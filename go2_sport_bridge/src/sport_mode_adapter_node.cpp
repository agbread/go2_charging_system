// sport_mode_adapter_node
// ------------------------
// Bridges the ArUco docking pipeline to the Go2's BUILT-IN (sport-mode)
// controller — NO reinforcement-learning policy involved.
//
// The docking controller (aruco_docking_controller_node) speaks a generic,
// controller-agnostic language:
//   • /cmd_vel (Twist)  — "drive this fast"
//   • /joy     (Joy)    — A = stand up, B = sit/lie down, RB+DPadUp = locomotion on
//   • needs /joint_states (sit/stand confirmation) and /charging_state.
//
// This node translates that language into unitree_sdk2 SportClient calls and
// republishes the robot's LowState (over rt/lowstate) as ROS topics:
//
//   /cmd_vel  ──►  SportClient::Move(vx, vy, vyaw)
//   /joy  A   ──►  SportClient::RecoveryStand()   (get up)
//   /joy  B   ──►  SportClient::StandDown()        (lie down onto the pad)
//   /joy  RB+DPadUp ─► SportClient::BalanceStand() (ready to walk again)
//   rt/lowstate.motor_state[]  ──►  /joint_states
//   rt/lowstate.bms_state      ──►  /charging_state
//
// So the docking *brain* (aruco_detector + aruco_docking_controller) is reused
// unchanged; only the actuation back-end differs from the Gazebo/RL setup.
//
// Usage:
//   ros2 run go2_sport_bridge sport_mode_adapter_node \
//       --ros-args -p network_interface:=eth0
//
// Prerequisite: the robot must be in sport (high-level) mode — i.e. the factory
// walking controller is active, not low-level/RL control.

#include <array>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"

namespace ug = unitree_go::msg::dds_;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

// BMS status codes (unitree_go BmsState.status)
static constexpr uint8_t BMS_PRECHG = 6;  // pre-charging
static constexpr uint8_t BMS_CHG    = 7;  // normal charging

// Go2 LowState motor order: leg*3 + joint, legs FR,FL,RR,RL; joints hip,thigh,calf.
// Names MUST contain "thigh"/"calf" — the docking controller's _is_sitting()/
// _is_standing() match joints by those substrings.
static const std::vector<std::string> JOINT_NAMES = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

static float clampf(double v, double lo, double hi)
{
    return static_cast<float>(std::max(lo, std::min(hi, v)));
}

class SportModeAdapter : public rclcpp::Node
{
public:
    SportModeAdapter() : rclcpp::Node("go2_sport_mode_adapter")
    {
        network_interface_   = declare_parameter<std::string>("network_interface", "eth0");
        cmd_vel_topic_       = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        joy_topic_           = declare_parameter<std::string>("joy_topic", "/joy");
        joint_states_topic_  = declare_parameter<std::string>("joint_states_topic", "/joint_states");
        charging_state_topic_= declare_parameter<std::string>("charging_state_topic", "/charging_state");
        max_vx_              = declare_parameter<double>("max_vx", 0.6);
        max_vy_              = declare_parameter<double>("max_vy", 0.4);
        max_vyaw_            = declare_parameter<double>("max_vyaw", 0.8);
        joint_rate_          = declare_parameter<double>("joint_state_rate", 50.0);
        charge_rate_         = declare_parameter<double>("charging_state_rate", 1.0);
        charge_cur_thresh_   = declare_parameter<int>("charge_current_threshold_ma", 0);
        cmd_debounce_        = declare_parameter<double>("joy_debounce_sec", 2.0);
        balance_on_start_    = declare_parameter<bool>("balance_stand_on_start", true);
        sport_timeout_       = declare_parameter<double>("sport_request_timeout_sec", 1.0);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            std::bind(&SportModeAdapter::cmdVelCb, this, std::placeholders::_1));
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 10,
            std::bind(&SportModeAdapter::joyCb, this, std::placeholders::_1));

        joint_pub_  = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
        charge_pub_ = create_publisher<std_msgs::msg::String>(charging_state_topic_, 10);

        last_cmd_time_ = now();
    }

    const std::string & networkInterface() const { return network_interface_; }

    // Called from main() AFTER ChannelFactory::Init().
    void initUnitree()
    {
        sport_ = std::make_unique<unitree::robot::go2::SportClient>();
        sport_->SetTimeout(static_cast<float>(sport_timeout_));
        sport_->Init();

        lowstate_sub_ = std::make_shared<ChannelSubscriber<ug::LowState_>>(TOPIC_LOWSTATE);
        lowstate_sub_->InitChannel(
            std::bind(&SportModeAdapter::lowStateHandler, this, std::placeholders::_1), 1);

        const auto joint_period = std::chrono::duration<double>(1.0 / std::max(1.0, joint_rate_));
        joint_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(joint_period),
            std::bind(&SportModeAdapter::publishJointStates, this));

        const auto charge_period = std::chrono::duration<double>(1.0 / std::max(0.1, charge_rate_));
        charge_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(charge_period),
            std::bind(&SportModeAdapter::publishChargingState, this));

        if (balance_on_start_) {
            RCLCPP_INFO(get_logger(), "BalanceStand() on start — robot ready to walk.");
            sport_->BalanceStand();
            walking_enabled_ = true;
        }

        RCLCPP_INFO(get_logger(),
            "Sport-mode adapter ready on '%s'.  cmd_vel='%s' joy='%s' → SportClient; "
            "lowstate → '%s' + '%s'.",
            network_interface_.c_str(), cmd_vel_topic_.c_str(), joy_topic_.c_str(),
            joint_states_topic_.c_str(), charging_state_topic_.c_str());
    }

private:
    // ── /cmd_vel → Move ────────────────────────────────────────────────────
    void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!sport_) return;
        // While sitting/lying (after StandDown) the docking controller keeps
        // publishing cmd_vel=0. Forwarding Move() then would make the robot try
        // to balance-stand and stand up off the pad — so gate on walking_enabled_.
        if (!walking_enabled_) return;

        const float vx   = clampf(msg->linear.x,  -max_vx_,   max_vx_);
        const float vy   = clampf(msg->linear.y,  -max_vy_,   max_vy_);
        const float vyaw = clampf(msg->angular.z, -max_vyaw_, max_vyaw_);
        sport_->Move(vx, vy, vyaw);
    }

    // ── /joy → stand / sit / locomotion ─────────────────────────────────────
    // The docking controller emits each gamepad command as a single one-shot
    // Joy message, so we act per-message (debounced) rather than on edges.
    void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!sport_) return;
        const auto &b = msg->buttons;
        const auto &a = msg->axes;
        const bool A       = b.size() > 0 && b[0] != 0;
        const bool B       = b.size() > 1 && b[1] != 0;
        const bool X       = b.size() > 2 && b[2] != 0;
        const bool RB      = b.size() > 5 && b[5] != 0;
        const bool dpad_up = a.size() > 7 && a[7] > 0.5f;

        if (RB && dpad_up) {                       // locomotion ON (RL: enter Locomotion)
            if (fire("loco")) {
                RCLCPP_INFO(get_logger(), "[joy] RB+DPadUp → BalanceStand (locomotion ready)");
                sport_->BalanceStand();
                walking_enabled_ = true;
            }
            return;
        }
        if (B) {                                   // sit / lie down onto pad
            if (fire("standdown")) {
                RCLCPP_INFO(get_logger(), "[joy] B → StandDown (lie down for charging)");
                walking_enabled_ = false;
                sport_->StopMove();
                sport_->StandDown();
            }
            return;
        }
        if (X) {                                   // gentle get up (paired with StandDown)
            if (fire("standup_soft")) {
                RCLCPP_INFO(get_logger(), "[joy] X → StandUp (gentle get up)");
                walking_enabled_ = false;
                sport_->StandUp();
            }
            return;
        }
        if (A) {                                   // get up (forceful recovery, from any pose)
            if (fire("standup")) {
                RCLCPP_INFO(get_logger(), "[joy] A → RecoveryStand (get up)");
                // stay gated until the locomotion trigger so we don't auto-walk
                walking_enabled_ = false;
                sport_->RecoveryStand();
            }
            return;
        }
    }

    // Debounce identical commands so a burst / retransmit doesn't restart an
    // animation, while genuinely repeated commands (seconds apart) still fire.
    bool fire(const std::string &cmd)
    {
        const auto t = now();
        if (cmd == last_cmd_ && (t - last_cmd_time_).seconds() < cmd_debounce_)
            return false;
        last_cmd_ = cmd;
        last_cmd_time_ = t;
        return true;
    }

    // ── rt/lowstate handler (runs on a unitree DDS thread) ───────────────────
    void lowStateHandler(const void *message)
    {
        const auto *ls = static_cast<const ug::LowState_ *>(message);
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (int i = 0; i < 12; ++i) {
            q_[i]   = ls->motor_state()[i].q();
            dq_[i]  = ls->motor_state()[i].dq();
            tau_[i] = ls->motor_state()[i].tau_est();
        }
        bms_status_  = ls->bms_state().status();
        bms_current_ = ls->bms_state().current();
        bms_soc_     = ls->bms_state().soc();
        have_state_  = true;
    }

    void publishJointStates()
    {
        std::array<double, 12> q{}, dq{}, tau{};
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (!have_state_) return;
            q = q_; dq = dq_; tau = tau_;
        }
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name = JOINT_NAMES;
        js.position.assign(q.begin(), q.end());
        js.velocity.assign(dq.begin(), dq.end());
        js.effort.assign(tau.begin(), tau.end());
        joint_pub_->publish(js);
    }

    void publishChargingState()
    {
        uint8_t status; int32_t cur; uint8_t soc; bool ok;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            ok = have_state_; status = bms_status_; cur = bms_current_; soc = bms_soc_;
        }
        if (!ok) return;
        const bool charging =
            (status == BMS_CHG || status == BMS_PRECHG) || (cur > charge_cur_thresh_);
        std_msgs::msg::String m;
        m.data = charging ? "charging success" : "charging failed";
        charge_pub_->publish(m);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "BMS status=%u soc=%u%% current=%dmA → %s",
            status, soc, cur, m.data.c_str());
    }

    // params
    std::string network_interface_, cmd_vel_topic_, joy_topic_,
                joint_states_topic_, charging_state_topic_;
    double max_vx_, max_vy_, max_vyaw_, joint_rate_, charge_rate_,
           cmd_debounce_, sport_timeout_;
    int charge_cur_thresh_;
    bool balance_on_start_{true};

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr charge_pub_;
    rclcpp::TimerBase::SharedPtr joint_timer_, charge_timer_;

    // unitree
    std::unique_ptr<unitree::robot::go2::SportClient> sport_;
    ChannelSubscriberPtr<ug::LowState_> lowstate_sub_;

    // shared state
    std::mutex state_mtx_;
    std::array<double, 12> q_{}, dq_{}, tau_{};
    uint8_t bms_status_{0}, bms_soc_{0};
    int32_t bms_current_{0};
    bool have_state_{false};

    // control gating / debounce
    bool walking_enabled_{true};
    std::string last_cmd_;
    rclcpp::Time last_cmd_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SportModeAdapter>();

    // ChannelFactory must be initialised before any unitree channel/client.
    // NOTE: do NOT pass the network interface here. ROS 2 already runs on
    // CycloneDDS (RMW_IMPLEMENTATION=rmw_cyclonedds_cpp) with CYCLONEDDS_URI
    // binding domain 0 to the robot NIC (e.g. eth0). Creating the node above
    // already opened domain 0, so passing an interface makes the SDK try to
    // create domain 0 *explicitly* a second time → CycloneDDS throws
    // PreconditionNotMetError ("Failed to create domain explicitly") and the
    // process aborts. With an empty interface the SDK just joins the existing
    // domain configured by CYCLONEDDS_URI. (network_interface param is kept on
    // the node for documentation / launch compatibility but is intentionally
    // not forwarded to the SDK.)
    unitree::robot::ChannelFactory::Instance()->Init(0);
    node->initUnitree();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
