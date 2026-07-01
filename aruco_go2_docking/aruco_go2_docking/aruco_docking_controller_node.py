#!/usr/bin/env python3
"""
aruco_docking_controller_node
------------------------------
State machine:

  DOCKING ──(target reached)──► SETTLING ──(settle_delay)──► SITTING
     ▲                                                           │
     │                             (joints confirm sit OR timeout)
     │                                    ↓
     │                          (charge_check_delay 대기)
     │                                    ↓
     │                                CHECKING
     │                              /          \\
     │                   success             failed
     │                      │                   │
     │                    DONE         (retry_count < max)
     │                                          │
     └──────────── RETRY_STANDUP ◄──────────────┘
          GetUp → loco ON → backup until marker visible
          (marker visible → immediately re-approach)
          (standup_delay elapsed → re-enter DOCKING to wait)
                              │ (max retries exceeded)
                            FAILED
"""

import enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String


class State(enum.Enum):
    DOCKING       = 'DOCKING'
    SETTLING      = 'SETTLING'
    SITTING       = 'SITTING'
    CHECKING      = 'CHECKING'
    RETRY_STANDUP = 'RETRY_STANDUP'
    DONE          = 'DONE'
    FAILED        = 'FAILED'


class ArucoDockingControllerNode(Node):

    def __init__(self):
        super().__init__('aruco_docking_controller_node')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('target_distance', 0.60)
        self.declare_parameter('max_linear_x', 0.20)
        # Minimum forward/back speed floor: the Go2 sport controller has a
        # velocity deadband, so commands below ~0.15 m/s barely move the robot.
        # Any non-zero drive command is floored to this to avoid stuttering.
        self.declare_parameter('min_linear_x', 0.15)
        self.declare_parameter('max_angular_z', 0.30)
        self.declare_parameter('Kp_linear', 0.5)
        self.declare_parameter('Kp_angular', 1.0)
        self.declare_parameter('goal_tol_dist', 0.05)
        self.declare_parameter('goal_tol_lateral', 0.02)
        self.declare_parameter('marker_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_pose_topic', '/aruco/marker_pose')
        self.declare_parameter('sit_on_arrival', True)
        self.declare_parameter('joy_topic', '/joy')
        # Timing
        self.declare_parameter('settle_delay_sec', 1.5)
        # After alignment, hold a full stop (cmd_vel=0) in locomotion for this long
        # so residual sideways velocity is killed before GetDown's damp mode (kp=0)
        # takes over — otherwise the robot coasts/slides sideways while "sitting".
        self.declare_parameter('pre_sit_stop_sec', 1.0)
        # Max time to wait for joint-confirmed sit before forcing proceed.
        self.declare_parameter('sit_confirm_timeout_sec', 15.0)
        # Time to wait AFTER sit is confirmed before reading /charging_state.
        self.declare_parameter('charge_check_delay_sec', 3.0)
        # 규격서: 약 10초간 충전 상태 유지 확인 (sustained charging duration)
        self.declare_parameter('charge_wait_timeout_sec', 3.0)
        # Max wait for the first /charging_state message before treating as failure.
        self.declare_parameter('charge_detect_timeout_sec', 5.0)
        # Retry
        self.declare_parameter('max_retries', 3)
        # Time budget for GetUp + locomotion trigger (before backup starts).
        self.declare_parameter('standup_delay_sec', 8.0)
        self.declare_parameter('locomotion_trigger_delay_sec', 3.0)
        self.declare_parameter('backup_start_delay_sec', 4.0)
        # Reverse speed [m/s].
        self.declare_parameter('backup_speed', 0.1)
        # Max backup duration [s] — safety limit if marker never appears.
        self.declare_parameter('backup_max_sec', 10.0)
        # Set false to skip charging check.
        self.declare_parameter('enable_charging_check', True)
        # Sit detection via /joint_states
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('sit_thigh_target', 1.36)
        self.declare_parameter('sit_calf_target', -2.65)
        self.declare_parameter('sit_joint_tolerance', 0.15)
        # Stand detection via /joint_states (calf straightens to ~-1.50 when standing)
        self.declare_parameter('stand_calf_target', -1.50)
        self.declare_parameter('stand_calf_tolerance', 0.50)
        # Heading alignment in SETTLING
        self.declare_parameter('heading_tol', 0.15)            # R[0,2] threshold (~8.6°)
        self.declare_parameter('settle_align_timeout_sec', 3.0) # extra time after settle_delay
        # Yaw gain to rotate base_link x-axis parallel to marker normal.
        # If the robot rotates the WRONG way during settling (heading_x grows
        # instead of → 0), set this negative.
        self.declare_parameter('align_yaw_gain', 1.0)
        # Pose filter for noisy real-camera marker detection. OFF by default.
        self.declare_parameter('pose_filter_alpha', 1.0)   # EMA: 1.0=off, lower=smoother (more lag)
        self.declare_parameter('pose_outlier_dist', 0.0)   # [m] reject frames jumping > this; 0=off

        # Camera mounting in base_link. The marker pose from the detector is in
        # the camera OPTICAL frame (x=right, y=down, z=forward). We convert it to
        # base_link distances. With the camera pitched DOWN by camera_pitch_deg
        # (rotation about base_link y), the forward distance is the projection:
        #   base_link x (forward) = camera_offset_x + cosθ·tvec.z − sinθ·tvec.y
        #   base_link y (left)    = camera_offset_y − tvec.x        (pitch-independent)
        # camera_pitch_deg = 0 → bl_x = camera_offset_x + tvec.z (flat/forward).
        # POSITIVE camera_pitch_deg = camera tilted DOWN (nose toward the ground).
        # target_distance is interpreted as the desired base_link-x range.
        self.declare_parameter('camera_offset_x', 0.345)   # base_link→camera forward [m]
        self.declare_parameter('camera_offset_y', 0.0)     # base_link→camera left(+) [m]
        self.declare_parameter('camera_pitch_deg', 0.0)    # down-tilt [deg], + = nose down

        self.target_dist          = self.get_parameter('target_distance').value
        self.cam_off_x            = self.get_parameter('camera_offset_x').value
        self.cam_off_y            = self.get_parameter('camera_offset_y').value
        self.cam_pitch            = np.deg2rad(self.get_parameter('camera_pitch_deg').value)
        self._cos_pitch           = float(np.cos(self.cam_pitch))
        self._sin_pitch           = float(np.sin(self.cam_pitch))
        self.max_lin              = self.get_parameter('max_linear_x').value
        self.min_lin              = self.get_parameter('min_linear_x').value
        self.max_ang              = self.get_parameter('max_angular_z').value
        self.kp_lin               = self.get_parameter('Kp_linear').value
        self.kp_ang               = self.get_parameter('Kp_angular').value
        self.tol_dist             = self.get_parameter('goal_tol_dist').value
        self.tol_lat              = self.get_parameter('goal_tol_lateral').value
        self.timeout              = self.get_parameter('marker_timeout_sec').value
        self.sit_on_arrival       = self.get_parameter('sit_on_arrival').value
        self.settle_delay         = self.get_parameter('settle_delay_sec').value
        self.pre_sit_stop         = self.get_parameter('pre_sit_stop_sec').value
        self.sit_confirm_timeout  = self.get_parameter('sit_confirm_timeout_sec').value
        self.charge_check_delay   = self.get_parameter('charge_check_delay_sec').value
        self.charge_wait_timeout  = self.get_parameter('charge_wait_timeout_sec').value
        self.charge_detect_timeout = self.get_parameter('charge_detect_timeout_sec').value
        self.max_retries          = self.get_parameter('max_retries').value
        self.standup_delay        = self.get_parameter('standup_delay_sec').value
        self.loco_trigger_delay   = self.get_parameter('locomotion_trigger_delay_sec').value
        self.backup_start_delay   = self.get_parameter('backup_start_delay_sec').value
        self.backup_speed         = self.get_parameter('backup_speed').value
        self.backup_max_sec       = self.get_parameter('backup_max_sec').value
        self.enable_charging_check = self.get_parameter('enable_charging_check').value
        self.sit_thigh_target     = self.get_parameter('sit_thigh_target').value
        self.sit_calf_target      = self.get_parameter('sit_calf_target').value
        self.sit_joint_tol        = self.get_parameter('sit_joint_tolerance').value
        self.stand_calf_target    = self.get_parameter('stand_calf_target').value
        self.stand_calf_tol       = self.get_parameter('stand_calf_tolerance').value
        self.heading_tol          = self.get_parameter('heading_tol').value
        self.settle_align_timeout = self.get_parameter('settle_align_timeout_sec').value
        self.align_yaw_gain       = self.get_parameter('align_yaw_gain').value
        self.pose_filter_alpha    = self.get_parameter('pose_filter_alpha').value
        self.pose_outlier_dist    = self.get_parameter('pose_outlier_dist').value
        cmd_vel_topic      = self.get_parameter('cmd_vel_topic').value
        pose_topic         = self.get_parameter('marker_pose_topic').value
        joy_topic          = self.get_parameter('joy_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # ── state ────────────────────────────────────────────────────────────
        self.state              = State.DOCKING
        self.state_enter_time   = None
        self.last_marker_time   = None
        self.last_tvec          = None
        self.last_quat          = None   # (qx, qy, qz, qw) for heading alignment
        self._pose_filt         = None   # EMA state [z_m, x_m, heading_x]
        self._pose_filt_time    = None
        self._pose_reject       = 0
        self.last_charging      = None
        self.retry_count        = 0
        self.loco_sent          = False
        self.backup_started     = False   # True once backup phase begins in RETRY_STANDUP
        self.standup_sent       = False   # True once GetUp (A) is sent in RETRY_STANDUP
        self.standup_sent_time  = None    # Clock time when GetUp was sent
        self.loco_sent_time     = None    # Clock time when locomotion trigger was sent
        self.backup_start_clock = None    # Clock time when backup phase began
        self.joint_positions    = {}
        self.sit_confirmed_time      = None
        self.settle_stop_time        = None   # when the pre-sit full-stop dwell began
        self.charging_confirmed_time = None   # when 'charging success' first detected (10 s sustain timer)
        self.aruco_arrive_published  = False  # prevent duplicate aruco_arrive per docking attempt

        # ── subscribers / publishers ─────────────────────────────────────────
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self.create_subscription(String, '/charging_state', self._charging_cb, 10)
        self.create_subscription(JointState, joint_states_topic,
                                 self._joint_states_cb, 10)
        self.cmd_pub         = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.joy_pub         = self.create_publisher(Joy, joy_topic, 10)
        self.aruco_state_pub = self.create_publisher(String, '/aruco_state', 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Docking controller ready  target={self.target_dist}m  '
            f'max_retries={self.max_retries}  '
            f'charging_check={"ON" if self.enable_charging_check else "OFF"}  '
            f'sit_confirm_timeout={self.sit_confirm_timeout}s  '
            f'charge_check_delay={self.charge_check_delay}s  '
            f'backup_max={self.backup_max_sec}s')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        self.last_marker_time = self.get_clock().now()
        self.last_tvec = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        self.last_quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def _charging_cb(self, msg: String):
        self.last_charging = msg.data

    def _joint_states_cb(self, msg: JointState):
        self.joint_positions = dict(zip(msg.name, msg.position))

    # ── helpers ───────────────────────────────────────────────────────────────

    def _elapsed(self) -> float:
        if self.state_enter_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_enter_time).nanoseconds * 1e-9

    def _enter(self, new_state: State):
        self.get_logger().info(f'[FSM] {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        if new_state == State.DONE:
            aruco_msg = String()
            aruco_msg.data = 'aruco_success'
            self.aruco_state_pub.publish(aruco_msg)
            self.get_logger().info('[aruco_state] aruco_success — shutting down in 0.5 s')
            self.create_timer(0.5, self._do_shutdown)
        elif new_state == State.FAILED:
            aruco_msg = String()
            aruco_msg.data = 'aruco_failed'
            self.aruco_state_pub.publish(aruco_msg)
            self.get_logger().error(
                '[aruco_state] aruco_failed — standing up, then shutting down.')
            # Drive GetUp from a clean state in _failed() (flags may be dirty
            # from a prior RETRY_STANDUP). Shutdown happens once standing.
            self.standup_sent      = False
            self.standup_sent_time = None
        elif new_state == State.SETTLING:
            self.settle_stop_time = None
        elif new_state == State.RETRY_STANDUP:
            self.aruco_arrive_published = False

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _do_shutdown(self):
        self._stop()
        self.get_logger().info('Aruco docking service terminated.')
        rclpy.shutdown()

    def _failed(self):
        """Docking failed: stand the robot up and leave it standing still, then
        shut down. No locomotion trigger — we only get it off the pad to a
        stable stand, we do not enable walking."""
        elapsed = self._elapsed()

        # ── Phase 0: wait for full sit, then send GetUp (A) ──────────────────
        if not self.standup_sent:
            if self._is_sitting():
                self.get_logger().info('[FAILED] Full sit confirmed — standing up (A).')
                self.joy_pub.publish(self._joy(1, 0))  # A → GetUp
                self.standup_sent      = True
                self.standup_sent_time = self.get_clock().now()
            elif elapsed >= self.sit_confirm_timeout:
                self.get_logger().warn(
                    f'[FAILED] Full sit not confirmed after {elapsed:.1f}s — '
                    'standing up anyway.')
                self.joy_pub.publish(self._joy(1, 0))
                self.standup_sent      = True
                self.standup_sent_time = self.get_clock().now()
            else:
                self._stop()
            return

        # ── Phase 1: wait until actually standing, then hold stand + shutdown ─
        since_getup = (self.get_clock().now() - self.standup_sent_time).nanoseconds * 1e-9
        if self._is_standing():
            self.get_logger().info(
                f'[FAILED] ✓ Standing confirmed ({since_getup:.1f}s) — '
                'holding stand, shutting down.')
        elif since_getup >= self.standup_delay:
            self.get_logger().warn(
                f'[FAILED] Stand not confirmed after {since_getup:.1f}s — '
                'shutting down anyway.')
        else:
            self.get_logger().info(
                f'[FAILED] Standing up… ({since_getup:.1f}/{self.standup_delay:.0f}s)',
                throttle_duration_sec=1.0)
            self._stop()
            return

        # Robot is standing; leave it there and terminate the controller.
        self._do_shutdown()

    def _joy(self, *buttons, axes=None) -> Joy:
        """Build a Joy message.

        F710 button map: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
        F710 axis  map:  Lx=0, Ly=1, LT=2, Rx=3, Ry=4, RT=5, DPadX=6, DPadY=7

        Compound gamepad actions (rl_sim):
          RB_DPadUp → buttons[5]=1  AND  axes[7]=1.0   (locomotion ON)
          LB_X      → buttons[4]=1  AND  buttons[2]=1  (passive mode)
        """
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.axes    = list(axes) if axes is not None else [0.0] * 8
        j.buttons = [0]   * 12
        for i, v in enumerate(buttons):
            j.buttons[i] = int(v)
        return j

    def _marker_visible(self) -> bool:
        """True if a fresh marker detection arrived within timeout."""
        if self.last_marker_time is None:
            return False
        age = (self.get_clock().now() - self.last_marker_time).nanoseconds * 1e-9
        return age < self.timeout

    def _marker_frame_pose(self):
        """Base_link pose in the FIXED marker frame (angle-invariant).

        Returns (z_m, x_m, heading_x, heading_deg) or None if no marker yet.
          z_m       : base_link distance along the marker normal (forward range)
          x_m       : base_link lateral offset along the marker x-axis (0 = centerline)
          heading_x : R[0,2]; 0 ⇒ base_link x-axis parallel to the marker normal
          heading_deg = asin(heading_x) in degrees

        solvePnP gives the marker pose in the camera optical frame (R, t). The
        camera origin in the marker frame is -Rᵀ·t; base_link sits camera_offset
        behind the camera, so that offset is transformed in too. Because these are
        POSITIONS in the fixed marker frame, they do NOT change when the robot
        merely rotates — unlike the raw camera-frame tvec.x / tvec.z.
        """
        if self.last_tvec is None or self.last_quat is None:
            return None
        tx, ty, tz = (float(v) for v in self.last_tvec)
        qx, qy, qz, qw = self.last_quat
        # rotation matrix (marker→camera): column 0 (marker x) and column 2 (normal)
        r00 = 1.0 - 2.0*(qy*qy + qz*qz); r10 = 2.0*(qx*qy + qz*qw); r20 = 2.0*(qx*qz - qy*qw)
        r02 = 2.0*(qx*qz + qy*qw);       r12 = 2.0*(qy*qz - qx*qw); r22 = 1.0 - 2.0*(qx*qx + qy*qy)
        # camera origin in marker frame = -Rᵀ·t
        z_cam = -(r02*tx + r12*ty + r22*tz)
        x_cam = -(r00*tx + r10*ty + r20*tz)
        # shift camera→base_link: base_link at (cam_off_y, 0, -cam_off_x) in optical coords
        ox, oy = self.cam_off_x, self.cam_off_y
        z_m = z_cam + (r02*oy - r22*ox)
        x_m = x_cam + (r00*oy - r20*ox)
        heading_x = r02
        z_m, x_m, heading_x = self._filter_pose(z_m, x_m, heading_x)
        heading_deg = float(np.degrees(np.arcsin(float(np.clip(heading_x, -1.0, 1.0)))))
        return z_m, x_m, heading_x, heading_deg

    def _filter_pose(self, z_m, x_m, heading_x):
        """Optional EMA smoothing + outlier rejection for a noisy real camera.
        Disabled by default (pose_filter_alpha=1.0, pose_outlier_dist=0.0): returns
        the raw values untouched. Re-seeds after a >0.5 s gap (marker was lost)."""
        a = self.pose_filter_alpha
        if a >= 1.0 and self.pose_outlier_dist <= 0.0:
            return z_m, x_m, heading_x                      # filtering fully OFF
        now = self.get_clock().now()
        gap = (self._pose_filt_time is None or
               (now - self._pose_filt_time).nanoseconds * 1e-9 > 0.5)
        self._pose_filt_time = now
        if gap or self._pose_filt is None:
            self._pose_filt = [z_m, x_m, heading_x]         # (re)seed after gap / first sample
            self._pose_reject = 0
            return z_m, x_m, heading_x
        fz, fx, fh = self._pose_filt
        # reject a single wild jump (keep last estimate); force-accept after 10 in a row
        if self.pose_outlier_dist > 0.0:
            jump = max(abs(z_m - fz), abs(x_m - fx))
            if jump > self.pose_outlier_dist and self._pose_reject < 10:
                self._pose_reject += 1
                return fz, fx, fh
        self._pose_reject = 0
        fz = a * z_m + (1.0 - a) * fz
        fx = a * x_m + (1.0 - a) * fx
        fh = a * heading_x + (1.0 - a) * fh
        self._pose_filt = [fz, fx, fh]
        return fz, fx, fh

    def _is_sitting(self) -> bool:
        """True when joint_states confirm the robot is fully in the sit pose."""
        if not self.joint_positions:
            return False
        thigh_joints = {n: p for n, p in self.joint_positions.items()
                        if 'thigh' in n.lower()}
        calf_joints  = {n: p for n, p in self.joint_positions.items()
                        if 'calf' in n.lower()}
        if not thigh_joints or not calf_joints:
            return False
        for _, pos in thigh_joints.items():
            if abs(pos - self.sit_thigh_target) > self.sit_joint_tol:
                return False
        for _, pos in calf_joints.items():
            if abs(pos - self.sit_calf_target) > self.sit_joint_tol:
                return False
        return True

    def _is_standing(self) -> bool:
        """True when calf joints have straightened to the standing pose (~-1.50).

        Used in RETRY_STANDUP to wait for GetUp to actually finish (~10-12 s)
        before triggering locomotion — fixed delays underestimate the animation.
        """
        if not self.joint_positions:
            return False
        calf_joints = {n: p for n, p in self.joint_positions.items()
                       if 'calf' in n.lower()}
        if not calf_joints:
            return False
        for _, pos in calf_joints.items():
            if abs(pos - self.stand_calf_target) > self.stand_calf_tol:
                return False
        return True

    # ── control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        {
            State.DOCKING:       self._docking,
            State.SETTLING:      self._settling,
            State.SITTING:       self._sitting,
            State.CHECKING:      self._checking,
            State.RETRY_STANDUP: self._retry_standup,
            State.DONE:          self._stop,
            State.FAILED:        self._failed,
        }[self.state]()

    # ── state handlers ────────────────────────────────────────────────────────

    def _docking(self):
        cmd = Twist()
        now = self.get_clock().now()

        if self.last_marker_time is None:
            self.get_logger().info(
                'Waiting for marker...', throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        age = (now - self.last_marker_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.get_logger().warn(
                f'Marker lost ({age:.2f}s) — holding.',
                throttle_duration_sec=1.0)
            self.cmd_pub.publish(cmd)
            return

        # Robot pose in the FIXED marker frame (angle-invariant). Distance is now
        # measured along the marker NORMAL (z_m), not the camera optical axis, so an
        # oblique approach no longer misreads distance. `lateral` is kept as the
        # camera-frame value ONLY for DOCKING steering (yaw to face the marker →
        # keeps it inside the camera FOV during the coarse approach).
        mf = self._marker_frame_pose()
        if mf is None:
            self.cmd_pub.publish(cmd)
            return
        z_m, x_m, heading_x, heading_deg = mf
        bl_x    = z_m                                        # base_link→marker along normal
        err     = bl_x - self.target_dist
        lateral = float(self.last_tvec[0]) - self.cam_off_y  # camera-frame (steer/FOV only)

        self.get_logger().info(
            f'z_m={z_m:.3f}m  err={err:+.3f}m  x_m={x_m:+.3f}m  '
            f'cam_lat={lateral:+.3f}m  heading={heading_deg:+.1f}°',
            throttle_duration_sec=0.5)

        # Arrive already aligned: require both the target range AND being on the
        # marker centerline (x_m ≈ 0) before handing off to SETTLING.
        if abs(err) < self.tol_dist and abs(x_m) < self.tol_lat:
            self.get_logger().info('Target reached (on marker centerline) — settling.')
            self._stop()
            self._enter(State.SETTLING)
            return

        # Forward/back on the marker-normal range (z_m → target). Zero within
        # tolerance so the robot HOLDS distance (no forward/back jitter) while it
        # is still strafing to center — otherwise the min-speed floor would make
        # it bang-bang past the target.
        if abs(err) < self.tol_dist:
            cmd.linear.x = 0.0
        else:
            v = self.kp_lin * err
            cmd.linear.x = (float(np.clip(v, self.min_lin, self.max_lin)) if v >= 0.0
                            else float(np.clip(v, -self.max_lin, -self.min_lin)))
        # Strafe onto the marker centerline WHILE approaching, so the robot
        # arrives already aligned instead of squaring up only at the end.
        # (Same sign convention as SETTLING: x_m>0 ⇒ strafe left.)
        if abs(x_m) < self.tol_lat:
            cmd.linear.y = 0.0
        else:
            vy_mag = float(np.clip(self.kp_lin * abs(x_m), self.min_lin, self.max_lin))
            cmd.linear.y = vy_mag if x_m > 0.0 else -vy_mag
        # Yaw keeps the marker centered in the camera FOV during the approach.
        cmd.angular.z = float(np.clip(-self.kp_ang * lateral, -self.max_ang, self.max_ang))

        self.get_logger().info(
            f'cmd  vx={cmd.linear.x:.2f}  vy={cmd.linear.y:.2f}  w={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5)
        self.cmd_pub.publish(cmd)

    def _settling(self):
        """Align so base_link x-axis becomes collinear with the marker normal,
        then sit squarely.

        DECOUPLED holonomic control (R is translation-invariant, so heading_x is
        a pure heading measure — independent of lateral position):
          • angular.z ← rotate until heading is parallel to marker normal
                        (heading_x = R[0,2] → 0). Directly removes head twist.
          • linear.y  ← strafe until the marker is centered (lateral = tvec[0] → 0).

        Converged state: parallel to normal AND centered ⇒ robot x-axis collinear
        with the marker normal through its center. Sit only once both are within
        tolerance (or the alignment timeout fires).
        """
        # ── Pre-sit full stop ───────────────────────────────────────────────
        # Once aligned, we hold cmd_vel=0 in locomotion for pre_sit_stop seconds
        # so the policy fully arrests sideways velocity. Only then send B, so the
        # robot doesn't coast/slide once GetDown enters damp mode (kp=0).
        if self.settle_stop_time is not None:
            self._stop()
            stopping = (self.get_clock().now() - self.settle_stop_time).nanoseconds * 1e-9
            if stopping < self.pre_sit_stop:
                self.get_logger().info(
                    f'Full-stopping before sit… ({stopping:.1f}/{self.pre_sit_stop:.1f}s)',
                    throttle_duration_sec=0.5)
                return
            if self.sit_on_arrival:
                self.get_logger().info('Stopped & aligned — commanding sit (B).')
                self.joy_pub.publish(self._joy(0, 1))  # B → GetDown
            self.sit_confirmed_time = None
            self._enter(State.SITTING)
            return

        elapsed = self._elapsed()

        if self.last_tvec is None or self.last_quat is None:
            self._stop()
            return

        # ── Angle-invariant alignment in the FIXED marker frame ──────────────
        mf = self._marker_frame_pose()
        if mf is None:
            self._stop()
            return
        z_m, x_m, heading_x, heading_deg = mf
        dist_err = z_m - self.target_dist

        heading_ok = abs(heading_x) < self.heading_tol
        lateral_ok = abs(x_m)       < self.tol_lat
        dist_ok    = abs(dist_err)  < self.tol_dist

        cmd = Twist()
        # Hold the forward range along the marker normal (z_m → target) throughout
        # both sub-phases. Measuring along the normal removes the oblique-approach
        # cos(φ) error.
        if dist_ok:
            cmd.linear.x = 0.0
        else:
            v = self.kp_lin * dist_err
            cmd.linear.x = (float(np.clip(v, self.min_lin, self.max_lin)) if v > 0.0
                            else float(np.clip(v, -self.max_lin, -self.min_lin)))

        # SEQUENTIAL: align HEADING first, THEN strafe to center. Once the robot is
        # perpendicular, base_link-y is parallel to the marker x-axis so the strafe
        # is a PURE lateral move (no distance coupling). Re-checked every tick, so if
        # heading drifts back out it rotates again before resuming the strafe.
        if not heading_ok:
            # Phase 1 — rotate to square up to the marker normal; no strafe yet.
            cmd.angular.z = float(np.clip(-self.align_yaw_gain * heading_x,
                                          -self.max_ang, self.max_ang))
            cmd.linear.y = 0.0
            phase = 'ROT'
        else:
            # Phase 2 — heading aligned → strafe onto the marker centerline.
            # (x_m>0 ⇒ robot on the marker's right ⇒ strafe LEFT, linear.y>0.)
            cmd.angular.z = 0.0
            if lateral_ok:
                cmd.linear.y = 0.0
            else:
                vy_mag = float(np.clip(self.kp_lin * abs(x_m), self.min_lin, self.max_lin))
                cmd.linear.y = vy_mag if x_m > 0.0 else -vy_mag
            phase = 'CENTER'
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Settling[{phase}]… z_m={z_m:.3f}m[{"OK" if dist_ok else ".."}] '
            f'x_m={x_m:+.3f}m[{"OK" if lateral_ok else ".."}] '
            f'heading={heading_deg:+.1f}°[{"OK" if heading_ok else ".."}] ({elapsed:.1f}s)',
            throttle_duration_sec=0.5)

        # Proceed once settle_delay elapsed AND aligned (or alignment timeout)
        if elapsed < self.settle_delay:
            return
        aligned = heading_ok and lateral_ok and dist_ok
        if not aligned and elapsed < self.settle_delay + self.settle_align_timeout:
            return
        if not aligned:
            self.get_logger().warn(
                f'Alignment timeout (z_m={z_m:.3f}, x_m={x_m:+.3f}, '
                f'heading={heading_deg:+.1f}°) — sitting anyway.')

        # Begin the pre-sit full-stop dwell (handled at the top of subsequent ticks).
        self.get_logger().info(
            f'Aligned in marker frame — full-stopping {self.pre_sit_stop:.1f}s before sit.')
        self.settle_stop_time = self.get_clock().now()
        self._stop()

    def _sitting(self):
        self._stop()
        elapsed = self._elapsed()

        # Step 1: wait for joints to confirm sit
        if self.sit_confirmed_time is None:
            if self._is_sitting():
                self.sit_confirmed_time = self.get_clock().now()
                if not self.aruco_arrive_published:
                    aruco_msg = String()
                    aruco_msg.data = 'aruco_arrive'
                    self.aruco_state_pub.publish(aruco_msg)
                    self.aruco_arrive_published = True
                    self.get_logger().info('[aruco_state] aruco_arrive published (sit confirmed by joints)')
                self.get_logger().info(
                    f'✓ Sit confirmed by joint states ({elapsed:.1f}s). '
                    f'Waiting {self.charge_check_delay:.1f}s before charging check.')
            elif elapsed >= self.sit_confirm_timeout:
                self.sit_confirmed_time = self.get_clock().now()
                if not self.aruco_arrive_published:
                    aruco_msg = String()
                    aruco_msg.data = 'aruco_arrive'
                    self.aruco_state_pub.publish(aruco_msg)
                    self.aruco_arrive_published = True
                    self.get_logger().info('[aruco_state] aruco_arrive published (sit timeout fallback)')
                self.get_logger().warn(
                    f'Sit not confirmed by joints after {elapsed:.1f}s '
                    '— proceeding with timer fallback.')
            else:
                self.get_logger().info(
                    f'Waiting for sit… ({elapsed:.1f}/{self.sit_confirm_timeout:.0f}s)',
                    throttle_duration_sec=2.0)
            return

        # Step 2: wait charge_check_delay AFTER sit confirmed
        post_sit = (self.get_clock().now() - self.sit_confirmed_time).nanoseconds * 1e-9
        if post_sit < self.charge_check_delay:
            self.get_logger().info(
                f'Post-sit stabilisation… ({post_sit:.1f}/{self.charge_check_delay:.0f}s)',
                throttle_duration_sec=2.0)
            return

        # Step 3: proceed to charging check
        self._go_to_checking()

    def _go_to_checking(self):
        if not self.enable_charging_check:
            self.get_logger().info('[DONE] Charging check disabled — done.')
            self._enter(State.DONE)
            return
        self.last_charging           = None
        self.charging_confirmed_time = None
        self._enter(State.CHECKING)

    def _checking(self):
        self._stop()

        # ── Phase 1: wait for initial /charging_state message ─────────────────
        if self.last_charging is None:
            if self._elapsed() > self.charge_detect_timeout:
                self.get_logger().warn(
                    f'No /charging_state received in {self.charge_detect_timeout:.0f}s '
                    '— treating as failure.')
                self.last_charging = 'charging failed'
            else:
                return

        # ── Phase 2: 규격서 — 약 10초간 충전 상태 유지 확인 ──────────────────────
        if self.last_charging == 'charging success':
            if self.charging_confirmed_time is None:
                self.charging_confirmed_time = self.get_clock().now()
                self.get_logger().info(
                    f'Charging detected — confirming for {self.charge_wait_timeout:.0f}s…')

            sustained = (self.get_clock().now() - self.charging_confirmed_time).nanoseconds * 1e-9
            self.get_logger().info(
                f'Charging sustained: {sustained:.1f}/{self.charge_wait_timeout:.0f}s',
                throttle_duration_sec=2.0)

            if sustained >= self.charge_wait_timeout:
                self.get_logger().info(
                    f'[SUCCESS] {self.charge_wait_timeout:.0f}s sustained charging confirmed.')
                self._enter(State.DONE)   # → publishes aruco_success + auto-shutdown
            return

        # ── Phase 3: not charging — keep sitting & checking for the full window ──
        # 규격: 패드 위에 엎드린 채 약 10초간 충전 여부를 확인한 뒤, 그래도 충전이
        # 안 되면 일어난다. 첫 'charging failed' 메시지에 즉시 일어나지 않는다.
        if self.charging_confirmed_time is not None:
            self.get_logger().warn('Charging interrupted — resetting sustain timer.')
            self.charging_confirmed_time = None

        if self._elapsed() < self.charge_wait_timeout:
            self.get_logger().info(
                f'Sitting & checking charging… not charging yet '
                f'({self._elapsed():.1f}/{self.charge_wait_timeout:.0f}s)',
                throttle_duration_sec=1.0)
            return

        # ── Window elapsed without sustained charging → fail / retry ────────────
        if self.retry_count >= self.max_retries:
            self.get_logger().error(
                f'Charging failed after {self.retry_count} retries — giving up.')
            self._enter(State.FAILED)   # → publishes aruco_failed + auto-shutdown
            return

        self.retry_count += 1
        self.get_logger().warn(
            f'No charging after {self._elapsed():.1f}s — retry {self.retry_count}/{self.max_retries}. '
            'Confirming full sit before stand-up.')
        self.loco_sent             = False
        self.loco_sent_time        = None
        self.backup_started        = False
        self.backup_start_clock    = None
        self.standup_sent          = False
        self.standup_sent_time     = None
        self.charging_confirmed_time = None
        self._enter(State.RETRY_STANDUP)

    def _retry_standup(self):
        elapsed = self._elapsed()

        # ── Phase 0: wait for robot to be fully sitting, then send GetUp ─────
        if not self.standup_sent:
            if self._is_sitting():
                self.get_logger().info('Full sit confirmed — sending GetUp (A).')
                self.joy_pub.publish(self._joy(1, 0))  # A → GetUp
                self.standup_sent      = True
                self.standup_sent_time = self.get_clock().now()
            elif elapsed >= self.sit_confirm_timeout:
                self.get_logger().warn(
                    f'Full sit not confirmed after {elapsed:.1f}s — sending GetUp anyway.')
                self.joy_pub.publish(self._joy(1, 0))
                self.standup_sent      = True
                self.standup_sent_time = self.get_clock().now()
            else:
                self.get_logger().info(
                    f'Waiting for full sit before GetUp… ({elapsed:.1f}/{self.sit_confirm_timeout:.0f}s)',
                    throttle_duration_sec=2.0)
                self._stop()
            return

        # Time elapsed since GetUp was sent
        since_getup = (self.get_clock().now() - self.standup_sent_time).nanoseconds * 1e-9

        # ── Phase 1: wait until the robot has ACTUALLY stood up, then trigger
        #            locomotion. GetUp animation takes ~10-12 s; triggering
        #            locomotion or cmd_vel before running_percent==1.0 is ignored
        #            by the FSM, so gate on joint-confirmed standing instead of a
        #            fixed (too-short) delay. ──────────────────────────────────
        if not self.loco_sent:
            if self._is_standing():
                self.get_logger().info(
                    f'✓ Stand confirmed by joints ({since_getup:.1f}s) — '
                    'sending locomotion trigger (RB_DPadUp).')
            elif since_getup >= self.standup_delay:
                self.get_logger().warn(
                    f'Stand not confirmed after {since_getup:.1f}s — '
                    'sending locomotion trigger anyway.')
            else:
                self.get_logger().info(
                    f'Standing up… ({since_getup:.1f}/{self.standup_delay:.0f}s)',
                    throttle_duration_sec=1.0)
                self._stop()
                return
            # RB_DPadUp: buttons[5]=1 (RB) AND axes[7]=1.0 (DPad Up)
            _axes = [0.0] * 8
            _axes[7] = 1.0
            self.joy_pub.publish(self._joy(0, 0, 0, 0, 0, 1, axes=_axes))
            self.loco_sent      = True
            self.loco_sent_time = self.get_clock().now()
            self._stop()
            return

        # ── Phase 2: let locomotion stabilize before commanding motion ───────
        since_loco = (self.get_clock().now() - self.loco_sent_time).nanoseconds * 1e-9
        if since_loco < self.loco_trigger_delay:
            self._stop()
            return

        # ── Phase 3+: back up until marker visible ───────────────────────────
        # Clear stale marker data once at the start of backup
        if not self.backup_started:
            self.get_logger().info('Backup phase started — clearing stale marker data.')
            self.last_marker_time   = None
            self.last_tvec          = None
            self.backup_started     = True
            self.backup_start_clock = self.get_clock().now()

        backup_elapsed = (self.get_clock().now() - self.backup_start_clock).nanoseconds * 1e-9

        # Marker re-acquired → stop and re-approach immediately
        if self._marker_visible():
            self._stop()
            self.get_logger().info(
                f'✓ Marker re-acquired after {backup_elapsed:.1f}s backup! '
                'Re-approaching.')
            self.last_charging      = None
            self.sit_confirmed_time = None
            self._enter(State.DOCKING)
            return

        # Still no marker — keep backing up within budget
        if backup_elapsed < self.backup_max_sec:
            cmd = Twist()
            cmd.linear.x = -self.backup_speed
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'Backing up… ({backup_elapsed:.1f}/{self.backup_max_sec:.0f}s) '
                'waiting for marker.',
                throttle_duration_sec=1.0)
            return

        # Backup timed out — re-enter DOCKING and wait in place for marker
        self._stop()
        self.get_logger().warn(
            f'Marker not found after {backup_elapsed:.1f}s backup. '
            'Re-entering DOCKING — waiting for marker in place.')
        self.last_charging      = None
        self.sit_confirmed_time = None
        self._enter(State.DOCKING)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockingControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
