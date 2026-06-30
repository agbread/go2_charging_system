#!/usr/bin/env python3
"""
mock_charging_node
------------------
Simulation-only: publishes /charging_state once the robot has lain down on the
charging pad (aruco_arrive received on /aruco_state).

Subscribes to:  /aruco_state    (std_msgs/String)
Publishes to:   /charging_state (std_msgs/String)

Toggle at runtime:
  ros2 param set /mock_charging_node charging_success true   # → "charging success"
  ros2 param set /mock_charging_node charging_success false  # → "charging failed"
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class MockChargingNode(Node):

    def __init__(self):
        super().__init__('mock_charging_node')
        self.declare_parameter('charging_success', False)

        self._active = False  # publish only after aruco_arrive

        self.pub = self.create_publisher(String, '/charging_state', 10)
        self.create_subscription(String, '/aruco_state',
                                 self._aruco_state_cb, 10)
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            'Mock charging node ready. '
            'Waiting for /aruco_state == aruco_arrive before publishing /charging_state.\n'
            'Toggle: ros2 param set /mock_charging_node charging_success true/false')

    def _aruco_state_cb(self, msg: String):
        if msg.data == 'aruco_arrive':
            if not self._active:
                self._active = True
                self.get_logger().info('[mock] aruco_arrive received — publishing /charging_state.')
        elif msg.data in ('aruco_success', 'aruco_failed'):
            if self._active:
                self._active = False
                self.get_logger().info(f'[mock] {msg.data} received — stopping /charging_state.')

    def _publish(self):
        if not self._active:
            return

        success = self.get_parameter('charging_success').value
        msg = String()
        msg.data = 'charging success' if success else 'charging failed'
        self.pub.publish(msg)
        self.get_logger().info(f'/charging_state: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MockChargingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
