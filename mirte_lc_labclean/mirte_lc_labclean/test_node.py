"""Simple ROS2 test node for Mirte sensor-driven base control.

This module defines a basic test controller that subscribes to front range
sensors and publishes Twist commands for simple motion behavior.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class MirteTestController(Node):
    """ROS2 node for testing Mirte base control via range sensors."""

    def __init__(self):
        """Create the test controller node and initialize ROS2 interfaces."""
        super().__init__("test_node")

        self.twist_pub = self.create_publisher(
            Twist, "/mirte_base_controller/cmd_vel_unstamped", 10
        )
        self.right_range_sub = self.create_subscription(
            Range, "/mirte/distance/front_right", self.right_sensor_callback, 1
        )
        self.left_range_sub = self.create_subscription(
            Range, "/mirte/distance/front_left", self.left_sensor_callback, 1
        )
        self.controller_timer = self.create_timer(0.05, self.controller_callback)

        self.distance_left = 0.01
        self.distance_right = 0.01
        self.k_p_t = 10
        self.k_p_t_2 = 10
        self.k_p_l = 0.5

    def controller_callback(self):
        """Compute and publish wheel commands from front range sensors."""
        if self.distance_left + self.distance_right < 0.4:
            ang_vel = 5.0
        else:
            ang_vel = (self.k_p_t * (self.distance_left - self.distance_right))
        lin_vel = self.k_p_l * (self.distance_left + self.distance_right)

        twist_msg = Twist()
        twist_msg.linear.x = min(lin_vel, 0.6)
        twist_msg.angular.z = ang_vel
        self.twist_pub.publish(twist_msg)
        return
    
    def left_sensor_callback(self, msg: Range):
        """Update the left range measurement from the sensor.

        Args:
            msg (:class:`sensor_msgs.msg.Range`): The left distance measurement.
        """
        try:
            self.distance_left = min(msg.range, 0.4)
        except Exception as e:
            self.get_logger().error(f'{e}')
        return
    
    def right_sensor_callback(self, msg: Range):
        """Update the right range measurement from the sensor.

        Args:
            msg (:class:`sensor_msgs.msg.Range`): The right distance measurement.
        """
        try:
            self.distance_right = min(msg.range, 0.4)
        except Exception as e:
            self.get_logger().error(f'{e}')
        return

def main(args=None):
    """Initialize and run the Mirte test controller node.

    Args:
        args (list, optional): Arguments forwarded to :func:`rclpy.init`.
    """
    rclpy.init(args=args)
    rclpy.spin(MirteTestController())
    rclpy.shutdown()

if __name__== '__main__':
    main()

    