import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class MirteTestController(Node):

    def __init__(self):
        super().__init__("test_node")

        self.twist_pub = self.create_publisher(
            Twist, "/mirte_base_controller/cmd_vel_unstamped", 10
        )
        self.right_range_sub = self.create_subscription(
            Range, "/mirte/distance/front_right", self.left_sensor_callback, 1
        )
        self.left_range_sub = self.create_subscription(
            Range, "/mirte/distance/front_left", self.right_sensor_callback, 1
        )
        self.controller_timer = self.create_timer(0.05, self.controller_callback)

        self.distance_left = 0.01
        self.distance_right = 0.01
        self.k_p_t_1 = 5
        self.k_p_t_2 = 0.1
        self.k_p_l = 1

    def controller_callback(self):
        ang_vel = self.k_p_t_1 * (
            self.distance_left - self.distance_right
        ) + self.k_p_t_2 / (self.distance_left + self.distance_right)
        lin_vel = self.k_p_l * (self.distance_left + self.distance_right)

        self.get_logger().info(
            f"Linear velocity: {lin_vel}, Angular velocity: {ang_vel}"
        )
        twist_msg = Twist()

        # Apply values to message (with clamping)
        twist_msg.linear.y = -lin_vel if lin_vel < 0.6 else -0.6
        twist_msg.angular.z = ang_vel if ang_vel < 2.0 else 2.0

        self.twist_pub.publish(twist_msg)
        return
    
    def left_sensor_callback(self, msg: Range):
        self.get_logger().info(f'Received {msg.range} from left')
        self.distance_left = msg.range
        return
    
    def right_sensor_callback(self, msg: Range):
        self.get_logger().info(f'Received {msg.range} from right')
        self.distance_right = msg.range
        return

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MirteTestController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    