# # 1. Start servo
# ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"

# # 2. Disable rotation control
# ros2 service call /servo_node/change_control_dimensions moveit_msgs/srv/ChangeControlDimensions "{x: true, y: true, z: true, roll: false, pitch: false, yaw: false}"

# # 3. Allow rotation to drift
# ros2 service call /servo_node/change_drift_dimensions moveit_msgs/srv/ChangeDriftDimensions "{drift_x_translation: false, drift_y_translation: false, drift_z_translation: false, drift_x_rotation: true, drift_y_rotation: true, drift_z_rotation: true}"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ServoPub(Node):
    def __init__(self):
        super().__init__('servo_pub', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        self.create_timer(0.1, self.publish)  # 100Hz

    def publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.z = -0.05
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ServoPub())

if __name__ == '__main__':
    main()