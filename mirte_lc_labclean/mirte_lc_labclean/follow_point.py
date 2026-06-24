#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener

from mirte_lc_msgs.msg import DetectedObjectArray
from mirte_lc_msgs.action import FollowPoint


class FollowPointServer(Node):

    def __init__(self):
        super().__init__("follow_point")

        self.target_distance = 0.15

        self.max_linear_speed = 0.1
        self.max_angular_speed = 0.4

        self.latest_objects = []

        self.following = False
        self.goal_reached = False

        self.distance_remaining = 999.0
        self.objects = []

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.detection_sub = self.create_subscription(
            DetectedObjectArray,
            "/perception/depth/detected_objects",
            self.detection_callback,
            10,
        )
        self.vel_timer = self.create_timer(0.1, self.vel_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.action_server = ActionServer(
            self,
            FollowPoint,
            "/follow_point",
            self.execute_callback,
        )

        self.get_logger().info(
            "FollowPoint action server ready"
        )


    def execute_callback(self, goal_handle):

        self.get_logger().info(
            "FollowPoint goal received"
        )

        self.following = True
        self.goal_reached = False

        self.target_distance = (
            goal_handle.request.target_distance
        )

        feedback = FollowPoint.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result = FollowPoint.Result()
                result.success = False
                result.message = "Cancelled"
                self.following = False
                return result

            feedback.distance_remaining = (
                self.distance_remaining
            )

            goal_handle.publish_feedback(
                feedback
            )

            if self.goal_reached:
                self.stop_robot()
                goal_handle.succeed()
                result = FollowPoint.Result()
                result.success = True
                result.message = "Object reached"
                self.following = False
                return result
            time.sleep(0.05)


        self.stop_robot()

        result = FollowPoint.Result()
        result.success = False
        result.message = "Node stopped"

        self.following = False

        return result



    def detection_callback(
        self,
        msg: DetectedObjectArray
    ):
        if len(msg.objects) == 0:
            self.stop_robot()
            return

        self.objects = msg.objects
    
    def vel_callback(self):
        if not self.following or len(self.objects) == 0:
            self.stop_robot()
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
            )

        except Exception as e:

            self.get_logger().debug(
                f"TF unavailable: {e}"
            )

            return


        rx = tf.transform.translation.x
        ry = tf.transform.translation.y


        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w


        yaw = math.atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        )


        target = min(
            self.objects,
            key=lambda obj:
                (obj.pose.position.x-rx)**2 +
                (obj.pose.position.y-ry)**2
        )


        dx = target.pose.position.x - rx
        dy = target.pose.position.y - ry


        x_rel = (
            math.cos(-yaw)*dx
            -
            math.sin(-yaw)*dy
        )

        y_rel = (
            math.sin(-yaw)*dx
            +
            math.cos(-yaw)*dy
        )


        distance = math.hypot(
            x_rel,
            y_rel
        )


        heading_error = math.atan2(
            y_rel,
            x_rel
        )


        distance_error = (
            distance
            -
            self.target_distance
        )


        self.distance_remaining = max(
            0.0,
            distance_error
        )


        if (
            abs(distance_error) < 0.02
            and abs(heading_error) < 0.04
        ):

            self.goal_reached = True
            self.stop_robot()
            return


        twist = Twist()


        twist.linear.x = (
            0.6
            *
            distance_error
        )

        twist.angular.z = (
            0.8
            *
            heading_error
        )


        twist.linear.x = max(
            -self.max_linear_speed,
            min(
                self.max_linear_speed,
                twist.linear.x
            )
        )


        twist.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                twist.angular.z
            )
        )


        if abs(heading_error) > 0.6:
            twist.linear.x *= 0.25

        self.get_logger().info(f"\033[31m sending cmdvels {twist.linear.x}, {twist.angular.z} \033[0m")
        self.cmd_pub.publish(
            twist
        )

    def stop_robot(self):

        self.cmd_pub.publish(
            Twist()
        )


def main(args=None):

    rclpy.init(args=args)

    node = FollowPointServer()

    executor = rclpy.executors.MultiThreadedExecutor(
        num_threads=2
    )

    executor.add_node(node)

    try:
        executor.spin()

    finally:

        node.stop_robot()
        node.destroy_node()

        rclpy.shutdown()



if __name__ == "__main__":
    main()