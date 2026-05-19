#!/usr/bin/env python3

from platform import node

import rclpy
from rclpy.node import Node

import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from sklearn.cluster import DBSCAN

from sensor_msgs_py import point_cloud2


class PointCloudObjectDetector(Node):

    def __init__(self):
        super().__init__('pc_node')

        # Subscribe to incoming point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            10
        )

        # Publisher for RViz bounding boxes
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/object_bounding_boxes',
            10
        )

        self.get_logger().info("PointCloud Object Detector Started")


    def cloud_callback(self, msg):

        ############################################################
        # 1. Convert ROS PointCloud2 -> NumPy array
        ############################################################

        points = []

        points = point_cloud2.read_points(msg)

        if len(points) == 0:
            return

        points_np = np.array(points)
        self.get_logger().info(f"Received point cloud with {len(points_np)} points")

        ############################################################
        # 2. Create Open3D point cloud
        ############################################################


        ############################################################
        # 3. Downsample using voxel grid
        ############################################################

        ############################################################
        # 4. Remove statistical outliers
        ############################################################

        downsampled_cloud, ind = downsampled_cloud.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0
        )

        ############################################################
        # 5. Ground plane removal using RANSAC
        ############################################################

        plane_model, inliers = downsampled_cloud.segment_plane(
            distance_threshold=0.15,
            ransac_n=3,
            num_iterations=100
        )

        # Keep only non-ground points
        object_cloud = downsampled_cloud.select_by_index(inliers, invert=True)

        ############################################################
        # 6. Convert back to NumPy for clustering
        ############################################################

        object_points = np.asarray(object_cloud.points)

        if len(object_points) == 0:
            return

        ############################################################
        # 7. Cluster points into individual objects
        ############################################################

        clustering = DBSCAN(
            eps=0.5,
            min_samples=20
        ).fit(object_points)

        labels = clustering.labels_

        unique_labels = set(labels)

        ############################################################
        # 8. Create bounding boxes
        ############################################################

        marker_array = MarkerArray()

        marker_id = 0

        for label in unique_labels:

            # Ignore noise points
            if label == -1:
                continue

            cluster_points = object_points[labels == label]

            # Ignore tiny clusters
            if len(cluster_points) < 30:
                continue

            ########################################################
            # Create Open3D cloud for this cluster
            ########################################################

            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(
                cluster_points
            )

            ########################################################
            # Compute oriented bounding box
            ########################################################

            obb = cluster_pcd.get_oriented_bounding_box()

            center = obb.center
            extent = obb.extent
            rotation = obb.R

            ########################################################
            # Convert rotation matrix to quaternion
            ########################################################

            quat = self.rotation_matrix_to_quaternion(rotation)

            ########################################################
            # Create RViz Marker
            ########################################################

            marker = Marker()

            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "bounding_boxes"
            marker.id = marker_id

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])

            # Orientation
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            # Box dimensions
            marker.scale.x = float(extent[0])
            marker.scale.y = float(extent[1])
            marker.scale.z = float(extent[2])

            # Color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

            marker_id += 1

        ############################################################
        # 9. Publish all bounding boxes
        ############################################################

        self.marker_pub.publish(marker_array)


    def rotation_matrix_to_quaternion(self, R):

        """
        Convert 3x3 rotation matrix into quaternion
        """

        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2

        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)

        return [qx, qy, qz, qw]


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudObjectDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()