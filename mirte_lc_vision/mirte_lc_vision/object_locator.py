#!/usr/bin/env python3

import time
import rclpy
import std_srvs.srv

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np
import open3d as o3d

from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as Rot


class ObjectLocator(Node):

    def __init__(self):

        super().__init__('object_locator')
        self.points = np.empty((0, 3))
        self.maxDim = 0.25

        ############################################################
        # Octomap reset service
        ############################################################

        self.client = self.create_client(
            std_srvs.srv.Empty,
            '/octomap_server_node/reset'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for octomap reset service...'
            )

        self.request = std_srvs.srv.Empty.Request()

        ############################################################
        # Point cloud subscriber
        ############################################################

        self.subscription = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.cloud_callback,
            10
        )

        ############################################################
        # Bounding box publisher
        ############################################################

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/object_bounding_boxes',
            10
        )

        ############################################################
        # Timer for octomap clearing
        ############################################################

        self.reset_timer = self.create_timer(
            10.0,
            self.reset_octomap
        )

        self.process_point_cloud_timer = self.create_timer(
            0.1,
            self.process_point_cloud
        )

        self.get_logger().info(
            "Object Locator Started"
        )

    ############################################################
    # Octomap reset callback
    ############################################################

    def reset_octomap(self):

        self.client.call_async(self.request)

        self.get_logger().info(
            'Octomap reset requested'
        )

    ############################################################
    # Main point cloud callback
    ############################################################

    def cloud_callback(self, msg):

        ############################################################
        # 1. Convert PointCloud2 -> NumPy
        ############################################################

        points = []

        for p in point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True):

            x, y, z = p

            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])

        if len(points) == 0:
            self.get_logger().warn(
                "No valid points received"
            )
            return

        points_np = np.array(points)

        self.get_logger().info(
            f"Received {len(points_np)} points"
        )
        self.points = points_np

    def process_point_cloud(self):
        ############################################################
        # 2. Create Open3D cloud
        ############################################################

        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(
            self.points
        )

        ############################################################
        # 3. OPTIONAL downsampling
        ############################################################
        # Octomap is already voxelized, so keep this tiny
        ############################################################

        voxel_size = 0.02

        try:
            pcd = pcd.voxel_down_sample(voxel_size)
        except RuntimeError:
            pass

        ############################################################
        # 4. OPTIONAL outlier removal
        ############################################################
        # Keep this weak because octomap is sparse
        ############################################################

        if len(pcd.points) > 20:

            pcd, ind = pcd.remove_statistical_outlier(
                nb_neighbors=5,
                std_ratio=2.5
            )

        ############################################################
        # 5. DIRECTLY use remaining points
        ############################################################

        object_points = np.asarray(pcd.points)

        if len(object_points) == 0:

            self.get_logger().warn(
                "No object points remain"
            )

            return

        ############################################################
        # 7. DBSCAN clustering
        ############################################################

        clustering = DBSCAN(
            eps=0.25,
            min_samples=3
        ).fit(object_points)

        labels = clustering.labels_

        unique_labels = set(labels)

        ############################################################
        # 8. Create MarkerArray
        ############################################################

        marker_array = MarkerArray()

        ############################################################
        # Clear old markers
        ############################################################

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        marker_array.markers.append(delete_marker)

        marker_id = 0

        ############################################################
        # Process each cluster
        ############################################################

        for label in unique_labels:

            ########################################################
            # Ignore noise
            ########################################################

            if label == -1:
                continue

            cluster_points = object_points[
                labels == label
            ]

            ########################################################
            # Ignore tiny clusters
            ########################################################

            if len(cluster_points) < 3:
                continue

            ########################################################
            # Create cluster cloud
            ########################################################

            cluster_pcd = o3d.geometry.PointCloud()

            cluster_pcd.points = (
                o3d.utility.Vector3dVector(
                    cluster_points
                )
            )

            ########################################################
            # Compute bounding box
            ########################################################

            try:

                obb = (
                    cluster_pcd
                    .get_oriented_bounding_box()
                )

            except RuntimeError:

                continue

            center = obb.center
            extent = obb.extent

            if (extent[0] > self.maxDim or
                extent[1] > self.maxDim or
                extent[2] > self.maxDim):
                continue

            rotation = obb.R #(3,3) float 64 array
            

            ########################################################
            # Rotation matrix -> quaternion
            ########################################################

            ########################################################
            # Lock roll/pitch and keep only yaw
            ########################################################

            rotation_copy = np.array(
                rotation,
                copy=True
            )

            # Extract yaw from rotation matrix
            yaw = np.arctan2(
                rotation_copy[1, 0],
                rotation_copy[0, 0]
            )

            # Create rotation using ONLY yaw
            quat = Rot.from_euler(
                'z',
                yaw
            ).as_quat()

            ########################################################
            # Create RViz marker
            ########################################################

            marker = Marker()

            marker.header.frame_id = 'map'

            marker.header.stamp = (
                self.get_clock().now().to_msg()
            )

            marker.ns = "bounding_boxes"

            marker.id = marker_id

            marker.type = Marker.CUBE

            marker.action = Marker.ADD

            ########################################################
            # Position
            ########################################################

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])

            ########################################################
            # Orientation
            ########################################################

            marker.pose.orientation.x = float(quat[0])
            marker.pose.orientation.y = float(quat[1])
            marker.pose.orientation.z = float(quat[2])
            marker.pose.orientation.w = float(quat[3])

            ########################################################
            # Dimensions
            ########################################################

            marker.scale.x = max(float(extent[0]), 0.05)
            marker.scale.y = max(float(extent[1]), 0.05)
            marker.scale.z = max(float(extent[2]), 0.05)

            ########################################################
            # Color
            ########################################################

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            ########################################################
            # Lifetime
            ########################################################

            marker.lifetime.sec = 1

            marker_array.markers.append(
                marker
            )

            marker_id += 1

        ############################################################
        # 9. Publish boxes
        ############################################################

        self.marker_pub.publish(
            marker_array
        )

        self.get_logger().info(
            f"Published {marker_id} boxes"
        )


############################################################
# Main
############################################################

def main(args=None):

    rclpy.init(args=args)

    node = ObjectLocator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()