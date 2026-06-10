#!/usr/bin/env python3

"""Object locator node for mirte_lc vision stack.

This module implements an `ObjectLocator` ROS 2 node that consumes a
PointCloud2 topic, processes the point cloud to find object clusters,
and publishes bounding boxes as a ``MarkerArray``. The docstrings
follow the Google style so they are compatible with Sphinx (Napoleon)
and common documentation tooling.

Example:

    $ ros2 run mirte_lc_vision object_locator

"""

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

# from mirte_lc_msgs import DetectedObject
# from mirte_lc_msgs import DetectedObjectArray


class ObjectLocator(Node):
    """ROS 2 node that locates objects from a point cloud.

    The node subscribes to a point cloud topic, removes planar surfaces
    (floor/walls), clusters remaining points with DBSCAN, and publishes
    bounding boxes as a ``MarkerArray``. It also periodically requests
    an octomap reset to keep the environment map fresh.

    Attributes:
        points (np.ndarray): Accumulated point array used for processing.
        marker_pub (rclpy.publisher.Publisher): Publisher for MarkerArray.
        reset_timer (rclpy.timer.Timer): Timer to periodically reset octomap.
    """

    def __init__(self):
        """Initialize the object locator node and its ROS 2 interfaces."""

        super().__init__('object_locator')
        self.points = np.empty((0, 3))
        self.maxDim = 0.06

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

        self.client.call_async(self.request)

        ############################################################
        # Point cloud subscriber
        ############################################################

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
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
        # Object Position Publisher
        ############################################################

        # self.bbox_pub = self.create_publisher(
        #     DetectedObjectArray,
        #     '/object_bounding_boxes',
        #     10
        # )

        ############################################################
        # Timer for octomap clearing
        ############################################################

        self.reset_timer = self.create_timer(
            12.0,
            self.reset_octomap
        )

        self.process_point_cloud_timer = self.create_timer(
            5.0,
            self.process_point_cloud
        )

        self.get_logger().info(
            "Object Locator Started"
        )

    ############################################################
    # Octomap reset callback
    ############################################################

    def reset_octomap(self):
        """Request an octomap reset service call."""

        self.client.call_async(self.request)

        self.get_logger().info(
            'Octomap reset requested'
        )

    ############################################################
    # Main point cloud callback
    ############################################################

    def cloud_callback(self, msg):
        """Handle incoming PointCloud2 messages and store their points.

        Args:
            msg (sensor_msgs.msg.PointCloud2): Incoming point cloud message.
        """

        ############################################################
        # 1. Convert PointCloud2 -> NumPy
        ############################################################

        points = []

        points = point_cloud2.read_points(
            msg, 
            field_names=['x', 'y', 'z'], 
            skip_nans=True
        )

        points_np = np.array(points)

        self.get_logger().info(
            f"Received {len(points_np)} points"
        )
        
        points_np = np.array(
            [[p[0], p[1], p[2]] for p in points],
            dtype=np.float64
        )

        points_np = points_np[
            np.isfinite(points_np).all(axis=1)
        ]

        pcd_HQ = o3d.geometry.PointCloud()

        pcd_HQ.points = o3d.utility.Vector3dVector(
            points_np
        )

        self.get_logger().info(
            f"Point cloud shape: {len(pcd_HQ.points)} points"
        )

        pcd_downsampled = pcd_HQ.voxel_down_sample(voxel_size = 0.05)

        self.get_logger().info(
            f"Downsampled to {len(pcd_downsampled.points)} points"
        )

    def process_point_cloud(self):
        """Process accumulated points to remove planes and identify objects."""

        ############################################################
        # 2. Create Open3D cloud
        ############################################################

        pcd_HQ = o3d.geometry.PointCloud()
        object_points = np.empty((0, 3))

        pcd_HQ.points = o3d.utility.Vector3dVector(
            self.points
        )

        ############################################################
        # 3. OPTIONAL downsampling
        ############################################################
        # Octomap is already voxelized, so keep this tiny
        ############################################################

        ############################################################
        # 4. OPTIONAL outlier removal
        ############################################################
        # Keep this weak because octomap is sparse
        ############################################################

        # if len(pcd_downsampled.points) > 20:

        #     pcd_downsampled, ind = pcd_downsampled.remove_statistical_outlier(
        #         nb_neighbors=5,
        #         std_ratio=2.5
        #     )

        ############################################################
        # 4.5 Remove planes (Ground and Walls)
        ############################################################

        pcd_downsampled = o3d.geometry.PointCloud()

        voxel_size = 0.05

        try:
            pcd_downsampled = pcd_HQ.voxel_down_sample(voxel_size)
        except RuntimeError:
            pass

        while True:

            self.get_logger().info(
                f"Using downsampled pointcloud with {len(pcd_downsampled.points)} points")

            if len(pcd_downsampled.points) < 250:
                break

            plane_model, inliers = pcd_downsampled.segment_plane(
                distance_threshold=0.003,
                ransac_n=3,
                num_iterations=2000
            )

            if len(inliers) < 150:
                break

            pcd_downsampled = pcd_downsampled.select_by_index(
                inliers,
                invert=True
            )

            points = np.asarray(pcd_HQ.points)

            dist = np.abs(
                plane_model[0]*points[:,0] +
                plane_model[1]*points[:,1] +
                plane_model[2]*points[:,2] +
                plane_model[3]
            ) / np.sqrt(plane_model[0]*plane_model[0] + plane_model[1]*plane_model[1] + plane_model[2]*plane_model[2])

            mask = dist > 0.04

            object_points = points[mask]

            object_pcd = o3d.geometry.PointCloud()

            object_pcd.points = o3d.utility.Vector3dVector(
                object_points
            )

        ############################################################
        # 5. DIRECTLY use remaining points
        ############################################################

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

        self.bbox_list = []

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

            

            self.bbox_list.append(obb)

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

            marker.lifetime.sec = 5

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

        self.bbox_pub.publish(
            self.bbox_list
        )

        self.get_logger().info(
            f"Published {marker_id} boxes"
        )


############################################################
# Main
############################################################

def main(args=None):

    """Entry point for running the object locator node.

    Args:
        args (List[str] | None): Optional list of command-line arguments.

    This function initializes the ROS client library, instantiates the
    `ObjectLocator` node, and spins until shutdown.
    """

    rclpy.init(args=args)

    node = ObjectLocator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()