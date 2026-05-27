import rclpy
import sensor_msgs
import std_msgs
import std_srvs.srv
import time

from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np
import open3d as o3d

from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as Rot

from mirte_lc_msgs.msg import DetectedObject
from mirte_lc_msgs.msg import DetectedObjectArray

from tf2_ros import Buffer, TransformListener
import tf_transformations

class ObjectLocator2(Node):

    def __init__(self):
        super().__init__('object_locator')
        self.points = np.empty((0, 3))
        self.maxDim = 0.06

        ############################################################
        # Point cloud subscriber
        ############################################################

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.process_point_cloud,
            1
        )

        ############################################################
        # VISUALISATION: Bounding box publisher
        ############################################################

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/object_bounding_boxes',
            10
        )

        ############################################################
        # Detected Object Publisher
        ############################################################

        self.object_pub = self.create_publisher(
            DetectedObjectArray,
            '/detected_objects',
            10
        )

        ############################################################
        # Preprocessed Point Cloud Publisher
        ############################################################

        self.preprocessed_pub = self.create_publisher(
            PointCloud2,
            '/preprocessed_points',
            10
        )   

        self.get_logger().info(
            "Object Locator Started"
        )

        ############################################################
        # TF2 Listener
        ############################################################

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )


    def process_point_cloud(self, msg):

        timestamp = self.get_clock().now().to_msg()
            

        object_points = np.empty((0, 3))

        points_np = point_cloud2.read_points_numpy(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True
        ).astype(np.float32)

        self.get_logger().info(
            f"Received {points_np.shape[0]} points"
        )

        # Remove non-finite points
        points_np = points_np[
            np.isfinite(points_np).all(axis=1)
        ]

        # Crop
        points_np = points_np[
            points_np[:, 1] >= -0.11
        ]

        self.get_logger().info(f"Converted to numpy array with shape {points_np.shape}")
        self.get_logger().info(f"Min point: {np.min(points_np, axis=0)}")
        self.get_logger().info(f"Max point: {np.max(points_np, axis=0)}")

        pcd_HQ = o3d.geometry.PointCloud()
        pcd_HQ.points = o3d.utility.Vector3dVector(
            points_np
        )

        pcd_LQ = pcd_HQ.voxel_down_sample(voxel_size = 0.01)
        
        object_pcd = o3d.geometry.PointCloud()

        points = np.asarray(pcd_HQ.points)

        remaining_mask = np.ones(len(points), dtype=bool)

        while True:

            self.get_logger().info(
                f"Using downsampled pointcloud with {len(pcd_LQ.points)} points")

            if len(pcd_LQ.points) < 100:
                break

            plane_model, inliers = pcd_LQ.segment_plane(
                distance_threshold=0.004,
                ransac_n=3,
                num_iterations=2000
            )

            if len(inliers) < 50:
                break

            pcd_LQ = pcd_LQ.select_by_index(
                inliers,
                invert=True
            )

            dist = np.abs(
                plane_model[0]*points[:,0] +
                plane_model[1]*points[:,1] +
                plane_model[2]*points[:,2] +
                plane_model[3]
            ) / np.sqrt(
                plane_model[0]**2 +
                plane_model[1]**2 +
                plane_model[2]**2
            )

            plane_mask = dist < 0.01

            remaining_mask &= ~plane_mask

        object_points = points[remaining_mask]
        object_pcd.points = o3d.utility.Vector3dVector(
            object_points
        )

        self.get_logger().info(
            f"Extracted {len(object_pcd.points)} object points after plane segmentation"
        )

        if (object_points.shape[0] < 5):
            return

        transform = self.tf_buffer.lookup_transform(
            target_frame='map',
            source_frame=msg.header.frame_id,
            time=msg.header.stamp,
            timeout=rclpy.duration.Duration(seconds=0.5)
        )

        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        q = transform.transform.rotation

        R = tf_transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]
        )[:3,:3]

        points = np.asarray(object_pcd.points)

        message = self.point_cloud(points, 'camera_depth_optical_frame')

        self.preprocessed_pub.publish(message)

        clustering = DBSCAN(
            eps=0.25,
            min_samples=3
        ).fit(object_points)

        labels = clustering.labels_

        unique_labels = set(labels)

        ############################################################
        # VISUALISATION: Create MarkerArray
        ############################################################

        marker_array = MarkerArray()

        ############################################################
        # VISUALISATION: Clear old markers
        ############################################################

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        marker_array.markers.append(delete_marker)

        marker_id = 0

        ############################################################
        # Detected Objects Array
        ############################################################

        detected_object_array = DetectedObjectArray()

        for label in unique_labels:

            if label == -1:
                continue

            cluster_points = object_points[
                labels == label
            ]

            if len(cluster_points) < 3:
                continue

            cluster_pcd = o3d.geometry.PointCloud()

            cluster_pcd.points = (
                o3d.utility.Vector3dVector(
                    cluster_points
                )
            )

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


            detected_object = DetectedObject()
            detected_object.pose.position.x = float(center[0])
            detected_object.pose.position.y = float(center[1])
            detected_object.pose.position.z = float(center[2])
            detected_object.pose.orientation.x = float(quat[0])
            detected_object.pose.orientation.y = float(quat[1])
            detected_object.pose.orientation.z = float(quat[2])
            detected_object.pose.orientation.w = float(quat[3])
            detected_object.size.x = float(extent[0])
            detected_object.size.y = float(extent[1])
            detected_object.size.z = float(extent[2])

            detected_object_array.objects.append(detected_object)

            ############################################################
            # VISUALISATION: create bounding box markers for RViz2
            ############################################################

            marker = Marker()

            marker.header.frame_id = 'map'

            marker.header.stamp = timestamp
            marker.ns = "bounding_boxes"

            marker.id = marker_id

            marker.type = Marker.CUBE

            marker.action = Marker.ADD

            ########################################################
            # Pose
            ########################################################

            marker.pose = detected_object.pose

            ########################################################
            # Dimensions
            ########################################################

            marker.scale = detected_object.size

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


    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message

        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(frame_id=parent_frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

############################################################
# Main
############################################################

def main(args=None):

    rclpy.init(args=args)

    node = ObjectLocator2()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()