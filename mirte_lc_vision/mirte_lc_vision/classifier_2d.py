'''
Only works on real robot as the simulation has no gripper camera yet.
'''

import os
from ament_index_python.packages import get_package_share_directory

import json
from unittest import result

import cv2
import torch
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Vector3

from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray

from cv_bridge import CvBridge

from ultralytics import YOLO


class Yolo26Cam:
    def __init__(self, targets:list, model_path= os.path.join(
            package_dir,
            "models",
            "ColourdetectionYOLO26n.pt",
            "best"
        ) , conf=0.4, imgsz=640):
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz

        self.objects = []

        self.target_classes = targets

    def predict_frame(self, frame):
        self.results = self.model.predict(
            frame,
            device=self.device,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False
        )[0]
        
        boxes = self.results.boxes

        if boxes is not None and len(boxes) > 0:
            classes = boxes.cls
            poses = boxes.xywhn.tolist()
            class_ids = classes.cpu().numpy().astype(int)

            for idx, class_id in enumerate(class_ids):
                class_name = self.detector.model.names[int(class_id)]

                if class_name in self.detector.target_classes:
                    label = "target"
                else:
                    label = "trash"

                self.detector.objects.append({
                    "label": label,
                    "pose": [poses[idx][0], poses[idx][1]],
                    "size": [poses[idx][2], poses[idx][3]]
                })

        self.objects = self.detector.objects.copy()

        # clear
        self.detector.objects.clear()
        return


class Yolo26RosNode(Node):
    def __init__(self):

        super().__init__("yolo26_object_detector")
        package_dir = get_package_share_directory("mirte_lc_vision")

        self.bridge = CvBridge()

        self.detector = Yolo26Cam(
            targets=["green", "red", "purple"],
            conf=0.3,
        )

        self.image_subscriber = self.create_subscription(
            Image,
            "/gripper_camera/image_raw",
            self.image_callback,
            10
        )

        self.object_publisher = self.create_publisher(
            DetectedObjectArray,
            "/object_bounding_boxes/planar",
            10
        )
        
        self.plotted_image_publisher = self.create_publisher(
            Image,
            "/gripper_camera/image_annotated",
            10
        )

        self.get_logger().info(f"Running YOLO on: {self.detector.device}")
        self.get_logger().info("Subscribed to: /gripper_camera/image_raw")
        self.get_logger().info("Publishing to: /object_bounding_boxes/planar")
        self.get_logger().info("Publishing annotated images to: /gripper_camera/image_annotated")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        self.detector.predict_frame(frame)

        msg = DetectedObjectArray()
        for obj in self.detector.objects:
            detected_object = DetectedObject()
            
            # Pose
            detected_object.pose = Pose()
            detected_object.pose.position.x = obj["pose"][0]
            detected_object.pose.position.y = obj["pose"][1]
            detected_object.pose.position.z = 0.0
            
            # Size
            detected_object.size = Vector3()
            detected_object.size.x = obj["size"][0]
            detected_object.size.y = obj["size"][1]
            detected_object.size.z = 0.0
            
            # Label
            detected_object.label = obj["label"]

            msg.objects.append(detected_object)

        self.object_publisher.publish(msg)

        annotated = self.detector.results.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8"
        )
        self.plotted_image_publisher.publish(annotated_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Yolo26RosNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()