'''
Only works on real robot as the simulation has no gripper camera yet.
'''

import os
from ament_index_python.packages import get_package_share_directory

import cv2
import torch
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Vector3

from mirte_lc_msgs.msg import DetectedObject, DetectedObjectArray
from mirte_lc_msgs.srv import GetDetectedObjects

from cv_bridge import CvBridge
from ultralytics import YOLO


class Yolo26Cam:
    def __init__(self, targets: list, model_path=os.path.join(
            get_package_share_directory("mirte_lc_vision"),
            "models",
            "ColourdetectionYOLO26n.pt",
        ), conf=0.4, imgsz=640):
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.target_classes = targets
        self.results = None

    def predict_frame(self, frame):
        self.results = self.model.predict(
            frame,
            device=self.device,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False
        )[0]

        objects = []
        boxes = self.results.boxes

        if boxes is not None and len(boxes) > 0:
            classes = boxes.cls
            poses = boxes.xywhn.tolist()
            class_ids = classes.cpu().numpy().astype(int)

            for idx, class_id in enumerate(class_ids):
                class_name = self.model.names[int(class_id)]
                label = "target" if class_name in self.target_classes else "trash"
                objects.append({
                    "label": label,
                    "pose": [poses[idx][0], poses[idx][1]],
                    "size": [poses[idx][2], poses[idx][3]]
                })

        return objects


class Yolo26RosNode(Node):
    def __init__(self):
        super().__init__("yolo26_object_detector")

        self.bridge = CvBridge()
        self.latest_frame = None

        self.detector = Yolo26Cam(
            targets=["green", "red", "purple"],
            conf=0.3,
        )

        gripper_cam_topic = "/camera/color/image_raw"
        annotated_topic = "/gripper_camera/image_annotated"

        self.image_subscriber = self.create_subscription(
            Image,
            gripper_cam_topic,
            self.image_callback,
            10
        )

        self.plotted_image_publisher = self.create_publisher(
            Image,
            annotated_topic,
            10
        )

        self.detection_service = self.create_service(
            GetDetectedObjects,
            "/perception/planar/get_detected_objects",
            self.detection_callback
        )

        self.get_logger().info(f"Running YOLO on: {self.detector.device}")
        self.get_logger().info(f"Subscribed to: {gripper_cam_topic}")
        self.get_logger().info(f"Service ready at: /perception/planar/get_detected_objects")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detection_callback(self, request, response):
        if self.latest_frame is None:
            self.get_logger().warn("Service called but no frame received yet")
            response.detected_objects = DetectedObjectArray()
            return response

        objects = self.detector.predict_frame(self.latest_frame)

        result_array = DetectedObjectArray()
        for obj in objects:
            detected_object = DetectedObject()
            detected_object.pose = Pose()
            detected_object.pose.position.x = obj["pose"][0]
            detected_object.pose.position.y = obj["pose"][1]
            detected_object.pose.position.z = 0.0
            detected_object.size = Vector3()
            detected_object.size.x = obj["size"][0]
            detected_object.size.y = obj["size"][1]
            detected_object.size.z = 0.0
            detected_object.label = obj["label"]
            result_array.objects.append(detected_object)

        response.detected_objects = result_array

        if self.detector.results is not None:
            annotated = self.detector.results.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.plotted_image_publisher.publish(annotated_msg)

        self.get_logger().info(f"Detected {len(objects)} objects")
        return response

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