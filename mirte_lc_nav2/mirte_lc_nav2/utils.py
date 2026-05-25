from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from enum import IntEnum

class LogType(IntEnum):
    INFO = 1
    WARN = 2
    ERR = 3
    DEBUG = 4

import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
np.float = float

# from tf_transformations import quaternion_from_euler
# def to_ros_path(points, frame_id="map", spacing=0.1):
#     """
#     Convert a list of (x, y) tuples into a uniformly sampled nav_msgs/Path.

#     Features:
#     - removes duplicate points
#     - resamples path at consistent spacing
#     - generates orientations from path tangent
#     """
#     # -----------------------------------------
#     # Remove duplicates
#     # -----------------------------------------
#     filtered = []

#     for p in points:
#         if len(filtered) == 0:
#             filtered.append(p)
#             continue

#         dx = p[0] - filtered[-1][0]
#         dy = p[1] - filtered[-1][1]

#         if math.hypot(dx, dy) > 1e-3:
#             filtered.append(p)

#     if len(filtered) < 2:
#         return Path()

#     # -----------------------------------------
#     # Uniform resampling
#     # -----------------------------------------
#     sampled = [filtered[0]]

#     for i in range(len(filtered) - 1):

#         x1, y1 = filtered[i]
#         x2, y2 = filtered[i + 1]

#         dx = x2 - x1
#         dy = y2 - y1

#         segment_length = math.hypot(dx, dy)

#         if segment_length < 1e-6:
#             continue

#         steps = max(1, int(segment_length / spacing))

#         for j in range(1, steps + 1):

#             t = j / steps

#             x = x1 + t * dx
#             y = y1 + t * dy

#             sampled.append((x, y))

#     # -----------------------------------------
#     # Build ROS Path
#     # -----------------------------------------
#     path = Path()
#     path.header.frame_id = frame_id

#     for i in range(len(sampled)):

#         pose = PoseStamped()
#         pose.header.frame_id = frame_id

#         x, y = sampled[i]

#         pose.pose.position.x = float(x)
#         pose.pose.position.y = float(y)
#         pose.pose.position.z = 0.0

#         # -----------------------------------------
#         # Orientation from tangent
#         # -----------------------------------------
#         if i < len(sampled) - 1:

#             nx, ny = sampled[i + 1]

#             yaw = math.atan2(nx - x, ny - y)

#         else:
#             px, py = sampled[i - 1]

#             yaw = math.atan2(x - px, y - py)

#         q = quaternion_from_euler(0.0, 0.0, yaw)

#         pose.pose.orientation.x = q[0]
#         pose.pose.orientation.y = q[1]
#         pose.pose.orientation.z = q[2]
#         pose.pose.orientation.w = q[3]

#         path.poses.append(pose)

#     return path

def log(node, msg_type: LogType, msg: str):
    if node is not None:
        match msg_type:
            case LogType.INFO:
                node.get_logger().info(msg)
            case LogType.WARN:
                node.get_logger().warn(msg)
            case LogType.ERR:
                node.get_logger().error(msg)
            case LogType.DEBUG:
                node.get_logger().debug(msg)
            case _:
                node.get_logger().info(msg)
    elif node is None:
        print(msg)