from geometry_msgs.msg import PoseStamped, Path

# Log Types
INFO = 1
WARN = 2
ERR = 3
DEBUG = 4

def to_ros_path(trajectory):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    for x, y in trajectory:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
    return path_msg

def log(node, msg_type:int, msg:str):
    if node is not None:
        match msg_type:
            case INFO:
                node.get_logger().info(msg)
            case WARN:
                node.get_logger().warn(msg)
            case ERR:
                node.get_logger().error(msg)
            case DEBUG:
                node.get_logger().debug(msg)
            case _:
                node.get_logger().info(msg)
