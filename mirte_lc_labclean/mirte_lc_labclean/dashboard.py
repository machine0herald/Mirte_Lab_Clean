# Creates rviz panel and visualization for labcleanining task
# start and stop buttons in rviz gui aswell as a visualization of led light states using markers

from rviz_visual_tools import RvizVisualTools

from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from mirte_msgs.srv import NeopixelColor

from rclpy.node import Node

class LabCleanDashboard(Node):
    def __init__(self):
        super().__init__('labclean_dashboard')
        self.visual_tools = RvizVisualTools(self, 'map', '/labclean_dashboard_visualization')
        self.visual_tools.deleteAllMarkers()
        self.led_server = self.create_service(NeopixelColor, '/io/leds/leds/set_color', self.led_callback)
        self.led_marker_publisher = self.create_publisher(Marker, '/labclean_led_markers', 10)

    def led_callback(self, request, response):
        # Create grid of markers of a specific color based on the request
        color = (request.r, request.g, request.b)
        self.publish_led_marker(color)
        return response

    def publish_led_marker(self, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.9
        marker.scale.y = 0.03
        marker.scale.z = 1.9
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.8
        marker.pose.position.z = 0.8
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        
        self.led_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    dashboard = LabCleanDashboard()
    rclpy.spin(dashboard)
    dashboard.destroy_node()
    rclpy.shutdown()