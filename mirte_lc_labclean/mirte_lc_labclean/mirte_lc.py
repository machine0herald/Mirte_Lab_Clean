import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LabcleanManager(Node):

    def __init__(self):
        super().__init__('labclean_manager')

        self.mapping_lifecycle = self.create_client(
            ChangeState,
            '/nav2/localization/change_state'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle service...')
    
    def change_state(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.mapping_lifecycle.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully requested {transition_id} of node {self.get_name()}')