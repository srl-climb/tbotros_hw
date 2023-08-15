from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class BasePosePubslisher(Node):

    def __init__(self):

        super().__init__('base_pose_publisher')

        self.create_timer(1, self.timer_callback)
        self._pub = self.create_publisher(PoseStamped, 'pose', 1)

    def timer_callback(self):
    
        msg = PoseStamped()
        self._pub.publish(msg)
        

def main():
    rclpy.init()

    node = BasePosePubslisher()
    
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
