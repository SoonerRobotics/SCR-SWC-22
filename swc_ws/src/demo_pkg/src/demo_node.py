import rclpy
from rclpy.node import Node

from swc_msgs.msg import Control


class DemoNode(Node):

    def __init__(self):
        super().__init__('demo_node')
        
        self.publisher_ = self.create_publisher(Control, '/sim/control', 10)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Control()
        msg.speed = 1.0
        msg.turn_angle = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    demo_node = DemoNode()

    rclpy.spin(demo_node)

    # Destroy the node explicitly
    demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()