import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CaptainNode(Node):
    i = 0

    def __init__(self):
        super().__init__('captain_node')
        # Create a publisher on topic '/boat_status'
        self.publisher_ = self.create_publisher(String, '/boat_status', 10)
        # Create a timer that fires every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Captain Node has started! The boat is alive.')

    def timer_callback(self):
        msg = String()
        msg.data = 'System Nominal: Ready for UGent Sailing Team' + str(self.i)
        self.i = self.i + 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = CaptainNode()
    rclpy.spin(node) # Keep the node running
    node.destroy_node()
    rclpy.shutdown()