import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TesterNode(Node):
    def __init__(self):
        super().__init__('tester_node')
        self.request_publisher = self.create_publisher(String, 'test/request', 10)
        self.response_subscriber = self.create_subscription(
            String, 'test/response', self.response_callback, 10)
        self.responses_received = 0
        self.get_logger().info("Tester Node is ready. Sending test request in 2 seconds...")
        
        self.send_timer = self.create_timer(2.0, self.send_request)
        self.shutdown_timer = self.create_timer(7.0, self.shutdown_callback)

    def send_request(self):
        self.get_logger().info("--------------------------------")
        self.get_logger().info(">>> Sending Test Request to all nodes...")
        msg = String()
        msg.data = 'report_for_duty'
        self.request_publisher.publish(msg)
        self.send_timer.cancel()

    def response_callback(self, msg):
        node_id = msg.data
        self.responses_received += 1
        self.get_logger().info(f"<<< Response received from: '{node_id}'")
    
    def shutdown_callback(self):
        self.get_logger().info(f"Test complete. Received {self.responses_received} responses. Shutting down.")
        self.shutdown_timer.cancel()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester_node = TesterNode()
    try:
        rclpy.spin(tester_node)
    except KeyboardInterrupt:
        pass
    finally:
        tester_node.destroy_node()

if __name__ == '__main__':
    main()
