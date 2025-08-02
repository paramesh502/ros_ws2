import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResponderNode(Node):
    def __init__(self):
        super().__init__('responder_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.request_subscriber = self.create_subscription(String, 'test/request', self.request_callback, 10)
        self.response_publisher = self.create_publisher(String, 'test/response', 10)
        self.get_logger().info(f"Responder '{self.get_name()}' is online and waiting for requests.")

    def request_callback(self, msg):
        if msg.data == 'report_for_duty':
            response = String()
            response.data = self.get_name()
            self.response_publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    responder_node = ResponderNode()
    rclpy.spin(responder_node)
    responder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
