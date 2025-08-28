import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ResponderNode(Node):
    """Simple test responder - listens for pings and responds.
    
    Part of the basic ROS communication test system.
    When it hears "report_for_duty", it sends back its node name.
    """
    def __init__(self):
        super().__init__(
            'responder_node', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )
        
        # Listen for test requests
        self.ping_subscriber = self.create_subscription(
            String, 'test/request', self.handle_ping, 10
        )
        
        # Send responses back
        self.response_pub = self.create_publisher(String, 'test/response', 10)
        
        # Track how many pings we've responded to
        self.ping_count = 0
        
        # Try to get a custom name from parameters, otherwise use node name
        try:
            self.my_id = self.get_parameter('node_name').get_parameter_value().string_value
            if not self.my_id:  # empty string
                self.my_id = self.get_name()
        except:
            self.my_id = self.get_name()
        
        self.get_logger().info(f"Responder '{self.my_id}' is ready to respond to network tests")

    def handle_ping(self, msg):
        """Respond to ping requests from test nodes."""
        if msg.data.strip() == 'report_for_duty':
            # Send back our identity
            response_msg = String()
            response_msg.data = self.my_id
            self.response_pub.publish(response_msg)
            
            self.ping_count += 1
            self.get_logger().debug(f"Responded to ping #{self.ping_count}")
        else:
            # Got some other message - probably not for us
            self.get_logger().debug(f"Ignored unknown request: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    responder_node = ResponderNode()
    rclpy.spin(responder_node)
    responder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
