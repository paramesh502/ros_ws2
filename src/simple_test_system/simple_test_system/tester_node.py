import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Simple timeouts that work well in practice
STARTUP_DELAY = 2.0  # give other nodes time to boot up
TEST_TIMEOUT = 7.0   # how long to wait for responses

class TesterNode(Node):
    """Basic node communication tester.
    
    Sends out a "report for duty" message and counts who responds.
    Good for making sure the ROS network is actually working.
    """
    def __init__(self):
        super().__init__('tester_node')
        
        # Set up our communication channels
        self.request_pub = self.create_publisher(String, 'test/request', 10)
        self.response_sub = self.create_subscription(
            String, 'test/response', self.handle_response, 10)
        
        # Keep track of what we've seen
        self.response_count = 0
        self.seen_nodes = set()  # track unique responders
        
        self.get_logger().info("Tester node starting up - will ping the network in a moment...")
        
        # Give everything a moment to settle, then run our test
        self.startup_timer = self.create_timer(STARTUP_DELAY, self.send_ping)
        self.timeout_timer = self.create_timer(TEST_TIMEOUT, self.finish_test)

    def send_ping(self):
        """Send out our network test message."""
        self.get_logger().info("" * 50)
        self.get_logger().info("Broadcasting network test - who's out there?")
        
        # Send the magic message that responder nodes should react to
        ping_msg = String()
        ping_msg.data = 'report_for_duty'  # keep this simple
        self.request_pub.publish(ping_msg)
        
        self.get_logger().info("Test message sent - waiting for responses...")
        self.startup_timer.cancel()  # one-shot timer

    def handle_response(self, msg):
        """Process incoming responses from other nodes."""
        responder_id = msg.data.strip()
        
        if responder_id not in self.seen_nodes:
            # New node responding
            self.seen_nodes.add(responder_id)
            self.response_count += 1
            self.get_logger().info(f"✓ Node '{responder_id}' is alive (#{self.response_count})")
        else:
            # We've seen this one before - probably a duplicate message
            self.get_logger().debug(f"Duplicate response from '{responder_id}'")
    
    def finish_test(self):
        """Wrap up the test and report results."""
        self.get_logger().info("" * 50)
        self.get_logger().info("Network test complete!")
        
        if self.response_count > 0:
            node_list = sorted(list(self.seen_nodes))
            self.get_logger().info(f"Found {self.response_count} responding nodes:")
            for node_name in node_list:
                self.get_logger().info(f"  - {node_name}")
            self.get_logger().info("Network looks good to go! 👍")
        else:
            self.get_logger().warn("No responses received - network might be empty or broken")
        
        self.get_logger().info("Shutting down tester...")
        self.timeout_timer.cancel()
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
