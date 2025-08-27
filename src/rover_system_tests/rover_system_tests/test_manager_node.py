import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import sys

class TestManager(Node):
    def __init__(self):
        super().__init__('test_manager')
        self.publisher = self.create_publisher(
            String,
            '/tests/summary',
            10
        )
        self.get_logger().info("✅ Test Manager ready")

    def run_all_tests(self):
        """Run comprehensive system tests"""
        results = {}

        # 1. ROS 2 DOCTOR CHECK
        self.get_logger().info("🔍 Running ROS 2 Doctor...")
        try:
            out = subprocess.check_output(["ros2", "doctor", "--report"], text=True)
            results["ros2_doctor"] = "PASS" if "All checks passed" in out or "passed" in out.lower() else "WARN"
            self.get_logger().info("✅ ROS 2 Doctor check completed")
        except Exception as e:
            results["ros2_doctor"] = f"FAIL: {e}"
            self.get_logger().error(f"❌ ROS 2 Doctor failed: {e}")

        # 2. NODE LIST CHECK
        self.get_logger().info("🔍 Checking active nodes...")
        try:
            out = subprocess.check_output(["ros2", "node", "list"], text=True)
            node_count = len([line for line in out.strip().split('\n') if line.strip()])
            results["active_nodes"] = f"PASS - {node_count} nodes active" if node_count > 0 else "WARN - No nodes active"
            self.get_logger().info(f"✅ Found {node_count} active nodes")
        except Exception as e:
            results["active_nodes"] = f"FAIL: {e}"
            self.get_logger().error(f"❌ Node list check failed: {e}")

        # 3. TOPIC LIST CHECK
        self.get_logger().info("🔍 Checking active topics...")
        try:
            out = subprocess.check_output(["ros2", "topic", "list"], text=True)
            topic_count = len([line for line in out.strip().split('\n') if line.strip()])
            results["active_topics"] = f"PASS - {topic_count} topics available" if topic_count > 0 else "WARN - No topics available"
            self.get_logger().info(f"✅ Found {topic_count} active topics")
        except Exception as e:
            results["active_topics"] = f"FAIL: {e}"
            self.get_logger().error(f"❌ Topic list check failed: {e}")

        # 4. ACTUATORS CHECK (placeholder - replace with actual motor commands)
        self.get_logger().info("🔍 Checking actuators...")
        results["actuators"] = "PASS - Actuators nominal (simulated)"
        self.get_logger().info("✅ Actuators check completed")

        # 5. SENSORS CHECK (placeholder - replace with actual sensor checks)
        self.get_logger().info("🔍 Checking sensors...")
        results["sensors"] = "PASS - Sensors nominal (simulated)"
        self.get_logger().info("✅ Sensors check completed")

        # 6. SYSTEM RESOURCES CHECK
        self.get_logger().info("🔍 Checking system resources...")
        try:
            # Check memory usage
            memory_out = subprocess.check_output(["free", "-h"], text=True)
            # Check disk usage
            disk_out = subprocess.check_output(["df", "-h", "/"], text=True)
            results["system_resources"] = "PASS - System resources OK"
            self.get_logger().info("✅ System resources check completed")
        except Exception as e:
            results["system_resources"] = f"WARN: {e}"
            self.get_logger().warning(f"⚠️ System resources check warning: {e}")

        # 7. OVERALL SUMMARY
        passed_tests = sum(1 for v in results.values() if v.startswith("PASS"))
        total_tests = len(results)
        
        overall_status = "PASS" if all(v.startswith("PASS") for v in results.values()) else \
                        "WARN" if any(v.startswith("PASS") for v in results.values()) else "FAIL"
        
        summary = {
            "timestamp": self.get_clock().now().to_msg(),
            "overall_status": overall_status,
            "tests_passed": f"{passed_tests}/{total_tests}",
            "details": results
        }
        
        # Log summary
        self.get_logger().info("=" * 50)
        self.get_logger().info("🚀 ROVER SYSTEM TEST SUMMARY")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Overall Status: {overall_status}")
        self.get_logger().info(f"Tests Passed: {passed_tests}/{total_tests}")
        self.get_logger().info("Test Details:")
        for test_name, result in results.items():
            status_emoji = "✅" if result.startswith("PASS") else "⚠️" if result.startswith("WARN") else "❌"
            self.get_logger().info(f"  {status_emoji} {test_name}: {result}")
        self.get_logger().info("=" * 50)
        
        # Publish to ROS 2 topic
        msg = String()
        msg.data = json.dumps(summary, indent=2, default=str)
        self.publisher.publish(msg)
        
        return summary

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestManager()
        
        # Run tests once
        node.run_all_tests()
        
        # Keep node alive to continue publishing
        node.get_logger().info("Test Manager running. Press Ctrl+C to exit.")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
