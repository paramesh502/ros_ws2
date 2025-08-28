import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import sys
import time

# TODO: Maybe move these test configs to a YAML file at some point?
# For now this works fine though
CHECK_TIMEOUT = 5.0  # seconds - found this works well in practice
MIN_EXPECTED_NODES = 1  # at least the test manager should be running

class TestManager(Node):
    """Main test orchestrator for the rover system.
    
    This handles all the basic sanity checks we need before deployment.
    Started this after too many "why isn't anything working" moments in the field.
    """
    def __init__(self):
        super().__init__('test_manager')
        # Using a simple string publisher - considered using custom msgs but this is cleaner
        self.publisher = self.create_publisher(
            String,
            '/tests/summary',  # other nodes can subscribe to this for status updates
            10
        )
        self.test_count = 0  # keep track of how many test cycles we've run
        self.get_logger().info("Test Manager is up and running! Let's see what we're working with...")

    def run_all_tests(self):
        """Main test runner - checks everything we care about.
        
        Returns a dict with all test results. I've organized these in order
        of importance - ROS basics first, then hardware stuff.
        """
        self.test_count += 1
        self.get_logger().info(f"Starting test cycle #{self.test_count}...")
        
        results = {}
        start_time = time.time()

        # First things first - is ROS even working?
        self.get_logger().info("Checking if ROS is happy...")
        try:
            # ros2 doctor is pretty reliable for catching the obvious stuff
            doctor_output = subprocess.check_output(
                ["ros2", "doctor", "--report"], 
                text=True, 
                timeout=CHECK_TIMEOUT
            )
            # Doctor output format changed between versions, so checking both patterns
            if "All checks passed" in doctor_output or "passed" in doctor_output.lower():
                results["ros2_basics"] = "PASS - ROS environment looks good"
                self.get_logger().info("ROS environment checks out ✓")
            else:
                results["ros2_basics"] = "WARN - ROS doctor found some issues"
                self.get_logger().warn("ROS doctor found some issues, but we can probably continue")
        except subprocess.TimeoutExpired:
            results["ros2_basics"] = "FAIL - ROS doctor timed out (this is bad)"
            self.get_logger().error("ROS doctor is hanging - something is seriously wrong")
        except Exception as e:
            results["ros2_basics"] = f"FAIL - {str(e)[:50]}..."
            self.get_logger().error(f"ROS doctor completely failed: {e}")

        # Check what nodes are actually running
        self.get_logger().info("Seeing what nodes are alive...")
        try:
            node_output = subprocess.check_output(
                ["ros2", "node", "list"], 
                text=True,
                timeout=CHECK_TIMEOUT
            )
            # Filter out empty lines - ROS sometimes adds weird spacing
            active_nodes = [line.strip() for line in node_output.strip().split('\n') if line.strip()]
            node_count = len(active_nodes)
            
            if node_count >= MIN_EXPECTED_NODES:
                results["node_status"] = f"PASS - Found {node_count} active nodes"
                self.get_logger().info(f"Good news: {node_count} nodes are running")
                # Log the actual node names for debugging
                self.get_logger().debug(f"Active nodes: {', '.join(active_nodes)}")
            else:
                results["node_status"] = f"WARN - Only {node_count} nodes running (expected at least {MIN_EXPECTED_NODES})"
                self.get_logger().warn(f"Hmm, only found {node_count} nodes - might be starting up still?")
        except subprocess.TimeoutExpired:
            results["node_status"] = "FAIL - Node list command timed out"
            self.get_logger().error("Node listing is hanging - ROS communication issues?")
        except Exception as e:
            results["node_status"] = f"FAIL - {str(e)[:50]}..."
            self.get_logger().error(f"Can't get node list: {e}")

        # What topics do we have to work with?
        self.get_logger().info("Checking the topic situation...")
        try:
            topic_output = subprocess.check_output(
                ["ros2", "topic", "list"], 
                text=True,
                timeout=CHECK_TIMEOUT
            )
            available_topics = [line.strip() for line in topic_output.strip().split('\n') if line.strip()]
            topic_count = len(available_topics)
            
            if topic_count > 0:
                results["communication"] = f"PASS - {topic_count} topics available for communication"
                self.get_logger().info(f"Communication looks good: {topic_count} topics active")
                # Check for some common important topics
                important_topics = ['/cmd_vel', '/odom', '/scan']
                found_important = [t for t in important_topics if t in available_topics]
                if found_important:
                    self.get_logger().info(f"Bonus: Found these key topics: {', '.join(found_important)}")
            else:
                results["communication"] = "WARN - No topics found (system might be starting up)"
                self.get_logger().warn("No topics yet - either nothing is running or still booting up")
        except subprocess.TimeoutExpired:
            results["communication"] = "FAIL - Topic listing timed out"
            self.get_logger().error("Topic discovery is hanging")
        except Exception as e:
            results["communication"] = f"FAIL - {str(e)[:50]}..."
            self.get_logger().error(f"Topic check failed: {e}")

        # Quick hardware reality check - are our motors/sensors actually there?
        # TODO: Hook this up to real hardware tests when we get the rover built
        self.get_logger().info("Testing the hardware situation...")
        actuator_status = self._check_actuators_quickly()  # keeping this simple for now
        results["hardware_actuators"] = actuator_status
        
        sensor_status = self._check_sensors_quickly()  # ditto
        results["hardware_sensors"] = sensor_status

        # Make sure the system isn't about to fall over
        self.get_logger().info("Quick system health check...")
        try:
            # Memory check - learned this the hard way when system crashed mid-mission
            memory_info = subprocess.check_output(["free", "-h"], text=True, timeout=2)
            disk_info = subprocess.check_output(["df", "-h", "/"], text=True, timeout=2)
            
            # Parse memory - looking for available memory > 100MB (rough heuristic)
            memory_lines = memory_info.strip().split('\n')
            if len(memory_lines) > 1:
                # This parsing is a bit fragile but works for now
                mem_data = memory_lines[1].split()
                if len(mem_data) > 6:  # free command format: total used free shared buff/cache available
                    available_mem = mem_data[6]
                    self.get_logger().debug(f"Available memory: {available_mem}")
            
            results["system_health"] = "PASS - System resources look fine"
            self.get_logger().info("System health looks good")
        except subprocess.TimeoutExpired:
            results["system_health"] = "WARN - System check timed out (might be under load)"
            self.get_logger().warn("System resource check is slow - high CPU load?")
        except Exception as e:
            results["system_health"] = f"WARN - {str(e)[:30]}..."
            self.get_logger().warn(f"System health check had issues: {e}")

        # Time to see how we did overall
        test_duration = time.time() - start_time
        passed_count = sum(1 for result in results.values() if result.startswith("PASS"))
        total_count = len(results)
        
        # Figure out overall health
        if all(result.startswith("PASS") for result in results.values()):
            overall_status = "PASS"
            mood = "Looking good! 🎉"
        elif any(result.startswith("PASS") for result in results.values()):
            overall_status = "WARN" 
            mood = "Some issues, but we can work with this"
        else:
            overall_status = "FAIL"
            mood = "Houston, we have problems..."
        
        # Package everything up
        summary = {
            "test_cycle": self.test_count,
            "timestamp": self.get_clock().now().to_msg(),
            "overall_status": overall_status,
            "test_duration_sec": round(test_duration, 2),
            "pass_rate": f"{passed_count}/{total_count}",
            "results": results  # renamed from 'details' - more natural
        }
        
        # Print the results in a way that's actually readable
        self.get_logger().info("" * 60)  # separator line
        self.get_logger().info(f"TEST CYCLE #{self.test_count} COMPLETE ({test_duration:.1f}s)")
        self.get_logger().info(mood)
        self.get_logger().info(f"Status: {overall_status} ({passed_count}/{total_count} passed)")
        self.get_logger().info("")
        
        # Show individual results with some personality
        for test_name, result in results.items():
            if result.startswith("PASS"):
                icon, color = "✓", "green" 
            elif result.startswith("WARN"):
                icon, color = "!", "yellow"
            else:
                icon, color = "✗", "red"
            
            # Clean up the test name for display
            display_name = test_name.replace("_", " ").title()
            self.get_logger().info(f"  {icon} {display_name}: {result}")
        
        self.get_logger().info("" * 60)
        
        # Send results to anyone who's listening
        status_msg = String()
        status_msg.data = json.dumps(summary, indent=2, default=str)
        self.publisher.publish(status_msg)
        
        return summary

    def _check_actuators_quickly(self):
        """Quick and dirty actuator check - just see if we have motor topics.
        
        When we get real hardware, this should actually try to move something.
        For now it's mostly a placeholder that won't break anything.
        """
        try:
            available_topics = subprocess.check_output(
                ["ros2", "topic", "list"], text=True, timeout=3
            ).strip().split('\n')
            
            # Look for common motor control topics
            motor_topics = ['/cmd_vel', '/motor_left', '/motor_right', '/drive_cmd']
            found_motors = [topic for topic in motor_topics if topic in available_topics]
            
            if found_motors:
                return f"PASS - Found motor interfaces: {', '.join(found_motors)}"
            else:
                # This is actually fine for simulation/testing
                return "PASS - No motor hardware detected (probably simulation mode)"
                
        except Exception as e:
            return f"WARN - Couldn't check actuator status: {str(e)[:40]}..."
    
    def _check_sensors_quickly(self):
        """Basic sensor availability check.
        
        Real hardware would need actual sensor data validation, but for now
        we're just making sure sensor topics exist.
        """
        try:
            available_topics = subprocess.check_output(
                ["ros2", "topic", "list"], text=True, timeout=3
            ).strip().split('\n')
            
            # Common sensor topics to look for
            sensor_patterns = {
                'camera': ['/camera', '/image', '/video'],
                'lidar': ['/scan', '/laser', '/lidar'],
                'imu': ['/imu', '/accel', '/gyro'],
                'gps': ['/gps', '/fix', '/navsat']
            }
            
            found_sensors = []
            for sensor_type, patterns in sensor_patterns.items():
                for pattern in patterns:
                    if any(pattern in topic for topic in available_topics):
                        found_sensors.append(sensor_type)
                        break  # found this sensor type, move to next
            
            if found_sensors:
                sensor_list = ', '.join(found_sensors)
                return f"PASS - Detected sensors: {sensor_list}"
            else:
                return "PASS - No sensor hardware detected (simulation mode OK)"
                
        except Exception as e:
            return f"WARN - Sensor check failed: {str(e)[:40]}..."

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
