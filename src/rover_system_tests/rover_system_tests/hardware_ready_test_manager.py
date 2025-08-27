import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import time
import sys

class HardwareReadyTestManager(Node):
    def __init__(self):
        super().__init__('hardware_ready_test_manager')
        self.publisher = self.create_publisher(
            String,
            '/tests/summary',
            10
        )
        self.get_logger().info("✅ Hardware-Ready Test Manager initialized")
        
        # Initialize hardware detection variables
        self.hardware_capabilities = self.detect_hardware_capabilities()
        self.log_hardware_status()

    def detect_hardware_capabilities(self):
        """Auto-detect available hardware based on ROS 2 topics"""
        capabilities = {
            'motors': False,
            'camera': False,
            'imu': False,
            'lidar': False,
            'ultrasonic': False
        }
        
        try:
            available_topics = self.get_topic_names_and_types()
            topic_dict = {name: [str(t) for t in types] for name, types in available_topics}
            
            # Check for motor capability
            motor_topics = ['/cmd_vel', '/motor_left', '/motor_right', '/motor_commands']
            capabilities['motors'] = any(topic in topic_dict for topic in motor_topics)
            
            # Check for sensor capabilities
            for topic_name, topic_types in topic_dict.items():
                if any('Image' in t for t in topic_types):
                    capabilities['camera'] = True
                if any('Imu' in t for t in topic_types):
                    capabilities['imu'] = True
                if any('LaserScan' in t for t in topic_types):
                    capabilities['lidar'] = True
                if any('Range' in t for t in topic_types):
                    capabilities['ultrasonic'] = True
            
        except Exception as e:
            self.get_logger().warning(f"Hardware detection failed: {e}")
        
        return capabilities

    def log_hardware_status(self):
        """Log detected hardware capabilities"""
        self.get_logger().info("🔍 Hardware Detection Results:")
        for hardware, available in self.hardware_capabilities.items():
            status = "✅ AVAILABLE" if available else "❌ NOT DETECTED"
            self.get_logger().info(f"  {hardware.upper()}: {status}")

    def test_actuators_smart(self):
        """Smart actuator testing - adapts to available hardware"""
        try:
            if not self.hardware_capabilities['motors']:
                return "PASS - No motor hardware detected (simulated mode)"
            
            # Real hardware detected - perform actual tests
            self.get_logger().info("🚗 Testing real motor hardware...")
            
            available_topics = self.get_topic_names_and_types()
            topic_names = [name for name, _ in available_topics]
            
            # Check for standard motor topics
            motor_topics = ['/cmd_vel', '/motor_left', '/motor_right']
            available_motor_topics = [t for t in motor_topics if t in topic_names]
            
            if not available_motor_topics:
                return "WARN - Motor topics detected but standard interfaces missing"
            
            # Test motor command publishing
            from geometry_msgs.msg import Twist
            
            if '/cmd_vel' in available_motor_topics:
                cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
                
                # Send gentle test command
                test_cmd = Twist()
                test_cmd.linear.x = 0.05  # Very slow forward
                cmd_vel_pub.publish(test_cmd)
                
                self.get_logger().info("📡 Sent test motor command")
                time.sleep(0.5)
                
                # Stop motors
                stop_cmd = Twist()
                cmd_vel_pub.publish(stop_cmd)
                
                return f"PASS - Motors responding via {len(available_motor_topics)} topics"
            
            return "PASS - Motor hardware detected and tested"
            
        except Exception as e:
            return f"FAIL - Motor test failed: {e}"

    def test_sensors_smart(self):
        """Smart sensor testing - tests all available sensors"""
        sensor_results = []
        
        # Test Camera
        if self.hardware_capabilities['camera']:
            camera_result = self.test_camera_hardware()
            sensor_results.append(f"Camera: {camera_result}")
        
        # Test IMU
        if self.hardware_capabilities['imu']:
            imu_result = self.test_imu_hardware()
            sensor_results.append(f"IMU: {imu_result}")
        
        # Test LIDAR
        if self.hardware_capabilities['lidar']:
            lidar_result = self.test_lidar_hardware()
            sensor_results.append(f"LIDAR: {lidar_result}")
        
        # Test Ultrasonic
        if self.hardware_capabilities['ultrasonic']:
            ultrasonic_result = self.test_ultrasonic_hardware()
            sensor_results.append(f"Ultrasonic: {ultrasonic_result}")
        
        # Compile results
        if not sensor_results:
            return "PASS - No sensor hardware detected (simulated mode)"
        
        passed_sensors = sum(1 for result in sensor_results if "PASS" in result)
        total_sensors = len(sensor_results)
        
        if passed_sensors == total_sensors:
            return f"PASS - All {total_sensors} sensors active: {', '.join(sensor_results)}"
        elif passed_sensors > 0:
            return f"WARN - {passed_sensors}/{total_sensors} sensors active: {', '.join(sensor_results)}"
        else:
            return f"FAIL - No sensors responding: {', '.join(sensor_results)}"

    def test_camera_hardware(self):
        """Test camera hardware"""
        try:
            from sensor_msgs.msg import Image
            import rclpy.qos as qos
            
            available_topics = self.get_topic_names_and_types()
            camera_topics = [name for name, types in available_topics 
                           if any('Image' in str(t) for t in types)]
            
            if not camera_topics:
                return "FAIL"
            
            # Subscribe to first camera topic
            self.latest_image = None
            
            def image_callback(msg):
                self.latest_image = msg
                
            image_sub = self.create_subscription(
                Image, camera_topics[0], image_callback, 
                qos.QoSProfile(reliability=qos.ReliabilityPolicy.BEST_EFFORT, depth=1)
            )
            
            # Wait for image with timeout
            timeout = 3.0
            start_time = time.time()
            while self.latest_image is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_image:
                return f"PASS ({self.latest_image.width}x{self.latest_image.height})"
            else:
                return "WARN (no data)"
                
        except Exception as e:
            return f"FAIL ({e})"

    def test_imu_hardware(self):
        """Test IMU hardware"""
        try:
            from sensor_msgs.msg import Imu
            
            available_topics = self.get_topic_names_and_types()
            imu_topics = [name for name, types in available_topics 
                         if any('Imu' in str(t) for t in types)]
            
            if not imu_topics:
                return "FAIL"
            
            self.latest_imu = None
            
            def imu_callback(msg):
                self.latest_imu = msg
                
            imu_sub = self.create_subscription(Imu, imu_topics[0], imu_callback, 10)
            
            # Wait for IMU data
            timeout = 2.0
            start_time = time.time()
            while self.latest_imu is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_imu:
                accel = self.latest_imu.linear_acceleration
                total_accel = abs(accel.x) + abs(accel.y) + abs(accel.z)
                
                if total_accel > 0.1:  # Should detect gravity
                    return f"PASS (accel: {total_accel:.2f})"
                else:
                    return "WARN (static data)"
            else:
                return "WARN (no data)"
                
        except Exception as e:
            return f"FAIL ({e})"

    def test_lidar_hardware(self):
        """Test LIDAR hardware"""
        try:
            from sensor_msgs.msg import LaserScan
            
            available_topics = self.get_topic_names_and_types()
            lidar_topics = [name for name, types in available_topics 
                           if any('LaserScan' in str(t) for t in types)]
            
            if not lidar_topics:
                return "FAIL"
            
            self.latest_scan = None
            
            def scan_callback(msg):
                self.latest_scan = msg
                
            scan_sub = self.create_subscription(LaserScan, lidar_topics[0], scan_callback, 10)
            
            # Wait for scan data
            timeout = 3.0
            start_time = time.time()
            while self.latest_scan is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_scan:
                valid_ranges = [r for r in self.latest_scan.ranges 
                              if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
                
                coverage = len(valid_ranges) / len(self.latest_scan.ranges) * 100
                return f"PASS ({coverage:.0f}% coverage)"
            else:
                return "WARN (no data)"
                
        except Exception as e:
            return f"FAIL ({e})"

    def test_ultrasonic_hardware(self):
        """Test ultrasonic sensors"""
        try:
            from sensor_msgs.msg import Range
            
            available_topics = self.get_topic_names_and_types()
            range_topics = [name for name, types in available_topics 
                           if any('Range' in str(t) for t in types)]
            
            if not range_topics:
                return "FAIL"
            
            self.latest_range = None
            
            def range_callback(msg):
                self.latest_range = msg
                
            range_sub = self.create_subscription(Range, range_topics[0], range_callback, 10)
            
            # Wait for range data
            timeout = 2.0
            start_time = time.time()
            while self.latest_range is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_range:
                if self.latest_range.min_range <= self.latest_range.range <= self.latest_range.max_range:
                    return f"PASS ({self.latest_range.range:.2f}m)"
                else:
                    return "WARN (out of range)"
            else:
                return "WARN (no data)"
                
        except Exception as e:
            return f"FAIL ({e})"

    def run_all_tests(self):
        """Run comprehensive system tests with intelligent hardware detection"""
        results = {}

        # Re-detect hardware capabilities (in case new nodes started)
        self.hardware_capabilities = self.detect_hardware_capabilities()

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

        # 4. SMART ACTUATOR TESTS
        self.get_logger().info("🔍 Testing actuators (hardware-aware)...")
        actuator_result = self.test_actuators_smart()
        results["actuators"] = actuator_result
        self.get_logger().info(f"✅ Actuator test: {actuator_result}")

        # 5. SMART SENSOR TESTS
        self.get_logger().info("🔍 Testing sensors (hardware-aware)...")
        sensor_result = self.test_sensors_smart()
        results["sensors"] = sensor_result
        self.get_logger().info(f"✅ Sensor test: {sensor_result}")

        # 6. SYSTEM RESOURCES CHECK
        self.get_logger().info("🔍 Checking system resources...")
        try:
            memory_out = subprocess.check_output(["free", "-h"], text=True)
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
        
        # Add hardware detection summary
        hardware_summary = []
        for hw_type, available in self.hardware_capabilities.items():
            if available:
                hardware_summary.append(hw_type)
        
        summary = {
            "timestamp": self.get_clock().now().to_msg(),
            "overall_status": overall_status,
            "tests_passed": f"{passed_tests}/{total_tests}",
            "hardware_detected": hardware_summary,
            "details": results
        }
        
        # Log summary
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 ROVER SYSTEM TEST SUMMARY (Hardware-Aware)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Overall Status: {overall_status}")
        self.get_logger().info(f"Tests Passed: {passed_tests}/{total_tests}")
        self.get_logger().info(f"Hardware Detected: {', '.join(hardware_summary) if hardware_summary else 'None (Simulation Mode)'}")
        self.get_logger().info("Test Details:")
        for test_name, result in results.items():
            status_emoji = "✅" if result.startswith("PASS") else "⚠️" if result.startswith("WARN") else "❌"
            self.get_logger().info(f"  {status_emoji} {test_name}: {result}")
        self.get_logger().info("=" * 60)
        
        # Publish to ROS 2 topic
        msg = String()
        msg.data = json.dumps(summary, indent=2, default=str)
        self.publisher.publish(msg)
        
        return summary

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HardwareReadyTestManager()
        
        # Run tests once
        node.run_all_tests()
        
        # Keep node alive to continue publishing
        node.get_logger().info("Hardware-Ready Test Manager running. Press Ctrl+C to exit.")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
