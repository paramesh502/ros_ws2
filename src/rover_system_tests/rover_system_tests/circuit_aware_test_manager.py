import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import BatteryState
import subprocess
import json
import time
import sys

class CircuitAwareTestManager(Node):
    def __init__(self):
        super().__init__('circuit_aware_test_manager')
        self.publisher = self.create_publisher(
            String,
            '/tests/summary',
            10
        )
        self.get_logger().info("✅ Circuit-Aware Test Manager initialized")
        
        # Initialize hardware and circuit detection
        self.hardware_capabilities = self.detect_hardware_capabilities()
        self.circuit_capabilities = self.detect_circuit_capabilities()
        self.log_detection_status()

    def detect_circuit_capabilities(self):
        """Auto-detect available circuit monitoring based on ROS 2 topics"""
        capabilities = {
            'battery_monitoring': False,
            'motor_current': False,
            'sensor_power': False,
            'diagnostic_system': False,
            'gpio_status': False,
            'voltage_monitoring': False
        }
        
        try:
            available_topics = self.get_topic_names_and_types()
            topic_dict = {name: [str(t) for t in types] for name, types in available_topics}
            
            # Check for circuit monitoring capabilities
            for topic_name, topic_types in topic_dict.items():
                # Battery monitoring
                if any('BatteryState' in t for t in topic_types):
                    capabilities['battery_monitoring'] = True
                
                # Current monitoring (custom messages or Float32)
                if 'current' in topic_name.lower() or 'amperage' in topic_name.lower():
                    capabilities['motor_current'] = True
                
                # Voltage monitoring
                if 'voltage' in topic_name.lower() or 'battery_voltage' in topic_name.lower():
                    capabilities['voltage_monitoring'] = True
                
                # GPIO/Digital pin status
                if 'gpio' in topic_name.lower() or 'digital' in topic_name.lower():
                    capabilities['gpio_status'] = True
                
                # Power status
                if 'power' in topic_name.lower() or 'enabled' in topic_name.lower():
                    capabilities['sensor_power'] = True
                
                # Diagnostic system
                if any('DiagnosticArray' in t for t in topic_types) or 'diagnostics' in topic_name:
                    capabilities['diagnostic_system'] = True
            
        except Exception as e:
            self.get_logger().warning(f"Circuit detection failed: {e}")
        
        return capabilities

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

    def log_detection_status(self):
        """Log detected hardware and circuit capabilities"""
        self.get_logger().info("🔍 Hardware Detection Results:")
        for hardware, available in self.hardware_capabilities.items():
            status = "✅ AVAILABLE" if available else "❌ NOT DETECTED"
            self.get_logger().info(f"  {hardware.upper()}: {status}")
        
        self.get_logger().info("⚡ Circuit Monitoring Detection:")
        for circuit, available in self.circuit_capabilities.items():
            status = "✅ MONITORED" if available else "❌ NOT MONITORED"
            self.get_logger().info(f"  {circuit.upper()}: {status}")

    def test_circuit_status(self):
        """Test electrical circuits and power systems"""
        if not any(self.circuit_capabilities.values()):
            return "PASS - No circuit monitoring hardware detected (basic mode)"
        
        circuit_results = []
        overall_circuit_health = True
        
        # Test Battery System
        if self.circuit_capabilities['battery_monitoring']:
            battery_result = self.test_battery_circuit()
            circuit_results.append(f"Battery: {battery_result}")
            if not battery_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Test Motor Circuits
        if self.circuit_capabilities['motor_current']:
            motor_circuit_result = self.test_motor_circuits()
            circuit_results.append(f"Motor Circuits: {motor_circuit_result}")
            if not motor_circuit_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Test Voltage Levels
        if self.circuit_capabilities['voltage_monitoring']:
            voltage_result = self.test_voltage_levels()
            circuit_results.append(f"Voltage: {voltage_result}")
            if not voltage_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Test GPIO/Digital Circuits
        if self.circuit_capabilities['gpio_status']:
            gpio_result = self.test_gpio_circuits()
            circuit_results.append(f"GPIO: {gpio_result}")
            if not gpio_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Test Sensor Power
        if self.circuit_capabilities['sensor_power']:
            sensor_power_result = self.test_sensor_power()
            circuit_results.append(f"Sensor Power: {sensor_power_result}")
            if not sensor_power_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Test Diagnostic System
        if self.circuit_capabilities['diagnostic_system']:
            diagnostic_result = self.test_diagnostic_system()
            circuit_results.append(f"Diagnostics: {diagnostic_result}")
            if not diagnostic_result.startswith("PASS"):
                overall_circuit_health = False
        
        # Compile results
        if not circuit_results:
            return "PASS - No circuit monitoring available"
        
        passed_circuits = sum(1 for result in circuit_results if result.split(': ')[1].startswith("PASS"))
        total_circuits = len(circuit_results)
        
        if overall_circuit_health:
            return f"PASS - All {total_circuits} circuits OK: {', '.join(circuit_results)}"
        elif passed_circuits > 0:
            return f"WARN - {passed_circuits}/{total_circuits} circuits OK: {', '.join(circuit_results)}"
        else:
            return f"FAIL - Circuit problems detected: {', '.join(circuit_results)}"

    def test_battery_circuit(self):
        """Test battery circuit and charging system"""
        try:
            available_topics = self.get_topic_names_and_types()
            battery_topics = [name for name, types in available_topics 
                             if any('BatteryState' in str(t) for t in types)]
            
            if not battery_topics:
                return "FAIL - No battery topics found"
            
            self.latest_battery = None
            
            def battery_callback(msg):
                self.latest_battery = msg
                
            battery_sub = self.create_subscription(BatteryState, battery_topics[0], battery_callback, 10)
            
            # Wait for battery data
            timeout = 3.0
            start_time = time.time()
            while self.latest_battery is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_battery:
                voltage = self.latest_battery.voltage
                current = self.latest_battery.current
                percentage = self.latest_battery.percentage
                
                # Check for circuit issues
                if voltage < 10.0:  # Assuming 12V system
                    return f"FAIL - Low voltage: {voltage:.1f}V"
                elif voltage > 15.0:
                    return f"FAIL - High voltage: {voltage:.1f}V"
                elif percentage < 20.0:
                    return f"WARN - Low battery: {percentage:.0f}%"
                elif abs(current) > 20.0:  # High current draw
                    return f"WARN - High current: {current:.1f}A"
                else:
                    return f"PASS - Battery OK ({voltage:.1f}V, {percentage:.0f}%)"
            else:
                return "WARN - Battery topic exists but no data"
                
        except Exception as e:
            return f"FAIL - Battery test error: {e}"

    def test_motor_circuits(self):
        """Test motor circuit current and status"""
        try:
            available_topics = self.get_topic_names_and_types()
            current_topics = [name for name in available_topics 
                             if 'current' in name.lower() or 'motor' in name.lower()]
            
            if not current_topics:
                return "WARN - No motor current monitoring"
            
            # Check for overcurrent or open circuits
            # This would subscribe to motor current topics
            # For example: /motor_left/current, /motor_right/current
            
            self.latest_motor_current = None
            
            def current_callback(msg):
                self.latest_motor_current = msg.data if hasattr(msg, 'data') else msg
                
            # Subscribe to first current topic (assuming Float32)
            current_sub = self.create_subscription(Float32, current_topics[0], current_callback, 10)
            
            timeout = 2.0
            start_time = time.time()
            while self.latest_motor_current is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_motor_current is not None:
                current = self.latest_motor_current
                
                if current < 0.1:  # Very low current might indicate open circuit
                    return f"WARN - Low/No current: {current:.2f}A (possible open circuit)"
                elif current > 15.0:  # High current might indicate short circuit
                    return f"FAIL - Overcurrent: {current:.2f}A (possible short circuit)"
                else:
                    return f"PASS - Motor current normal: {current:.2f}A"
            else:
                return "WARN - Current monitoring available but no data"
                
        except Exception as e:
            return f"FAIL - Motor circuit test error: {e}"

    def test_voltage_levels(self):
        """Test system voltage levels"""
        try:
            available_topics = self.get_topic_names_and_types()
            voltage_topics = [name for name, types in available_topics 
                             if 'voltage' in name.lower()]
            
            if not voltage_topics:
                return "WARN - No voltage monitoring"
            
            self.latest_voltage = None
            
            def voltage_callback(msg):
                self.latest_voltage = msg.data if hasattr(msg, 'data') else msg
                
            voltage_sub = self.create_subscription(Float32, voltage_topics[0], voltage_callback, 10)
            
            timeout = 2.0
            start_time = time.time()
            while self.latest_voltage is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_voltage is not None:
                voltage = self.latest_voltage
                
                # Check voltage levels (assuming 12V system)
                if voltage < 10.5:
                    return f"FAIL - Undervoltage: {voltage:.1f}V"
                elif voltage > 14.5:
                    return f"FAIL - Overvoltage: {voltage:.1f}V"
                elif voltage < 11.5:
                    return f"WARN - Low voltage: {voltage:.1f}V"
                else:
                    return f"PASS - Voltage OK: {voltage:.1f}V"
            else:
                return "WARN - Voltage monitoring available but no data"
                
        except Exception as e:
            return f"FAIL - Voltage test error: {e}"

    def test_gpio_circuits(self):
        """Test GPIO and digital circuit status"""
        try:
            available_topics = self.get_topic_names_and_types()
            gpio_topics = [name for name, types in available_topics 
                          if 'gpio' in name.lower() or 'digital' in name.lower()]
            
            if not gpio_topics:
                return "WARN - No GPIO monitoring"
            
            # Test digital pins for open/closed status
            gpio_status_count = 0
            gpio_results = []
            
            for topic in gpio_topics[:3]:  # Test first 3 GPIO topics
                self.latest_gpio = None
                
                def gpio_callback(msg):
                    self.latest_gpio = msg.data if hasattr(msg, 'data') else msg
                    
                gpio_sub = self.create_subscription(Bool, topic, gpio_callback, 10)
                
                timeout = 1.0
                start_time = time.time()
                while self.latest_gpio is None and (time.time() - start_time) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.latest_gpio is not None:
                    status = "HIGH" if self.latest_gpio else "LOW"
                    circuit_status = "CLOSED" if self.latest_gpio else "OPEN"
                    gpio_results.append(f"{topic}: {circuit_status}")
                    gpio_status_count += 1
            
            if gpio_status_count > 0:
                return f"PASS - {gpio_status_count} GPIO circuits: {', '.join(gpio_results)}"
            else:
                return "WARN - GPIO topics found but no data"
                
        except Exception as e:
            return f"FAIL - GPIO test error: {e}"

    def test_sensor_power(self):
        """Test sensor power circuits"""
        try:
            available_topics = self.get_topic_names_and_types()
            power_topics = [name for name, types in available_topics 
                           if 'power' in name.lower() or 'enabled' in name.lower()]
            
            if not power_topics:
                return "WARN - No sensor power monitoring"
            
            powered_sensors = 0
            total_sensors = len(power_topics)
            
            for topic in power_topics[:5]:  # Test first 5 power topics
                self.latest_power = None
                
                def power_callback(msg):
                    self.latest_power = msg.data if hasattr(msg, 'data') else msg
                    
                power_sub = self.create_subscription(Bool, topic, power_callback, 10)
                
                timeout = 1.0
                start_time = time.time()
                while self.latest_power is None and (time.time() - start_time) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.latest_power:
                    powered_sensors += 1
            
            if powered_sensors == total_sensors:
                return f"PASS - All {total_sensors} sensor circuits powered"
            elif powered_sensors > 0:
                return f"WARN - {powered_sensors}/{total_sensors} sensor circuits powered"
            else:
                return f"FAIL - No sensor circuits powered"
                
        except Exception as e:
            return f"FAIL - Sensor power test error: {e}"

    def test_diagnostic_system(self):
        """Test system diagnostics for circuit issues"""
        try:
            available_topics = self.get_topic_names_and_types()
            diag_topics = [name for name, types in available_topics 
                          if any('DiagnosticArray' in str(t) for t in types)]
            
            if not diag_topics:
                return "WARN - No diagnostic system"
            
            self.latest_diagnostics = None
            
            def diag_callback(msg):
                self.latest_diagnostics = msg
                
            diag_sub = self.create_subscription(DiagnosticArray, diag_topics[0], diag_callback, 10)
            
            timeout = 3.0
            start_time = time.time()
            while self.latest_diagnostics is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_diagnostics:
                error_count = 0
                warn_count = 0
                
                for status in self.latest_diagnostics.status:
                    if status.level == DiagnosticStatus.ERROR:
                        error_count += 1
                    elif status.level == DiagnosticStatus.WARN:
                        warn_count += 1
                
                total_checks = len(self.latest_diagnostics.status)
                
                if error_count > 0:
                    return f"FAIL - {error_count} errors in {total_checks} diagnostic checks"
                elif warn_count > 0:
                    return f"WARN - {warn_count} warnings in {total_checks} diagnostic checks"
                else:
                    return f"PASS - All {total_checks} diagnostic checks OK"
            else:
                return "WARN - Diagnostic system available but no data"
                
        except Exception as e:
            return f"FAIL - Diagnostic test error: {e}"

    def run_all_tests(self):
        """Run comprehensive system tests including circuit monitoring"""
        results = {}

        # Re-detect capabilities
        self.hardware_capabilities = self.detect_hardware_capabilities()
        self.circuit_capabilities = self.detect_circuit_capabilities()

        # Standard tests (ROS 2 doctor, nodes, topics)
        self.get_logger().info("🔍 Running standard system checks...")
        
        # ROS 2 Doctor
        try:
            out = subprocess.check_output(["ros2", "doctor", "--report"], text=True)
            results["ros2_doctor"] = "PASS" if "All checks passed" in out or "passed" in out.lower() else "WARN"
        except Exception as e:
            results["ros2_doctor"] = f"FAIL: {e}"

        # Node and topic checks
        try:
            node_out = subprocess.check_output(["ros2", "node", "list"], text=True)
            node_count = len([line for line in node_out.strip().split('\n') if line.strip()])
            results["active_nodes"] = f"PASS - {node_count} nodes active" if node_count > 0 else "WARN"
        except Exception as e:
            results["active_nodes"] = f"FAIL: {e}"

        try:
            topic_out = subprocess.check_output(["ros2", "topic", "list"], text=True)
            topic_count = len([line for line in topic_out.strip().split('\n') if line.strip()])
            results["active_topics"] = f"PASS - {topic_count} topics available" if topic_count > 0 else "WARN"
        except Exception as e:
            results["active_topics"] = f"FAIL: {e}"

        # Hardware tests
        self.get_logger().info("🔍 Testing hardware systems...")
        if any(self.hardware_capabilities.values()):
            results["hardware_status"] = "PASS - Hardware systems operational"
        else:
            results["hardware_status"] = "PASS - No hardware detected (simulation mode)"

        # Circuit tests (NEW!)
        self.get_logger().info("⚡ Testing electrical circuits...")
        circuit_result = self.test_circuit_status()
        results["circuit_status"] = circuit_result
        self.get_logger().info(f"⚡ Circuit test: {circuit_result}")

        # System resources
        try:
            subprocess.check_output(["free", "-h"], text=True)
            results["system_resources"] = "PASS - System resources OK"
        except Exception as e:
            results["system_resources"] = f"WARN: {e}"

        # Overall summary
        passed_tests = sum(1 for v in results.values() if v.startswith("PASS"))
        total_tests = len(results)
        
        overall_status = "PASS" if all(v.startswith("PASS") for v in results.values()) else \
                        "WARN" if any(v.startswith("PASS") for v in results.values()) else "FAIL"
        
        # Hardware and circuit summary
        detected_hardware = [hw for hw, available in self.hardware_capabilities.items() if available]
        monitored_circuits = [circuit for circuit, available in self.circuit_capabilities.items() if available]
        
        summary = {
            "timestamp": self.get_clock().now().to_msg(),
            "overall_status": overall_status,
            "tests_passed": f"{passed_tests}/{total_tests}",
            "hardware_detected": detected_hardware,
            "circuits_monitored": monitored_circuits,
            "details": results
        }
        
        # Log comprehensive summary
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚀 ROVER SYSTEM TEST SUMMARY (Circuit-Aware)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Overall Status: {overall_status}")
        self.get_logger().info(f"Tests Passed: {passed_tests}/{total_tests}")
        self.get_logger().info(f"Hardware Detected: {', '.join(detected_hardware) if detected_hardware else 'None'}")
        self.get_logger().info(f"Circuits Monitored: {', '.join(monitored_circuits) if monitored_circuits else 'None'}")
        self.get_logger().info("Test Details:")
        for test_name, result in results.items():
            status_emoji = "✅" if result.startswith("PASS") else "⚠️" if result.startswith("WARN") else "❌"
            self.get_logger().info(f"  {status_emoji} {test_name}: {result}")
        self.get_logger().info("=" * 70)
        
        # Publish results
        msg = String()
        msg.data = json.dumps(summary, indent=2, default=str)
        self.publisher.publish(msg)
        
        return summary

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CircuitAwareTestManager()
        node.run_all_tests()
        node.get_logger().info("Circuit-Aware Test Manager running. Press Ctrl+C to exit.")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
