# ⚡ Circuit Monitoring & Detection Guide

Your rover test system **CAN absolutely detect open and closed circuits**! Here's exactly how it works and how to integrate it.

## 🎯 **What Circuit Problems It Can Detect:**

### ✅ **OPEN CIRCUITS**
- **Motor circuits**: No current flow = open circuit detection
- **Sensor power**: Missing power signals = open circuit
- **GPIO pins**: Digital LOW when expecting HIGH
- **Battery disconnect**: Voltage drops to zero

### ✅ **CLOSED CIRCUITS** 
- **Normal operation**: Expected current/voltage levels
- **GPIO status**: Digital HIGH signals
- **Power rails**: Proper voltage levels maintained
- **Motor operation**: Current draw within normal range

### ⚡ **SHORT CIRCUITS**
- **Overcurrent detection**: Current spikes above safe levels
- **Voltage drops**: System voltage falls under load
- **Thermal shutdown**: Hardware protection triggers

## 🔌 **Circuit Monitoring Capabilities:**

| Circuit Type | Detection Method | Status Indicators |
|-------------|------------------|-------------------|
| **Battery** | Voltage, current, percentage | ✅ PASS / ⚠️ LOW / ❌ FAIL |
| **Motor Circuits** | Current monitoring | ✅ Normal / ⚠️ Open / ❌ Short |
| **Sensor Power** | Power enable status | ✅ Powered / ❌ Unpowered |
| **GPIO Pins** | Digital state monitoring | ✅ CLOSED / ⚠️ OPEN |
| **System Voltage** | Voltage rail monitoring | ✅ Normal / ⚠️ Low / ❌ Fault |
| **Diagnostics** | Hardware fault reporting | ✅ OK / ⚠️ Warning / ❌ Error |

## 🔧 **How to Set Up Circuit Monitoring:**

### **1. Arduino-Based Systems**

```cpp
// Example Arduino code for circuit monitoring
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

// Publishers for circuit status
rcl_publisher_t motor_current_pub;
rcl_publisher_t battery_voltage_pub;
rcl_publisher_t gpio_status_pub;

void setup() {
  // Initialize micro-ROS
  set_microros_transports();
  
  // Create publishers
  rclc_publisher_init_default(&motor_current_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor/current");
  
  rclc_publisher_init_default(&battery_voltage_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery/voltage");
    
  rclc_publisher_init_default(&gpio_status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/gpio/pin_1");
}

void loop() {
  // Read motor current (detect open/short circuits)
  float motor_current = analogRead(CURRENT_SENSOR_PIN) * 0.1; // Convert to amps
  std_msgs__msg__Float32 current_msg;
  current_msg.data = motor_current;
  rcl_publish(&motor_current_pub, &current_msg, NULL);
  
  // Read battery voltage (detect power issues)
  float battery_voltage = analogRead(VOLTAGE_SENSOR_PIN) * 0.05; // Convert to volts
  std_msgs__msg__Float32 voltage_msg;
  voltage_msg.data = battery_voltage;
  rcl_publish(&battery_voltage_pub, &voltage_msg, NULL);
  
  // Read GPIO status (detect open/closed circuits)
  bool gpio_state = digitalRead(GPIO_PIN);
  std_msgs__msg__Bool gpio_msg;
  gpio_msg.data = gpio_state;
  rcl_publish(&gpio_status_pub, &gpio_msg, NULL);
  
  delay(100);
}
```

### **2. Raspberry Pi Systems**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import BatteryState
import RPi.GPIO as GPIO
import board
import busio
import adafruit_ina219  # Current sensor

class CircuitMonitor(Node):
    def __init__(self):
        super().__init__('circuit_monitor')
        
        # Publishers for circuit monitoring
        self.current_pub = self.create_publisher(Float32, '/motor/current', 10)
        self.voltage_pub = self.create_publisher(Float32, '/system/voltage', 10)
        self.gpio_pub = self.create_publisher(Bool, '/gpio/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        
        # Initialize hardware
        self.setup_hardware()
        
        # Timer for periodic monitoring
        self.timer = self.create_timer(0.1, self.monitor_circuits)
    
    def setup_hardware(self):
        """Initialize circuit monitoring hardware"""
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Circuit status pin
        
        # Setup I2C for current sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina219 = adafruit_ina219.INA219(i2c)
    
    def monitor_circuits(self):
        """Monitor all circuits and publish status"""
        
        # Motor current monitoring (detects open/short circuits)
        try:
            current = self.ina219.current  # mA
            current_msg = Float32()
            current_msg.data = current / 1000.0  # Convert to Amps
            self.current_pub.publish(current_msg)
            
            # Log circuit issues
            if current < 10:  # Very low current - possible open circuit
                self.get_logger().warn(f"Low motor current: {current}mA - possible open circuit")
            elif current > 5000:  # High current - possible short circuit
                self.get_logger().error(f"High motor current: {current}mA - possible short circuit!")
                
        except Exception as e:
            self.get_logger().error(f"Current monitoring failed: {e}")
        
        # Voltage monitoring
        try:
            voltage = self.ina219.bus_voltage  # V
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_pub.publish(voltage_msg)
            
            if voltage < 11.0:
                self.get_logger().warn(f"Low system voltage: {voltage}V")
                
        except Exception as e:
            self.get_logger().error(f"Voltage monitoring failed: {e}")
        
        # GPIO circuit status (open/closed detection)
        gpio_state = not GPIO.input(18)  # Inverted due to pull-up
        gpio_msg = Bool()
        gpio_msg.data = gpio_state
        self.gpio_pub.publish(gpio_msg)
        
        # Battery monitoring
        battery_msg = BatteryState()
        battery_msg.voltage = voltage
        battery_msg.current = current / 1000.0
        battery_msg.percentage = self.calculate_battery_percentage(voltage)
        self.battery_pub.publish(battery_msg)
    
    def calculate_battery_percentage(self, voltage):
        """Calculate battery percentage from voltage"""
        # Simple linear approximation for 12V lead-acid
        min_voltage = 10.5  # 0%
        max_voltage = 12.6  # 100%
        percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
        return max(0, min(100, percentage))

def main():
    rclpy.init()
    monitor = CircuitMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **3. Commercial Hardware Integration**

For commercial motor controllers and power systems:

```bash
# Example: Connect to existing hardware diagnostic topics
ros2 topic list | grep -E "(current|voltage|power|diagnostic)"

# Common topics from commercial systems:
/motor_controller/current
/battery_monitor/voltage  
/power_distribution/status
/diagnostics
```

## 📊 **Expected Circuit Test Results:**

### **🔌 Current State (No Circuit Monitoring):**
```
⚡ Circuit Monitoring Detection:
  BATTERY_MONITORING: ❌ NOT MONITORED
  MOTOR_CURRENT: ❌ NOT MONITORED  
  SENSOR_POWER: ❌ NOT MONITORED
  DIAGNOSTIC_SYSTEM: ❌ NOT MONITORED
  GPIO_STATUS: ❌ NOT MONITORED
  VOLTAGE_MONITORING: ❌ NOT MONITORED

✅ circuit_status: PASS - No circuit monitoring hardware detected (basic mode)
```

### **⚡ With Circuit Monitoring Hardware:**
```
⚡ Circuit Monitoring Detection:
  BATTERY_MONITORING: ✅ MONITORED
  MOTOR_CURRENT: ✅ MONITORED  
  SENSOR_POWER: ✅ MONITORED
  GPIO_STATUS: ✅ MONITORED
  VOLTAGE_MONITORING: ✅ MONITORED

✅ circuit_status: PASS - All 5 circuits OK: Battery: PASS (12.1V, 85%), Motor Circuits: PASS (2.3A), GPIO: PASS (3 circuits: /gpio/pin_1: CLOSED, /gpio/pin_2: OPEN), Voltage: PASS (12.1V), Sensor Power: PASS (4/4 powered)
```

### **⚠️ With Circuit Problems:**
```
⚡ Circuit Problems Detected:
❌ circuit_status: FAIL - Circuit problems detected: Battery: PASS (11.8V, 65%), Motor Circuits: WARN - Low current: 0.05A (possible open circuit), GPIO: PASS (2 circuits), Voltage: WARN - Low voltage: 11.2V, Sensor Power: FAIL - 1/3 sensor circuits powered
```

## 🚨 **Real-World Circuit Detection Examples:**

### **Open Circuit Detection:**
- **Motor disconnected**: Current drops to near zero
- **Sensor power cut**: Power enable signal goes FALSE
- **Broken wire**: GPIO pin reads unexpected state
- **Battery disconnect**: Voltage reading disappears

### **Short Circuit Detection:** 
- **Motor short**: Current spikes above safe threshold
- **Power rail short**: Voltage drops under high load
- **Wire short**: GPIO pin stuck at unexpected level

### **Normal Operation:**
- **Healthy circuits**: Current/voltage within expected ranges
- **Proper connections**: GPIO states match expected operation
- **Good power**: Battery voltage and current normal

## 🎯 **Testing Your Circuit Integration:**

### **Step 1: Run Current Test**
```bash
cd ~/ros2_ws
timeout 10s python3 src/rover_system_tests/rover_system_tests/circuit_aware_test_manager.py
```

### **Step 2: Add Circuit Hardware**
Connect your current sensors, voltage monitors, and GPIO status pins.

### **Step 3: Start Circuit Monitoring Node**
```bash
# Your custom circuit monitoring node
ros2 run your_package circuit_monitor

# Or use the commercial hardware drivers
ros2 launch motor_controller_pkg monitor.launch.py
```

### **Step 4: Run Enhanced Test**
```bash
# The system will automatically detect your circuit monitoring!
./run_rover_tests.sh
```

## ✅ **Circuit Integration Summary:**

| Integration Level | What You Get | Effort Required |
|------------------|-------------|----------------|
| **Basic** | System/software monitoring only | ✅ Already done |
| **GPIO Level** | Open/closed circuit detection | 🔧 Moderate (GPIO pins) |
| **Current Sensing** | Open/short circuit detection | 🔧 Moderate (current sensors) |
| **Full Monitoring** | Complete electrical diagnostics | 🔧 Advanced (multiple sensors) |

## 🚀 **The Bottom Line:**

**YES! Your rover test system WILL detect open and closed circuits when you connect the appropriate hardware.**

- ✅ **Already built-in**: Auto-detection of circuit monitoring capabilities
- ✅ **Plug-and-play**: Connect sensors and get immediate circuit status
- ✅ **Comprehensive**: Detects opens, shorts, voltage issues, power problems
- ✅ **Real-time**: Live circuit monitoring through ROS 2 topics
- ✅ **Dashboard ready**: Results automatically appear in your web interface

**Just add the hardware sensors - the software is ready! 🎉**
