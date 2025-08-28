# ROS 2 Rover Testing Workspace

My collection of ROS 2 testing tools for rover development. Started this after too many "why isn't it working?" debugging sessions that could have been avoided with some basic health checks.

## What's Here

This workspace contains a few different testing systems I've built over time:

- **System Health Checker**: Basic ROS environment validation (is everything actually running?)
- **Communication Tester**: Simple ping system to verify nodes can talk to each other  
- **Hardware Detection**: Auto-detects connected motors/sensors when available
- **Development Utilities**: Scripts and tools for easier manual testing

Nothing too fancy - just practical stuff that saves debugging time.

## Quick Start

```bash
# Build everything
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run the main system test
ros2 run rover_system_tests test_manager_node

# Or try the simple communication test
ros2 run simple_test_system tester_node
```

There's also a quick test script if you don't want to remember the ROS commands:

```bash
./quick_test.py basic    # basic system check
./quick_test.py ping     # communication test
./quick_test.py all      # run everything
```

## What Gets Tested

**Basic System Checks:**
- ROS 2 environment (ros2 doctor)
- Active nodes and topics
- System resources (memory/disk)
- Basic hardware detection

**Communication Tests:**
- Node discovery and messaging
- Publisher/subscriber functionality
- Network connectivity

The system is designed to work in both simulation and with real hardware. If no hardware is connected, it just notes that and continues - no failures for missing sensors in development mode.

## Project Structure

```
~/ros2_ws/
├── src/
│   ├── rover_system_tests/          # Main system health checks
│   │   ├── rover_system_tests/
│   │   │   ├── test_manager_node.py         # Main test orchestrator
│   │   │   ├── hardware_ready_test_manager.py  # Hardware detection version
│   │   │   └── circuit_aware_test_manager.py   # Circuit monitoring (experimental)
│   │   └── launch/
│   │       └── full_test_dashboard.launch.py
│   └── simple_test_system/          # Basic communication tests
│       └── simple_test_system/
│           ├── tester_node.py       # Sends ping requests
│           └── responder_node.py    # Responds to pings
├── DEVELOPMENT_NOTES.md             # TODOs and random thoughts
├── quick_test.py                    # Manual test runner
└── README.md                        # This file
```

## Package Details

### rover_system_tests
The main testing package. Runs through a comprehensive checklist of ROS and system health:
- ROS 2 environment validation
- Node and topic discovery
- Hardware detection (when available)
- System resource monitoring

### simple_test_system  
A dead-simple communication test. One node broadcasts "report for duty", others respond with their names. Good for verifying basic ROS networking is working.

This is usually the first thing I run when something seems broken - if this doesn't work, nothing else will.

## Usage Examples

**Basic system health check:**
```bash
ros2 run rover_system_tests test_manager_node
```

**Communication test (run these in separate terminals):**
```bash
# Terminal 1 - start some responders
ros2 run simple_test_system responder_node --ros-args -p node_name:=robot1
ros2 run simple_test_system responder_node --ros-args -p node_name:=sensor_hub

# Terminal 2 - run the test
ros2 run simple_test_system tester_node
```

**Monitor test results:**
```bash
ros2 topic echo /tests/summary
```

## Hardware Integration

The tests automatically detect available hardware based on ROS topic names. In simulation mode (no hardware), everything still passes with appropriate messages.

When you connect real hardware that publishes to standard topics like `/cmd_vel`, `/image_raw`, `/imu/data`, etc., the system will detect and test it automatically.

## Sample Output

```
TEST CYCLE #1 COMPLETE (2.1s)
Looking good! 🎉
Status: PASS (5/5 passed)

  ✓ Ros2 Basics: PASS - ROS environment looks good
  ✓ Node Status: PASS - Found 3 active nodes
  ✓ Communication: PASS - 12 topics available for communication
  ✓ Hardware Actuators: PASS - No motor hardware detected (probably simulation mode)
  ✓ Hardware Sensors: PASS - No sensor hardware detected (simulation mode OK)
```

## Development Notes

Check `DEVELOPMENT_NOTES.md` for TODOs, known issues, and random thoughts about the codebase.

## Why This Exists

After too many "worked on my machine" moments and debugging sessions that could have been avoided with basic checks, I figured we needed something simple to validate the system state before wasting time on complex issues.

It's not meant to be comprehensive - just catch the obvious stuff early.
