# Rover System Tests

Basic health checks for our ROS 2 rover project. Started building this after getting tired of debugging "why isn't it working" issues in the field.

## What This Does

Runs through a bunch of sanity checks to make sure everything is basically working before you try to drive the rover around. Think of it as a pre-flight checklist.

**Current checks:**
- ROS 2 environment (is `ros2 doctor` happy?)
- Node discovery (who's actually running?)
- Topic communication (can nodes talk to each other?)
- Hardware detection (what motors/sensors do we have?)
- System resources (enough memory/disk space?)

## Quick Start

```bash
# From your ROS 2 workspace
cd /path/to/your/ros2_ws

# Build it
colcon build --packages-select rover_system_tests

# Source the workspace
source install/setup.bash

# Run the basic test manager
ros2 run rover_system_tests test_manager_node

# Or use the launch file for the full dashboard
ros2 launch rover_system_tests full_test_dashboard.launch.py
```

## What You'll See

The test manager spits out a bunch of info as it runs through checks. At the end you get a summary that looks something like:

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

## Files in Here

- `test_manager_node.py` - Main test orchestrator (this does most of the work)
- `hardware_ready_test_manager.py` - Smart hardware detection version (WIP)
- `circuit_aware_test_manager.py` - Circuit-specific tests (experimental)
- `main.py` - Simple CLI runner if you don't want to use ros2 run

## Hardware vs Simulation

The tests are designed to work in both simulation and with real hardware. In simulation mode, it's totally fine if no motor/sensor topics exist - we just note that and move on.

When you hook up real hardware, the system should auto-detect available sensors and actuators based on what ROS topics are published.

## Adding New Tests

If you want to add more checks, look at the `run_all_tests()` method in `test_manager_node.py`. The pattern is pretty straightforward:

1. Try to check something
2. Catch any errors gracefully  
3. Return a status string that starts with "PASS", "WARN", or "FAIL"
4. Log what happened

## Known Issues

- Memory parsing is a bit fragile (works on Ubuntu, might break on other distros)
- Hardware detection is pretty basic - just looks for common topic names
- No real timeout handling for hardware tests yet
- The circuit-aware tests are still experimental

## Why This Exists

After too many "worked on my machine" moments and debugging sessions that could have been avoided with basic checks, I figured we needed something simple to validate the system state.

It's not meant to be comprehensive - just catch the obvious stuff that wastes time.

---

*Last updated: When I got tired of manually checking if ROS was working*
