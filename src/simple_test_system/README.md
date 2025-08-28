# Simple Test System

A dead-simple ROS 2 communication test system. Helps figure out if nodes can actually talk to each other.

## What's this for?

The ROS network can be finicky sometimes. This package gives you a simple way to check if:

1. Nodes can discover each other
2. Publishers and subscribers are working
3. Messages are making it through the system

It's basically "ping" for ROS 2 - nothing fancy, just a basic connectivity check.

## How it works

The system consists of two node types:

- **Tester Node**: Broadcasts a "report for duty" message and counts responses
- **Responder Node**: Listens for the request and sends back its identity

The tester waits a bit for responses, then shows you what it found.

## Running the test

```bash
# Terminal 1 - start the tester
ros2 run simple_test_system tester_node

# Terminal 2+ - start as many responders as you want
ros2 run simple_test_system responder_node --ros-args -p node_name:=robot1
ros2 run simple_test_system responder_node --ros-args -p node_name:=robot2
ros2 run simple_test_system responder_node --ros-args -p node_name:=sensor_hub
```

The tester will wait about 7 seconds to collect responses, then report what it found.

## Sample output

```
Network test complete!
Found 3 responding nodes:
  - robot1
  - robot2
  - sensor_hub
Network looks good to go! 👍
```

## Why I made this

Got tired of more complex tools when I just needed to know "is ROS working?" This is just the minimum to verify basic communication without any fancy extras.

I use this as a first step whenever something isn't working right - if this doesn't work, nothing else will.

## Known limitations

- Super basic - not meant for performance testing
- No QoS configuration options
- Fixed 7-second timeout (change TEST_TIMEOUT in the code if needed)
- Doesn't check for message content/corruption

## TODO (maybe)

- [ ] Add launch file for common configs
- [ ] Add parameter for custom timeout
- [ ] Report discovery time for each node
- [ ] Add message timing stats
