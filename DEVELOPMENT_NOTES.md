# Development Notes & TODOs

Random thoughts and todos for the rover test system. Mostly just keeping track of ideas and things that need fixing.

## Current Status

The basic test system is working pretty well. Test manager catches most of the obvious ROS issues, and the simple ping system helps verify basic communication.

## Near-term TODOs

### rover_system_tests
- [ ] Fix the memory parsing - it's fragile and probably breaks on non-Ubuntu systems
- [ ] Add actual hardware integration tests (need to wait for hardware)
- [ ] Make the hardware detection smarter - right now it just looks for topic names
- [ ] Add configuration file support (YAML?) for test parameters
- [ ] Better error handling in subprocess calls - some can hang indefinitely

### simple_test_system  
- [ ] Add launch file for common test scenarios
- [ ] Make the timeout configurable via parameter
- [ ] Add timing stats for response latency
- [ ] Maybe add a \"stress test\" mode with rapid-fire messages?

## Ideas for Later

- **Performance testing**: Could extend the simple test system to do throughput/latency measurements
- **Hardware simulation**: Mock hardware nodes for testing without physical components
- **Web dashboard**: Real-time test results in a web interface (probably overkill)
- **Integration with CI**: Automated testing in GitHub Actions or similar

## Known Issues

1. **Memory check fragility**: The `free` command parsing assumes a specific format. Works on Ubuntu 20.04/22.04 but might break elsewhere.

2. **Subprocess timeouts**: Some ROS 2 commands can hang if the middleware is having issues. Current timeouts help but aren't comprehensive.

3. **Hardware detection**: Very basic pattern matching on topic names. Real hardware might use different naming conventions.

4. **No QoS testing**: The simple test system uses default QoS settings. In the real world, QoS mismatches cause communication issues.

## Debugging Tips (for future me)

- If tests are timing out, check if ROS_DOMAIN_ID is set correctly
- Memory issues usually show up as the system getting sluggish during tests
- Hardware detection works best when nodes are already running (obvious, but easy to forget)
- The circuit-aware test manager is still experimental - don't rely on it yet

## Random Thoughts

- Should probably add some kind of configuration system instead of hardcoding timeouts and parameters
- The emoji in log messages might be annoying in some terminals - could make it configurable
- Considered using ROS parameters more extensively but string-based results are easier to debug
- The \"mood\" messages in test results are silly but actually help during debugging sessions

## Test Scenarios to Consider

1. **Slow startup**: Some nodes take forever to initialize
2. **Intermittent communication**: WiFi dropping packets, USB disconnects, etc.
3. **Resource exhaustion**: What happens when we run out of memory/CPU?
4. **Multiple ROS domains**: Accidental domain conflicts
5. **Mixed ROS versions**: ROS 1 bridge complications

---

*Last updated: Whenever I remembered to update this file*

## Archive of Old TODOs (completed)

- ✅ Make test output more readable (was way too verbose before)
- ✅ Add basic hardware detection (simple but functional)
- ✅ Create simple communication test (the ping system)
- ✅ Better error handling in main test loop
- ✅ Add documentation that doesn't suck
