#!/bin/bash

# ===========================================
# 🚀 ROVER ONE-BUTTON SYSTEM TEST LAUNCHER
# ===========================================

echo "🚀 Starting Rover System Test Dashboard..."
echo "=========================================="

# Navigate to workspace
cd ~/ros2_ws

# Source the environment
echo "📦 Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if package is built
if [ ! -d "install/rover_system_tests" ]; then
    echo "❌ Package not built. Building now..."
    colcon build --packages-select rover_system_tests
    source install/setup.bash
fi

echo "✅ Environment ready!"
echo ""

# Give user options
echo "Choose an option:"
echo "1. Run test system (launches ROS 2 node)"
echo "2. Open web dashboard"
echo "3. Monitor test results (ros2 topic echo)"
echo "4. Run single test manually"
echo "5. Exit"
echo ""

read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        echo "🔍 Launching ROS 2 test manager..."
        ros2 launch rover_system_tests full_test_dashboard.launch.py
        ;;
    2)
        echo "🌐 Opening web dashboard..."
        if command -v xdg-open > /dev/null; then
            xdg-open ~/ros2_ws/web/index.html
        elif command -v open > /dev/null; then
            open ~/ros2_ws/web/index.html
        else
            echo "📂 Please open: ~/ros2_ws/web/index.html in your browser"
        fi
        ;;
    3)
        echo "👂 Monitoring test results on /tests/summary topic..."
        echo "   (Run the test manager in another terminal first)"
        ros2 topic echo /tests/summary
        ;;
    4)
        echo "🔧 Running single test node..."
        ros2 run rover_system_tests test_manager_node
        ;;
    5)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice. Please run the script again."
        exit 1
        ;;
esac
