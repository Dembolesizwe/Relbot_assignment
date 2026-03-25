Run this before you start in your terminal:
# 1. Source ROS2
source /opt/ros/humble/setup.bash

Ensure you are in the correct repository by inputting
# 2. Go to workspace
cd ~/Relbot_ws

To build after you have saved run this in the terminal:
# 3. Build
colcon build

Then run this to launch files and run everything: 
# 4. Source workspace
source install/setup.bash

# 5. Launch
ros2 launch relbot_launch relbot_sequence_controller.launch.py

