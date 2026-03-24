Run this before you start in your terminal:
source /opt/ros/humble/setup.bash
source ~/Relbot_ws/install/setup.bash

Ensure you are in the correct repository by inputting
cd Relbot_ws

To build after you have saved run this in the terminal:
colcon build

Then run this to launch files and run everything: 
ros2 launch relbot_launch relbot_sequence_controller.launch.py

