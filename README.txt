# 1. Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. Go to workspace
Ensure you are in the correct repository by inputting
cd ~/Relbot_ws

# 3. Build
To build after you have saved run this in the terminal:
colcon build

# 4. Run the videoserver.py
Before you run the files, you have to enable your webcam, so open a second terminal, go to ...\cam2image_host2vm and from there run
python videoserver.py

# 5. Launch
In the first terminal run the launch file:
ros2 launch object_detection 6.2_object_detection.launch.py

