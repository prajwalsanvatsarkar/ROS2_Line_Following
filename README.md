# ROS2 Line Following

This project demonstrates a **ROS 2 line-following robot** using TurtleBot3 in Gazebo.  
It uses **OpenCV** to detect lane lines (white/yellow) from the robot’s camera and publishes velocity commands to follow the line.  

---

## Features
- Runs in **Gazebo** with TurtleBot3 (Stage1 / AutoRace world)
- Subscribes to the robot’s camera feed
- Uses OpenCV (`cv2.findContours`, `cv2.moments`) to detect line centroid
- Publishes velocity commands to `/cmd_vel` for line following
- Clean ROS 2 Python node (`follow_line_node.py`) + launch file

---

## Requirements
- ROS 2 Humble (or compatible)
- `turtlebot3_gazebo` and `turtlebot3_autorace` packages
- Python 3.x with OpenCV (`cv2`)

---

## Build

```bash
# 1) Create a workspace if not already
mkdir -p ~/autorace_ws/src
cd ~/autorace_ws/src

# 2) Clone this repo
git clone https://github.com/prajwalsanvatsarkar/ROS2_Line_Following.git

# 3) Build with colcon
cd ..
colcon build --symlink-install

# 4) Source
source install/setup.bash
```

## Run
```bash

cd ~/autorace_ws
source install/setup.bash

# set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# avoid DDS multicast warnings
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7

# launch
ros2 launch follow_line follow_line.launch.py

After Gazebo loads, the robot detects the lane line and follows it automatically.
```
