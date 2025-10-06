turtlebot3_ws

ROS 2 workspace containing:
- ecn_turtlebot3x – custom TurtleBot3 package (launch, config, maps, rviz)
- turtlebot3 – upstream TurtleBot3 packages
- turtlebot3_simulations – Gazebo/RViz simulation for TurtleBot3

Tested with ROS 2 Humble + colcon. Works on Ubuntu 22.04 and Apple Silicon (via UTM VM).


------------------------------------------------------------
How to clone and build this repository
------------------------------------------------------------
# Clone the repository
git clone git@github.com:Amruth-bn/turtlebot3_ws.git
cd turtlebot3_ws

# Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the setup file
source install/setup.bash


------------------------------------------------------------
1. Workspace layout
------------------------------------------------------------
turtlebot3_ws/
├─ ecn_turtlebot3x/
│  ├─ launch/
│  ├─ config/
│  ├─ maps/
│  ├─ rviz/
│  ├─ package.xml
│  └─ setup.py / setup.cfg
├─ turtlebot3/
└─ turtlebot3_simulations/


------------------------------------------------------------
2. Prerequisites
------------------------------------------------------------
- ROS 2 Humble
- colcon, rosdep
- Gazebo (Classic / Fortress / Garden)
- Python 3.10+

sudo apt update
rosdep update
rosdep install --from-paths . --ignore-src -r -y


------------------------------------------------------------
3. Build the workspace
------------------------------------------------------------
colcon build --symlink-install
source install/setup.bash


------------------------------------------------------------
4. Environment setup
------------------------------------------------------------
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/turtlebot3_ws/turtlebot3_simulations/turtlebot3_gazebo/models

You can add these lines to your ~/.bashrc file for convenience.


------------------------------------------------------------
5. Running examples
------------------------------------------------------------
Replace launch file names with your actual ones inside ecn_turtlebot3x/launch.

A) Simulation (Gazebo + RViz)
    ros2 launch ecn_turtlebot3x bringup_sim.launch.py

B) Teleoperation
    ros2 run turtlebot3_teleop teleop_keyboard

C) Navigation (Nav2)
    ros2 launch ecn_turtlebot3x nav2.launch.py

D) SLAM / Localization
    ros2 launch ecn_turtlebot3x slam.launch.py
    ros2 launch ecn_turtlebot3x localization.launch.py map:=maps/my_map.yaml


------------------------------------------------------------
6. Maps and RViz
------------------------------------------------------------
Maps → ecn_turtlebot3x/maps/
RViz configs → ecn_turtlebot3x/rviz/

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix ecn_turtlebot3x)/share/ecn_turtlebot3x/rviz/my_config.rviz


------------------------------------------------------------
7. Development tips
------------------------------------------------------------
rm -rf build/ install/ log/
colcon build --symlink-install

ament_flake8 .
ament_pep257 .


------------------------------------------------------------
8. Troubleshooting
------------------------------------------------------------
ros2: command not found → source /opt/ros/humble/setup.bash
Packages not found → source install/setup.bash
Gazebo models missing → export GAZEBO_MODEL_PATH
Wrong robot model → export TURTLEBOT3_MODEL=burger


------------------------------------------------------------
9. License
------------------------------------------------------------
Add a LICENSE file (MIT or BSD-3-Clause) if you make the repo public.


------------------------------------------------------------
10. Acknowledgements
------------------------------------------------------------
- ROBOTIS TurtleBot3 packages
- Gazebo & ROS 2 community
- École Centrale de Nantes – Robotics Lab
