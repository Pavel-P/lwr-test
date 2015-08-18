# lwr-test
KUKA LWR Gazebo Simulation with Descartes Motion Planning

#Installation
To install, clone the repository recursively into your catkin workspace src folder
 and ```catkin_make``` it. You may need to install the following packages if you 
have not done so already:
- ros-indigo-desktop-full
- gazebo5
- libgazebo5-dev
- gazebo-ros-pkgs
- moveit-full
- realtime-tools

If a missing dependency causes the build to fail, it should be easy to find through the ROS wiki or APT

#Usage
1. Source ```update_model_path.bash``` in the lwr_robot package to update the Gazebo model path with the relevant ros packages
2. Run ```update_model_sdf.bash``` to update the sdf files for each model in lwr_robot/models based on corresponding xacro files
3. Use ```roslaunch lwr_robot lwr.launch load_moveit:=true``` to load Gazebo and Moveit (note: if the simulation is paused Moveit will hang)
4. Use ```roslaunch lwr_descartes_demo demo.launch``` to load Descartes (MoveIt must be running)
