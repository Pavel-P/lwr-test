This package contains sdf models for the LWR arm, Robotiq S gripper, and a complete gripper-arm robot
The package also contains a demo world for picking and placing a metal peg.

To launch, source the "update model path" script and use ```roslaunch lwr_robot lwr.launch load_moveit:=true```
If you change the xacro files make sure to run the "update model sdf" script if you are loading the sdf models through the gazebo world
