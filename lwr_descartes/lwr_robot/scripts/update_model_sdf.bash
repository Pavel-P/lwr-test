#!/bin/bash
#Script to update model sdf's from xacro's
for dir in $(rospack find lwr_robot)/models/*; do
    dir=${dir##*/}
    xacro_file="$(rospack find lwr_robot)/models/$dir/$dir.urdf.xacro"
    if [ -f "$xacro_file" ]; then
        rosrun xacro xacro.py "$xacro_file" -o tmp
        gz sdf -p tmp > "$(rospack find lwr_robot)/models/$dir/model.sdf"
        rm tmp
    fi
done
