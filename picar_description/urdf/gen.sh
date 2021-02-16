#!/bin/bash

URDF_FILE="model.urdf"
SDF_FILE="model.sdf"

SIMU_FOLDER="simulation"

#apt update && apt install nano liburdfdom-tools

echo "Generate URDF from xacro..."
xacro -v picar.urdf.xacro -o ./$URDF_FILE

echo "Validate herarchy..."
urdf_to_graphiz $URDF_FILE

echo "Generate Gazebo SDF..."
cp ./$URDF_FILE $SIMU_FOLDER/models/picar/$URDF_FILE
gz sdf -p $SIMU_FOLDER/models/picar/$URDF_FILE > $SIMU_FOLDER/models/picar/$SDF_FILE

echo "Display on rviz."
roslaunch urdf_tutorial display.launch model:=$URDF_FILE

#source /usr/share/gazebo-11/setup.sh
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH/root/ros_ws/ros_ws/src/picar_ros/picar_description/urdf/simulation/models/:
#gazebo simulation/worlds/picar.world