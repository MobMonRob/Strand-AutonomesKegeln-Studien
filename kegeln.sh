#!/bin/bash

# Start lidar sensor
roslaunch sick_scan sick_lms_5xx.launch > sick_scan_log &
ROS_SICK_PID=$!

echo "Starting lidar, waiting for 20 seconds..."
sleep 20

# Start pointcloud 
rosrun tf static_transform_publisher 0 0 0 0 0 0 map cloud 50 &
ROS_POINTCLOUD_PID=$!

# Start object tracking
rosrun sphero_bolt_node object_tracking > object_tracking_log &
ROS_OBJECT_TRACKING_PID=$!

# fordern das sphero richtig vor die kegel gelegt wird
echo "Sphero bolt, FRONT TOWARD ENEMY"
echo "Press any key to continue..."
read -n 1

# Start sphero
rosrun sphero_bolt_node move_sphero.py > move_sphero_log &
ROS_SPHERO_PID=$!

echo "Take Sphero if he glows green, then press any key to continue..."
read -n 1

# Start control
rosrun sphero_bolt_node control > control_log &
ROS_CONTROL_PID=$!

echo "Ready to play!"
echo "Press any key to exit..."
read -n 1

echo "exiting..."
kill $ROS_CONTROL_PID
sleep 1
kill $ROS_SPHERO_PID
sleep 1
kill $ROS_OBJECT_TRACKING_PID
sleep 1
kill $ROS_POINTCLOUD_PID
sleep 1
kill $ROS_SICK_PID
echo "stoped"
