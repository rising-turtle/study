#!/bin/bash

# David Z, Sep 9, 2018 (hzhang8@vcu.edu)
# try to run 'rosbag play **.bag' and 'roslauch **.launch'
# 

cur_dir=`pwd`
rosbag_dir="/home/davidz/work/data/drone/dataset_3"
rosbag_fname="rgbd_imu.bag"
roslaunch_dir="/home/davidz/work/ros/indigo/src/vslam_node/VINS-mono/launch"
roslaunch_file="robocane_data_no_view.launch"
result_dir="/home/davidz/work/ros/indigo/src/vslam_node/VINS-mono"

cd $roslaunch_dir
echo "roslaunch $roslaunch_dir/$roslaunch_file"
roslaunch $roslaunch_file & >/dev/null 2>&1

# Save progress() PID
# You need to use the PID to kill the function
ROS_PID=$!

# wait for roslaunch start 
sleep 2 

cd $rosbag_dir
echo "rosbag play $rosbag_dir/$rosbag_fname"
rosbag play $rosbag_fname # >/dev/null 2>&1
echo "finish rosbag play!"

# Kill progresse
echo "kill pid $ROS_PID"
kill $ROS_PID >/dev/null 2>&1

### process the result 
cd $result_dir
echo "handle $result_dir"
cp "vins_result.csv" "./result/exp1.csv"

echo "finish the job, return to $cur_dir"
cd $cur_dir

exit 0



