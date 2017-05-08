#!/bin/bash

# David Z, June 18, 2015 (hxzhang1@ualr.edu)
# record sr data by sr_record, through USB, or TCP 
# into bdat format, z, x, y, intensity, confidence 
# 

sr_bin_dir="/home/davidz/work/EmbMess/mesa/pcl_mesa/build/bin"
data_dir="/home/davidz/work/data/SwissRanger4000/exp"
# data_dir="/home/davidz/work/data/SwissRanger4000/with_gt"
# data_dir="/home/davidz/work/data/SwissRanger4000/bdat/angle_test/pitch_v4"
# data_dir="/home/davidz/work/data/SwissRanger4000/bdat/angle_test/yaw_v1" 
#  data_dir="/home/davidz/work/data/SwissRanger4000/bdat/angle_test/tx_v2" 
# data_dir="/home/davidz/work/data/SwissRanger4000/bdat/angle_test/tx_v2" 


cur_dir=`pwd`
mode="USB"
sr_ip="192.168.0.11"
data_version="old"

i=0
echo '$#= '$#
if [ $# -gt 0 ]; then
  i=$1
  if [ $# -gt 1 ]; then
    sr_ip=$2
  fi
fi
data_dir="$data_dir/dataset_$i"
echo "record data in $data_dir"

cd $sr_bin_dir
echo "./sr_record $data_dir $mode"
# ./sr_record $data_dir "TCP" "IP" "$data_version"
# ./sr_record $data_dir "USB" "$data_version"
./sr_record $data_dir $mode $data_version
# ./sr_record $data_dir "TCP" "192.168.0.11" $data_version

echo "finish the job, return to $cur_dir"
cd $cur_dir


