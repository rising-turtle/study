#!/bin/bash

# David Z, June 16, 2015 (hxzhang1@ualr.edu)
# record initial imu data by imu_record, then record sr data by sr_record
# implement body_main $1-$2 times and copy the created file into $dst/motion_record_$#.log 
#

imu_bin_dir="/home/davidz/work/imu_reader/Bluetooth/build/bin"
sr_bin_dir="/home/davidz/work/EmbMess/mesa/pcl_mesa/build/bin"
data_dir="/home/davidz/work/data/SwissRanger4000/exp"
cur_dir=`pwd`

i=0
echo '$#= '$#
if [ $# -gt 0 ]; then
  i=$1
fi
data_dir="$data_dir/dataset_$i"
echo "record data in $data_dir"

cd $imu_bin_dir 
echo "./imu_recorder $data_dir"
./imu_recorder $data_dir >/dev/null 2>&1
echo "finish record imu data"

cd $sr_bin_dir
echo "./sr_record $data_dir"
./sr_record $data_dir 

echo "finish the job, return to $cur_dir"
cd $cur_dir


