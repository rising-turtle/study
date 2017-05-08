#!/bin/bash

# David Z, June 18, 2015 (hxzhang1@ualr.edu)
# record sr data by sr_record, through TCP 
#

sr_bin_dir="/home/davidz/work/EmbMess/mesa/pcl_mesa/build/bin"
# data_dir="/home/davidz/work/data/SwissRanger4000/exp"
data_dir="/home/davidz/work/data/SwissRanger4000/with_gt"
cur_dir=`pwd`
mode="TCP"
sr_ip="192.168.1.42"

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
./sr_record $data_dir $mode $sr_ip

echo "finish the job, return to $cur_dir"
cd $cur_dir


