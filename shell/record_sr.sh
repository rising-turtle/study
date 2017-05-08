#!/bin/bash

# David Z, June 18, 2015 (hxzhang1@ualr.edu)
# record sr data by sr_record, through USB
#

sr_bin_dir="/home/davidz/work/EmbMess/mesa/pcl_mesa/build/bin"
data_dir="/home/davidz/work/data/SwissRanger4000/exp"
# data_dir="/home/davidz/work/data/SwissRanger4000/with_gt"
cur_dir=`pwd`

i=0
echo '$#= '$#
if [ $# -gt 0 ]; then
  i=$1
fi
data_dir="$data_dir/dataset_$i"
echo "record data in $data_dir"

cd $sr_bin_dir
echo "./sr_record $data_dir"
./sr_record $data_dir 

echo "finish the job, return to $cur_dir"
cd $cur_dir


