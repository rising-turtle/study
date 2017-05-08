#!/bin/bash

# David Z, Jan 30, (hxzhang1@ualr.edu)
# implement body_main $1-$2 times and copy the created file into $dst/motion_record_$#.log 
#
root_dir="/home/davidz/work/motion_recog"
bin_dir="$root_dir/build/bin/body_recog"
dst_dir="$root_dir/matlab_dir/records"
cur_dir=`pwd`
echo '$# = ' $#
if [ $# -eq 0 ]; then
  echo "usage: run_copy_file.sh (number of files to be written in sequence)"
else
  s=$1
  e=$1
  if [ $# -ge 2 ]; then
    e=$2
  fi
  i=$s
  cd $bin_dir # cd the bin directory to run the program
  while [ $i -le $e ]; do
    echo "./body_main $i"
    ./body_main $i >/dev/null 2>&1
    src="motion_record.log"
    dst="$dst_dir/motion_record_$i.log"
    echo "copy file $src to $dst"
    cp $src $dst
    i=$((i+1))
    sleep 2 # default unit is second, sleep 2 seconds
  done
  cd $cur_dir # return back to the current_dir 
  echo "finish the job, return to $cur_dir"
fi
