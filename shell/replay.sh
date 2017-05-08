#!/bin/bash

# David Z, Jan 14, 2015 
# to replay the video from $1 to $2 

root_dir="/home/davidz/work/motion_recog"
bin_dir="$root_dir/build/bin/body_recog"
cur_dir=`pwd`

if [ $# -ge 2 ]; then
  s=$1
  e=$2
  if [ $s -le $e ]; then
    i=$s
    cd $bin_dir
    while [ $i -le $e ]; do
      echo "replay_video $i"
      ./replay_video $i >/dev/null 2>&1 &
      i=$((i+1))
    done
    cd $cur_dir
  fi
fi


