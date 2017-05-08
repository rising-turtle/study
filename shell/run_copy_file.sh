#!/bin/bash

# David Z, Jan 30, (hxzhang1@ualr.edu)
# implement write_n $1 times and copy the created file into tmp_$1.log 
#
cur_dir=`pwd`;
echo "cur_dir: $cur_dir"
dir="/home/davidz/work/motion_recog/matlab_dir/records"
echo '$# = ' $#
if [ $# -eq 0 ]; then
  echo "usage: run_copy_file.sh (number of files to be written in sequence)"
else
  i=1
  while [ $i -lt $1 ]; do
    echo "./write_n $i"
    ./write_n $i >/dev/null 2>&1
    src="write_$i.log"
    # dst="$dir/tmp_$i.log"
    dst="$cur_dir/tmp_$i.log"
    echo "copy file $src to $dst"
    cp $src $dst
    i=$((i+1))
  done
fi
