#!/bin/bash
#
# recusively compute the size of a folder 
#
usage="usage: fold_size dir"
if [ "$#" -ne 1 ]; then
  echo $usage
  exit
fi

SIZE=$(du -sb $1 | cut -f 1)
echo "$1 size: $SIZE bytes"



