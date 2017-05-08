#!/bin/bash
#
# cp_dir dir_from dir_to, cp files in dir1 to dir2
#   
#

usage="usage: cp_dir dir_from dir_to"
entry_f=""
entry_t=""
if [ "$#" -ne 2 ]; then
  if  [ "$#" -eq 1 ] && [ "$1" = "-help" ]  || [ "$1" = "--h" ]; then
    echo "$1: cp_dir_ls dir_from dir_to"
    exit
  fi
  echo $usage
  exit 
else
  if [ -d "$1" ]; then
    entry_f=`ls $1` 
  else
    echo "$1 not a directory"
    echo $usage
    exit
  fi
  if [ -d "$2" ]; then
    entry_t=`ls $2`
  else
    echo "$2 not a directory"
    echo $usage
    exit
  fi
fi

 for entry in $entry_f
 do
  if [[ "$entry_t" =~ $entry ]]; then
     echo "$2 contains $entry"
     #cp $entry $2/
   else
     echo "missing $1/$entry in $2"
     echo "cp $1/$entry to $2/"
     cp "$1"/$entry "$2"/
  fi
 done

# echo "in $1: $entry_f"
# echo "in $2: $entry_t"


