#!/bin/bash
#
# cp_dir dir_from dir_to, cp files in dir1 to dir2
#
#

usage="usage: cp_dir dir_from dir_to"
entry_f=""
entry_t=""
if [ "$#" -ne 2 ]; then
  echo $usage
  exit 
else
  if [ -d "$1" ]; then
    for entry in "$1"/*
    do
      if [ -f $entry ]; then
        entry_f="$entry_f $entry"
      fi
    done
  else
    echo "$1 not a directory"
    echo $usage
    exit
  fi
  if [ -d "$2" ]; then
    for entry in "$2"/*
    do
      if [ -f $entry ]; then
        entry_t="$entry_t $entry"
      fi
    done
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
     echo "cp $entry to $2/"
     cp $entry "$2"/
  fi
 done

# echo "entry in $1 : $entry_f"
# echo "entry in $2 : $entry_t"

