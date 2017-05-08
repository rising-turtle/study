#!/bin/bash 

# control the input parameters using $#

# echo '$0 = ' $0
echo '$1 = ' $1

number=0
# while [ $number -lt 10 ]; do
#    echo "Number = $number"
#    number=$((number + 1))
# done

echo '$# = ' $#
while [ "$1" != "" ]; do
  echo $1
  ./print_n $1 >/dev/null 2>&1 & # redirect the output to the back ground session, 
shift
done

