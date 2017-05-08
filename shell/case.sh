#!/bin/bash

echo -n "Enter a number >"

#read number
# case $number in
#  1) echo "You entered one"
#    ;;
#  2) echo "you entered two"
#    ;;
#  *) echo "you entered $number"
#esac

read ch
case $ch in
  [0-9]) echo "You entered a number $ch"
    ;;
  [a-z]|[A-Z]) echo "You entered a letter $ch"
    ;;
  *) echo "You entered what? $ch"
esac

