#!/bin/bash
# Program:
#	The first argument must be hello
# History:
# 	2012/7/17 Zhanghe first release

if [ "$1" == "" ]; then
	echo "No arguments!" 
elif [ "$1" == "hello" ]; then
	echo "Hello! How are you"
else 
	echo "The first argu must be hello!"
fi

echo "first $1"
echo "total $#"
