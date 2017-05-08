#!/bin/bash
# Program:
#	output the executed file, and parse arguments
#	User input the arguments $1,$2,$3...
# History
#	2012/7/17 Zhanghe First Release

echo "filename: $0"
echo "The N of arguments is: $#"
[ "$#" -lt "2" ] && echo "too few arguments" && exit 0
echo "Whole argument: $@"
echo "first argu: $1"
echo "second argu: $2"

	
