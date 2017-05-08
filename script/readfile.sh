#!/bin/bash
# Program:
#	This program will read file /etc/passwd and output the first column
# History:
# 	2012/7/17 Zhanghe fr

echo "This program will read file /etc/passwd, and output the first col"

content=$( cut -d ":" -f1 /etc/passwd )
declare -i num=1
for item in $content
do
	echo "The $num is $item"
	num=($num+1)
done
