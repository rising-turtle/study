#!/bin/bash 
# Program:
#	User input a filename, program will check the flowing:
#	1.) exist? 2.) file/directory? 3.) file permissions
# History:
#	2012/7/17 ZhangHe First release
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

# 1. let the user input the filename
echo "input a filename!"
read -p "filename: " filename
test -z $filename && echo "You must input a filename: " && exit 0

# 2. If this file not exist, exit
test ! -e $filename && echo "The filename ${filename} do not exit" && exit 0

# 3. check the type of this file
test -f $filename && filetype="regular file"
test -d $filename && filetype="directory"
test -r $filename && perm="readable"
test -w $filename && perm="${perm} writable"
test -x $filename && perm="${perm} executable"

# 4. output its information
echo "The file: $filename is a $filetype"
echo "And the permission is: $perm"
