#!/bin/bash
# Program :
#	check ports: 80,22, 
# History:
#	2012/7/17 Zhanghe FR

testing=$(netstat -tuln | grep ":80")
if [ "$testing" != "" ]; then
	echo "WWW is running in your pc!"
	echo "testing = $testing"
fi
testing=$(netstat -tuln | grep ":22")
if [ "$testing" != "" ]; then
	echo "SSH is running in your pc!"
	echo "testing = $testing"
fi
