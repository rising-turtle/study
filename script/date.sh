#!/bin/bash
# Program :
#	User input a date, and the dis between this date with current one will be calculated 
# Example : 
#	User input 20101015, and current date is 20100903 output is 42days
# 
# History :
#	2012/7/17 Zhanghe

echo "This program will calculate: "
echo "How many days between the input date with current date."
read -p "Please input a date (YYYYMMDD ex>20090401): " date2

date_d=$(echo "$date2" | grep "[0-9]\{8\}")
if [ "date_d" == "" ]; then
	echo "Your input date must be (YYYYMMDD)!" && exit 0
fi

echo "date2: $date2"
declare -i date_dem=`date --date="$date2" +%s`
declare -i date_cur=`date +%s`
declare -i date_total=$( ($date_dem-$date_cur) )
declare -i date_day=$( ($date_total/60/60/24) )

if [ "$date_d" -lt "0" ]; then
	declare -i date_d1=$(-1*$date_day)
	echo "dis: $date_d1 days"
else
	echo "dis: $date_day days"
fi





