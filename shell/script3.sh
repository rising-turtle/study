#!/bin/sh
#world clock program
printf "TimeinBoston: "
TZ=EST5EDT; export TZ; date
printf "TimeinChicago: "
TZ=CST6CDT; date
