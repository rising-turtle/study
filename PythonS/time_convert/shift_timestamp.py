#!/usr/bin/python
#
# shift the timestamps in imu_vn100.log, to synchronize imu's timestamp 
# with camera's timestamp 
#
import sys
from x_imu_vn100 import Time 

def shift(fin, fout, delay_s = 0, delay_nano_s = 467116630):
    fi = open(fin, 'r')
    fo = open(fout, 'w')
    
    delay_T = Time(delay_s, delay_nano_s)

    for line in fi:
        items = line.split('\t')
        timestamp = items[0]
        Timestamp = Time.fromStr(timestamp)
        Timestamp.subTime(delay_T)
        items[0] = Timestamp.tostr()
        fo.write('\t'.join(items))
    
if __name__ == '__main__':
    inf = 'imu_vn100.log'
    ouf = 'shift_imu_vn100.log'
    total = len(sys.argv)
    if total >= 2:
        inf = str(sys.argv[1])
    if total >= 3:
        ouf = str(sys.argv[2])
    shift(inf, ouf)




