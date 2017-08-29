#!/usr/bin/python
#
# shift a group of timestamps, with about 10ms as step
# based on imu_vn100.log, in order to synchronize imu's timestamp 
# with camera's timestamp 
#
import sys
from x_imu_vn100 import Time 
from shift_timestamp import shift

def shift_group(fin, folder, delay_s, delay_nano_s, step_ms = 10):
   
    delay_T = Time(delay_s, delay_nano_s)
    # print('delay_T: sec = {} nsec = {}'.format(delay_T.sec, delay_T.nano_sec))

    # '-'
    for i in range(10):
        step_T = Time(0, step_ms*1000000)
        delay_T.subTime(step_T)
        ouf = folder + '/imu_vn100_' + str(i) + '.log'
        print('{} -> {} s: {} ns: {}'.format(fin, ouf, delay_T.sec, delay_T.nano_sec))
        shift(fin, ouf, delay_T.sec, delay_T.nano_sec)

    delay_T = Time(delay_s, delay_nano_s)
    # print('delay_T: sec = {} nsec = {}'.format(delay_T.sec, delay_T.nano_sec))

    # '-'
    for i in range(10):
        step_T = Time(0, step_ms*1000000)
        delay_T.addTime(step_T)
        ouf = folder + '/imu_vn100_' + str(i+10) + '.log'
        print('{} -> {} s: {} ns: {}'.format(fin, ouf, delay_T.sec, delay_T.nano_sec))
        shift(fin, ouf, delay_T.sec, delay_T.nano_sec)
  
   
if __name__ == '__main__':
    inf = 'imu_vn100.log'
    folder = 'shift_group'
    total = len(sys.argv)
    delay_nano_s = 467116630
    delay_s = 0
    step_ms  = 10 
    if total >= 2:
        inf = str(sys.argv[1])
    # if total >= 3:
    #    ouf = str(sys.argv[2])
    if total >= 3:
        delay_s = int(sys.argv[2])
    if total >= 4:
        delay_nano_s = int(sys.argv[3])  
    if total >= 5:
        step_ms = int(sys.argv[4])
    print('delay_s = {} delay_nano_s = {}'.format(delay_s, delay_nano_s))
    print('step_ms = {}'.format(step_ms))
    shift_group(inf, folder, delay_s, delay_nano_s, step_ms)




