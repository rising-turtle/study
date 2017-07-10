#!/usr/bin/python
#
# change the timestamps in imu_vn100.log, 
# e.g. 200 Hz means 5ms between two imu measurements
# timestamp follows seconds.nano_seconds 
#

import sys
import numpy as np

class Time:
    NANO_MAX = 1000000000;
    MICO_MAX = 1000000;
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nano_sec = nanosec
    
    @classmethod
    def fromStr(cls, t_str):
        [sec, microsec] = t_str.split('.') # it stores us or ns
        sec = int(sec)
        nanosec = 0
        if len(microsec) <= 6: # us
            nanosec = int(microsec) * 1000
        elif len(microsec) <= 9:
            nanosec = int(microsec)
        else:
            print('what? microsec = {}'.format(microsec))
        return cls(sec, nanosec)
    def tostr(self):
        return (str(self.sec) + '.' + str(self.nano_sec).zfill(9))
    def addSec(self, sec):
        self.sec += sec
    def addNanoSec(self, nanosec):
        self.nano_sec += nanosec
        if self.nano_sec >= Time.NANO_MAX:
            self.sec += 1
            self.nano_sec -= Time.NANO_MAX

def update(fin, fout, dt=5000000): # 5ms = 5000000 ns
    fi = open(fin, 'r')
    fo = open(fout, 'w')

    # start timestamp 
    line = fi.readline()
    items = line.split('\t')
    timestamp = items[0]
    st = Time.fromStr(timestamp)
    items[0] = st.tostr(); 
    fo.write('\t'.join(items));

    for line in fi:
        items = line.split('\t')
        st.addNanoSec(dt)
        items[0] = st.tostr()
        fo.write('\t'.join(items))

    # print('st: {}'.format(st.tostr()))
    # st.addNanoSec(dt)
    # print('now timestamp: {}'.format(st.tostr()))
    
if __name__=='__main__':
    
    input_f = 'imu_vn100.log'
    output_f = 'new_imu_vn100.log'
    total = len(sys.argv)
    if total >= 2: 
        input_f = str(sys.argv[1])
    if total >= 3:
        output_f = str(sys.argv[2])

    update(input_f, output_f)
    print('change {} into {}!'.format(input_f, output_f))
    

