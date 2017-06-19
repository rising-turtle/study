#!/usr/bin/python
# 
# try to convert the rgbd dataset's timestamps format into dso dataset 
#

def convert(fin, fout):
    fi = open(fin, "r")
    fo = open(fout, "w")
    fi.readline() # filter out the first line "index timestamp"
    id = 0
    for line in fi:
        # print('line: {}'.format(line))
        items = line.split(' ')
        # print('line[0]: {}'.format(items[0]))
        fo.write(str(id).zfill(5) + ' ' + items[0] + '\n')
        id = id + 1
    fi.close()
    fo.close()

if __name__=='__main__':
    convert("timestamp.txt", "times.txt")
    print('convert (rgbd) timestamp.txt into (dso) times.txt!')
