#!/usr/bin/python
def filerw(f):
    fr = open(f, "w")
    fr.write("I love Python! \n Hello, World!\n")
    fr.close()
    fw = open(f, "r")
    text = fw.readline()
    print(text)

if __name__=='__main__':
    filerw("1.txt")
