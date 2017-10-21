#!/usr/bin/python
# Filename:  add.py
def Hello(s):
    print "Hello World"
    print s

def PassListFromCToPython(List1, List2):

    PyList = List1
    print (PyList)
    print (List2)
    return 1.5

def Add(a, b):
    print 'a=', a
    print 'b=', b
    return a + b

class Test:
    def __init__(self):
        print "Init"
    def SayHello(self, name):
        print "Hello,", name
        return name
