#!/usr/bin/python
#ModuleOne.py文件

import numpy as np

def ArrayListReturn():

    ArrList = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

    Array_A  = np.asarray(ArrList, dtype='float' )
    Array_B  = np.asarray(ArrList, dtype='double')

    return [Array_A, Array_B]

def IntegerListReturn():

    IntegerList = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

    return IntegerList

def FloatListReturn():

    FloatList = [[1.2, 2.3, 3.5], [0.5, 5.2, 6.5], [7.2, 8.8, 9.3]]

    return FloatList

def PassListFromCToPython(List):

    PyList = List
    print (PyList)

def PassArrayFromCToPython(Array):

    print "Shape Of Array:", Array.shape

    print Array
