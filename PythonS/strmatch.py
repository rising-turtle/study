#!coding=utf-8
import re
def StrMatch(filename,str):
    f=open(filename,"r")
    file_content=f.read()
    print file_content
    l=file_content.split('£¬')
    print l
    