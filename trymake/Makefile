name1 := $(lastword $(MAKEFILE_LIST))
include ../trymake/file1/file1.mk

name2 := $(lastword $(MAKEFILE_LIST))
dir1 :=$(dir $(name2))..
pwd1 := $(PWD)
path := $(pwd1)/$(dir1)
root := $(abspath $(PWD)/$(dir $(name2))..)
subst1 := $(patsubst $(root)/%,%,$(pwd1))

.PHONY : all 
all:
	@echo name1=$(name1)
	@echo name2=$(name2)
	@echo dir1=$(dir1)
	@echo pwd1=$(pwd1)
	@echo path=$(path)
	@echo root=$(root)
	@echo subst=$(subst1)
