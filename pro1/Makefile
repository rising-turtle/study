MD := mkdir -p
DIR := ./sub
SUB := $(MD) $(DIR)

src:=$(wildcard *.c ./sub/*.c)
dir:=$(notdir $(src))
obj:=$(patsubst %.c, %.o, $(dir))

all: $(src)
	@echo $(src)
