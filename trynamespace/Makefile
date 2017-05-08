SRC = $(wildcard *.cpp)
OBJ = $(patsubst %.cpp, %.o, $(wildcard *.cpp))
.PHONY : all
all : $(SRC)
	g++ $^ -o $@
#$(OBJ) : %.o : %.cpp
#	g++ -c $< -o $@
