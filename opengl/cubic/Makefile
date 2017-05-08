#
# makefile for cubic - draw a cubic and display it in opengl, and rotate it using SIGALRM 
#

cubic: cubic.o set_ticker.o
	cc -o cubic cubic.o set_ticker.o -lglut -lGLU

clean:
	rm -f *.o
