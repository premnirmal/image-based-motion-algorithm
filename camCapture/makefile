# Makefile for camCapture.c
# Sept. 23, 2009

# include directories for any compile

CFLAGS = -I/usr/include/opencv/

# Link phase of compile; -L denotes the libraries, -lcv and -lcvaux specify
# Files libcv.so and libcvaux.so

#LDFLAGS = -L/usr/local/lib/ -lopencv_core -lopencv_highgui

LDLIBS = `pkg-config --libs opencv`