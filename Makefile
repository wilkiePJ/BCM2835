#the compilerr: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS = -g -Wall -std=c++0x 

INCLUDES = -I/home/pi/BCM2835

SRCS = BCM2835.cpp TestBed.cpp

# the build target executable:
TARGET = BCM2835

all: $(TARGET)
	
$(TARGET):$(TARGET).cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS)
	
clean:
	$(RM) $(TARGET)
