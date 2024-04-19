INCLUDE_DIRS = -I/usr/include/opencv4
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpigpio

src = $(wildcard *.cpp)
obj = $(src:.cpp=.o)

all: build final

build:
	mkdir -p bin

final: $(obj)
	$(CC) $(CFLAGS) -o $@ $^ `pkg-config --libs opencv4` $(LIBS)

.cpp.o: $(src)
	$(CC) $(CFLAGS) -c $<

.PHONY: clean

clean:
	rm -f *.o *.d
	rm -f $(obj) final
	rm -r bin