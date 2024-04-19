INCLUDE_DIRS = -I/usr/include/opencv4
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpigpio

# SRC_NAME= capture sensor

HFILES= 
CFILES= capture.cpp sensor.cpp

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.cpp=.o}
# OBJS= bin/${SRC_NAME}.o
# TRGT= bin/${SRC_NAME}

all: capture sensor

clean:
	-rm -f *.o *.d
	-rm -f capture sensor

# build:
# 	mkdir -p bin

# ${SRC_NAME}: ${SRC_NAME}.o
# 	$(CC)

# ${SRC_NAME}.o: build
# 	$(CC) $(LIBS) ${CFLAGS} -c ${SRCS} -o ${OBJS} `pkg-config --libs opencv4`

capture: capture.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o `pkg-config --libs opencv4` $(LIBS)

sensor: sensor.o
	$(CC) $(CFLAGS) sensor.o -o sensor -lpigpio -lrt

sensor.o: 
	$(CC) $(CFLAGS) -c sensor.cpp -o sensor.o -lpigpio -lrt

depend:

.cpp.o: $(SRCS)
	$(CC) $(CFLAGS) -c $<
