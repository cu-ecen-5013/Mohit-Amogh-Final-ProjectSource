ifeq ($(CC),)
	CC = $(CROSS_COMPILE)gcc
endif

ifeq ($(CCFLAGS),)
	CFLAGS = -g -Wall -Werror
endif

ifeq ($(LDFLAGS),)
	LDFLAGS = -pthread -lgcc_s -lrt
endif

ifeq ($(OPTFLAGS),)
	OPTFLAGS = -O0
endif

all: cap_sensor

cap_sensor: cap_sensor.o gpio.o
	$(CC) $(CCFLAGS) cap_sensor.o gpio.o -o cap_sensor

cap_sensor.o: test_src/cap_sensor/cap_sensor.c test_src/cap_sensor/gpio.h
	$(CC) $(CCFLAGS) -c test_src/cap_sensor/cap_sensor.c

gpio.o: test_src/cap_sensor/gpio.c test_src/cap_sensor/gpio.h
	$(CC) $(CCFLAGS) -c test_src/cap_sensor/gpio.c

clean:
	-rm -f *.o cap_sensor
