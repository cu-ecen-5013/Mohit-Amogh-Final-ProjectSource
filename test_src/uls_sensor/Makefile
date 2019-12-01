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

all: uls_sensor

uls_sensor: uls_sensor.o gpio.o
	$(CC) $(CCFLAGS) uls_sensor.o gpio.o -o uls_sensor

uls_sensor.o: test_src/uls_sensor/uls_sensor.c test_src/uls_sensor/gpio.h
	$(CC) $(CCFLAGS) -c test_src/uls_sensor/uls_sensor.c

gpio.o: test_src/uls_sensor/gpio.c test_src/uls_sensor/gpio.h
	$(CC) $(CCFLAGS) -c test_src/uls_sensor/gpio.c

clean:
	-rm -f *.o uls_sensor
