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

all: lux_sensor

lux_sensor: test_src/lux_sensor/lux_sensor.c
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o test_src/lux_sensor/lux_sensor test_src/lux_sensor/lux_sensor.c

clean:
	-rm -f *.o *.d test_src/lux_sensor/lux_sensor/*.o test_src/lux_sensor/lux_sensor/*.d
	-rm -f test_src/lux_sensor/lux_sensor
