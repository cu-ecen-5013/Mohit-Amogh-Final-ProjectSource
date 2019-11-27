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

all: led_blink

led_blink: led_blink.o gpio.o
	$(CC) $(CCFLAGS) led_blink.o gpio.o -o led_blink

led_blink.o: test_src/led_blink/led_blink.c test_src/led_blink/gpio.h
	$(CC) $(CCFLAGS) -c test_src/led_blink/led_blink.c

gpio.o: test_src/led_blink/gpio.c test_src/led_blink/gpio.h
	$(CC) $(CCFLAGS) -c test_src/led_blink/gpio.c

clean:
	-rm -f *.o led_blink
