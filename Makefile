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

INCPATH = test_src/led_blink/
SRCPATH = test_src/led_blink/

all: led_blink

led_blink: led_blink.o gpio.o
	$(CC) $(CCFLAGS) led_blink.o gpio.o -o led_blink

led_blink.o: $(SRCPATH)led_blink.c $(INCPATH)gpio.h
	$(CC) $(CCFLAGS) -c $(SRCPATH)led_blink.c

gpio.o: $(SRCPATH)gpio.c $(INCPATH)gpio.h
	$(CC) $(CCFLAGS) -c $(SRCPATH)gpio.c

clean:
	-rm -f *.o led_blink
