ifeq ($(CC),)
	CC = $(CROSS_COMPILE)gcc
endif

ifeq ($(CCFLAGS),)
	CFLAGS = -g -Wall -Werror
endif

ifeq ($(LDFLAGS),)
	LDFLAGS = -pthread -lgcc_s -lrt -lm
endif

ifeq ($(OPTFLAGS),)
	OPTFLAGS = -O0
endif

all: bbb_main

bbb_main: main.o gpio.o
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o bbb_main main.o gpio.o $(LDFLAGS)

main.o: src/main.c inc/main.h inc/gpio.h
	$(CC) $(CCFLAGS) $(OPTFLAGS) -c src/main.c $(LDFLAGS)

gpio.o: src/gpio.c inc/gpio.h
	$(CC) $(CCFLAGS) -c src/gpio.c

# bbb_main: src/main.c inc/main.h
# 	$(CC) $(CCFLAGS) $(OPTFLAGS) -o bbb_main src/main.c $(LDFLAGS)

clean:
	-rm -f *.o bbb_main