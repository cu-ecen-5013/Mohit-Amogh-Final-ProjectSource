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

bbb_main: src/main.c inc/main.h
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o bbb_main src/main.c $(LDFLAGS)

clean:
	-rm -f *.o bbb_main