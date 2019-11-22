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

all: hello_world

hello_world: hello_world.c
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o hello_world hello_world.c

clean:
	-rm -f *.o *.d hello_world/*.o hello_world/*.d
	-rm -f hello_world