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

hello_world: test_src/hello_world/hello_world.c
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o test_src/hello_world/hello_world test_src/hello_world/hello_world.c

clean:
	-rm -f *.o *.d test_src/hello_world/hello_world/*.o test_src/hello_world/hello_world/*.d
	-rm -f test_src/hello_world/hello_world
