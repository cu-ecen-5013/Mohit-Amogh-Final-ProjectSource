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

all: basic_uart

basic_uart: test_src/basic_uart/basic_uart.c
	$(CC) $(CCFLAGS) $(OPTFLAGS) -o test_src/basic_uart/basic_uart test_src/basic_uart/basic_uart.c

clean:
	-rm -f *.o *.d test_src/basic_uart/basic_uart/*.o test_src/basic_uart/basic_uart/*.d
	-rm -f test_src/basic_uart/basic_uart
