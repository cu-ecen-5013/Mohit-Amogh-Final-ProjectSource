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

ifeq ($(SRC),)
	SRC := main.c
endif

ifeq ($(OBJ),)
	OBJ := $(SRC:.c=.o)
endif

vpath %.c src/
vpath %.h inc/

bbb_main: $(OBJ)
	$(CC) -o bbb_main $(OBJ) $(CCFLAGS) $(OPTFLAGS) $(LDFLAGS)

clean:
	-rm -f *.o bbb_main