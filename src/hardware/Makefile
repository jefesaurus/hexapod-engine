CC := gcc
CFLAGS := -g -Wall -std=gnu99 -gdwarf-3
LDFLAGS := -lrt -lm

COMMON_SRC := serialport.c maestro.c
COMMON_HEADERS := serialport.h maestro.h
TARGET := test_serial

ifeq ($(DEBUG),1)
CFLAGS := -DDEBUG -O0 $(CFLAGS)
else
CFLAGS := -O3 -DNDEBUG $(CFLAGS)
endif
CFLAGS := $(CFLAGS)

test_serial: test_serial.c $(COMMON_SRC) $(COMMON_HEADERS) Makefile
	$(CC) test_serial.c $(COMMON_SRC) $(CFLAGS) $(LDFLAGS) -o test_serial

clean:
	rm -f ./test_serial
