CC := gcc
CFLAGS := -g -Wall -std=gnu99 -gdwarf-3
LDFLAGS := -lrt -lm

COMMON_SRC := ik_3dof.c gen/basic_leg_transforms.c
COMMON_HEADERS := ik_3dof.h gen/basic_leg_transforms.h
TARGET := test_ik

ifeq ($(DEBUG),1)
CFLAGS := -DDEBUG -O0 $(CFLAGS)
else
CFLAGS := -O3 -DNDEBUG $(CFLAGS)
endif
CFLAGS := $(CFLAGS)

test_ik: ik_3dof.c $(COMMON_SRC) $(COMMON_HEADERS) Makefile
	$(CC) $(COMMON_SRC) $(CFLAGS) $(LDFLAGS) -o test_ik

clean:
	rm -f ./test_ik
