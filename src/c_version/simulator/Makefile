WRKDIR = `pwd`

CC = gcc
CPP = g++
LD = g++
AR = ar

INC = 
CFLAGS = -Wall -O2
RESINC = 
RCFLAGS = 
LIBDIR = 
LIB = -lglut -lGLU -lGL -lm
LDFLAGS =

INC_RELEASE = $(INC)
CFLAGS_RELEASE = $(CFLAGS)
RESINC_RELEASE = $(RESINC)
RCFLAGS_RELEASE = $(RCFLAGS)
LIBDIR_RELEASE = $(LIBDIR)
LIB_RELEASE = $(LIB)
LDFLAGS_RELEASE = $(LDFLAGS) -s
OBJDIR_RELEASE = objs
DEP_RELEASE = 
OUTDIR_RELEASE = bin
OUT_RELEASE = simulator

OBJ_RELEASE = $(OBJDIR_RELEASE)/main.o $(OBJDIR_RELEASE)/Timer.o

all: release

clean: clean_release

release: $(OUT_RELEASE)

$(OUT_RELEASE): $(OBJ_RELEASE) $(DEP_RELEASE)
	test -d $(OUTDIR_RELEASE) || mkdir -p $(OUTDIR_RELEASE)
	$(LD) $(LDFLAGS_RELEASE) $(LIBDIR_RELEASE) -o $(OUTDIR_RELEASE)/$(OUT_RELEASE) $(OBJ_RELEASE) $(LIB_RELEASE)

$(OBJDIR_RELEASE)/main.o: main.cpp
	test -d $(OBJDIR_RELEASE) || mkdir -p $(OBJDIR_RELEASE)
	$(CPP) $(CFLAGS_RELEASE) $(INC_RELEASE) -c -o $(OBJDIR_RELEASE)/main.o main.cpp

$(OBJDIR_RELEASE)/Timer.o: Timer.cpp
	test -d $(OBJDIR_RELEASE) || mkdir -p $(OBJDIR_RELEASE)
	$(CPP) $(CFLAGS_RELEASE) $(INC_RELEASE) -c -o $(OBJDIR_RELEASE)/Timer.o Timer.cpp

clean_release:
	rm -rf $(OBJDIR_RELEASE) $(OUTDIR_RELEASE)

.PHONY: clean clean_release

