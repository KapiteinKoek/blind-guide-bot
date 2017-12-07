BINARIES = blindguide

CC = gcc
CFLAGS = -Wall -g -c
LDLIBS = -lm

all:	$(BINARIES)

clean:
	rm -f *.o $(BINARIES)

blindguide: blindguide.o

blindguide.o: blindguide.c blindguide.h

