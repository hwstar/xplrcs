# Makefile

PACKAGE = rc65
VERSION = 0.0.1
CONTACT = <hwstar@rodgers.sdcoxmail.com>

CC = gcc
CFLAGS = -O2 -Wall  -D'PACKAGE="$(PACKAGE)"' -D'VERSION="$(VERSION)"' -D'EMAIL="$(CONTACT)"'
#CFLAGS = -g3 -Wall  -D'PACKAGE="$(PACKAGE)"' -D'VERSION="$(VERSION)"' -D'EMAIL="$(CONTACT)"'

# Install paths for built executables

DAEMONDIR = /usr/local/bin

#Libraries

# Object file lists

OBJS = rc65.o serio.o notify.o

#Dependencies

all: rc65 

rc65.o: Makefile rc65.c notify.h serio.h

#Rules

rc65: $(OBJS)
	$(CC) $(CFLAGS) -o rc65 $(OBJS) -lxPL

clean:
	-rm -f rc65 *.o core

install:
	cp rc65 $(DAEMONDIR)

