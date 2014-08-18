# Makefile of sample for a project

#OS=MAC
OS=LINUX

DAEMON=n
DEBUG=y

INC_DIR=$(shell pwd)
SRC_DIR=$(shell pwd)

ifeq ($(OS), MAC)
CFLAGS= -g -Wall -D OS_MAC -I$(INC_DIR)
else
CFLAGS= -g -Wall -D OS_LINUX
endif

ifeq ($(OS), MAC)
LDFLAGS=
else
LDFLAGS=
endif

SRC_FILES=$(wildcard *.c)
OBJS=$(patsubst %.c,%.o, $(SRC_FILES))
TARGET=serialtool

all:$(TARGET)

$(TARGET):$(OBJS)
	$(CC) -o $@ $(LDFLAGS) $?
$(SRC_DIR)/%.o:%.c
	$(CC) $(CFLAGS) -c $^

.PHONY:clean
clean:
	rm -rf $(OBJS) $(TARGET)
