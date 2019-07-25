
CFLAGS += -std=c99 -Wall
CFLAGS += -I.
CFLAGS += -I../../include
CFLAGS += -Izoo/jtbr.endianness

SRCS = test_uuid.c UniversallyUniqueIdentifier.c

OBJS := $(SRCS:.c=.o)

VPATH += ../../common

all: test-uuid

test-uuid: $(OBJS)
	gcc $^ -o $@

%.o: %.c
	gcc -c -o $@ $< $(CFLAGS)

clean:
	rm *.o
	rm test-uuid
