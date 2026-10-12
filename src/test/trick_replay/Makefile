# Host build of the trick detector replay tool: make -C src/test/trick_replay
ROOT    := ../../..
OUT     := $(ROOT)/obj/trick_replay
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu99

$(OUT): trick_replay.c $(ROOT)/src/main/flight/trick_detector.c $(ROOT)/src/main/flight/trick_detector.h
	mkdir -p $(dir $(OUT))
	$(CC) $(CFLAGS) -I$(ROOT)/src/main -o $@ trick_replay.c $(ROOT)/src/main/flight/trick_detector.c -lm

clean:
	rm -f $(OUT)

.PHONY: clean
