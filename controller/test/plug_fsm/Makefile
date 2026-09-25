CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -g
MAIN    := ../../main

test_plug_fsm: test_plug_fsm.c $(MAIN)/plug_fsm.c $(MAIN)/plug_fsm.h
	$(CC) $(CFLAGS) -I$(MAIN) -o $@ test_plug_fsm.c $(MAIN)/plug_fsm.c

.PHONY: test clean
test: test_plug_fsm
	./test_plug_fsm

clean:
	rm -rf test_plug_fsm test_plug_fsm.dSYM
