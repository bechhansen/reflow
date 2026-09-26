CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -g
MAIN    := ../../main
SRCS    := $(MAIN)/reflow_algo.c $(MAIN)/reflow_curve.c
HDRS    := $(MAIN)/reflow_algo.h $(MAIN)/reflow_curve.h

test_reflow_algo: test_reflow_algo.c $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -I$(MAIN) -o $@ test_reflow_algo.c $(SRCS) -lm

.PHONY: test clean
test: test_reflow_algo
	./test_reflow_algo

clean:
	rm -rf test_reflow_algo test_reflow_algo.dSYM
