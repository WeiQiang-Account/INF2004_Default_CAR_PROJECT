# PC tests for subsystem 1 (telemetry JSON + command checks). No robot needed:  make
SRC_DIR := ../../../app_program/subsystem1
CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -O1 -I$(SRC_DIR)
SRCS    := test_s1.c $(SRC_DIR)/s1_selftest.c $(SRC_DIR)/s1_json.c

all: run
test_s1: $(SRCS) ; $(CC) $(CFLAGS) $(SRCS) -o $@
run: test_s1
	./test_s1
clean: ; rm -f test_s1
.PHONY: all run clean
