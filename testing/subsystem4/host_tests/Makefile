# PC tests for subsystem 4 (IMU). No robot needed:  make
SRC_DIR := ../../../app_program/subsystem4
CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -O1 -DAPP_TEST=0 -I$(SRC_DIR)
SRCS    := test_s4.c $(SRC_DIR)/s4_selftest.c $(SRC_DIR)/s4_logic.c

all: run
test_s4: $(SRCS) ; $(CC) $(CFLAGS) $(SRCS) -o $@
run: test_s4
	./test_s4
clean: ; rm -f test_s4
.PHONY: all run clean
