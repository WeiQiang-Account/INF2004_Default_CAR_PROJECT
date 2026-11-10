# PC tests for subsystem 3 (line + barcode). No robot needed:  make
SRC_DIR := ../../../app_program/subsystem3
CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -O1 -DS3_HOST_TEST -I$(SRC_DIR)
SRCS    := host_test.c $(SRC_DIR)/s3_selftest.c $(SRC_DIR)/s3_decode.c $(SRC_DIR)/s3_line.c $(SRC_DIR)/s3_barcode.c

all: run
host_test: $(SRCS) ; $(CC) $(CFLAGS) $(SRCS) -o $@
run: host_test
	./host_test
clean: ; rm -f host_test
.PHONY: all run clean
