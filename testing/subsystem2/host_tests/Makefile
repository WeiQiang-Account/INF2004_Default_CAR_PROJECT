# PC tests for subsystem 2 (Motion). No robot needed.
#   make            -> builds and runs the packed and the baseline version
# Uses the real firmware sources from app_program/subsystem2 with -DHOST_TEST.
SRC_DIR := ../../../app_program/subsystem2
CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -O1 -DHOST_TEST -DMOTION_RIGHT_MOTOR_OK=1 -I$(SRC_DIR)
SRCS    := test_motion.c host_mocks.c $(SRC_DIR)/task_motion.c $(SRC_DIR)/drv_encoder.c $(SRC_DIR)/pid.c

all: run
packed:   ; $(CC) $(CFLAGS) -DMOTION_USE_PACKED=1 $(SRCS) -lm -o test_packed
baseline: ; $(CC) $(CFLAGS) -DMOTION_USE_PACKED=0 $(SRCS) -lm -o test_baseline
run: packed baseline
	./test_packed
	./test_baseline
clean: ; rm -f test_packed test_baseline
.PHONY: all packed baseline run clean
