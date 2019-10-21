
UNITY_ROOT=../zoo/ThrowTheSwitch.Unity

CFLAGS = -std=c99
CFLAGS += -Wall

SRC_FILES  = $(UNITY_ROOT)/src/unity.c

SRC_FILES += test_serial_protocol.c
SRC_FILES += cmsis_os2_mock.c
SRC_FILES += test_runners/TestProductionCode_Runner.c

SRC_FILES += ../../common/serial_protocol.c

SRC_FILES += ../zoo/thinnect.lll/logging/loggers_std.c

INC_DIRS  = -I. -I$(UNITY_ROOT)/src
INC_DIRS += -I../../include
INC_DIRS += -I../zoo/thinnect.lll/logging

TARGET = test-serial-protocol.out

all: clean default

default: $(SRC_FILES)
	gcc $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES) -o $(TARGET)
	- ./$(TARGET)

test_runners/TestProductionCode_Runner.c: test_serial_protocol.c
	mkdir -p test_runners
	ruby $(UNITY_ROOT)/auto/generate_test_runner.rb test_serial_protocol.c test_runners/TestProductionCode_Runner.c

clean:
	rm -f $(TARGET)
