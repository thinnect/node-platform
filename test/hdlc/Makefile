
UNITY_ROOT=../zoo/ThrowTheSwitch.Unity

CFLAGS = -std=c99
CFLAGS += -Wall

SRC_FILES  = $(UNITY_ROOT)/src/unity.c test_hdlc_tools.c
SRC_FILES += test_runners/TestProductionCode_Runner.c
SRC_FILES += ../../common/hdlc_tools.c

INC_DIRS  = -I. -I$(UNITY_ROOT)/src
INC_DIRS += -I../../include

TARGET = test-hdlc.out

all: clean default

default: $(SRC_FILES)
	gcc $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES) -o $(TARGET)
	- ./$(TARGET)

test_runners/TestProductionCode_Runner.c: test_hdlc_tools.c
	mkdir -p test_runners
	ruby $(UNITY_ROOT)/auto/generate_test_runner.rb test_hdlc_tools.c test_runners/TestProductionCode_Runner.c

clean:
	rm -f $(TARGET)
