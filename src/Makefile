CXX ?= g++
PKG_CONFIG ?= pkg-config

TARGET := robot_core
SOURCES := \
	robot_core.cpp \
	source/adafruitmotorhat.cpp \
	source/adafruitdcmotor.cpp \
	source/pwm.cpp \
	source/i2cdevice.cpp

CPPFLAGS += $(shell $(PKG_CONFIG) --cflags libwebsockets)
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra
LDLIBS += $(shell $(PKG_CONFIG) --libs libwebsockets)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -pthread $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

clean:
	$(RM) $(TARGET)
