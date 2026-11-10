CXX ?= g++
PKG_CONFIG ?= pkg-config

TARGET := robot_core
SOURCES := \
	main.cpp \
	gnss_thread.cpp \
	motor_control_thread.cpp \
	system_stats_thread.cpp \
	web_server_thread.cpp \
	lib/adafruitmotorhat.cpp \
	lib/adafruitdcmotor.cpp \
	lib/pwm.cpp \
	lib/i2cdevice.cpp

HEADERS := robot_state.h thread_functions.h

CPPFLAGS += $(shell $(PKG_CONFIG) --cflags libwebsockets)
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra
LDLIBS += $(shell $(PKG_CONFIG) --libs libwebsockets)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -pthread $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

clean:
	$(RM) $(TARGET)
