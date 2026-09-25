# iqstream host tool. Needs a C++17 compiler, zlib, FFTW3 (single precision)
# and FTDI's D3XX library for Linux (libftd3xx.so and ftd3xx.h), found
# through D3XX_DIR. The web UI in web/ is compiled into the tool.
D3XX_DIR ?= /usr/local/lib/d3xx
CXX ?= c++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++17 -pthread -Wall -Wextra -Werror -I../protocol -isystem $(D3XX_DIR)
LDFLAGS += -pthread -L$(D3XX_DIR) -Wl,-rpath,$(D3XX_DIR)
LDLIBS += -lftd3xx -lz -lfftw3f

SOURCES := $(wildcard src/*.cpp) ../protocol/iq_record.cpp
OBJECTS := $(patsubst %.cpp,build/%.o,$(notdir $(SOURCES))) build/web_assets.o
vpath %.cpp src ../protocol

.PHONY: all clean
all: build/iqstream

build/iqstream: $(OBJECTS)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

# FTDI's header includes the deprecated <cstdbool>.
build/ft600.o: CXXFLAGS += -Wno-cpp

build/%.o: %.cpp $(wildcard src/*.h) ../protocol/control.h ../protocol/iq_record.h | build
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/web_assets.cpp: embed.sh $(wildcard web/*) | build
	sh embed.sh web > $@

build/web_assets.o: build/web_assets.cpp src/web.h
	$(CXX) $(CXXFLAGS) -Isrc -c $< -o $@

build:
	mkdir -p $@

clean:
	rm -rf build
