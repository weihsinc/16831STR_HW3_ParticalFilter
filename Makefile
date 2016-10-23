CXX=clang++-3.5
# CXX=g++
CXX_FLAGS=-std=c++11 # -Wall -Wextra -Werror

SRC=sensor_model.cpp \
    map.cpp \
    sensor_msg.cpp \
    data_parser.cpp \
    utils.cpp \
    particle_filter.cpp

SRC:=$(addprefix src/, $(SRC))

EXECUTABLES=particle_filter
	    # find_max_range

EXECUTABLES:=$(addprefix bin/, $(EXECUTABLES))

INCLUDES=-I include \
	 -I /usr/local/include/opencv2 \
	 -L /usr/local/lib

LIBS=-lopencv_core \
     -lopencv_imgproc \
     -lopencv_highgui

.PHONY: debug all o3 clean

all: $(EXECUTABLES)

o3: CXX_FLAGS+=-O3
o3: all
debug: CXX_FLAGS+=-g -DDEBUG
debug: all

bin/particle_filter: $(SRC) particle_filter.cpp
	$(CXX) $(CXX_FLAGS) particle_filter.cpp $(SRC) -o $@ $(INCLUDES) $(LIBS)

bin/find_max_range: $(SRC) find_max_range.cpp
	$(CXX) $(CXX_FLAGS) find_max_range.cpp $(SRC) -o $@ $(INCLUDES) $(LIBS)

clean:
	rm bin/*
