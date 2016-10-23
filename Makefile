CXX=clang++-3.5
CXX_FLAGS=-std=c++11 -O3 # -Wall -Wextra

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

all: $(EXECUTABLES)

bin/particle_filter: $(SRC) particle_filter.cpp
	$(CXX) $(CXX_FLAGS) particle_filter.cpp $(SRC) -o $@ $(INCLUDES) $(LIBS)

bin/find_max_range: $(SRC) find_max_range.cpp
	$(CXX) $(CXX_FLAGS) find_max_range.cpp $(SRC) -o $@ $(INCLUDES) $(LIBS)

.PHONY: clean

clean:
	rm bin/*
