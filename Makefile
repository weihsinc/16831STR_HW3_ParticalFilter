CXX=clang++-3.5
CXX_FLAGS=-std=c++11 -O3 # -Wall -Wextra

SRC=src/map.cpp \
    src/data_parser.cpp \
    src/utils.cpp \
    src/particle_filter.cpp

all:
	$(CXX) $(CXX_FLAGS) main.cpp $(SRC) -o particle_filter -I include/
