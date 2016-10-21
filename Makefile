SRC=src/map.cpp \
    src/data_parser.cpp \
    src/utils.cpp \
    src/particle_filter.cpp

all:
	g++ -std=c++11 main.cpp $(SRC) -o out -I include/
