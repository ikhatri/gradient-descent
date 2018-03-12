CXX = clang++
FLAGS = -std=c++11 -Wall

.DEFAULT = all

all: main

main: gradientDescent.cpp
	$(CXX) $(FLAGS) $^ -o GradientDescent.app

clean:
	rm *.app