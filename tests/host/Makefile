CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I../../include

.PHONY: test clean

test: test_motions
	./test_motions

test_motions: test_motions.cpp ../../include/ez_compat/pid.hpp ../../include/ez_compat/motion_math.hpp
	$(CXX) $(CXXFLAGS) test_motions.cpp -o test_motions

clean:
	rm -f test_motions
