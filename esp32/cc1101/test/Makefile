CXX ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

.PHONY: test clean

test: test_cc1101_math
	./test_cc1101_math

test_cc1101_math: test_cc1101_math.cpp ../cc1101_math.h
	$(CXX) $(CXXFLAGS) test_cc1101_math.cpp -o test_cc1101_math

clean:
	rm -f test_cc1101_math
