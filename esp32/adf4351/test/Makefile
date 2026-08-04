CXX ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

.PHONY: test clean

test: test_adf4351_math
	./test_adf4351_math

test_adf4351_math: test_adf4351_math.cpp ../adf4351_math.h
	$(CXX) $(CXXFLAGS) test_adf4351_math.cpp -o test_adf4351_math

clean:
	rm -f test_adf4351_math
