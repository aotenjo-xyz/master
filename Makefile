# Makefile

SRC_FILES := $(wildcard src/*.cpp src/*.h include/*.h lib/**/*.cpp lib/**/*.h)

format:
	clang-format -i $(SRC_FILES)

clean:
	rm -f $(SRC_FILES:.cpp=.o)

.PHONY: format clean