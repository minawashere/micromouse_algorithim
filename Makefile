CXX ?= g++
CXXFLAGS ?= -std=c++17 -O3 -Wall -Wextra -Iinclude

SRCS_CORE = src/lib/maze_io.cpp
BUILD_DIR = build

all: $(BUILD_DIR)/run $(BUILD_DIR)/mms $(BUILD_DIR)/test_core

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/run: src/apps/run.cpp $(SRCS_CORE) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/mms: src/apps/mms_main.cpp $(SRCS_CORE) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/test_core: tests/test_core.cpp $(SRCS_CORE) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(BUILD_DIR)/test_core
	./$(BUILD_DIR)/test_core

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all test clean
