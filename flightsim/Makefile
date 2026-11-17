# Makefile - compiles the simulator
#   make          compile  -> ./flightsim
#   make run      compile and run
#   make clean    remove the compiled files
# Fedora: sudo dnf install gcc-c++ make glfw-devel glew-devel glm-devel mesa-libGL-devel

CXX      ?= g++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++17 -Wall -Wextra -MMD -MP
# the program finds assets/ and shaders/ in this folder, wherever it is run from
CPPFLAGS += -Isrc -DASSET_ROOT="\"$(CURDIR)/\""
LDLIBS   += -lglfw -lGLEW -lGL

TARGET := flightsim
SRC    := $(wildcard src/*.cpp src/*/*.cpp)
OBJ    := $(patsubst src/%.cpp,obj/%.o,$(SRC))
DEP    := $(OBJ:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

obj/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf obj $(TARGET) screenshot_*.ppm

.PHONY: all run clean

-include $(DEP)
