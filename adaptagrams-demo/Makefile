CXX = g++
# Add third_party headers so includes like "libavoid/libavoid.h" are found
CXXFLAGS = -Iinclude -std=c++17 -I../cola -I.. -Ithird_party/include -O2
# Link against the static libraries built under cola/.libs
LIB_DIR = ../cola
LDFLAGS = -Wl,--start-group \
         $(LIB_DIR)/libavoid/.libs/libavoid.a \
         $(LIB_DIR)/libcola/.libs/libcola.a \
         $(LIB_DIR)/libtopology/.libs/libtopology.a \
         $(LIB_DIR)/libvpsc/.libs/libvpsc.a \
         $(LIB_DIR)/libdialect/.libs/libdialect.a \
         -Wl,--end-group -pthread

SRC = adaptagrams_demo.cpp
OBJ = $(SRC:.cpp=.o)
TARGET = adaptagrams_demo

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)