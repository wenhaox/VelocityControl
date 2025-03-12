CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -Ijsoncpp/include
LDFLAGS = -Ljsoncpp/build/lib -ljsoncpp

TARGET = zynq_test
SRCS = zynq_test.cpp jsoncpp/src/lib_json/json_reader.cpp jsoncpp/src/lib_json/json_value.cpp jsoncpp/src/lib_json/json_writer.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)
