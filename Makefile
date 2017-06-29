TARGET = example

SRC := src
SRC_URG = $(SRC)/URG
INCLUDE := include
CC = g++ -std=c++11
CFLAGS = -I. -I$(INCLUDE)/LiDAR -I$(SRC_URG) -I$(SRC) -I$(INCLUDE)
LIBS = -pthread -L$(SRC_URG)/src -L$(SRC_URG) -lrt -lserialport -lsbp 
AUX = $(SRC)/*.cpp

all: $(TARGET)

$(TARGET): $(TARGET).cpp $(AUX)
	$(CC) -fpermissive -o $(TARGET) $(TARGET).cpp $(AUX)  $(SRC_URG)/Connection_information.o $(SRC_URG)/src/liburg_cpp.a $(CFLAGS) $(LIBS) 
	
clean:
	$(RM) $(TARGET)
