CXX = gcc

targets = DSPUARTcomm

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
serial_dev.o: serial_dev.c
	$(CXX) -c serial_dev.c	

DSPUARTcomm.o: DSPUARTcomm.c
	$(CXX) -c DSPUARTcomm.c	 

DSPUARTcomm: netapi.o serial_dev.o DSPUARTcomm.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 
clean:
	rm -rf *.o $(targets)
  
