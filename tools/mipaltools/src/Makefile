all: EmonLog.o  Function.o  FunctionStore.o  StackAnalyser.o  StackDump.o  main.o
	$(CXX) -o ../StackedIt EmonLog.o  Function.o  FunctionStore.o  StackAnalyser.o  StackDump.o  main.o 

Function.o: Function.cc Function.h
	$(CXX) -c -O3  Function.cc

FunctionStore.o: FunctionStore.cc FunctionStore.h Function.h 
	$(CXX) -c -O3  FunctionStore.cc

StackAnalyser.o: StackAnalyser.cc StackAnalyser.h StackDump.h FunctionStore.h Function.h Trace.h
	$(CXX) -c -O3  StackAnalyser.cc

StackDump.o: StackDump.cc StackDump.h 
	$(CXX) -c -O3  StackDump.cc

main.o: main.cc StackDump.h FunctionStore.h Function.h StackAnalyser.h
	$(CXX) -c -O3  main.cc

EmonLog.o:	EmonLog.h EmonLog.cc
	$(CXX) -c -O3  EmonLog.cc

clean:
	rm -f *.o ../StackedIt ../StackedIt.exe
