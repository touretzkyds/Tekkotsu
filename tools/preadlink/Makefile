.PHONY: all clean test

all: preadlink

preadlink: preadlink.cc
	$(CXX) -o preadlink -Wall -O2 preadlink.cc
	@ln -fs ../$@/$@ ../bin

clean:
	rm -f preadlink

test:

