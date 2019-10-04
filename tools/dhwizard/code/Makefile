SRCS:=$(shell find . -name "*.java")

OBJS:=${SRCS:.java=.class}

%.class: %.java
	javac -encoding UTF8 $<

all: build

build: ${OBJS}

clean:
	@rm *.class
