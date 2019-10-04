SRCS:=$(shell find . -name "*.java")

OBJS:=${SRCS:.java=.class}

CXX=javac
CURDIR:=$(shell pwd | sed 's/.*\///')

# SEP is to use ';' on windows and ':' on unix because windows is mentally deficient
SEP:=$(shell if [ "`uname`" = "CYGWIN" -o "`uname`" \> "CYGWIN" -a "`uname`" \< "CYGWIO" ] ; then echo ";" ; else echo ":" ; fi )
CXXFLAGS=-deprecation -classpath ".$(SEP)../mon/ftp.jar"

.PHONY: all clean msg build clearbuildlist test

all: clearbuildlist build

%.class: %.java
	@printf " $<" >> tmp_buildList.txt;

clearbuildlist:
	@rm -f tmp_buildList.txt;

tmp_buildList.txt: ${OBJS}
#	@echo "Build list constructed...";

build: tmp_buildList.txt
	@if [ -r tmp_buildList.txt ] ; then \
		cat README; \
		$(if $(shell which $(CXX)), \
			echo "Compiling`cat tmp_buildList.txt`..."; $(CXX) $(CXXFLAGS) `cat tmp_buildList.txt`; , \
			printf "  ***** WARNING: You don't have java... skipping TekkotsuMon *****\n"; \
		) \
	fi;

clean:
	rm -f $(OBJS) *~ 

test:

