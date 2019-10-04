SRCS:=$(shell find . -name "*.java")

OBJS:=${SRCS:.java=.class}

JAVAC=javac
CURDIR:=$(shell pwd | sed 's/.*\///')
PROJECT_BUILDDIR?=/tmp
USER?=$(if $(USERNAME),$(USERNAME),$(shell whoami))
BUILDLIST:=$(PROJECT_BUILDDIR)/$(CURDIR)_$(USER)_buildlist.txt

# SEP is to use ';' on windows and ':' on unix because windows is mentally deficient
SEP:=$(shell if [ "`uname`" = "CYGWIN" -o "`uname`" \> "CYGWIN" -a "`uname`" \< "CYGWIO" ] ; then echo ";" ; else echo ":" ; fi )
JAVACFLAGS=-deprecation -classpath ".$(SEP)../mon/ftp.jar"

.PHONY: all clean msg build clearbuildlist test

all: clearbuildlist build

%.class: %.java
	@printf " $<" >> "$(BUILDLIST)";
	@chown $(USER) "$(BUILDLIST)";

clearbuildlist:
	@rm -f "$(BUILDLIST)";

"$(BUILDLIST)": ${OBJS} ;

#	@echo "Build list constructed...";

build: "$(BUILDLIST)"
	@if [ -r "$(BUILDLIST)" ] ; then \
		$(if $(shell which $(JAVAC)), \
			echo "Compiling`cat \"$(BUILDLIST)\"`..."; \
			$(JAVAC) $(JAVACFLAGS) `cat "$(BUILDLIST)"` || exit 1; , \
			printf "  ***** WARNING: You don't have java... skipping EasyTrain *****\n"; \
		) \
		rm -f "$(BUILDLIST)"; \
	else \
		echo "EasyTrain: Nothing to be done."; \
	fi;

clean:
	rm -f "$(BUILDLIST)" $(OBJS) *~ 

test:

