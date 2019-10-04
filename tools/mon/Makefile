SRCS:=$(shell find . -name "*.java")

OBJS:=${SRCS:.java=.class}

JAVAC?=javac
CURDIR:=$(shell pwd | sed 's/.*\///')
PROJECT_BUILDDIR?=/tmp
USER?=$(if $(USERNAME),$(USERNAME),$(shell whoami))
BUILDLIST:=$(PROJECT_BUILDDIR)/$(CURDIR)_$(USER)_buildlist.txt

# SEP is to use ';' on windows and ':' on unix because windows is mentally deficient
SEP:=$(shell if [ "`uname`" = "CYGWIN" -o "`uname`" \> "CYGWIN" -a "`uname`" \< "CYGWIO" ] ; then echo ";" ; else echo ":" ; fi )
J3D:="java3d/j3dcore.jar$(SEP)java3d/j3dutils.jar$(SEP)java3d/vecmath.jar"
JAVAFLAGSnodep:=$(JAVAFLAGS) -classpath ".$(SEP)/usr/share/java/jutils.jar$(SEP)/usr/share/java/jinput.jar$(SEP)./jargs.jar$(SEP)$(J3D)" 
JAVAFLAGS:=$(JAVAFLAGS) -deprecation -classpath ".$(SEP)/usr/share/java/jutils.jar$(SEP)/usr/share/java/jinput.jar$(SEP)./jargs.jar$(SEP)$(J3D)" 

AIBO3DCACHE:=.aibo3dcache
ifneq ($(MAKECMDGOALS),clean)
AIBO3D:=$(shell if [ ! -r "$(AIBO3DCACHE)" ] ; then $(JAVAC) $(JAVAFLAGSnodep) org/tekkotsu/aibo3d/Java3DTest.java >/dev/null 2>&1; echo $$? > "$(AIBO3DCACHE)"; fi; cat "$(AIBO3DCACHE)" )
endif

.PHONY: all clean build clearbuildlist test

all: clearbuildlist build

org/tekkotsu/aibo3d/%.class: org/tekkotsu/aibo3d/%.java
	@if [ $(AIBO3D) -eq 0 ] ; then \
		echo Compiling "$<"; \
		$(JAVAC) $(JAVAFLAGSnodep) "$<" >/dev/null 2>&1; \
	fi;

%.class: %.java
	@printf " $<" >> "$(BUILDLIST)";
	@chown $(USER) "$(BUILDLIST)";

clearbuildlist:
	@rm -f "$(BUILDLIST)";

"$(BUILDLIST)": ${OBJS} ;

#	@echo "Build list constructed...";

build: "$(BUILDLIST)"
	@if [ -r "$(BUILDLIST)" ] ; then \
		echo " **     These tools require the JDK 1.5/5.0 or higher.                  **"; \
		if [ $(AIBO3D) -ne 0 ] ; then \
			echo " *************************************************************************"; \
			echo " **     Java3D package not found.  Can't compile SketchGUI.             **"; \
			echo " **     More information: http://java.sun.com/products/java-media/3D/   **"; \
			echo " **     (Status cached for faster recompiles - make clean to clear)     **"; \
		else \
			echo " **     (Using cached Java3D status - make clean to clear cache)        **"; \
		fi;	\
		$(if $(shell which $(JAVAC)), \
			if [ `cat "$(BUILDLIST)" | wc -c` -gt 400 ] ; then \
				list=' [...]'; \
			else \
				list="`cat \"$(BUILDLIST)\"`"; \
			fi; \
			echo "Compiling$$list ..."; $(JAVAC) $(JAVAFLAGS) `cat "$(BUILDLIST)"` || exit 1; , \
			printf "  ***** WARNING: You don't have Java SDK... skipping TekkotsuMon *****\n"; \
		) \
		rm -f "$(BUILDLIST)"; \
		ln -fs ../mon/ControllerGUI ../bin; \
	else \
		echo "TekkotsuMon: Nothing to be done."; \
	fi;

clean:
	rm -f "$(BUILDLIST)" "$(AIBO3DCACHE)"
	find . -name "*.class" -exec rm \{\} \;
	find . -name "*~" -exec rm \{\} \;

test:

