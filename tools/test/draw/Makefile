
# This Makefile will handle most aspects of compiling and
# linking a tool against the Tekkotsu framework.  You probably
# won't need to make any modifications, but here's the major controls

# Target model to compile for... if model agnostic, use the default 'dynamic' target
TEKKOTSU_TARGET_MODEL?=TGT_DYNAMIC

# Executable name, defaults to:
#   `basename \`pwd\``
# with a '-$(TEKKOTSU_TARGET_MODEL)' suffix if not DYNAMIC
BIN:=$(shell pwd | sed 's@.*/@@')
ifeq ($(findstring TGT_DYNAMIC,$(TEKKOTSU_TARGET_MODEL)),)
	BIN:=$(BIN)-$(shell echo $(patsubst TGT_%,%,$(TEKKOTSU_TARGET_MODEL)))
endif

# Build directory
PROJECT_BUILDDIR:=build

# Other default values are drawn from the template project's
# Environment.conf file.  This is found using $(TEKKOTSU_ROOT)
# Remove the '?' if you want to override an environment variable
# with a value of your own.
TEKKOTSU_ROOT=../../..

# Source files, defaults to all files ending matching *$(SRCSUFFIX)
SRCSUFFIX:=.cc
PROJ_SRC:=$(shell find . -name "*$(SRCSUFFIX)")
TK_SRC:=$(wildcard $(addsuffix $(SRCSUFFIX), $(addprefix $(TEKKOTSU_ROOT)/, \
	$(addprefix Shared/,fmat BoundingBox Draw) \
)))

.PHONY: all test

TEMPLATE_PROJECT:=$(TEKKOTSU_ROOT)/project
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(TEMPLATE_PROJECT)/Environment.conf
$(if $(shell [ -r $(TEKKOTSU_ENVIRONMENT_CONFIGURATION) ] || echo "failure"),$(error An error has occured, '$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)' could not be found.  You may need to edit TEKKOTSU_ROOT in the Makefile))

TEKKOTSU_TARGET_PLATFORM:=PLATFORM_LOCAL
include $(shell echo "$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)" | sed 's/ /\\ /g')
FILTERSYSWARN:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(FILTERSYSWARN))
COLORFILT:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(COLORFILT))
$(shell mkdir -p $(PROJ_BD))

PROJ_OBJ:=$(patsubst ./%$(SRCSUFFIX),$(PROJ_BD)/%.o,$(PROJ_SRC))
TK_OBJ:=$(patsubst $(TEKKOTSU_ROOT)/%$(SRCSUFFIX),$(PROJ_BD)/%.o,$(TK_SRC))

LIBSUFFIX:=$(suffix $(LIBTEKKOTSU))
LIBS:=
#$(TK_BD)/$(LIBTEKKOTSU) $(TK_BD)/../Shared/newmat/libnewmat$(LIBSUFFIX)

DEPENDS:=$(PROJ_OBJ:.o=.d) $(TK_OBJ:.o=.d)

CXXFLAGS:=-g -Wall -I$(TEKKOTSU_ROOT) $(shell xml2-config --cflags) \
	     $(shell pkg-config --exists cairo && pkg-config --cflags cairo && echo -DHAVE_CAIRO) \
         -D$(TEKKOTSU_TARGET_PLATFORM) -D$(TEKKOTSU_TARGET_MODEL) \

LDFLAGS:=$(LDFLAGS) $(shell xml2-config --libs) \
	    $(shell pkg-config --exists cairo && pkg-config --libs cairo)

all: $(BIN)

$(BIN): $(PROJ_OBJ) $(TK_OBJ) $(LIBS)
	@echo "Linking $@..."
	@$(CXX) $(PROJ_OBJ) $(TK_OBJ) $(LIBS) $(LDFLAGS) -o $@

ifeq ($(findstring clean,$(MAKECMDGOALS)),)
-include $(DEPENDS)
endif

%.a :
	@echo "ERROR: $@ was not found.  You may need to compile the Tekkotsu framework."
	@echo "Press return to attempt to build it, ctl-C to cancel."
	@read;
	$(MAKE) -C $(TEKKOTSU_ROOT) compile

$(TK_OBJ:.o=.d): %.d :
	@mkdir -p $(dir $@)
	@src=$(patsubst %.d,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,$(TEKKOTSU_ROOT)/%,$@)); \
	echo "$@..." | sed 's@.*$(TGT_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > $@

$(PROJ_OBJ:.o=.d): %.d :
	@mkdir -p $(dir $@)
	@src=$(patsubst %.d,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,%,$@)); \
	echo "$@..." | sed 's@.*$(TGT_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > $@

$(TK_OBJ): %.o:
	@mkdir -p $(dir $@)
	@src=$(patsubst %.o,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,$(TEKKOTSU_ROOT)/%,$@)); \
	echo "Compiling $$src..."; \
	$(CXX) $(CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

$(PROJ_OBJ): %.o:
	@mkdir -p $(dir $@)
	@src=$(patsubst %.o,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,%,$@)); \
	echo "Compiling $$src..."; \
	$(CXX) $(CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

clean:
	rm -rf $(BIN) $(PROJECT_BUILDDIR) test-* *~
	rm -rf circGrads.txt log.txt poly-circ.txt poly-point.txt poly-poly.txt poly-rect.txt polyGrads.txt rect-circ.txt rect-point.txt rect-rect.txt rectGrads.txt

test: ./$(BIN)
	./$(BIN)
	@for x in ideal.* ; do \
		y=`echo "$$x" | sed 's/^ideal/test/'` ; \
		if diff -u "$$x" "$$y" ; then \
			echo "'$$y' passed"; \
		else \
			echo "'$$y' does not match ideal"; \
		fi; \
	done
