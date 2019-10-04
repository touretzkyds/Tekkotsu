
# This Makefile will handle most aspects of compiling and
# linking a tool against the Tekkotsu framework.  You probably
# won't need to make any modifications, but here's the major controls

# Executable name, defaults to:
#   `basename \`pwd\``
BIN:=$(shell pwd | sed 's@.*/@@')

# Build directory
PROJECT_BUILDDIR:=build

# Other default values are drawn from the template project's
# Environment.conf file.  This is found using $(TEKKOTSU_ROOT)
TEKKOTSU_ROOT:=../..

# Source files, defaults to all files ending matching *$(SRCSUFFIX)
SRCSUFFIX:=.cc
PROJ_SRC:=$(shell find . -name "*$(SRCSUFFIX)")

TK_SRC:=$(wildcard $(TEKKOTSU_ROOT)/Shared/plist*$(SRCSUFFIX)) \
	$(addsuffix $(SRCSUFFIX), $(addprefix $(TEKKOTSU_ROOT)/, \
		Shared/LoadSave Shared/XMLLoadSave Shared/string_util Shared/fmat Shared/BoundingBox \
		Shared/DynamicInfo Shared/RobotInfo Motion/KinematicJoint Motion/SensorInfo \
		Planners/PlannerObstacles \
		Wireless/netstream \
	))

.PHONY: all test

TEMPLATE_PROJECT:=$(TEKKOTSU_ROOT)/project
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(TEMPLATE_PROJECT)/Environment.conf
$(if $(shell [ -r $(TEKKOTSU_ENVIRONMENT_CONFIGURATION) ] || echo "failure"),$(error An error has occured, '$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)' could not be found.  You may need to edit TEKKOTSU_ROOT in the Makefile))

TEKKOTSU_TARGET_MODEL=TGT_DYNAMIC
TEKKOTSU_TARGET_PLATFORM:=PLATFORM_LOCAL
include $(shell echo "$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)" | sed 's/ /\\ /g')
FILTERSYSWARN:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(FILTERSYSWARN))
COLORFILT:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(COLORFILT))
$(shell mkdir -p $(PROJ_BD))

PROJ_OBJ:=$(patsubst %$(SRCSUFFIX),$(PROJ_BD)/%.o,$(PROJ_SRC)) $(patsubst $(TEKKOTSU_ROOT)/%$(SRCSUFFIX),$(PROJ_BD)/%.o,$(TK_SRC))

LIBSUFFIX:=$(suffix $(LIBTEKKOTSU))
LIBS:= 
#$(TK_BD)/$(LIBTEKKOTSU) $(TEKKOTSU_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM)/Shared/newmat/libnewmat$(LIBSUFFIX)

DEPENDS:=$(PROJ_OBJ:.o=.d)

CXXFLAGS:=-g -Wall -fmessage-length=0 \
         -I$(TEKKOTSU_ROOT) $(shell xml2-config --cflags) \
         -D$(TEKKOTSU_TARGET_PLATFORM) -D$(TEKKOTSU_TARGET_MODEL) -DFMAT_DEFAULT_DOUBLE

LDFLAGS:=$(LDFLAGS) `xml2-config --libs` $(if $(shell locate librt.a 2> /dev/null),-lrt)

all: $(BIN)

$(BIN): $(PROJ_OBJ) $(LIBS)
	@echo "Linking $@..."
	@$(CXX) $(PROJ_OBJ) $(LIBS) $(LDFLAGS) -o $@
	@ln -fs ../$@/$@ ../bin

ifeq ($(findstring clean,$(MAKECMDGOALS)),)
-include $(DEPENDS)
endif

%.a :
	@echo "ERROR: $@ was not found.  You may need to compile the Tekkotsu framework."
	@echo "Press return to attempt to build it, ctl-C to cancel."
	@read;
	$(MAKE) -C $(TEKKOTSU_ROOT) compile

%.d :
	@mkdir -p $(dir $@)
	@src=$(patsubst %.d,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,$(TEKKOTSU_ROOT)/%,$(patsubst $(PROJ_BD)/./%,%,$@))); \
	echo "$@..." | sed 's@.*$(TGT_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > $@

%.o:
	@mkdir -p $(dir $@)
	@src=$(patsubst %.o,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,$(TEKKOTSU_ROOT)/%,$(patsubst $(PROJ_BD)/./%,%,$@))); \
	echo "Compiling $$src..."; \
	$(CXX) $(CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

clean:
	rm -rf $(BIN) $(PROJECT_BUILDDIR) test-* *~

test: ./$(BIN)
	@for x in * ; do \
		if [ -r "ideal-$$x" ] ; then \
			if diff -u "ideal-$$x" "$$x" ; then \
				echo "Test '$$x' passed"; \
			else \
				echo "Test output '$$x' does not match ideal"; \
			fi; \
		fi; \
	done
