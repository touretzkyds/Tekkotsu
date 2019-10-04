
# This Makefile will handle most aspects of compiling and
# linking a tool against the Tekkotsu framework.  You probably
# won't need to make any modifications, but here's the major controls

# Target model to compile for...
# If model agnostic, use the default 'dynamic' target and add files
#   to the TK_SRC list (LIBTEKKOTSU is unavailable for 'dynamic')
# If model dependent, set the model, and you may want to uncomment LIBS
#   below to use LIBTEKKOTSU instead of managing the TK_SRC list
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
TEKKOTSU_ROOT:=../../..

# Source files, defaults to all files ending matching *$(SRCSUFFIX)
SRCSUFFIX:=.cc
PROJ_SRC:=$(shell find . -name "*$(SRCSUFFIX)")
TK_SRC:=$(addsuffix $(SRCSUFFIX), $(addprefix $(TEKKOTSU_ROOT)/, \
	Shared/Resource Shared/TimeET Shared/StackTrace IPC/Thread IPC/ProcessID IPC/MutexLock \
))

.PHONY: all test

TEMPLATE_PROJECT:=$(TEKKOTSU_ROOT)/project
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(TEMPLATE_PROJECT)/Environment.conf
$(if $(shell [ -r $(TEKKOTSU_ENVIRONMENT_CONFIGURATION) ] || echo "failure"),$(error An error has occured, '$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)' could not be found.  You may need to edit TEKKOTSU_ROOT in the Makefile))

TEKKOTSU_TARGET_PLATFORM:=
include $(shell echo "$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)" | sed 's/ /\\ /g')
FILTERSYSWARN:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(FILTERSYSWARN))
COLORFILT:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(COLORFILT))
$(shell mkdir -p $(PROJ_BD))

PROJ_OBJ:=$(patsubst ./%$(SRCSUFFIX),$(PROJ_BD)/%.o,$(PROJ_SRC))
TK_OBJ:=$(patsubst $(TEKKOTSU_ROOT)/%$(SRCSUFFIX),$(PROJ_BD)/%.o,$(TK_SRC))


LIBSUFFIX:=$(suffix $(LIBTEKKOTSU))
#LIBS:= $(TK_BD)/$(LIBTEKKOTSU) $(TK_LIB_BD)/Shared/newmat/libnewmat$(LIBSUFFIX)

DEPENDS:=$(PROJ_OBJ:.o=.d) $(TK_OBJ:.o=.d)

CXXFLAGS:=-g -Wall -O2 \
         -I$(TEKKOTSU_ROOT) \
         -I$(TEKKOTSU_ROOT)/Shared/jpeg-6b `xml2-config --cflags` \
         -D$(TEKKOTSU_TARGET_PLATFORM) -D$(TEKKOTSU_TARGET_MODEL) 

LDFLAGS:=$(LDFLAGS) $(shell xml2-config --libs) -lpng -ljpeg \
		$(if $(ISMACOSX),,-lrt) \
		$(if $(ISMACOSX), $(shell if [ $(TEST_MACOS_MAJOR) -gt 10 -o $(TEST_MACOS_MAJOR) -eq 10 -a $(TEST_MACOS_MINOR) -ge 6 ] ; \
		then echo -framework QTKit -framework CoreVideo -framework Cocoa; \
		else echo -framework Quicktime -framework Carbon; fi))

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

test: ./$(BIN)
	./$(BIN) | sed 's/@VAR.*/@VAR/' > test-output.txt
	@for x in * ; do \
		if [ -r "test-$$x" ] ; then \
			if diff -u "$$x" "test-$$x" ; then \
				echo "Test '$$x' passed"; \
			else \
				echo "Test output '$$x' does not match ideal"; \
			fi; \
		fi; \
	done