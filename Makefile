############### FRAMEWORK MAKEFILE ################

# Make sure the default target is 'all' by listing it first
all:


###################################################
##             ENVIRONMENT SETUP                 ##
###################################################
# Use the default project Environment.conf if none has been
# specified.  If a project initiated make, it should have
# defined this variable for us to use its configuration.
TEKKOTSU_ROOT:=$(shell pwd | sed 's/ /\\ /g')
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(TEKKOTSU_ROOT)/project/Environment.conf
include $(shell echo "$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)" | sed 's/ /\\ /g')

ifeq ($(XCODE_VERSION_ACTUAL),)
ifeq ($(MAKELEVEL),0)
INDENT=#empty string just to tokenize leading whitespace removal from the *actual* indentation
  $(shell echo "  ** Targeting $(TEKKOTSU_TARGET_MODEL) for build on $(TEKKOTSU_TARGET_PLATFORM) **" > /dev/tty)
  $(shell echo "  ** TEKKOTSU_DEBUG is $(if $(TEKKOTSU_DEBUG),ON: $(TEKKOTSU_DEBUG),OFF) **" > /dev/tty)
  $(shell echo "  ** TEKKOTSU_OPTIMIZE is $(if $(TEKKOTSU_DEBUG),DISABLED BY DEBUG,$(if $(TEKKOTSU_OPTIMIZE),ON: $(TEKKOTSU_OPTIMIZE),OFF)) **" > /dev/tty)
endif
endif

#sanity checks
ifeq ($(filter fsms clean% distclean docs alldocs,$(MAKECMDGOALS)),)
  ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
    $(if $(shell [ -d "$(OPENRSDK_ROOT)" ] || echo "not found"),$(error OPEN-R SDK not found at '$(OPENRSDK_ROOT)', check installation.))
    $(if $(shell [ -d "$(OPENRSDK_ROOT)/OPEN_R" ] || echo "not found"),$(error OPEN-R SDK header files missing, check installation.))
  endif
  $(if $(shell $(CXX) -v > /dev/null 2>&1 || echo "not found"),$(error C++ compiler not found at '$(CXX)', check installation.))
endif

$(shell mkdir -p $(TK_BD))

ifeq ($(MAKELEVEL),0)
  export ENV_CXXFLAGS:=$(CXXFLAGS)
  export ENV_LDFLAGS:=$(LDFLAGS)
endif
unexport CXXFLAGS
unexport LDFLAGS


#############  MAKEFILE VARIABLES  ################

# Would you like some more compiler flags?  We like lots of warnings.
# There are some files with exceptions to these flags - MMCombo*.cc
# need to have optimizations turned off, and several TinyFTPD sources
# have -Weffc++ and -DOPENR_DEBUG turned off.  If you want to modify
# these exceptions, look in the middle of the 'Makefile Machinery'
# section. (grep/search for the file name)

ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
  PLATFORM_FLAGS:= \
	  -isystem $(OPENRSDK_ROOT)/OPEN_R/include/MCOOP \
	  -isystem $(OPENRSDK_ROOT)/OPEN_R/include/R4000 \
	  -isystem $(OPENRSDK_ROOT)/OPEN_R/include \
	  -isystem aperios/include \
	  $(if $(TEKKOTSU_DEBUG),-DOPENR_DEBUG,) -DLOADFILE_NO_MMAP \
	  $(shell aperios/bin/xml2-config --cflags)
else
  PLATFORM_FLAGS:=$(shell xml2-config --cflags) -Winvalid-pch \
	$(shell if [ -d "$(CWIID_ROOT)" ] ; then echo "-DCWIID_ICE -I$(CWIID_ROOT)/include"; fi) \
  $(shell if [ -r /usr/lib/libOpenNI.so ] ; then echo -DHAVE_OPENNI -I/usr/include/ni; fi) \
	$(if $(FLYCAP_ROOT),-I$(FLYCAP_ROOT))
  #enable -fPIC if we are building shared libraries on x86_64/amd64
  ifneq ($(filter __amd64__ __x86_64__,$(shell g++ $(CXXFLAGS) -dM -E - < /dev/null)),)
    ifneq ($(suffix $(LIBTEKKOTSU)),.a)
      PLATFORM_FLAGS:=$(PLATFORM_FLAGS) -fPIC
    endif
  endif
  LDFLAGS:= $(LDFLAGS) \
	$(shell xml2-config --libs) -lpng -ljpeg \
	$(shell if [ -r /usr/lib/libOpenNI.so ] ; then echo -lOpenNI; fi) \
	$(if $(ISMACOSX), $(shell if [ $(TEST_MACOS_MAJOR) -gt 10 -o $(TEST_MACOS_MAJOR) -eq 10 -a $(TEST_MACOS_MINOR) -ge 6 ] ; \
	then echo -framework QTKit -framework CoreVideo -framework Cocoa; \
	else echo -framework Quicktime -framework Carbon; fi))
endif

CXXFLAGS:= -std=c++11 \
	$(if $(TEKKOTSU_DEBUG),$(TEKKOTSU_DEBUG),$(TEKKOTSU_OPTIMIZE)) \
	$(if $(TEKKOTSU_STRICT_MODEL),-DSTRICT_TGT_MODEL) \
	-pipe -fno-common \
	-Wall -Wshadow -Wlarger-than=200000 -Wpointer-arith -Wcast-qual \
	-Woverloaded-virtual -Weffc++ -Wdeprecated -Wnon-virtual-dtor \
	-fmessage-length=0 \
	-I$(TEKKOTSU_ROOT) -I$(TEKKOTSU_ROOT)/Shared/newmat -I$(TK_BD) \
	-D$(TEKKOTSU_TARGET_PLATFORM)  -D$(TEKKOTSU_TARGET_MODEL) \
	$(CPPFLAGS) $(PLATFORM_FLAGS) $(ENV_CXXFLAGS)

# This is kind of annoying due to flagging double-to-float casts, but good
# for a quick check now and then for 'real' 64bit issues (e.g. size_t clipping)
# $(if $(findstring Darwin,$(shell uname)),-Wshorten-64-to-32) 

INCLUDE_PCH=$(if $(TEKKOTSU_PCH),-include $(TK_BD)/$(TEKKOTSU_PCH))


###################################################
##              SOURCE CODE LIST                 ##
###################################################

# Find all of the source files: (except temp files in build directory)
# You shouldn't need to change anything here unless you want to add
# external libraries or new directories for the search
SRCSUFFIX:=.cc
SRC_DIRS:=Behaviors Crew DualCoding Events IPC Kodu Motion Planners Localization Shared Sound Vision Wireless
SRCS:=$(shell find -L $(SRC_DIRS) -name "[^.]*$(SRCSUFFIX)")
FSMS:=$(shell find -L $(SRC_DIRS) -name "[^.]*\.fsm")

# We will do recursive makes on these directories
# If you want to add something here, you'll probably also have to
# add to CXXFLAGS and LDFLAGS above to pull in the resulting binaries
# Source code files found within these will be filtered out of $(SRCS)
USERLIBS:= Shared/newmat Vision/AprilTags Vision/SIFT


###################################################
##             MAKEFILE MACHINERY                ##
###################################################
# Hopefully, you shouldn't have to change anything down here...

#delete automatic suffix list
.SUFFIXES:

.PHONY: all compile clean distclean cleanDeps docs alldocs cleanDoc updateTools updateLibs $(USERLIBS) platformBuild update install static shared fsms

# Clear any previous lockfile just in case we are using tools/sbin/pager to
# serialize error output from parallel builds
TMPDIR?=/tmp
JUNK:=$(shell rm -f $(TMPDIR)/TekkotsuBuildPagerLock-$(USER))

USERLIB_SRCS:=$(shell find -L $(USERLIBS) -name "[^.]*$(SRCSUFFIX)")
SRCS:=$(filter-out $(USERLIB_SRCS),$(SRCS))

ifeq ($(filter TGT_ERS%,$(TEKKOTSU_TARGET_MODEL)),)
all:
	@echo "Running $(MAKE) from the root directory will build the"
	@echo "Tekkotsu library which is linked against by executables."
	@echo "The Environment.conf from the template 'project' directory"
	@echo "will be used, which can be overridden by environment"
	@echo "variables.  Current settings are:"
	@echo "";
	@echo "  Target model: $(TEKKOTSU_TARGET_MODEL)"
	@echo "  Build directory: $(TEKKOTSU_BUILDDIR)"
	@echo "";
	@echo "You will want to run 'make' from your project directory in order"
	@echo "to produce executables..."
	@echo ""
	$(MAKE) TEKKOTSU_TARGET_PLATFORM= compile static shared
	@echo "Build successful."
else
all:
	@echo "Running $(MAKE) from the root directory will first build the"
	@echo "Tekkotsu library for Aperios (AIBO), and then for the local"
	@echo "platform.  The Environment.conf from the template 'project'"
	@echo "directory will be used, which can be overridden by environment"
	@echo "variables.  Current settings are:"
	@echo "";
	@echo "  Target model: $(TEKKOTSU_TARGET_MODEL)"
	@echo "  Build directory: $(TEKKOTSU_BUILDDIR)"
	@echo "";
	@echo "You will want to run 'make' from your project directory in order"
	@echo "to produce executables..."
	@echo ""
	$(MAKE) TEKKOTSU_TARGET_PLATFORM=PLATFORM_APERIOS compile static
	$(MAKE) TEKKOTSU_TARGET_PLATFORM= compile static shared
	@echo "Build successful."
endif

update install sim:
	@echo ""
	@echo "You probably want to be running make from within your project's directory"
	@echo ""
	@echo "You can run $(MAKE) from within the root Tekkotsu directory to build"
	@echo "libtekkotsu for both Aperios and the host platform, which will then"
	@echo "be linked against by the projects and tools."
	@echo ""
	@echo "However, you can only install or update to memory stick from within a project."
	@echo "You can use the template project directory if you want to build a stick"
	@echo "with the standard demo behaviors."

# Don't want to try to remake this - give an error if not found
$(TEKKOTSU_ROOT)/project/Environment.conf:
	@echo "Could not find Environment file - check the default project directory still exists"
	@exit 1

TOOLS_BUILT_FLAG:=$(TEKKOTSU_BUILDDIR)/.toolsBuilt

ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
include aperios/Makefile.aperios
else
include local/Makefile.local
endif

# Sort by modification date
SRCS:=$(shell ls -t $(SRCS) $(filter %$(SRCSUFFIX).fsm,$(FSMS)) )

# The object file for each of the source files
OBJS:=$(addprefix $(TK_BD)/,$(SRCS:$(SRCSUFFIX)=.o))
OBJS:=$(OBJS:.mm=.o)
OBJS:=$(OBJS:$(SRCSUFFIX).fsm=-fsm.o)

# list of all source files of all components, sorted to remove
# duplicates.  This gives us all the source files which we care about,
# all in one place.
DEPENDS:=$(addprefix $(TK_BD)/,$(SRCS:$(SRCSUFFIX)=.d) )
DEPENDS:=$(DEPENDS:$(SRCSUFFIX).fsm=-fsm.d)
DEPENDS:=$(DEPENDS:.mm=.d)
ifneq ($(TEKKOTSU_PCH),)
DEPENDS:=$(DEPENDS) $(TK_BD)/$(TEKKOTSU_PCH).d
endif

SRCS:=$(patsubst %$(SRCSUFFIX).fsm,$(PROJ_BD)/%-fsm$(SRCSUFFIX),$(SRCS))

$(patsubst %.fsm,$(TK_BD)/%,$(filter %.h.fsm,$(FSMS))) : $(TK_BD)/%.h : %.h.fsm
	@mkdir -p $(dir $@)
	@if [ -n "${XCODE_VERSION_MAJOR}" ] ; then prefix="$(CURDIR)/"; fi; \
	tools/sbin/stateparser "$${prefix}$^" "$(subst $(CURDIR)/,,$@)" "${TEKKOTSU_ROOT}/tools/perl"

$(patsubst %$(SRCSUFFIX).fsm,$(TK_BD)/%-fsm$(SRCSUFFIX),$(filter %$(SRCSUFFIX).fsm,$(FSMS))) : $(TK_BD)/%-fsm$(SRCSUFFIX) : %$(SRCSUFFIX).fsm
	@mkdir -p $(dir $@)
	@if [ -n "${XCODE_VERSION_MAJOR}" ] ; then prefix="$(CURDIR)/"; fi; \
	tools/sbin/stateparser "$${prefix}$^" "$(subst $(CURDIR)/,,$@)" "${TEKKOTSU_ROOT}/tools/perl"

$(filter %-fsm.d,$(DEPENDS)) : $(TK_BD)/%.d : $(TK_BD)/%$(SRCSUFFIX)

$(filter-out %-fsm.d %$(TEKKOTSU_PCH).d $(OBJCPPDEPENDS),$(DEPENDS)) : $(TK_BD)/%.d : %$(SRCSUFFIX)

$(DEPENDS) : | $(patsubst %.fsm,$(TK_BD)/%,$(filter %.h.fsm,$(FSMS)))

fsms: $(patsubst %.fsm,$(TK_BD)/%,$(filter %.h.fsm,$(FSMS))) $(patsubst %$(SRCSUFFIX).fsm,$(TK_BD)/%-fsm$(SRCSUFFIX),$(filter %$(SRCSUFFIX).fsm,$(FSMS)))

$(TK_BD)/$(TEKKOTSU_PCH).d : $(TK_BD)/%.d : % | $(patsubst %.fsm,$(TK_BD)/%,$(filter %.h.fsm,$(FSMS)))

%$(PCHSUFFIX):
	@mkdir -p $(dir $@)
	@src=$(patsubst $(TK_BD)/%,%,$*); \
	echo "Pre-compiling $$src..."; \
	$(CXX) $(CXXFLAGS) -x c++-header -c $$src -o $@ > $*.log 2>&1; \
        retval=$$?; \
        cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
        test $$retval -eq 0;

$(TK_BD)/%.d:
	@mkdir -p $(dir $@)
	@src=$(if $(filter %-fsm.d,$@),$(@:.d=$(SRCSUFFIX)),$*$(SRCSUFFIX)); \
	if [ ! -f "$$src" ] ; then \
		echo "ERROR: Missing source file '$$src'... you shouldn't be seeing this"; \
		exit 1; \
	fi; \
	echo "$@..." | sed 's@.*$(TK_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > "$@" || (rm -f "$@"; exit 1)

$(TK_BD)/$(TEKKOTSU_PCH).d:
	@mkdir -p $(dir $@)
	@src=$(TEKKOTSU_PCH); \
	echo "$@..." | sed 's@.*$(TK_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -x c++-header -MP -MG -MT "$@" -MT "$(@:.d=$(PCHSUFFIX))" -MM "$$src" > $@

EMPTYDEPS:=$(shell find -L $(TK_BD) -type f -name "*\.d" -size 0 -print -exec rm \{\} \;)
ifneq ($(EMPTYDEPS),)
  $(shell echo "Empty dependency files detected: $(EMPTYDEPS)" > /dev/tty)
endif

ifeq ($(filter fsms clean% distclean docs alldocs newstick,$(MAKECMDGOALS)),)
-include $(DEPENDS)
ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
-include $(TK_BD)/aperios/aperios.d
endif
endif

compile: platformBuild

platformBuild shared : updateTools updateLibs

$(TOOLS_BUILT_FLAG):
	@$(MAKE) TOOLS_BUILT_FLAG="$(TOOLS_BUILT_FLAG)" -C tools

docs:
	docs/builddocs --update --tree --search

alldocs:
	docs/builddocs --update --all --tree --search

updateTools: | $(TOOLS_BUILT_FLAG) 
	$(MAKE) -C tools

updateLibs: $(USERLIBS)

$(USERLIBS): | $(TOOLS_BUILT_FLAG)
	@echo "$@:"; \
	export TEKKOTSU_ENVIRONMENT_CONFIGURATION="$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)"; \
	$(MAKE) -C "$@"

ifeq ($(findstring compile,$(MAKECMDGOALS)),compile)
ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
static: $(TK_BD)/libtekkotsu.a ;
shared:
	@echo "PLATFORM_APERIOS does not support shared libraries... Make goal 'shared' ignored."
else
static: $(TK_BD)/libtekkotsu.a ;
shared: $(TK_BD)/libtekkotsu.$(if $(findstring Darwin,$(shell uname)),dylib,so) ;
endif
else
static shared: all ;
endif

$(TK_BD)/libtekkotsu.a: $(OBJS) VERSION
	@echo "Updating build timestamp..."; $(CXX) $(CXXFLAGS) -o $(TK_BD)/VERSION.o -c -x c++ VERSION;
	@echo "Linking object files..."
	@printf "$@ <- "; echo "[...]" | sed 's@$(TK_BD)/@@g';
	@rm -f $@;
	@$(AR) $@ $(OBJS) $(TK_BD)/VERSION.o;
	@$(AR2) $@
	@$(if $(filter PLATFORM_LOCAL%,$(TEKKOTSU_TARGET_PLATFORM)),echo "Redirecting PLATFORM_LOCAL to $(TEKKOTSU_TARGET_PLATFORM)"; rm -rf $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL; ln -s $(TEKKOTSU_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM) $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL)

$(TK_BD)/libtekkotsu.dylib: $(OBJS) VERSION | updateLibs
	@echo "Updating build timestamp..."; $(CXX) $(CXXFLAGS) -o $(TK_BD)/VERSION.o -c -x c++ VERSION;
	@echo "Linking object files..."
	@printf "$@ <- "; echo "[...]" | sed 's@$(TK_BD)/@@g';
	@$(CXX) -dynamiclib -o $@ $(OBJS) $(TK_BD)/VERSION.o -L$(TK_LIB_BD) $(addprefix -l,$(notdir $(USERLIBS))) $(LDFLAGS)
	@$(if $(filter PLATFORM_LOCAL%,$(TEKKOTSU_TARGET_PLATFORM)),echo "Redirecting PLATFORM_LOCAL to $(TEKKOTSU_TARGET_PLATFORM)"; rm -rf $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL; ln -s $(TEKKOTSU_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM) $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL)

$(TK_BD)/libtekkotsu.so: $(OBJS) VERSION | updateLibs
	@echo "Linking object files..."
	@printf "$@ <- "; echo "[...]" | sed 's@$(TK_BD)/@@g';
	@$(CXX) -shared -o $@ $(OBJS) -x c++ VERSION -Wl,-rpath,$(TK_LIB_BD) -L$(TK_LIB_BD) $(addprefix -l,$(notdir $(USERLIBS))) $(LDFLAGS);
	@$(if $(filter PLATFORM_LOCAL%,$(TEKKOTSU_TARGET_PLATFORM)),echo "Redirecting PLATFORM_LOCAL to $(TEKKOTSU_TARGET_PLATFORM)"; rm -rf $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL; ln -s $(TEKKOTSU_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM) $(TEKKOTSU_BUILDDIR)/PLATFORM_LOCAL)

%.h :
	@echo "ERROR: Seems to be a missing header file '$@'...";
	@if [ "$(notdir $@)" = "def.h" -o "$(notdir $@)" = "entry.h" ] ; then \
		echo "WARNING: You shouldn't be seeing this message.  Report that you did." ; \
		echo "         Try a clean recompile." ; \
		exit 1; \
	fi;
	@echo "       Someone probably forgot to check a file into CVS.";
	@echo "       I'll try to find where it's being included from:";
	@echo "       if this was a file you recently deleted, just make again after this completes. (will update dependency files)";
	@find -L . -name "*.h" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L . -name "*.cc" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L $(TK_BD) -name "*.d" -exec grep -qH "$(notdir $@)" \{\} \; -exec rm \{\} \; ;
	@exit 1

#don't try to make random files via this implicit chain
%:: %.o ;

$(TK_BD)/%.o: $(if $(TEKKOTSU_PCH),$(TK_BD)/$(TEKKOTSU_PCH)$(PCHSUFFIX)) | $(TOOLS_BUILT_FLAG)
	@mkdir -p $(dir $@)
	@src=$(if $(filter %-fsm.o,$@),$(subst $(PWD)/,,$(@:.o=$(SRCSUFFIX))),$*$(SRCSUFFIX)); \
	echo "Compiling $$src..."; \
	$(CXX) $(CXXFLAGS) $(INCLUDE_PCH) -o $@ -c $$src > $(TK_BD)/$*.log 2>&1; \
	retval=$$?; \
	cat $(TK_BD)/$*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0;

clean:
	@printf "\nCleaning all framework build files for ${TEKKOTSU_TARGET_MODEL}...\n"
	@echo "(Use 'make distclean' to clean all targets and tools)"
	rm -rf $(TK_BD)

cleanStubs:
	@cd aperios; for dir in `ls -d [A-Z]*` ; do \
		if [ "$$dir" = "CVS" ] ; then continue; fi; \
		if [ "$$dir" = ".svn" ] ; then continue; fi; \
		if [ -f "$$dir" ] ; then continue; fi; \
		echo rm -f "aperios/$$dir/$${dir}Stub.h" "aperios/$$dir/$${dir}Stub.cc" "aperios/$$dir/def.h" "aperios/$$dir/entry.h" ; \
		rm -f "$$dir/$${dir}Stub.h" "$$dir/$${dir}Stub.cc" "$$dir/def.h" "$$dir/entry.h" ; \
	done

distclean: cleanStubs
	rm -rf $(TEKKOTSU_BUILDDIR)
	$(MAKE) TOOLS_BUILT_FLAG="$(TOOLS_BUILT_FLAG)" -C tools clean;

cleanDeps:
	@printf "Cleaning all .d files from build directory..."
	@find -L "$(TEKKOTSU_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM)/$(TEKKOTSU_TARGET_MODEL)" -name "*.d" -exec rm \{\} \;
	@printf "done.\n"

cleanDoc:
	docs/builddocs --clean



