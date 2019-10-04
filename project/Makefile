################ PROJECT MAKEFILE #################

# Make sure the default target is 'all' by listing it first
all:

###################################################
##             ENVIRONMENT SETUP                 ##
###################################################
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(shell pwd | sed 's/ /\\ /g')/Environment.conf
ifneq ($(filter sim% tekkotsu%,$(MAKECMDGOALS)),)
	ifneq ($(filter sim-%,$(MAKECMDGOALS)),)
		TEKKOTSU_TARGET_MODEL:=$(patsubst sim-%,TGT_%,$(filter sim-%,$(MAKECMDGOALS)))
	endif
	ifneq ($(filter tekkotsu-%,$(MAKECMDGOALS)),)
		TEKKOTSU_TARGET_MODEL:=$(patsubst tekkotsu-%,TGT_%,$(filter tekkotsu-%,$(MAKECMDGOALS)))
	endif
	ifneq ($(filter TGT_ERS%,$(TEKKOTSU_TARGET_MODEL)),)
		# For Aibos, override to select local platform
		TEKKOTSU_TARGET_PLATFORM:=
	endif
endif
include $(TEKKOTSU_ENVIRONMENT_CONFIGURATION)

ifeq ($(XCODE_VERSION_ACTUAL),)
ifeq ($(MAKELEVEL),0)
  $(shell echo "  ** Project targeting $(TEKKOTSU_TARGET_MODEL) for build on $(TEKKOTSU_TARGET_PLATFORM) **" > /dev/tty)
  $(shell echo "  ** TEKKOTSU_DEBUG is $(if $(TEKKOTSU_DEBUG),ON: $(TEKKOTSU_DEBUG),OFF) **" > /dev/tty)
  $(shell echo "  ** TEKKOTSU_OPTIMIZE is $(if $(TEKKOTSU_DEBUG),DISABLED BY DEBUG,$(if $(TEKKOTSU_OPTIMIZE),ON: $(TEKKOTSU_OPTIMIZE),OFF)) **" > /dev/tty)
endif
endif

#sanity checks
$(if $(shell [ -d "$(TEKKOTSU_ROOT)" ] || echo "not found"),$(error Could not access TEKKOTSU_ROOT: $(TEKKOTSU_ROOT)))
ifeq ($(filter clean% distclean fsms,$(MAKECMDGOALS)),)
  ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
    $(if $(shell [ -d "$(OPENRSDK_ROOT)" ] || echo "not found"),$(error OPEN-R SDK not found at '$(OPENRSDK_ROOT)', check installation.))
    $(if $(shell [ -d "$(OPENRSDK_ROOT)/OPEN_R" ] || echo "not found"),$(error OPEN-R SDK header files missing, check installation.))
  endif
  $(if $(shell $(CXX) -v > /dev/null 2>&1 || echo "not found"),$(error C++ compiler not found at '$(CXX)', check installation.))
endif

$(shell mkdir -p $(PROJ_BD))

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
	  -isystem $(TEKKOTSU_ROOT)/aperios/include \
	  $(if $(TEKKOTSU_DEBUG),-DOPENR_DEBUG,) -DLOADFILE_NO_MMAP \
	  $(shell $(TEKKOTSU_ROOT)/aperios/bin/xml2-config --cflags)
  LDFLAGS:=$(LDFLAGS) -lObjectComm -lOPENR  -lInternet -lantMCOOP -lERA201D1 \
	  $(shell $(TEKKOTSU_ROOT)/aperios/bin/xml2-config --libs) -ljpeg -lpng -lregex
else
  PLATFORM_FLAGS:=$(shell xml2-config --cflags) -Winvalid-pch \
  $(shell if [ -r /usr/lib/libOpenNI.so ] ; then echo -DHAVE_OPENNI -I/usr/include/ni; fi) \
  #enable -fPIC if we are building shared libraries on x86_64/amd64
  ifneq ($(filter __amd64__ __x86_64__,$(shell g++ $(CXXFLAGS) -dM -E - < /dev/null)),)
    ifneq ($(suffix $(LIBTEKKOTSU)),.a)
      PLATFORM_FLAGS:=$(PLATFORM_FLAGS) -fPIC
    endif
  endif
  LDFLAGS:=$(LDFLAGS) -rdynamic $(shell xml2-config --libs) -lreadline -lncurses -lpthread -ldl \
          $(shell if [ -r /usr/lib/libOpenNI.so ] ; then echo -lOpenNI; fi) \
          $(if $(FLYCAP_ROOT),-lflycapture) \
          $(if $(ISMACOSX),-bind_at_load -framework AudioUnit -framework CoreFoundation -framework CoreServices, -lasound -lrt)
endif

ifneq ($(SKEWLZONE_ROOT),)
  PLATFORM_FLAGS:=$(PLATFORM_FLAGS) -I$(SKEWLZONE_ROOT)/include
  LDFLAGS:=$(LDFLAGS) -L$(SKEWLZONE_ROOT)/lib -lBotSense -lrnr -lRCB3 -li2c -lserial
endif

CXXFLAGS:= -std=c++11 \
	$(if $(TEKKOTSU_DEBUG),$(TEKKOTSU_DEBUG),$(TEKKOTSU_OPTIMIZE)) \
	$(if $(TEKKOTSU_STRICT_MODEL),-DSTRICT_TGT_MODEL) \
	-pipe -fno-common \
	-Wall -Wshadow -Wlarger-than=200000 -Wpointer-arith -Wcast-qual \
	-Woverloaded-virtual -Weffc++ -Wdeprecated -Wnon-virtual-dtor \
	-fmessage-length=0 \
	-I. -I$(TEKKOTSU_ROOT) -I$(TEKKOTSU_ROOT)/Shared/newmat -I$(PROJ_BD) -I$(TK_BD) \
	-isystem $(TEKKOTSU_ROOT)/Shared/jpeg-6b \
	-D$(TEKKOTSU_TARGET_PLATFORM)  -D$(TEKKOTSU_TARGET_MODEL) \
	$(if $(FLYCAP_ROOT),-DHAVE_FLYCAP) \
	$(CPPFLAGS) $(PLATFORM_FLAGS) $(ENV_CXXFLAGS)

INCLUDE_PCH=$(if $(TEKKOTSU_PCH),-include $(TK_BD)/$(TEKKOTSU_PCH))

TEKKOTSU_VERSION:=$(shell cat $(TEKKOTSU_ROOT)/VERSION)

###################################################
##              SOURCE CODE LIST                 ##
###################################################

# Find all of the source files: (except temp files in build directory,
# or platform-specific files in either aperios or sim)
# You shouldn't need to change anything here unless you want to add
# external libraries
SRCSUFFIX=.cc
SRCS:=$(patsubst ./%,%,$(shell find -L . -name "[^.]*$(SRCSUFFIX)" -or -path "*$(PROJECT_BUILDDIR)" \! -prune -or -name "templates" \! -prune -or -name "aperios" \! -prune -or -name "local" \! -prune))
FSMS:=$(patsubst ./%,%,$(shell find -L . -name "[^.]*\.fsm" -or -path "*$(PROJECT_BUILDDIR)" \! -prune -or -name "templates" \! -prune -or -name "aperios" \! -prune -or -name "local" \! -prune))

# These are libraries expected to be built in/by the framework directory
TK_FW_LIBS:=$(TK_BD)/$(LIBTEKKOTSU) \
           $(TK_LIB_BD)/libnewmat$(suffix $(LIBTEKKOTSU)) \
           $(TK_LIB_BD)/libAprilTags$(suffix $(LIBTEKKOTSU)) \
           $(TK_LIB_BD)/libSIFT$(suffix $(LIBTEKKOTSU))

# We can also add project specific third-party libraries if you want:
USERLIBS:=$(TK_FW_LIBS)


###################################################
##             MAKEFILE MACHINERY                ##
###################################################
# Hopefully, you shouldn't have to change anything down here,
# except one or two little things ;)

.PHONY: all compile clean distclean cleanDeps cleanProj checkLibs testLibs buildLibs sim tekkotsu fsms

sim all update install ftpupdate: %: checkLibs compile-%

# Clear any previous lockfile just in case we are using tools/sbin/pager to
# serialize error output from parallel builds
TMPDIR?=/tmp
JUNK:=$(shell rm -f $(TMPDIR)/TekkotsuBuildPagerLock-$(USER))

#DEFTARGET is set when using XCode to trigger 'update' instead of 'all'
ifeq ($(TEKKOTSU_DEFTARGET),)
compile-all: compile
else
compile-all: compile-$(TEKKOTSU_DEFTARGET) ;
endif

# Don't want to try to remake this - give an error if not found
$(TEKKOTSU_ROOT)/project/Environment.conf:
	@echo "Could not find Environment file - check the default project directory still exists"
	@exit 1

checkLibs: $(if $(TEKKOTSU_ALWAYS_BUILD),buildLibs,testLibs)	

$(TK_BD)/$(TEKKOTSU_PCH)$(PCHSUFFIX): checkLibs ;

$(TK_BD)/%.o $(TK_BD)/%.h: checkLibs ;

ifeq ($(TEKKOTSU_ALWAYS_BUILD),)
$(TK_FW_LIBS): testLibs ;

else
$(TK_FW_LIBS): buildLibs ;

endif

testLibs:
	@broken=false; \
	for x in $(USERLIBS) ; do \
		if [ \! -r "$$x" ] ; then \
			echo "Could not read framework library '$$x'"; \
			broken=true; \
		fi; \
	done; \
	if $$broken ; then \
		echo "It appears that the framework itself has not been compiled."; \
		echo "TEKKOTSU_ROOT = $(TEKKOTSU_ROOT)"; \
		echo "TK_BD = $(TK_BD)"; \
		echo "Press enter to compile it now, ctl-C to cancel."; \
		read; \
		export TEKKOTSU_ENVIRONMENT_CONFIGURATION=$(TEKKOTSU_ENVIRONMENT_CONFIGURATION); \
		$(MAKE) TEKKOTSU_TARGET_MODEL=$(TEKKOTSU_TARGET_MODEL) TEKKOTSU_TARGET_PLATFORM=$(TEKKOTSU_TARGET_PLATFORM) -C "$(TEKKOTSU_ROOT)" compile ; \
	fi;

buildLibs:
	@echo "Updating framework build...";
	@export TEKKOTSU_ENVIRONMENT_CONFIGURATION=$(TEKKOTSU_ENVIRONMENT_CONFIGURATION); \
	$(MAKE) TEKKOTSU_TARGET_MODEL=$(TEKKOTSU_TARGET_MODEL) TEKKOTSU_TARGET_PLATFORM=$(TEKKOTSU_TARGET_PLATFORM) -C "$(TEKKOTSU_ROOT)" compile ; 

ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
include aperios/Makefile.aperios
else
include local/Makefile.local
endif

# Sort by modification date
SRCS:=$(shell ls -t $(SRCS) $(filter %$(SRCSUFFIX).fsm,$(FSMS)))

# The object file for each of the source files
OBJS:=$(addprefix $(PROJ_BD)/,$(SRCS:$(SRCSUFFIX)=.o))
OBJS:=$(OBJS:$(SRCSUFFIX).fsm=-fsm.o)

# list of all source files of all components, sorted to remove
# duplicates.  This gives us all the source files which we care about,
# all in one place.
DEPENDS:=$(addprefix $(PROJ_BD)/,$(SRCS:$(SRCSUFFIX)=.d))
DEPENDS:=$(DEPENDS:$(SRCSUFFIX).fsm=-fsm.d)

SRCS:=$(patsubst %$(SRCSUFFIX).fsm,$(PROJ_BD)/%-fsm$(SRCSUFFIX),$(SRCS))

EMPTYDEPS:=$(shell find -L $(PROJ_BD) -type f -name "*\.d" -size 0 -print -exec rm \{\} \;)
ifneq ($(EMPTYDEPS),)
EMPTYDEPS:=$(shell echo "Empty dependency files detected: $(EMPTYDEPS)" > /dev/tty)
endif

ifeq ($(filter fsms clean% distclean docs newstick,$(MAKECMDGOALS)),)
-include $(DEPENDS)
ifeq ($(TEKKOTSU_TARGET_PLATFORM),PLATFORM_APERIOS)
-include $(PROJ_BD)/aperios/aperios.d
else
-include $(PROJ_BD)/local/local.d
endif
endif

$(patsubst %.fsm,$(PROJ_BD)/%,$(filter %.h.fsm,$(FSMS))) : $(PROJ_BD)/%.h : %.h.fsm
	@mkdir -p $(dir $@)
	@if [ -n "${XCODE_VERSION_MAJOR}" ] ; then prefix="$(CURDIR)/"; fi; \
	$(TEKKOTSU_ROOT)/tools/sbin/stateparser "$${prefix}$^" "$@" "${TEKKOTSU_ROOT}/tools/perl"

$(patsubst %$(SRCSUFFIX).fsm,$(PROJ_BD)/%-fsm$(SRCSUFFIX),$(filter %$(SRCSUFFIX).fsm,$(FSMS))) : $(PROJ_BD)/%-fsm$(SRCSUFFIX) : %$(SRCSUFFIX).fsm
	@mkdir -p $(dir $@)
	@if [ -n "${XCODE_VERSION_MAJOR}" ] ; then prefix="$(CURDIR)/"; fi; \
	$(TEKKOTSU_ROOT)/tools/sbin/stateparser "$${prefix}$^" "$@" "${TEKKOTSU_ROOT}/tools/perl"

$(filter %-fsm.d,$(DEPENDS)) : $(PROJ_BD)/%.d : $(PROJ_BD)/%$(SRCSUFFIX) | $(TK_FW_LIBS)

$(filter-out %-fsm.d,$(DEPENDS)) : $(PROJ_BD)/%.d : %$(SRCSUFFIX)

$(DEPENDS) : | $(TK_FW_LIBS) $(patsubst %.fsm,$(PROJ_BD)/%,$(filter %.h.fsm,$(FSMS)))

fsms: $(patsubst %.fsm,$(PROJ_BD)/%,$(filter %.h.fsm,$(FSMS))) $(patsubst %$(SRCSUFFIX).fsm,$(PROJ_BD)/%-fsm$(SRCSUFFIX),$(filter %$(SRCSUFFIX).fsm,$(FSMS)))
	@$(if $(TEKKOTSU_ALWAYS_BUILD), $(MAKE) TEKKOTSU_TARGET_MODEL=$(TEKKOTSU_TARGET_MODEL) TEKKOTSU_TARGET_PLATFORM=$(TEKKOTSU_TARGET_PLATFORM) -C "$(TEKKOTSU_ROOT)" fsms)

$(PROJ_BD)/%.d : 
	@mkdir -p $(dir $@)
	@src=$(if $(filter %-fsm.d,$@),$(@:.d=$(SRCSUFFIX)),$*$(SRCSUFFIX)); \
	echo "$@..." | sed 's@$(PROJ_BD)/@Generating @'; \
	$(CXX) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > "$@" || (rm -f "$@"; exit 1)

%.h :
	@if [ "$(notdir $@)" = "def.h" -o "$(notdir $@)" = "entry.h" ] ; then \
		echo "WARNING: You shouldn't be seeing this message.  Report that you did." ; \
		echo "         Try a clean recompile." ; \
	fi;
	@echo "ERROR: Seems to be a missing header file '$@'...";
	@echo "       Someone probably forgot to check a file into CVS.";
	@echo "       I'll try to find where it's being included from:";
	@echo "       if this was a file you recently deleted, just make again after this completes. (will update dependency files)";
	@find -L . -name "*.h" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L . -name "*.cc" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L $(TEKKOTSU_ROOT) -name "*.h" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L $(TEKKOTSU_ROOT) -name "*.cc" -exec grep -H "$(notdir $@)" \{\} \; ;
	@find -L $(TK_BD) -name "*.d" -exec grep -qH "$(notdir $@)" \{\} \; -exec rm \{\} \; ;
	@find -L $(PROJ_BD) -name "*.d" -exec grep -qH "$(notdir $@)" \{\} \; -exec rm \{\} \; ;
	@exit 1

#don't try to make random files via this implicit chain
%:: %.o
	echo "I don't know anything about '$*'";
	exit 1;

# Don't locally rebuild framework files
# (sometimes platform_local targets want to doubly-compile the target specific object files... might be a make bug)
$(TK_BD)/%.o: checkLibs

$(PROJ_BD)/%.o: $(if $(TEKKOTSU_PCH),$(TK_BD)/$(TEKKOTSU_PCH)$(PCHSUFFIX))
	@mkdir -p $(dir $@)
	@src=$(if $(filter %-fsm.o,$@),$(@:.o=$(SRCSUFFIX)),$*$(SRCSUFFIX)); \
	echo "Compiling $$src..."; \
	$(CXX) $(CXXFLAGS) $(INCLUDE_PCH) -o $@ -c $$src > $(PROJ_BD)/$*.log 2>&1; \
	retval=$$?; \
	cat $(PROJ_BD)/$*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

$(PROJ_BD)/project.a: $(OBJS)
	@echo "Linking project files..."
	@printf "$@ <- "; echo "$(OBJS)" | sed 's@$(PROJ_BD)/@@g';
	@rm -f $@;
	@$(AR) $@ $(OBJS);
	@$(AR2) $@

clean: cleanProj
	$(if $(TEKKOTSU_ALWAYS_BUILD),$(MAKE) -C $(TEKKOTSU_ROOT) $@)

cleanProj:
	@printf "\nCleaning all project build files for ${TEKKOTSU_TARGET_MODEL}...\n"
	@echo "(Use 'make distclean' to clean all targets and tools)"
	rm -rf $(PROJ_BD)
	@if [ -n "$(INSTALL_BINS)" ] ; then echo rm -f $(INSTALL_BINS); rm -f $(INSTALL_BINS); fi

cleanStubs:
	@cd aperios; for dir in `ls -d [A-Z]*` ; do \
		if [ "$$dir" = "CVS" ] ; then continue; fi; \
		if [ "$$dir" = ".svn" ] ; then continue; fi; \
		if [ -f "$$dir" ] ; then continue; fi; \
		echo rm -f "aperios/$$dir/$${dir}Stub.h" "aperios/$$dir/$${dir}Stub.cc" "aperios/$$dir/def.h" "aperios/$$dir/entry.h" ; \
		rm -f "$$dir/$${dir}Stub.h" "$$dir/$${dir}Stub.cc" "$$dir/def.h" "$$dir/entry.h" ; \
	done

distclean: cleanStubs
	rm -rf $(PROJECT_BUILDDIR)
	$(if $(TEKKOTSU_ALWAYS_BUILD),$(MAKE) -C $(TEKKOTSU_ROOT) $@)

cleanDeps:
	@echo "Cleaning all .d files from build directory..."
	find -L $(PROJECT_BUILDDIR)/$(TEKKOTSU_TARGET_PLATFORM)/$(TEKKOTSU_TARGET_MODEL) -name "*.d" -exec rm \{\} \;
	@echo "done."
	$(if $(TEKKOTSU_ALWAYS_BUILD),$(MAKE) -C $(TEKKOTSU_ROOT) $@)
