
# This Makefile will handle most aspects of compiling and
# linking a tool against the Tekkotsu framework.  You probably
# won't need to make any modifications, but here's the major controls

# Executable name, defaults to:
#   `basename \`pwd\``-$(TEKKOTSU_TARGET_MODEL)
TEKKOTSU_TARGET_MODEL?=TGT_DYNAMIC
BIN:=$(shell pwd | sed 's@.*/@@')

# Build directory
PROJECT_BUILDDIR:=build

# Other default values are drawn from the template project's
# Environment.conf file.  This is found using $(TEKKOTSU_ROOT)
# Remove the '?' if you want to override an environment variable
# with a value of your own.
TEKKOTSU_ROOT:=../..

# Non-OS X only, install path for Ogre3D, will search for headers at $(OGRE_ROOT)/include/OGRE
OGRE_ROOT?=/usr/local

# Non-OS X only, install path for Bullet physics library
BULLET_ROOT?=/usr/local

# Source files, defaults to all files ending matching *$(SRCSUFFIX)
SRCSUFFIX:=.cc
PROJ_SRC:=$(shell find src -name "*$(SRCSUFFIX)") \
	$(wildcard $(TEKKOTSU_ROOT)/Shared/plist*$(SRCSUFFIX)) \
	$(wildcard $(TEKKOTSU_ROOT)/Shared/jpeg-6b/*$(SRCSUFFIX)) \
	$(addsuffix $(SRCSUFFIX), $(addprefix $(TEKKOTSU_ROOT)/, \
		$(addprefix Shared/,LoadSave BoundingBox XMLLoadSave DynamicInfo RobotInfo ImageUtil TimeET string_util StackTrace Resource fmat) \
		IPC/Thread IPC/ProcessID IPC/DriverMessaging Wireless/netstream Motion/KinematicJoint Motion/SensorInfo \
		Planners/PlannerObstacles Planners/Dynamics/PhysicsBody Planners/Dynamics/PhysicsWorld Planners/Dynamics/MotorControllers \
	))

ifeq ($(shell uname),Darwin)
OBJC_SRC:=$(shell find src -name "*.mm")
endif

NEWMAT_SRC:=$(wildcard $(TEKKOTSU_ROOT)/Shared/newmat/*.cpp)

LIBSUFFIX:=$(if $(findstring Darwin,$(shell uname)),.dylib,.so)

.PHONY: all test

ifeq ($(MAKECMDGOALS),docdump)
  PROJECT_BUILDDIR:=$(PROJECT_BUILDDIR)/docdump
  PROJ_SRC:=src/docdump.cc $(TEKKOTSU_ROOT)/Shared/string_util.cc
  CXXFLAGS:=-DDOCDUMP $(CXXFLAGS)
endif

TEMPLATE_PROJECT:=$(TEKKOTSU_ROOT)/project
TEKKOTSU_ENVIRONMENT_CONFIGURATION?=$(TEMPLATE_PROJECT)/Environment.conf
$(if $(shell [ -r $(TEKKOTSU_ENVIRONMENT_CONFIGURATION) ] || echo "failure"),$(error An error has occured, '$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)' could not be found.  You may need to edit TEKKOTSU_ROOT in the Makefile))

TEKKOTSU_TARGET_PLATFORM:=PLATFORM_LOCAL
include $(shell echo "$(TEKKOTSU_ENVIRONMENT_CONFIGURATION)" | sed 's/ /\\ /g')
FILTERSYSWARN:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(FILTERSYSWARN))
COLORFILT:=$(patsubst $(TEKKOTSU_ROOT)/%,$(TEKKOTSU_ROOT)/%,$(COLORFILT))
CXX:=$(filter-out -m%,$(CXX))
$(shell mkdir -p $(PROJ_BD))

PROJ_OBJ:=$(patsubst %$(SRCSUFFIX),$(PROJ_BD)/%.o,$(PROJ_SRC))

OBJC_OBJ:=$(patsubst %.mm,$(PROJ_BD)/objc/%.o,$(OBJC_SRC))

NEWMAT_OBJ:=$(patsubst %.cpp,$(PROJ_BD)/%.o,$(NEWMAT_SRC))

LIBS:=$(PROJ_BD)/libnewmat$(LIBSUFFIX)

DEPENDS:=$(PROJ_OBJ:.o=.d)

NEWMAT_CXXFLAGS:=-g -Wall -pipe -O3 -ffast-math -fno-common \
	$(if $(findstring Darwin,$(shell uname)),-arch i386,$(if $(filter __amd64__ __x86_64__,$(shell g++ $(CXXFLAGS) -dM -E - < /dev/null)),-fPIC)) \
	$(CXXFLAGS)

CXXFLAGS:=-std=c++11 -g -Wall -pipe -fvisibility=hidden \
          $(if $(findstring Darwin,$(shell uname)), \
              -arch i386, \
              $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(OGRE_ROOT)/lib/pkgconfig pkg-config --cflags OGRE OIS) \
          ) \
          -I$(TEKKOTSU_ROOT) -I$(BULLET_ROOT)/include/bullet -I$(BULLET_ROOT)/src -DHAVE_BULLET \
          -I$(TEKKOTSU_ROOT)/Shared/jpeg-6b $(shell xml2-config --cflags) \
          -D$(TEKKOTSU_TARGET_PLATFORM) -D$(TEKKOTSU_TARGET_MODEL) -DNO_TEKKOTSU_CONFIG $(CXXFLAGS) $(CPPFLAGS)

LDFLAGS:=-ljpeg -lpng -ldl \
         $(if $(findstring Darwin,$(shell uname)), \
             -arch i386 -framework Ogre -framework OIS -framework Carbon -framework IOKit -framework Cocoa \
                 -framework BulletCollision -framework BulletDynamics -framework LinearMath, \
             $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(OGRE_ROOT)/lib/pkgconfig pkg-config --libs OGRE OIS) \
                 -L$(BULLET_ROOT)/lib/ -Wl,-rpath=$(OGRE_ROOT)/lib -Wl,-rpath=$(BULLET_ROOT)/lib \
                 -Wl,-rpath=$(shell cd $(PROJ_BD);pwd) -rdynamic -lBulletCollision -lBulletDynamics -lLinearMath \
         ) \
         -L$(PROJ_BD) $(LDFLAGS)

all: $(BIN)

$(PROJ_BD)/libnewmat.dylib: $(NEWMAT_OBJ)
	@echo "Linking libnewmat$(LIBSUFFIX)..."
	@libtool -dynamic -undefined dynamic_lookup -o $@ $(NEWMAT_OBJ)

$(PROJ_BD)/libnewmat.so: $(NEWMAT_OBJ)
	@echo "Linking libnewmat$(LIBSUFFIX)..."
	@$(CXX) -shared -o $@ $(NEWMAT_OBJ)

$(BIN): $(PROJ_OBJ) $(OBJC_OBJ) $(LIBS)
	@echo "Linking $@..."
	@$(if $(CBUILD),$(CC),$(CXX)) $(PROJ_OBJ) $(OBJC_OBJ) $(LDFLAGS) $(patsubst $(PROJ_BD)/lib%$(LIBSUFFIX),-l%,$(LIBS)) $(shell xml2-config --libs) -lpthread -o $@
	ln -fs ../$@/$@ ../bin

docdump: $(PROJ_OBJ)
	@echo "Linking $@..."
	@$(if $(CBUILD),$(CC),$(CXX)) $(PROJ_OBJ) $(LDFLAGS) -o $@

ifeq ($(findstring clean,$(MAKECMDGOALS)),)
-include $(DEPENDS)
endif

%.h :
	@echo "Could not find header file $@, you will need fix the header(s) and then 'make cleanDeps'" 
	@false

%.a :
	@echo "ERROR: $@ was not found.  You may need to compile the Tekkotsu framework."
	@echo "Press return to attempt to build it, ctl-C to cancel."
	@read;
	$(MAKE) -C $(TEKKOTSU_ROOT) compile

%.d :
	@mkdir -p $(dir $@)
	@src=$(patsubst %.d,%.cc,$(patsubst $(PROJ_BD)/%,%,$@)); \
	echo "$@..." | sed 's@.*$(TGT_BD)/@Generating @'; \
	$(if $(CBUILD),$(CC),$(CXX)) $(CXXFLAGS) -MP -MG -MT "$@" -MT "$(@:.d=.o)" -MM "$$src" > $@

$(NEWMAT_OBJ): %.o: $(wildcard $(TEKKOTSU_ROOT)/Shared/newmat/*.h)
	@mkdir -p $(dir $@)
	@src=$(patsubst %.o,%.cpp,$(patsubst $(PROJ_BD)/%,%,$@)); \
	echo "Compiling $$src..."; \
	$(CXX) $(NEWMAT_CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

$(PROJ_OBJ): %.o:
	@mkdir -p $(dir $@)
	@src=$(patsubst %.o,%$(SRCSUFFIX),$(patsubst $(PROJ_BD)/%,%,$@)); \
	echo "Compiling $$src..."; \
	$(if $(CBUILD),$(CC),$(CXX)) $(CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

$(OBJC_OBJ): %.o:
	@mkdir -p $(dir $@)
	@src=$(patsubst $(PROJ_BD)/objc/%.o,%.mm,$@); \
	echo "Compiling $$src..."; \
	$(if $(CBUILD),$(CC),$(CXX)) $(CXXFLAGS) -o $@ -c $$src > $*.log 2>&1; \
	retval=$$?; \
	cat $*.log | $(FILTERSYSWARN) | $(COLORFILT) | $(TEKKOTSU_LOGVIEW); \
	test $$retval -eq 0; \

clean:
	rm -rf $(BIN) $(BIN).dSYM $(PROJECT_BUILDDIR) Motion Shared Wireless IPC *~

cleanDeps:
	@rm $(DEPENDS)

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
