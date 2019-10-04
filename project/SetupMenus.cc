#include "StartupBehavior.h"
#include "Shared/RobotInfo.h"

/* This file configures the menu structure see in the ControllerGUI,
 *  partly via a call to StartupBehavior::processRegisteredMenus(), and
 *  partly via 'manual' addition of remaining controls which cannot be
 *  instantiated during static initialization.
 *
 *  Most behaviors can be registered via REGISTER_BEHAVIOR(Foo),
 *  so generally you should not need to edit this file.
 *
 *  See also StartupBehavior::doStart(), which performs some
 *  additional configuration before calling this function. */

// STATUS REPORTS
#include "Behaviors/Controls/EventLogger.h"

// FILE ACCESS
#include "Behaviors/Controls/PostureEditor.h"
#include "Behaviors/Controls/RunSequenceControl.h"
#include "Behaviors/Controls/PlaySoundControl.h"
#include "Behaviors/Controls/DumpFileControl.h"
#if defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS)
#  include "Behaviors/Controls/WaypointWalkControl.h"
#endif
#include "Behaviors/Controls/LoadPostureControl.h"
#include "Behaviors/Controls/SavePostureControl.h"
#include "Behaviors/Controls/ConfigurationEditor.h"
#include "Motion/WalkMC.h"
#ifdef TGT_HAS_CMPACKWALK
#  include "Behaviors/Controls/WalkEdit.h"
#elif defined(TGT_HAS_XWALK)
#  include "Behaviors/Controls/XWalkEdit.h"
#endif

// SHUTDOWN
#include "Behaviors/Controls/ShutdownControl.h"
#ifdef PLATFORM_APERIOS
#  include "Behaviors/Controls/RebootControl.h"
#endif

#include "Behaviors/Controls/HelpControl.h"

ControlBase*
StartupBehavior::SetupMenus() {
	// Fix the order of root menus by pre-defining them now
	// Later startSubMenu calls will reuse these instead of adding duplicate entries
	startSubMenu("User Behaviors","Behaviors written by YOU!"); endSubMenu();
	startSubMenu("Framework Demos","Behaviors provided with the framework to demonstrate various features"); endSubMenu();
	startSubMenu("Background Behaviors","Background daemons and monitors"); endSubMenu();
	startSubMenu("TekkotsuMon","Servers for GUIs"); endSubMenu();
	startSubMenu("Status Reports","Displays information about the runtime environment on the console"); endSubMenu();
	startSubMenu("File Access","Access/load files on the memory stick"); endSubMenu();
	startSubMenu("Vision Pipeline","Start/Stop stages of the vision pipeline"); endSubMenu();
	
	
	// Adds entries from REGISTER_BEHAVIOR and REGISTER_CONTROL macro calls
	processRegisteredMenus();
	
	
	// ** Manual configuration of remaining Controller entries **
	// Most of these use 'config' in their constructors, which is not available during static initialization
	// Thus they are not instantiated until StartupBehavior is called after the environment is set up.
	startSubMenu("Status Reports"); {
		addItem(new EventLogger());
	} endSubMenu();
	
	startSubMenu("File Access"); {
		addItem(new ConfigurationEditor("Tekkotsu Configuration"));
		addItem(new LoadPostureControl("Load Posture"));
		addItem(new SavePostureControl("Save Posture"));
		addItem(new PostureEditor);
		addItem(new RunSequenceControl<XLargeMotionSequenceMC::CAPACITY*2>("Run Motion Sequence"));
		addItem(new PlaySoundControl("Play Sound"));
#if defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS)
		addItem(new WaypointWalkControl());
#endif
#ifdef TGT_HAS_CMPACKWALK
		addItem(new WalkEdit);
#elif defined(TGT_HAS_XWALK)
		addItem(new XWalkEdit);
#endif
		addItem(new DumpFileControl("Files",config->portPath("")));
	} endSubMenu();
	
	SetupVision(); // puts entries in "Vision Pipeline" menu
	
	startSubMenu("Shutdown?","Confirms decision to reboot or shut down"); {
		addItem(new ShutdownControl());
#ifdef PLATFORM_APERIOS
		addItem(new RebootControl());
#endif
	} endSubMenu();
	
	addItem(new HelpControl(setup.front(),2));
	
	if(setup.size()!=1)
		std::cout << "\n*** WARNING *** menu setup stack corrupted" << std::endl;
	
	return setup.front();
}
