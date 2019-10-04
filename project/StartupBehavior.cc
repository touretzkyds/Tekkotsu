#include "StartupBehavior.h"
#include "Shared/RobotInfo.h"
#include "Shared/string_util.h"
#include "Shared/ProjectInterface.h"

#include "Behaviors/Controller.h"
#ifdef TGT_HAS_POWER_STATUS
#  include "Behaviors/Controls/BatteryCheckControl.h"
#endif
#include "Behaviors/Controls/ControlBase.h"

#include "Motion/EmergencyStopMC.h"
#include "Motion/MotionSequenceMC.h"

#include "Sound/SoundManager.h"

#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)
#  include "Shared/CameraData.h"
#endif

using namespace std;


BehaviorBase& ProjectInterface::startupBehavior() {
	// used by Main process, called after environmental setup is complete
	static StartupBehavior * theStartup=NULL;
	if(!theStartup)
		theStartup=new StartupBehavior;
	return *theStartup;
}

StartupBehavior::StartupBehavior()
	: BehaviorBase("StartupBehavior"), spawned(), setup(), pids(), behGroup()
{	
	setAutoDelete(false); // this is a global, so there's a global reference, don't delete yourself
	behGroup.setAutoDelete(false); // this is a member, so let destructor handle it
}

StartupBehavior::~StartupBehavior() {}

void StartupBehavior::doStart() {
	//Initialize the Vision pipeline (it's probably a good idea to do this
	//first in case later stuff wants to reference the vision stages)
	initVision();

#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)
	CameraData().loadCameraData(); // copies data to RobotInfo::CameraHomography
#endif
	
	//This will "fade" in the PIDs so the joints don't jerk to full
	//power, also looks cooler
	pids = SharedObject<PIDMC>(0);
	addMotion(pids, PERSISTENT, MotionManager::kEmergencyPriority+2);
	//also, pause before we start fading in, PIDs take effect right
	//away, before the emergencystop is picked up
	erouter->addTimer(this,0,4*FrameTime*NumFrames,true);

	bool startInEStop=false;
#ifdef TGT_IS_AIBO
	startInEStop=true;
#endif
	bool makeNoise=false;
	ProjectInterface::estop->setStopped(startInEStop, makeNoise); 
	addMotion(ProjectInterface::estop, PERSISTENT, MotionManager::kEmergencyPriority);
	
#ifdef TGT_HAS_POWER_STATUS
	//This displays the current battery conditions on the console
	if(state->framesProcessed!=0 && state->frameNumber!=0) { // skip it if we haven't gotten any sensors
		BatteryCheckControl batchk;
		batchk.activate(MotionManager::invalid_MC_ID,NULL);
		batchk.deactivate();
	}
#endif

	//This is what runs the menu system
	Controller * controller=new Controller;
	controller->start();
	setup.push_back(new ControlBase("Root Control"));
	controller->setRoot( SetupMenus() ); //this triggers the layout of the menus themselves
	wireless->setReceiver(sout, Controller::console_callback);
	spawned.push_back(controller);
	
#if defined(TGT_CHIARA) or defined(TGT_CHIARA2) 
	sndman->playFile("chiara-ready.wav");
#elif defined(TGT_IS_AIBO)
	sndman->playFile("roar.wav");
#else
	sndman->playFile("beepdoub.wav");
#endif

#ifdef TGT_HAS_MOUTH
	//This will close the mouth so it doesn't look stupid or get in the
	//way of head motions (ERS-210 or ERS-7 only)
	SharedObject<TinyMotionSequenceMC> closeMouth;
	closeMouth->advanceTime(3000); //take 3 seconds to close the mouth
	closeMouth->setOutputCmd(MouthOffset,outputRanges[MouthOffset][MaxRange]);
	closeMouth->advanceTime(500); //and hold it for another .5 seconds
	closeMouth->setOutputCmd(MouthOffset,outputRanges[MouthOffset][MaxRange]);
	addMotion(closeMouth,PRUNABLE,MotionManager::kEmergencyPriority+1);
	erouter->addTimer(this,1,3250,false);
#endif
	
#ifdef PLATFORM_APERIOS
	sout->printf("Remember, telnet to port 10001 for text entry (port 59000 is read only)\n");
#endif
	
	//if you didn't want to start off paused, you should throw an
	//un-estop event.  This will make it clear to any background
	//behaviors (namely WorldStateVelDaemon) that we're not in estop
	if(!startInEStop)
		erouter->postEvent(EventBase::estopEGID,ProjectInterface::estop.getID(),EventBase::deactivateETID,0);
}

void StartupBehavior::doStop() {
	erouter->removeListener(this);
	for(vector<BehaviorBase*>::iterator it=spawned.begin(); it!=spawned.end(); it++) {
		//cout << "StartupBehavior stopping spawned: " << (*it)->getName() << endl;
		(*it)->stop();
	}
	spawned.clear();
	ProjectInterface::estop.clear();
	pids.clear();
}

/*!Uses a few timer events at the beginning to fade in the PID values, and closes the mouth too*/
void StartupBehavior::doEvent() {
	if(event->getGeneratorID()==EventBase::timerEGID) {
		if(event->getSourceID()==0) {
			//this will do the work of fading in the PID values.  It helps the joints
			//to power up without twitching
			static unsigned int start_time=-1U;
			const unsigned int tot_time=2000; 
			if(start_time==-1U) { //first time
				start_time=get_time();
			} else {
				float power=(get_time()-start_time)/(float)tot_time;
				if(power>1)
					power=1;
				pids->setAllPowerLevel(power*power);
			}
			if((get_time()-start_time)>=(tot_time+250)) { // leave extra time to assure we reach power=1
				erouter->removeTimer(this,0);
				//when this goes through, it will trigger the event handler below to follow through cleanup of the id
				removeMotion(pids);
				pids.clear();
			}
#ifdef TGT_HAS_MOUTH
		} else if(event->getSourceID()==1) {
			//we're done closing the mouth... set the mouth to closed in the estop too
			//otherwise it might twitch a little when the MotionSequence expires and the estop takes over
			// (little == +/-.1 radians, aka estop's threshold for resetting an "out of place" joint) (details, details)
			float weight=ProjectInterface::estop->getOutputCmd(MouthOffset).weight;
			ProjectInterface::estop->setOutputCmd(MouthOffset,OutputCmd(outputRanges[MouthOffset][MaxRange],weight));
#endif
		} else {
			cerr << "Warning: StartupBehavior got unknown timer event: " << event->getName() << endl;
		}
	}
}

void
StartupBehavior::processRegisteredMenus() {
	ControlBase::ControlRegistry_t& entries = ControlBase::getControllerEntries();
	for(ControlBase::ControlRegistry_t::const_iterator it = entries.begin(); it!=entries.end(); ++it) {
		ControlBase * menu = findMenu(setup.front(), it->first);
		for(ControlBase::ControlRegistryEntry_t::const_iterator it2 = it->second.begin(); it2!=it->second.end(); ++it2) {
			ControlBase * entry = it2->second.first;
			int flags = it2->second.second;
			menu->pushSlot(entry);
			if(flags & BEH_START) {
				if(BehaviorSwitchControlBase* behsw = dynamic_cast<BehaviorSwitchControlBase*>(entry)) {
					behsw->start();
				} else if(BehaviorBase* beh = dynamic_cast<BehaviorBase*>(entry)) {
					beh->start();
				} else {
					std::cerr << "WARNING: BEH_START flag set for " << it2->first << " in menu " << it->first << " but is not a behavior" << std::endl;
				}
			}
			if((flags & BEH_NONEXCLUSIVE) == 0) {
				if(BehaviorSwitchControlBase* behsw = dynamic_cast<BehaviorSwitchControlBase*>(entry)) {
					behsw->setGroup(&behGroup);
				} else {
					// controls are non-exclusive anyway, so just let this slide
					//std::cerr << "WARNING: BEH_NONEXCLUSIVE flag set for " << it2->first << " in menu " << it->first << " but is not a behavior" << std::endl;
				}
			}
			// BEH_RETAINED is handled in BehaviorBase::registerControllerEntry(), nothing to do here
			entry->registered();
		}
	}
}

void
StartupBehavior::startSubMenu(const std::string& name, const std::string& description) {
	ControlBase * c = setup.back()->findSlot(name);
	if(c==NULL) {
		c=new ControlBase(name,description);
		setup.back()->pushSlot(c);
	} else {
		c->setDescription(description);
	}
	setup.push_back(c);
}

void
StartupBehavior::addItem(ControlBase * control) {
	setup.back()->pushSlot(control);
}

ControlBase*
StartupBehavior::endSubMenu() {
	ControlBase * tmp=setup.back();
	setup.pop_back();
	return tmp;
}

ControlBase*
StartupBehavior::findMenu(ControlBase* cur, const std::string& name) const {
	std::vector<std::string> path = string_util::tokenize(name,"/");
	for(std::vector<std::string>::const_iterator it=path.begin(); it!=path.end(); ++it) {
		if(it->size()==0)
			continue;
		ControlBase * c = cur->findSlot(*it);
		if(c==NULL) {
			c=new ControlBase(*it);
			cur->pushSlot(c);
		}
		cur=c;
	}
	return cur;
}
