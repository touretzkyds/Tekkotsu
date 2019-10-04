#include "MMCombo.h"
#include "Shared/WorldState.h"
#include "Shared/Profiler.h"
#include "Shared/debuget.h"
#include "Shared/Config.h"
#include "Shared/zignor.h"
#include "IPC/SharedObject.h"
#include "Events/EventRouter.h"
#include "Events/EventTranslator.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/Kinematics.h"
#include "Sound/SoundManager.h"
#include "Events/DataEvent.h"
#include "Events/TextMsgEvent.h"
#include "Events/FilterBankEvent.h"
#include "Shared/WMclass.h"

#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"

#include "Shared/ProjectInterface.h"

#include "Events/EventBase.h"

#include <OPENR/OSyslog.h>
#include <OPENR/core_macro.h>
#include <OPENR/OFbkImage.h>
#include "aperios/MMCombo/entry.h"

using namespace std;

//#define NO_PROFILING
#ifdef NO_PROFILING
#undef PROFSECTION
#define PROFSECTION(...) {}
#endif

// no-op, just eat a pointer
inline std::istream& operator>>(std::istream& s, const OFbkImageVectorData*& rs) { void * x; s>>x; return s; }
inline std::istream& operator>>(std::istream& s, const OSoundVectorData*& rs) { void * x; s>>x; return s; }

MMCombo::MMCombo()
	: OObject(), motmanMemRgn(NULL), motionProfilerMemRgn(NULL), soundProfilerMemRgn(NULL),
		worldStateMemRgn(NULL), soundManagerMemRgn(NULL), processMapMemRgn(NULL),
		runLevel(0), num_open(0), etrans(NULL), entryPt(), isStopped(true),
		gainListener(), shutterListener(), wbListener()
{
try {
	for(unsigned int i=0; i<NumOutputs; i++) {
		primIDs[i]=oprimitiveID_UNDEF;
		open[i]=false;
	}
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo construction",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo construction",NULL))
		throw;
}
}

OStatus
MMCombo::DoInit(const OSystemEvent&)
{
try {
	cout << objectName << "::DoInit() " << endl;

	isStopped=false;

	NEW_ALL_SUBJECT_AND_OBSERVER;
	REGISTER_ALL_ENTRY;
	SET_ALL_READY_AND_NOTIFY_ENTRY;
	
	// make sure the library doesn't drop data "for" us on this reliable communication channel
	observer[obsReceiveWorldState]->SetBufCtrlParam(0,1,1);
	observer[obsReceiveMotionManager]->SetBufCtrlParam(0,1,1);
	observer[obsReceiveSoundManager]->SetBufCtrlParam(0,1,1);
	observer[obsReceiveProcessMap]->SetBufCtrlParam(0,1,1);
	observer[obsEventTranslatorComm]->SetBufCtrlParam(0,1,20); //allows up to 20 messages to be sent in a burst before dropping any
	observer[obsMotionManagerComm]->SetBufCtrlParam(0,1,MotionManager::MAX_MOTIONS+1);
	observer[obsReceiveMotionProfiler]->SetBufCtrlParam(0,1,1);
	observer[obsReceiveSoundProfiler]->SetBufCtrlParam(0,1,2);
	//+1 to MAX_MOTIONS so we can get a delete message after we've filled up

	cout << objectName << ": sbjRegisterWorldState==" << sbjRegisterWorldState << " selector==" << subject[sbjRegisterWorldState]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveWorldState==" << obsReceiveWorldState << " selector==" << observer[obsReceiveWorldState]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjRegisterMotionManager==" << sbjRegisterMotionManager << " selector==" << subject[sbjRegisterMotionManager]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveMotionManager==" << obsReceiveMotionManager << " selector==" << observer[obsReceiveMotionManager]->GetID().GetSelector() << '\n'
			 << objectName << ": obsEventTranslatorComm==" << obsEventTranslatorComm << " selector==" << observer[obsEventTranslatorComm]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjEventTranslatorComm==" << sbjEventTranslatorComm << " selector==" << observer[sbjEventTranslatorComm]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveMotionProfiler==" << obsReceiveMotionProfiler << " selector==" << observer[obsReceiveMotionProfiler]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveSoundProfiler==" << obsReceiveSoundProfiler << " selector==" << observer[obsReceiveSoundProfiler]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjRegisterProfiler==" << sbjRegisterProfiler << " selector==" << observer[sbjRegisterProfiler]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjMoveJoint==" << sbjMoveJoint << " selector==" << subject[sbjMoveJoint]->GetID().GetSelector() << '\n'
			 << objectName << ": obsSensorFrame==" << obsSensorFrame << " selector==" << observer[obsSensorFrame]->GetID().GetSelector() << '\n'
			 << objectName << ": obsImage==" << obsImage << " selector==" << observer[obsImage]->GetID().GetSelector() << '\n'
			 << objectName << ": obsMic==" << obsMic << " selector==" << observer[obsMic]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjMotionManagerComm==" << sbjMotionManagerComm << " selector==" << subject[sbjMotionManagerComm]->GetID().GetSelector() << '\n'
			 << objectName << ": obsMotionManagerComm==" << obsMotionManagerComm << " selector==" << observer[obsMotionManagerComm]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveSoundManager==" << obsReceiveSoundManager << " selector==" << observer[obsReceiveSoundManager]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjRegisterProcessMap==" << sbjRegisterProcessMap << " selector==" << subject[sbjRegisterProcessMap]->GetID().GetSelector() << '\n'
			 << objectName << ": obsReceiveProcessMap==" << obsReceiveProcessMap << " selector==" << observer[obsReceiveProcessMap]->GetID().GetSelector() << '\n'
			 << objectName << ": sbjSoundManagerComm==" << sbjSoundManagerComm << " selector==" << subject[sbjSoundManagerComm]->GetID().GetSelector() << '\n'
			 << flush;

	if(strcmp(objectName,"MainObj")==0) {
		ProcessID::setID(ProcessID::MainProcess);
		
		//processMapMemRgn -> ProcessID::setMap() setup
		processMapMemRgn = InitRegion(sizeof(stacktrace::StackFrame)*ProcessID::NumProcesses);
		memset(processMapMemRgn->Base(),0,sizeof(stacktrace::StackFrame)*ProcessID::NumProcesses);
		ProcessID::setMap(reinterpret_cast<stacktrace::StackFrame*>(processMapMemRgn->Base()));
	} else if(strcmp(objectName,"MotoObj")==0)
		ProcessID::setID(ProcessID::MotionProcess);

	MarkScope ep(entryPt);
	
	
	//Read config file
	::config = new Config();
	::config->setFileSystemRoot("/ms");
	::config->vision.gain.addPrimitiveListener(&gainListener);
	::config->vision.shutter_speed.addPrimitiveListener(&shutterListener);
	::config->vision.white_balance.addPrimitiveListener(&wbListener);
	if(::config->loadFile("config/tekkotsu.xml")==0) {
		if(::config->loadFile("config/tekkotsu.cfg")==0)
			std::cerr << std::endl << " *** ERROR: Could not load configuration file config/tekkotsu.xml *** " << std::endl << std::endl;
		else
			std::cerr << "Successfully imported settings from old-format tekkotsu.cfg" << std::endl;
	}
	
	erouter = new EventRouter;
	
	if(strcmp(objectName,"MainObj")==0) {
		bool isSlowOutput[NumOutputs];
		for(unsigned int i=0; i<NumOutputs; i++)
			isSlowOutput[i]=!IsFastOutput[i];

		SetupOutputs(isSlowOutput);

		//Request power status updates
		OPowerStatus observationStatus;
		observationStatus.Set(orsbALL,obsbALL,opsoREMAINING_CAPACITY_NOTIFY_EVERY_CHANGE,opsoTEMPERATURE_NOTIFY_EVERY_CHANGE,opsoTIME_DIF_NOTIFY_EVERY_CHANGE,opsoVOLUME_NOTIFY_EVERY_CHANGE);
		OServiceEntry entry(myOID_, Extra_Entry[entryGotPowerEvent]);
		OStatus result = OPENR::ObservePowerStatus(observationStatus, entry);
		if(result != oSUCCESS) {
			OSYSLOG1((osyslogERROR, "%s : %s %d","MMCombo::DoStart()","OPENR::ObservePowerStatus() FAILED", result));
			return oFAIL;
		}
		
		//Setup wireless
		wireless = new Wireless();
		sout=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*12);
		serr=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*4);
		wireless->setDaemon(sout);
		wireless->setDaemon(serr);
			serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);
		
		//Have erouter listen for incoming stuff
		cout << "Telling erouter to serve remote event requests" << endl;
		erouter->serveRemoteEventRequests();
		
		//worldStateMemRgn -> state setup
		worldStateMemRgn = InitRegion(sizeof(WorldState));
		state = new (worldStateMemRgn->Base()) WorldState;
		
		mainProfiler = new mainProfiler_t;

		etrans=new NoOpEventTranslator(*erouter);
		MotionManager::setTranslator(etrans);
	}
	if(strcmp(objectName,"MotoObj")==0) {
		SetupOutputs(IsFastOutput);
		OPENR::SetMotorPower(opowerON);
		OPENR::EnableJointGain(oprimitiveID_UNDEF); //oprimitiveID_UNDEF means enable all

		//Setup wireless
		wireless = new Wireless();
		sout=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*6);
		serr=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*2);
		wireless->setDaemon(sout);
		wireless->setDaemon(serr);
		serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);
		
		//motmanMemRgn -> motman setup
		motmanMemRgn = InitRegion(sizeof(MotionManager));
		motman = new (motmanMemRgn->Base()) MotionManager;
		motman->InitAccess(subject[sbjMotionManagerComm]);
		
		//motionProfilerMemRgn -> motionProfiler setup
		motionProfilerMemRgn = InitRegion(sizeof(motionProfiler_t));
		motionProfiler = new (motionProfilerMemRgn->Base()) motionProfiler_t;
		
		etrans=new IPCEventTranslator(*subject[sbjEventTranslatorComm]);
		MotionManager::setTranslator(etrans);
		//MotionCommands enqueue directly, so there shouldn't be any riff-raff to catch
		//but just in case, subscribe to everything except erouterEGID
		for(unsigned int i=0; i<EventBase::numEGIDs; i++)
			if(i!=EventBase::erouterEGID)
				erouter->addTrapper(etrans,static_cast<EventBase::EventGeneratorID_t>(i));
	}
	kine = new Kinematics();
	
	cout << objectName << "::DoInit()-DONE" << endl;
	return oSUCCESS;

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoInit()",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoInit()",NULL))
		throw;
}
return oSUCCESS;
}

OStatus
MMCombo::DoStart(const OSystemEvent&)
{
	MarkScope ep(entryPt);
try {
	cout << objectName << "::DoStart() " << endl;

	// initialize the current power status, doesn't always give us
	// a power update right away otherwise
	if(strcmp(objectName,"MainObj")==0) {
		wireless->listen(sout, config->main.console_port);
		wireless->listen(serr, config->main.stderr_port);
		OPowerStatus power;
		OPENR::GetPowerStatus(&power);
		state->read(power,erouter);
	}

	if(strcmp(objectName,"MotoObj")==0) {
		wireless->listen(sout, config->motion.console_port);
		wireless->listen(serr, config->motion.stderr_port);
	}
	
	isStopped=false;

	ENABLE_ALL_SUBJECT;
	ASSERT_READY_TO_ALL_OBSERVER;

	if(strcmp(objectName,"MainObj")==0) {
		addRunLevel();
	}
	
	cout << objectName << "::DoStart()-DONE" << endl;
	return oSUCCESS;

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoStart()",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoStart()",NULL))
		throw;
}
return oSUCCESS;
}

OStatus
MMCombo::DoStop(const OSystemEvent&)
{
	MarkScope ep(entryPt);
try {
	erouter->postEvent(EventBase::runtimeEGID,0,EventBase::deactivateETID);
	cout << objectName << "::DoStop()..." << endl;
	if(strcmp(objectName,"MainObj")==0) {
		ProjectInterface::startupBehavior().stop();
		wireless->close(sout);
		wireless->close(serr);
		motman->RemoveAccess();
	}
	DISABLE_ALL_SUBJECT;
	DEASSERT_READY_TO_ALL_OBSERVER;
	isStopped=true;
	cout << objectName << "::DoStop()-DONE" << endl;
	return oSUCCESS;

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoStop()",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoStop()",NULL))
		throw;
}
return oSUCCESS;
}

OStatus
MMCombo::DoDestroy(const OSystemEvent&)
{
	MarkScope ep(entryPt);
try {
	cout << objectName << "::DoDestroy()..." << endl;
	delete etrans;
	etrans=NULL;
	MotionManager::setTranslator(NULL);
	if(strcmp(objectName,"MainObj")==0) {
		delete erouter;
		if(motmanMemRgn!=NULL) {
			motmanMemRgn->RemoveReference();
			motmanMemRgn=NULL;
		}
		if(motionProfilerMemRgn!=NULL) {
			motionProfilerMemRgn->RemoveReference();
			motionProfilerMemRgn=NULL;
		}
		delete mainProfiler;
	}
	if(strcmp(objectName,"MotoObj")==0) {
		if(worldStateMemRgn!=NULL) {
			worldStateMemRgn->RemoveReference();
			worldStateMemRgn=NULL;
		}
		if(processMapMemRgn!=NULL) {
			processMapMemRgn->RemoveReference();
			processMapMemRgn=NULL;
		}
	}
	if(soundManagerMemRgn!=NULL) {
		soundManagerMemRgn->RemoveReference();
		soundManagerMemRgn=NULL;
	}
	
	DELETE_ALL_SUBJECT_AND_OBSERVER;
	cout << objectName << "::DoDestroy()-DONE" << endl;
	return oSUCCESS;

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoDestroy()",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MMCombo::DoDestroy()",NULL))
		throw;
}
return oSUCCESS;
}

/*! Called when MotoObj is initially ready as well as when it has finished
 *  processing the previous message - we only want to do this the first time
 *  otherwise we infinite loop. */
void
MMCombo::ReadyRegisterWorldState(const OReadyEvent&){
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		cout << objectName << " Registering WorldState" << endl;
		if(strcmp(objectName,"MainObj")==0) {
			subject[sbjRegisterWorldState]->SetData(worldStateMemRgn);
			subject[sbjRegisterWorldState]->NotifyObservers();
		}
	}
}

void
MMCombo::GotWorldState(const ONotifyEvent& event){
	MarkScope ep(entryPt);
try {
	cout << objectName << "-GOTWORLDSTATE..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(strcmp(objectName,"MotoObj")==0) {
		ASSERT(event.NumOfData()==1,"Too many WorldStates");
		worldStateMemRgn = event.RCData(0);
		worldStateMemRgn->AddReference();
		state = reinterpret_cast<WorldState*>(worldStateMemRgn->Base());
	}
	observer[obsReceiveWorldState]->AssertReady();
	cout << "done" << endl;
} catch(const std::exception& ex) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during GotWorldState",&ex))
		throw;
} catch(...) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during GotWorldState",NULL))
		throw;
}
}

		
/*! Called when MainObj is initially ready as well as when it has finished
 *  processing the previous message - we only want to do this the first time
 *  otherwise we infinite loop. */
void
MMCombo::ReadyRegisterMotionManager(const OReadyEvent&){
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		cout << objectName << " Registering MotionManager" << endl;
		if(strcmp(objectName,"MotoObj")==0) {
			subject[sbjRegisterMotionManager]->SetData(motmanMemRgn);
			subject[sbjRegisterMotionManager]->NotifyObservers();
		}
	}
}

void
MMCombo::GotMotionManager(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	cout << objectName << "-GOTMOTIONMANAGER..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(strcmp(objectName,"MainObj")==0) {
		ASSERT(event.NumOfData()==1,"Too many MotionManagers");
		motmanMemRgn = event.RCData(0);
		motmanMemRgn->AddReference();
		motman = reinterpret_cast<MotionManager*>(motmanMemRgn->Base());
		cout << "MAIN INIT MOTMAN..." << flush;
		//			hexout(event.RCData(event_data_id)->Base(),128);
		motman->InitAccess(subject[sbjMotionManagerComm]);
		addRunLevel();
	}
  observer[obsReceiveMotionManager]->AssertReady();
	cout << "done" << endl;
}


void
MMCombo::GotInterProcessEvent(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	EventBase* evt=NULL;
try {
	//cout << objectName << "-GOTInterProcessEvent " << event.NumOfData() << "..." << flush;
	//cout << TimeET() << endl;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(etrans==NULL)
		return;
	static unsigned int lastSensorTime=-1U;
	static unsigned int lastSensorFrame=-1U;
	for(int i=0; i<event.NumOfData(); i++) {
		RCRegion * msg = event.RCData(i);
		msg->AddReference();
		evt=etrans->decodeEvent(msg->Base(),msg->Size());
		if(evt->getGeneratorID()==EventBase::sensorEGID) {
			// sensor events are dropped if there's a backlog
			// so if we get one of these, update the timestamp info
			if(lastSensorTime==-1U) {
				addRunLevel();
				lastSensorTime=0;
			}
			if(state->frameNumber==lastSensorFrame) { //already saw this one
				delete evt;
				evt=NULL;
			} else {
				evt->setTimeStamp(get_time());
				evt->setDuration(evt->getTimeStamp()-lastSensorTime);
				lastSensorTime=evt->getTimeStamp();
				//if(state->frameNumber-lastSensorFrame!=NumFrames && lastSensorFrame!=-1U)
				//cout << ProcessID::getIDStr() << " dropped " << (state->frameNumber-lastSensorFrame-NumFrames)/NumFrames << " sensor frames" << endl;
				lastSensorFrame=state->frameNumber;
			}
		}
		if(evt!=NULL)
			erouter->postEvent(*evt);
		delete evt;
		msg->RemoveReference();
	}
	if(state->frameNumber>lastSensorFrame && lastSensorTime!=-1U) {
		// there's new sensor info, but we didn't get the event
		//(Motion drops event if there's anything else already in the queue, might not have been a sensor event in the queue though)
		unsigned int t=get_time();
		erouter->postEvent(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID,t-lastSensorTime,"SensorSouceID::UpdatedSID",1);
		lastSensorTime=t;
		//if(state->frameNumber-lastSensorFrame!=NumFrames && lastSensorFrame!=-1U)
		//cout << ProcessID::getIDStr() << " dropped " << (state->frameNumber-lastSensorFrame-NumFrames)/NumFrames << " sensor frames" << endl;
		lastSensorFrame=state->frameNumber;
	}
	observer[obsEventTranslatorComm]->AssertReady();
	//cout << "done" << endl;

} catch(const std::exception& ex) {
  observer[obsEventTranslatorComm]->AssertReady();
	std::string msg("Occurred during inter-process event processing");
	if(evt!=NULL)
		msg+=": "+evt->getName();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),&ex))
		throw;
} catch(...) {
  observer[obsEventTranslatorComm]->AssertReady();
	std::string msg("Occurred during inter-process event processing");
	if(evt!=NULL)
		msg+=": "+evt->getName();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),NULL))
		throw;
}
}

		
void
MMCombo::ReadyRegisterProfiler(const OReadyEvent&){
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		if(strcmp(objectName,"MotoObj")==0) {
			cout << objectName << " Registering Motion Profiler" << endl;
			subject[sbjRegisterProfiler]->SetData(motionProfilerMemRgn);
			subject[sbjRegisterProfiler]->NotifyObservers();
		}
	}
}

void
MMCombo::GotMotionProfiler(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	cout << objectName << "-GOTMOTIONPROFILER..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(strcmp(objectName,"MainObj")==0) {
		ASSERT(event.NumOfData()==1,"Too many Profilers");
		motionProfilerMemRgn = event.RCData(0);
		motionProfilerMemRgn->AddReference();
		motionProfiler = reinterpret_cast<motionProfiler_t*>(motionProfilerMemRgn->Base());
		addRunLevel();
	}
	observer[obsReceiveMotionProfiler]->AssertReady();
	cout << "done" << endl;
}
void
MMCombo::GotSoundProfiler(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	cout << objectName << "-GOTSOUNDPROFILER..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(strcmp(objectName,"MainObj")==0) {
		ASSERT(event.NumOfData()==1,"Too many Profilers");
		soundProfilerMemRgn = event.RCData(0);
		soundProfilerMemRgn->AddReference();
		soundProfiler = reinterpret_cast<soundProfiler_t*>(soundProfilerMemRgn->Base());
		addRunLevel();
	}
	observer[obsReceiveSoundProfiler]->AssertReady();
	cout << "done" << endl;
}


void
MMCombo::ReadySendJoints(const OReadyEvent& sysevent) {
	MarkScope ep(entryPt);
try {

	if(isStopped) {
		//cout << "BAH!ReadySendJoints" << endl;
		return;
	}

#ifndef NO_PROFILING
	static unsigned int id=-1U;
	Profiler::Timer timer;
	if(ProcessID::getID()==ProcessID::MotionProcess) {
		if(id==-1U)
			id=motionProfiler->getNewID("ReadySendJoints()");
		timer.setID(id,&motionProfiler->prof);
	}	else if(ProcessID::getID()==ProcessID::MainProcess) {
		if(id==-1U)
			id=mainProfiler->getNewID("ReadySendJoints()");
		timer.setID(id,&mainProfiler->prof);
	}
#endif

	if(num_open==0) //If we don't have any joints to open, leave now. (i.e. MainObj on a 220, has no ears)
		return;

	// Find an unused command vector
	RCRegion* rgn=NULL;
	for (unsigned int i = 0; i < NUM_COMMAND_VECTOR; i++) {
		if (region[i]->NumberOfReference() == 1) {
			rgn=region[i];
			/*			if(strcmp(objectName,"MainObj")==0) {
							static unsigned int lasttime=get_time();
							unsigned int thistime=get_time();
							cout << '*' << i << ' ' << thistime << '\t' << (thistime-lasttime) << endl;
							lasttime=thistime;
							}*/
			break;
		}
	}
	ASSERTRET(rgn!=NULL,"Could not find unused command vector");
	ASSERTRET(rgn->Base()!=NULL,"Bad Command Vector");
	OCommandVectorData* cmdVecData = reinterpret_cast<OCommandVectorData*>(rgn->Base());
	
	// Update the outputs (note that Main is doing the ears)
	//I'm using an id compare instead of the slightly more readable strcmp for a tiny bit of speed
	bool isERS7=(RobotName == ERS7Info::TargetName);
	if(ProcessID::getID()==ProcessID::MotionProcess) {
		float outputs[NumFrames][NumOutputs];
		if(state!=NULL) {
			static bool seededRNG=false;
			if(!seededRNG) {
				seededRNG=true;
				initRNG();
			}
			motman->getOutputs(outputs);
			motman->updatePIDs(primIDs);
			
			//cout << "updateWorldState" << endl;
			for(uint output=LEDOffset; output<LEDOffset+NumLEDs; output++)
				state->outputs[output]=motman->getOutputCmd(output).value;
			for(uint output=BinJointOffset; output<BinJointOffset+NumBinJoints; output++)
				state->outputs[output]=motman->getOutputCmd(output).value;
			
			// these parts check to see if there are "fake" joints, and sets their "sensed" values
			// to be the current target value, just in case a behavior is waiting for a non-existant
			// non-existant joint to move to a certain position.
			const std::set<unsigned int>& fakes = RobotInfo::capabilities.getFakeOutputs();
			for(std::set<unsigned int>::const_iterator it=fakes.begin(); it!=fakes.end(); ++it)
				state->outputs[*it]=motman->getOutputCmd(*it).value;
			
		} else {
			for(unsigned int f=0; f<NumFrames; f++)
				for(unsigned int i=0; i<NumOutputs; i++)
					outputs[f][i]=0;
		}
			
		// Should be a relatively simple matter to copy angles into commands...
		unsigned int used=0; //but only copy open joints (so main does ears on 210, motion does everything else)
		for(unsigned int i=PIDJointOffset; i<PIDJointOffset+NumPIDJoints; i++)
			if(open[i]) {
				OJointCommandValue2* jval = reinterpret_cast<OJointCommandValue2*>(cmdVecData->GetData(used)->value);
				for(unsigned int frame=0; frame<NumFrames; frame++)
					jval[frame].value = (slongword)(outputs[frame][i]*1.0e6f);
				used++;
			}
		if(isERS7) {
			// except if it's an ERS-7, we have to use different data structures for some of the leds and the ears
			for(unsigned int i=LEDOffset; i<ERS7Info::FaceLEDPanelOffset; i++)
				if(open[i]) {
					OLEDCommandValue2* jval = reinterpret_cast<OLEDCommandValue2*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].led = calcLEDValue(i-LEDOffset,outputs[frame][i]);
					used++;
				}
			// for instance, this virtual mode thing, which is global to all the affected LEDs
			OLED3Mode curMode[NumFrames];
			for(unsigned int frame=0; frame<NumFrames; frame++)
				curMode[frame]=(calcLEDValue(ERS7Info::LEDABModeOffset-LEDOffset,sqrt(clipRange01(outputs[frame][ERS7Info::LEDABModeOffset])))==oledON?oled3_MODE_B:oled3_MODE_A);
			for(unsigned int i=ERS7Info::FaceLEDPanelOffset; i<LEDOffset+NumLEDs; i++)
				if(open[i]) {
					OLEDCommandValue3* jval = reinterpret_cast<OLEDCommandValue3*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++) {
						jval[frame].intensity = static_cast<sword>(255*clipRange01(outputs[frame][i]));
						jval[frame].mode=curMode[frame];
					}
					used++;
				}
			for(unsigned int i=BinJointOffset; i<BinJointOffset+NumBinJoints; i++)
				if(open[i]) {
					OJointCommandValue4* jval = reinterpret_cast<OJointCommandValue4*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumSlowFrames; frame++)
						jval[frame].value = (outputs[frame][i]<.5?ojoint4_STATE0:ojoint4_STATE1);
					used++;
				}
		} else {
			for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
				if(open[i]) {
					OLEDCommandValue2* jval = reinterpret_cast<OLEDCommandValue2*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].led = calcLEDValue(i-LEDOffset,outputs[frame][i]);
					used++;
				}
			for(unsigned int i=BinJointOffset; i<BinJointOffset+NumBinJoints; i++)
				if(open[i]) {
					OJointCommandValue3* jval = reinterpret_cast<OJointCommandValue3*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumSlowFrames; frame++)
						jval[frame].value = (outputs[frame][i]<.5?ojoint3_STATE1:ojoint3_STATE0);
					used++;
				}
		}
	}	else if(ProcessID::getID()==ProcessID::MainProcess) {
		// Just copy over the current ear state from WorldState
		unsigned int used=0; //but only copy open joints (so main does ears, motion does everything else)
		if(isERS7) {
			for(unsigned int i=BinJointOffset; i<BinJointOffset+NumBinJoints; i++)
				if(open[i]) {
					OJointCommandValue4* jval = reinterpret_cast<OJointCommandValue4*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumSlowFrames; frame++)
						jval[frame].value = (state->outputs[i]<.5?ojoint4_STATE0:ojoint4_STATE1);
					used++;
				}
		} else {
			for(unsigned int i=BinJointOffset; i<BinJointOffset+NumBinJoints; i++)
				if(open[i]) {
					OJointCommandValue3* jval = reinterpret_cast<OJointCommandValue3*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumSlowFrames; frame++)
						jval[frame].value = (state->outputs[i]<.5?ojoint3_STATE1:ojoint3_STATE0);
					used++;
				}
		}
	}

	// Send outputs to system
	subject[sbjMoveJoint]->SetData(rgn);

	// The first time this is called, we actually need to send *two* buffers
	// in order to get the double buffering going... (well, actually generalized
	// for NUM_COMMAND_VECTOR level buffering)
	static unsigned int initCount=1;
	if(initCount<NUM_COMMAND_VECTOR) {
		initCount++;
		ReadySendJoints(sysevent);
	} else //recursive base case
		subject[sbjMoveJoint]->NotifyObservers();

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during joint angle updates",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during joint angle updates",NULL))
		throw;
}
}

void
MMCombo::GotSensorFrame(const ONotifyEvent& event){
	MarkScope ep(entryPt);
try {
	erouter->processTimers();
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
		throw;
}
try {
	//	if(state && state->buttons[RFrPawOffset])
	//	cout << "SENSOR..."<<flush;
	if(isStopped) {
		//cout << "BAH!GotSensorFrame" << endl;
		return;
	}

#ifndef NO_PROFILING
	static unsigned int id=-1U;
	Profiler::Timer timer;
	if(ProcessID::getID()==ProcessID::MotionProcess) {
		if(id==-1U)
			id=motionProfiler->getNewID("GotSensorFrame()");
		timer.setID(id,&motionProfiler->prof);
	}	else if(ProcessID::getID()==ProcessID::MainProcess) {
		if(id==-1U)
			id=mainProfiler->getNewID("GotSensorFrame()");
		timer.setID(id,&mainProfiler->prof);
	}
#endif
	//TimeET t;

	OSensorFrameVectorData* rawsensor = reinterpret_cast<OSensorFrameVectorData*>(event.RCData(0)->Base());
	static unsigned int throwaway=1; //i thought the first few sensor updates might be flakey, but now i think not.  But a good way to delay startup.
	bool wasLastThrowAway=false;
	if(throwaway!=0) {
		if(--throwaway==0)
			wasLastThrowAway=true;
	}
	//cout << objectName << " got sensors " << rawsensor[0].GetInfo(0)->frameNumber << " at " << t << endl;
	if(state==NULL) {
		if(wasLastThrowAway)
			throwaway++; //postpone until worldstate comes through
		observer[obsSensorFrame]->AssertReady();
		//cout << "burning throwaway, left: " << throwaway << endl;
		return;
	}
	//cout << objectName << " writing " << wsw.frame << " state=" << state << "(" << wsw.bufUsed << ") source=" << wsw.src << "("<<wsw.srcRequest.bufUsed<<") status="<<wsw.getStatus() << " complete="<<wsw.getComplete() << endl;

	if(erouter!=NULL)
		state->read(rawsensor[0],erouter);

	// notify main of update, but only if there's no backlog
	etrans->encodeEvent(EventBase(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID,0,"SensorSouceID::UpdatedSID",1),true);
		
	observer[obsSensorFrame]->AssertReady();
	//	if(state && state->buttons[RFrPawOffset])
	//	cout << "done" << endl;

} catch(const std::exception& ex) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during sensor update processing",&ex))
		throw;
} catch(...) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during sensor update processing",NULL))
		throw;
}
}

void
MMCombo::GotImage(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	if(isStopped) {
		//cout << "BAH!GotImage" << endl;
		return;
	}

try {
	erouter->processTimers();
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
		throw;
}
try {
	PROFSECTION("GotImage()",*mainProfiler);
  
	WMvari(int, frame_counter, 0);
	++frame_counter;
	
	erouter->postEvent(DataEvent<const OFbkImageVectorData*>(reinterpret_cast<const OFbkImageVectorData*>(event.Data(0)),EventBase::visOFbkEGID,0,EventBase::activateETID));
	erouter->postEvent(DataEvent<const OFbkImageVectorData*>(reinterpret_cast<const OFbkImageVectorData*>(event.Data(0)),EventBase::visOFbkEGID,0,EventBase::statusETID));
	erouter->postEvent(DataEvent<const OFbkImageVectorData*>(reinterpret_cast<const OFbkImageVectorData*>(event.Data(0)),EventBase::visOFbkEGID,0,EventBase::deactivateETID));
	
  observer[obsImage]->AssertReady();

} catch(const std::exception& ex) {
  observer[obsImage]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during camera image processing",&ex))
		throw;
} catch(...) {
  observer[obsImage]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during camera image processing",NULL))
		throw;
}
try {
	erouter->processTimers();
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
		throw;
}
}

void
MMCombo::GotAudio(const ONotifyEvent& event){
	MarkScope ep(entryPt);
try {
	if(isStopped) {
		//cout << "BAH!GotAudio" << endl;
		return;
	}

	PROFSECTION("GotAudio()",*mainProfiler);

	for (int i = 0; i < event.NumOfData(); i++) {
		erouter->postEvent(DataEvent<const OSoundVectorData*>(reinterpret_cast<const OSoundVectorData*>(event.Data(i)),EventBase::micOSndEGID,0,EventBase::statusETID));
		try {
			erouter->processTimers();
		} catch(const std::exception& ex) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
				throw;
		} catch(...) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
				throw;
		}
	}
  
  observer[obsMic]->AssertReady();

} catch(const std::exception& ex) {
  observer[obsMic]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during audio processing",&ex))
		throw;
} catch(...) {
  observer[obsMic]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during audio processing",NULL))
		throw;
}
}

void
MMCombo::GotPowerEvent(void * msg){
	MarkScope ep(entryPt);
	if(isStopped) {
		//cout << "BAH!GotPowerEvent" << endl;
		return;
	}
try {
	erouter->processTimers();
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
		throw;
}
try {

	//	cout << "POWER..."<<flush;
	PROFSECTION("PowerEvent()",*mainProfiler);

	static bool first=true;
	if(first) {
		addRunLevel();
		first=false;
	}
	const OPowerStatus* result = &static_cast<OPowerStatusMessage*>(msg)->powerStatus;
	state->read(*result,erouter);
	// this part watches to see if the power button is pressed to shutdown the robot
	// i'm leaving this low-level because there's not much else you can do anyway...
	// the hardware kills power to the motors, and as far as we can tell, you can't
	// turn them back on.
	if(state->powerFlags[PowerSrcID::PauseSID]) {
		cout << "%%%%%%%  Pause button was pushed! %%%%%%%" << endl;
		OBootCondition bc(0);
		OPENR::Shutdown(bc);
	}
	//	cout << "done" << endl;

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during power status update",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during power status update",NULL))
		throw;
}
}

void
MMCombo::GotMotionMsg(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	if(isStopped) {
		//cout << "BAH!GotMotionMsg" << endl;
		return;
	}

	//	cout << "RECEIVE..."<<flush;
	if(motman!=NULL)
		motman->receivedMsg(event);
	else
		cout << "*** WARNING Main dropping MotionCommand (motman not ready) " << endl;
	observer[obsMotionManagerComm]->AssertReady();
	//	cout << "done" << endl;
}

void
MMCombo::GotSoundManager(const ONotifyEvent& event) {
	MarkScope ep(entryPt);
	cout << objectName << "-GOTSOUNDMANAGER..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	ASSERT(event.NumOfData()==1,"Too many SoundManagers");
	soundManagerMemRgn = event.RCData(0);
	soundManagerMemRgn->AddReference();
	sndman = reinterpret_cast<SoundManager*>(soundManagerMemRgn->Base());
	observer[obsReceiveSoundManager]->AssertReady();
	sndman->InitAccess(subject[sbjSoundManagerComm]);
	if(strcmp(objectName,"MainObj")==0) {
		addRunLevel();
	}
	cout << "done" << endl;
}

void
MMCombo::ReadyRegisterProcessMap(const OReadyEvent&){
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		cout << objectName << " Registering Process Map" << endl;
		if(strcmp(objectName,"MainObj")==0) {
			subject[sbjRegisterProcessMap]->SetData(processMapMemRgn);
			subject[sbjRegisterProcessMap]->NotifyObservers();
		}
	}
}

void
MMCombo::GotProcessMap(const ONotifyEvent& event){
try {
	cout << objectName << "-GOTPROCESSMAP..." << flush;
	//	PROFSECTION("GotMemRegion()",*mainProfiler);
	if(strcmp(objectName,"MotoObj")==0) {
		ASSERT(event.NumOfData()==1,"Too many ProcessMaps");
		processMapMemRgn = event.RCData(0);
		processMapMemRgn->AddReference();
		ProcessID::setMap(reinterpret_cast<stacktrace::StackFrame*>(processMapMemRgn->Base()));
		//doesn't handle setting map within entry point, have to do that part first
		MarkScope ep(entryPt);
	}
	observer[obsReceiveProcessMap]->AssertReady();
	cout << "done" << endl;
} catch(const std::exception& ex) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during GotProcessMap",&ex))
		throw;
} catch(...) {
  observer[obsSensorFrame]->AssertReady();
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during GotProcessMap",NULL))
		throw;
}
}

void
MMCombo::OpenPrimitives()
{
	for(unsigned int i=0; i<NumOutputs; i++)
		if(open[i]) {
			OStatus result = OPENR::OpenPrimitive(PrimitiveName[i], &primIDs[i]);
			if (result != oSUCCESS)
				OSYSLOG1((osyslogERROR, "%s : %s %d","MMCombo::DoInit()","OPENR::OpenPrimitive() FAILED", result));
		}
}

void
MMCombo::SetupOutputs(const bool to_open[NumOutputs])
{
	const std::set<unsigned int>& fakes = RobotInfo::capabilities.getFakeOutputs();
	for(unsigned int j=0; j<NumOutputs; j++)
		open[j] = to_open[j] && fakes.find(j)==fakes.end();
	
	// count how many we're opening
	for(unsigned int j=0; j<NumOutputs; j++)
		if(open[j])
			num_open++;

	if(num_open==0) //If we don't have any joints to open, leave now. (i.e. MainObj on a 220, has no ears, and on ERS-7, all joints are full speed)
		return;

	OpenPrimitives();

	// request memory regions
	for (unsigned int i = 0; i < NUM_COMMAND_VECTOR; i++) {
		MemoryRegionID      cmdVecDataID;
		OCommandVectorData* cmdVecData;
		OStatus result = OPENR::NewCommandVectorData(num_open,&cmdVecDataID,&cmdVecData);
		if (result != oSUCCESS)
			OSYSLOG1((osyslogERROR, "%s : %s %d","MMCombo::NewCommandVectorData()","OPENR::NewCommandVectorData() FAILED", result));
		region[i] = new RCRegion(cmdVecData->vectorInfo.memRegionID,cmdVecData->vectorInfo.offset,(void*)cmdVecData,cmdVecData->vectorInfo.totalSize);
		cmdVecData->SetNumData(num_open);

		// initialize the outputs we just opened
		unsigned int used=0;
		ASSERT(cmdVecData==reinterpret_cast<OCommandVectorData*>(region[i]->Base())," should be equal!?");
		for(unsigned int j=PIDJointOffset; j<PIDJointOffset+NumPIDJoints; j++)
			if(open[j]) {
				OCommandInfo* info = cmdVecData->GetInfo(used++);
				info->Set(odataJOINT_COMMAND2, primIDs[j], NumFrames);
			}
		if(RobotName == ERS7Info::TargetName) {
			// this part's the same as usual, except stop when we get to face leds
			for(unsigned int j=LEDOffset; j<ERS7Info::FaceLEDPanelOffset; j++)
				if(open[j]) {
					OCommandInfo* info = cmdVecData->GetInfo(used);
					info->Set(odataLED_COMMAND2, primIDs[j], NumFrames);
					OLEDCommandValue2* jval = reinterpret_cast<OLEDCommandValue2*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].period = 1;
					used++;
				}
			//we have to use OLEDCommandValue3 on the face and back LEDs if it's an ERS-7
			for(unsigned int j=ERS7Info::FaceLEDPanelOffset; j<LEDOffset+NumLEDs; j++)
				if(open[j]) {
					OCommandInfo* info = cmdVecData->GetInfo(used);
					info->Set(odataLED_COMMAND3, primIDs[j], NumFrames);
					OLEDCommandValue3* jval = reinterpret_cast<OLEDCommandValue3*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].period = 1;
					used++;
				}
			//also have to use OJointCommandValue4 on the ears now
			for(unsigned int j=BinJointOffset; j<BinJointOffset+NumBinJoints; j++)
				if(open[j]) {
					OCommandInfo* info = cmdVecData->GetInfo(used);
					info->Set(odataJOINT_COMMAND4, primIDs[j], NumSlowFrames);
					OJointCommandValue4* jval = reinterpret_cast<OJointCommandValue4*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].period = 1;
					used++;
				}
		} else {
			for(unsigned int j=LEDOffset; j<LEDOffset+NumLEDs; j++)
				if(open[j]) {
					OCommandInfo* info = cmdVecData->GetInfo(used);
					info->Set(odataLED_COMMAND2, primIDs[j], NumFrames);
					OLEDCommandValue2* jval = reinterpret_cast<OLEDCommandValue2*>(cmdVecData->GetData(used)->value);
					for(unsigned int frame=0; frame<NumFrames; frame++)
						jval[frame].period = 1;
					used++;
				}
			for(unsigned int j=BinJointOffset; j<BinJointOffset+NumBinJoints; j++)
				if(open[j]) {
					OCommandInfo* info = cmdVecData->GetInfo(used);
					info->Set(odataJOINT_COMMAND3, primIDs[j], NumSlowFrames);
					used++;
				}
		}
	}
}

/*! Will round up size to the nearest page */
RCRegion*
MMCombo::InitRegion(unsigned int size) {
	unsigned int pagesize=4096;
	sError err=GetPageSize(&pagesize);
	if(err!=sSUCCESS)
		cerr << "Error "<<err<<" getting page size " << pagesize << endl;
	unsigned int pages=(size+pagesize-1)/pagesize;
	return new RCRegion(pages*pagesize);
}

void
MMCombo::initRNG() {
	//seed the random number generator with time value and sensor noise
	if(config->main.seed_rng) {
		double tv=TimeET().Value(); //current time with nanosecond resolution
		unsigned int * tm=reinterpret_cast<unsigned int*>(&tv);
		unsigned int seed=tm[0]+tm[1];
		for(unsigned int i=0; i<NumPIDJoints; i++) { //joint positions
			unsigned int * x=reinterpret_cast<unsigned int*>(&state->outputs[i]);
			seed+=(*x)<<((i%sizeof(unsigned int))*8);
		}
		for(unsigned int i=0; i<NumPIDJoints; i++) { //joint forces
			unsigned int * x=reinterpret_cast<unsigned int*>(&state->pidduties[i]);
			seed+=(*x)<<((i%sizeof(unsigned int))*8);
		}
		for(unsigned int i=0; i<NumSensors; i++) {
			unsigned int * x=reinterpret_cast<unsigned int*>(&state->sensors[i]);
			seed+=(*x)<<((i%sizeof(unsigned int))*8); //sensor values
		}
		cout << ProcessID::getIDStr() << " RNG seed=" << seed << ", zignor seeds: " << *tm << ',' << seed << endl;
		srand(seed);
		RanNormalSetSeedZig32((int*)&seed,1);
		RanNormalSetSeedZig((int*)&seed,1);
	} else {
		int s=12345;
		RanNormalSetSeedZig32(&s,1);
		RanNormalSetSeedZig(&s,1);
	}
}

void
MMCombo::addRunLevel() {
	runLevel++;
	if(runLevel==readyLevel) {
		try {
			initRNG();
			cout << "START UP BEHAVIOR..." << flush;
			ProjectInterface::startupBehavior().start();
			cout << "START UP BEHAVIOR-DONE" << endl;
			erouter->postEvent(EventBase::runtimeEGID,0,EventBase::activateETID);
		} catch(const std::exception& ex) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during StartupBehavior construction and startup",&ex))
				throw;
		} catch(...) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during StartupBehavior construction and startup",NULL))
				throw;
		}
	}
}

void MMCombo::GainSettingListener::plistValueChanged(const plist::PrimitiveBase& pl) {
	typedef plist::NamedEnumeration<Config::vision_config::gain_levels> GainSetting;
	OPrimitiveID fbkID = 0;
	if(OPENR::OpenPrimitive(CameraLocator, &fbkID) != oSUCCESS){
		std::cout << "Open FbkImageSensor failure." << std::endl;
	} else if(const GainSetting* v=dynamic_cast<const GainSetting*>(&pl)) {
		OPrimitiveControl_CameraParam ogain(*v);
		if(OPENR::ControlPrimitive(fbkID, oprmreqCAM_SET_GAIN, &ogain, sizeof(ogain), 0, 0) != oSUCCESS)
			std::cout << "CAM_SET_GAIN : Failed!" << std::endl;
		OPENR::ClosePrimitive(fbkID);
	} else {
		std::cerr << "GainSettingListener got a value changed notification for a non-gain_levels primitive, ignoring..." << std::endl;
	}
}

void MMCombo::ShutterSettingListener::plistValueChanged(const plist::PrimitiveBase& pl) {
	typedef plist::NamedEnumeration<Config::vision_config::shutter_speeds> SpeedSetting;
	OPrimitiveID fbkID = 0;
	if(OPENR::OpenPrimitive(CameraLocator, &fbkID) != oSUCCESS){
		std::cout << "Open FbkImageSensor failure." << std::endl;
	} else if(const SpeedSetting* v=dynamic_cast<const SpeedSetting*>(&pl)) {
		OPrimitiveControl_CameraParam oshutter(*v);
		if(OPENR::ControlPrimitive(fbkID,oprmreqCAM_SET_SHUTTER_SPEED, &oshutter, sizeof(oshutter), 0, 0) != oSUCCESS)
			std::cout << "CAM_SET_SHUTTER_SPEED : Failed!" << std::endl;
		OPENR::ClosePrimitive(fbkID);
	} else {
		std::cerr << "ShutterSettingListener got a value changed notification for a non-shutter_speeds primitive, ignoring..." << std::endl;
	}
}

void MMCombo::WhiteBalanceSettingListener::plistValueChanged(const plist::PrimitiveBase& pl) {
	typedef plist::NamedEnumeration<Config::vision_config::white_balance_levels> WBSetting;
	OPrimitiveID fbkID = 0;
	if(OPENR::OpenPrimitive(CameraLocator, &fbkID) != oSUCCESS){
		std::cerr << "Open FbkImageSensor failure." << std::endl;
	} else if(const WBSetting* v=dynamic_cast<const WBSetting*>(&pl)) {
		OPrimitiveControl_CameraParam owb(*v);
		if(OPENR::ControlPrimitive(fbkID, oprmreqCAM_SET_WHITE_BALANCE, &owb, sizeof(owb), 0, 0) != oSUCCESS)
			std::cerr << "CAM_SET_WHITE_BALANCE : Failed!" << std::endl;
		OPENR::ClosePrimitive(fbkID);
	} else {
		std::cerr << "WhiteBalanceSettingListener got a value changed notification for a non-white_balance_levels primitive, ignoring..." << std::endl;
	}
}


/*! @file
 * @brief Implements MMCombo, the OObject which "forks" (sort of) into Main and Motion processes
 * @author ejt (Creator)
 */


