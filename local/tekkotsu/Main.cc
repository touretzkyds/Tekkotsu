#include "Main.h"
#include "SoundPlay.h"
#include "Motion.h"
#include "Simulator.h"
#include "TimerExecThread.h"
#include "SimConfig.h"
#include "MotionExecThread.h"

#include "IPC/RegionRegistry.h"
#include "IPC/MessageReceiver.h"
#include "IPC/FailsafeThread.h"
#include "IPC/PollThread.h"
#include "IPC/ThreadedMessageQueue.h"
#include "Motion/Kinematics.h"
#include "Motion/PostureEngine.h"
#include "Wireless/Wireless.h"
#include "Shared/ProjectInterface.h"
#include "Behaviors/BehaviorBase.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Shared/Config.h"
#include "Shared/MarkScope.h"
#include "Shared/WorldState.h"
#include "Events/EventBase.h"

using namespace std;

Main::Main()
	: Process(getID(),getClassName()),
	sounds(ipc_setup->registerRegion(SoundPlay::getSoundPlayID(),sizeof(sim::SoundPlayQueue_t))),
	motions(ipc_setup->registerRegion(Motion::getMotionCommandID(),sizeof(sim::MotionCommandQueue_t))),
	events(ipc_setup->registerRegion(getEventsID(),sizeof(sim::EventQueue_t))),
	cameraFrames(ipc_setup->registerRegion(Simulator::getCameraQueueID(),sizeof(sim::CameraQueue_t))),
	sensorFrames(ipc_setup->registerRegion(Simulator::getSensorQueueID(),sizeof(sim::SensorQueue_t))),
	timerWakeup(ipc_setup->registerRegion(Simulator::getTimerWakeupID(),sizeof(sim::TimerWakeup_t))),
	statusRequest(ipc_setup->registerRegion(Simulator::getStatusRequestID(),sizeof(sim::StatusRequest_t))),
	motionmanager(ipc_setup->registerRegion(Motion::getMotionManagerID(),sizeof(MotionManager))),
	soundmanager(ipc_setup->registerRegion(SoundPlay::getSoundManagerID(),sizeof(SoundManager))),
	motionProf(ipc_setup->registerRegion(Motion::getMotionProfilerID(),sizeof(motionProfiler_t))),
	soundProf(ipc_setup->registerRegion(SoundPlay::getSoundProfilerID(),sizeof(soundProfiler_t))),
	visrecv(NULL), sensrecv(NULL), evtrecv(NULL), timerrecv(NULL), statusrecv(NULL), timerExec(NULL),
	visionRead(true), wireless_thread(), worldStateCache(), behaviorLock(),
	curimgregion(NULL), img(), lastVisionSN(-1U)
{
	new (&(*events)) sim::EventQueue_t;
	motman=&(*motionmanager);
	sndman=&(*soundmanager);
	::mainProfiler=new mainProfiler_t;
	::motionProfiler=&(*motionProf);
	::soundProfiler=&(*soundProf);

	if(sim::config.multiprocess) {
		//Setup wireless
		ASSERT(wireless==NULL,"global wireless already initialized before Main?");
		wireless = new Wireless();
		sout=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*12);
		serr=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*4);
		wireless->setDaemon(sout);
		wireless->setDaemon(serr);
		serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);
		
		//Setup Kinematics
		ASSERT(kine==NULL,"global kine already initialized before Main?");
		kine=new Kinematics();
	}
	wireless->setCallbackLock(behaviorLock);

	//EventRouter and Config are set up for all processes by main() before fork
}

Main::~Main() {
	if(sim::config.multiprocess) {
		delete wireless;
		wireless=NULL;
		delete kine;
		kine=NULL;
	}
}


void Main::doStart() {
try {
	Process::doStart();
	//These are constructed by other processes, so need to wait
	//until the construction runlevel is complete
	sndman->InitAccess(*sounds);
	motman->InitAccess(*motions,behaviorLock);
	
	// initialize output feedback from initial pose loaded by Simulator constructor
	if(globals->motion.startPose.size()>0) {
		for(unsigned int i=0; i<NumOutputs; ++i)
			state->outputs[i]=globals->sensorState.outputs[i];
	}

	wireless->listen(sout, config->main.console_port);
	wireless->listen(serr, config->main.stderr_port);
	wireless_thread.start();
	statusrecv=new MessageReceiver(*statusRequest,statusReport);

	if(globals->waitForSensors)
		erouter->addListener(this,EventBase::sensorEGID);
	evtrecv=new MessageReceiver(*events,gotEvent,false);
	visrecv=new MessageReceiver(*cameraFrames,gotCamera,false);
	sensrecv=new MessageReceiver(*sensorFrames,gotSensors,false);
	timerrecv=new MessageReceiver(*timerWakeup,gotTimer,false);
	timerExec=new TimerExecThread(behaviorLock,false);
	erouter->serveRemoteEventRequests();
	erouter->getEventQueue().spawnCallback(&Main::gotThreadedEvent,*this);
} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main doStart",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main doStart",NULL))
		throw;
}
}

void Main::run() {
try {
	evtrecv->start();
	if(globals->waitForSensors) {
		sensrecv->start();
		CallbackThread waitSensors(&SharedGlobals::waitSensors,*globals,true);
		FailsafeThread failsafe(waitSensors,2.0,true);
		waitSensors.join(); // block until either we get the sensors or failsafe cuts in
		if(globals->waitForSensors && !globals->haveSensors())
			std::cerr << "Blocked on first sensor reading (WaitForSensors is true)... is a driver wedged?" << std::endl;
		// block for real now... either user unsets WaitForSensors flag and Simulator sends the signal, or we wait until shut down.
		globals->waitSensors();
	}

	// might have shutdown triggered while waiting
	// (perhaps device isn't available, user's killing the process...)
	if(globals->isShutdown())
		return; // skip running altogether
	
	{
		MarkScope bl(behaviorLock);
		ProjectInterface::startupBehavior().start();
		globals->setNextTimer(erouter->getNextTimer());
	}
	
	if(!globals->waitForSensors)
		if(!sensrecv->isStarted()) // this could happen if we originally blocked on waitForSensors and then user cleared the configuration flag
			sensrecv->start();
	visrecv->start();
	timerrecv->start();
	timerExec->reset();
	
	{
		MarkScope bl(behaviorLock);
		erouter->postEvent(EventBase::runtimeEGID,0,EventBase::activateETID);
	}
	
	// this is a bit of a hack, but once we're done launching, display the prompt:
	cout << sim::config.cmdPrompt << flush;
	
	Process::run();

	{
		MarkScope bl(behaviorLock);
		erouter->postEvent(EventBase::runtimeEGID,0,EventBase::deactivateETID);
	}
	sensrecv->finish();
	visrecv->finish();
	evtrecv->finish();
	timerrecv->finish();
	erouter->getEventQueue().finishCallback();
	
	{
		MarkScope bl(behaviorLock);
		ProjectInterface::startupBehavior().stop();
		globals->setNextTimer(erouter->getNextTimer());
	}
	timerExec->reset();

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main 'run' runlevel (startupBehavior initialization and startup)",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main 'run' runlevel (startupBehavior initialization and startup)",NULL))
		throw;
}
}

void Main::doStop() {
try {

	delete sensrecv;
	sensrecv=NULL;
	delete visrecv;
	visrecv=NULL;
	delete evtrecv;
	evtrecv=NULL;
	delete timerrecv;
	timerrecv=NULL;
	delete statusrecv;
	statusrecv=NULL;
	
	motman->RemoveAccess();
	
	if(curimgregion!=NULL)
		curimgregion->RemoveReference();

	wireless_thread.stop();
	wireless_thread.join();
	wireless->setDaemon(sout,false);
	wireless->close(sout);
	sout=NULL;
	wireless->setDaemon(serr,false);
	wireless->close(serr);
	serr=NULL;
	
	Process::doStop();

} catch(const std::exception& ex) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main doStop",&ex))
		throw;
} catch(...) {
	if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during Main doStop",NULL))
		throw;
}
}

void Main::processEvent(const EventBase&) {
	if(state->frameNumber==0)
		return; // heartbeats don't count for signalHaveSensors()
	erouter->removeListener(this);
	for(unsigned int i=0; i<NumOutputs; i++)
		motman->setOutput(NULL,i,state->outputs[i]);
	globals->signalHaveSensors();
}

bool Main::gotCamera(RCRegion* msg) {
	if(msg==NULL)
		return true;
	Main * main=dynamic_cast<Main*>(Process::getCurrent());
	ASSERTRETVAL(main!=NULL,"gotCamera, but not within Main process!",true);
	MarkScope bl(main->behaviorLock);
	PROFSECTION("GotImage()",*mainProfiler);

	try {
		BufferedImageGenerator::ImageSource& img=main->img;
		
		DataSource::ImageHeader* header = reinterpret_cast<DataSource::ImageHeader*>(msg->Base());
		char* buf=msg->Base()+sizeof(*header);
		unsigned int remain=header->width * header->height * header->components;
		if(globals->vision.verbose>=1 && header->frameNumber-main->lastVisionSN!=1)
			cout << "Main dropped " << (header->frameNumber-main->lastVisionSN-1) << " camera frame(s)" << endl;
		main->lastVisionSN=header->frameNumber;
		if(globals->vision.verbose>=3 && remain==0)
			cout << "Main received image heartbeat at " << get_time() << endl;
		else if(globals->vision.verbose>=2 && remain!=0)
		  cout << "Main received image data \"" << header->name << "\" at " << get_time() << " with source: " << header->sourceID << endl;
		
		img.frameIndex=header->frameNumber;
		if(remain==0) {
			if(img.width==0 || img.height==0 || img.img==NULL)
				return true; // can't do the heartbeat, don't have an initial image to replicate
		} else {
			img.width=header->width;
			img.height=header->height;
			img.channels=header->components;
			if(header->layer!=0) {
				if(header->layer<0)
					img.layer = ProjectInterface::defRawCameraGenerator->getNumLayers()+header->layer;
				else
					img.layer = header->layer-1; // have ensured layer >=1 due to outer conditional
			} else {
				// using "automatic" mode, pick the layer closest to resolution of provided image
				// assumes each layer doubles in size, with smallest layer at 0
				float fullRes=std::sqrt((float)(CameraResolutionX*CameraResolutionY)); // from RobotInfo
				float givenRes=std::sqrt((float)(img.width*img.height));
				if(givenRes==0) {
					cerr << "Main received empty image!" << endl;
					return true;
				} else {
					float ratio=log2f(givenRes/fullRes);
					int layerOff=static_cast<int>(rintf(ratio));
					int tgtLayer=static_cast<int>(ProjectInterface::fullLayer)+layerOff;
					if(tgtLayer<0)
						img.layer=0;
					else if(ProjectInterface::defRawCameraGenerator!=NULL && static_cast<unsigned int>(tgtLayer)>=ProjectInterface::defRawCameraGenerator->getNumLayers())
						img.layer=ProjectInterface::defRawCameraGenerator->getNumLayers()-1;
					else
						img.layer=tgtLayer;
					if(static_cast<unsigned int>(tgtLayer)!=img.layer)
						cerr << "Image dimensions of " << img.width << "x" << img.height << " are well beyond the available resolution layers (full is " << CameraResolutionX << "x" << CameraResolutionY << ")" << endl;
				}
			}
			img.img=reinterpret_cast<unsigned char*>(buf);
			msg->AddReference();
			if(main->curimgregion!=NULL)
				main->curimgregion->RemoveReference();
			main->curimgregion=msg;
		}
		DataEvent<BufferedImageGenerator::ImageSource> dev(img,EventBase::visOFbkEGID,header->sourceID,EventBase::activateETID);
		erouter->postEvent(dev);
		dev.setTypeID(EventBase::statusETID);
		erouter->postEvent(dev);
		dev.setTypeID(EventBase::deactivateETID);
		erouter->postEvent(dev);

	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during camera frame processing",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during camera frame processing",NULL))
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
	if(globals->setNextTimer(erouter->getNextTimer()))
		main->timerExec->reset();
	return true;
}

bool Main::gotSensors(RCRegion* /*msg*/) {
	Main * main=dynamic_cast<Main*>(Process::getCurrent());
	ASSERTRETVAL(main!=NULL,"gotSensors, but not within Main process!",true);

	PROFSECTION("GotSensorFrame()",*mainProfiler);
	MarkScope l(main->behaviorLock);

	try {
		MarkScope sensorLock(globals->sensorState);
		unsigned int frameInc = globals->sensorState.frameNumber - state->frameNumber;
		if(frameInc>2 && globals->sensors.verbose>=1)
			std::cout << "Main dropped " << (frameInc-1) << " sensor updates... " << std::endl;
		if(globals->sensors.verbose>=4)
			std::cout << "Processing " << (frameInc==0 && state->frameNumber!=0 ? "duplicate " : "") << "sensor update #" << globals->sensorState.frameNumber << " @" << get_time() << "... ";
		state->read(globals->sensorState,true);
		if(globals->sensors.verbose>=4)
			std::cout << "sensor processing completed @" << get_time() << std::endl;
	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during sensor processing",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during sensor processing",NULL))
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
	if(globals->setNextTimer(erouter->getNextTimer()))
		main->timerExec->reset();
	return true;
}

bool Main::gotEvent(RCRegion* msg) {
	if(msg==NULL)
		return true;
	Main * main=dynamic_cast<Main*>(Process::getCurrent());
	ASSERTRETVAL(main!=NULL,"gotEvent, but not within Main process!",true);
	MarkScope l(main->behaviorLock);
	EventBase* evt=NULL;
	try {
		evt=EventTranslator::decodeEvent(msg->Base(),msg->Size());
		if(evt==NULL) {
			cerr << "ERROR: Main::gotEvent() failed to decode message" << endl;
			return true;
		}
		erouter->postEvent(*evt);
	} catch(const std::exception& ex) {
		std::string emsg("Occurred during inter-process event processing");
		if(evt!=NULL)
			emsg+=": "+evt->getName();
		delete evt;
		evt=NULL;
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,emsg.c_str(),&ex))
			throw;
	} catch(...) {
		std::string emsg("Occurred during inter-process event processing");
		if(evt!=NULL)
			emsg+=": "+evt->getName();
		delete evt;
		evt=NULL;
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,emsg.c_str(),NULL))
			throw;
	}
	delete evt;
	evt=NULL;
	try {
		erouter->processTimers();
	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
			throw;
	}
	if(globals->setNextTimer(erouter->getNextTimer()))
		main->timerExec->reset();
	return true;
}

bool Main::gotTimer(RCRegion* /*msg*/) {
	Main * main=dynamic_cast<Main*>(Process::getCurrent());
	ASSERTRETVAL(main!=NULL,"gotTimer, but not within Main process!",true);

	ASSERTRETVAL(main->timerExec!=NULL,"timerExec thread is NULL when timer wakeup received",true);
	main->timerExec->reset();
	
	if(globals->timeScale<=0) {
		MarkScope bl(main->behaviorLock);
		try {
			erouter->processTimers();
		} catch(const std::exception& ex) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
				throw;
		} catch(...) {
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
				throw;
		}
		globals->setNextTimer(erouter->getNextTimer());
		// don't need to reset timerExec (again) because we're only here if speed<=0, in which case timerExec isn't running	main->timerExec->reset();
	}
	return true;
}

void Main::gotThreadedEvent(EventBase* evt) {
#ifdef DEBUG
	Main * main=dynamic_cast<Main*>(Process::getCurrent());
	ASSERTRET(main!=NULL,"gotThreadedEvent, but not within Main process!");
#endif
	// We (the callback) are responsible for deleting this event.
	MarkScope l(behaviorLock);
	try {
		erouter->postEvent(*evt);
	} catch(const std::exception& ex) {
		std::string emsg("Occurred during queued/inter-thread event processing");
		if(evt!=NULL)
			emsg+=": "+evt->getName();
		delete evt;
		evt=NULL;
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,emsg.c_str(),&ex))
			throw;
	} catch(...) {
		std::string emsg("Occurred during queued/inter-process event processing");
		if(evt!=NULL)
			emsg+=": "+evt->getName();
		delete evt;
		evt=NULL;
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,emsg.c_str(),NULL))
			throw;
	}
	delete evt;
	evt=NULL;
	try {
		erouter->processTimers();
	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during timer processing",NULL))
			throw;
	}
	if(globals->setNextTimer(erouter->getNextTimer()))
		timerExec->reset();
}

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

