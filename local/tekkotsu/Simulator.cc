#include "Simulator.h"
#include "Main.h"
#include "Motion.h"
#include "SoundPlay.h"
#include "sim.h"
#include "Shared/string_util.h"
#include "Shared/RobotInfo.h"
#include "SimConfig.h"
#include "Shared/debuget.h"
#include "Shared/MarkScope.h"
#include "IPC/MessageReceiver.h"
#include "IPC/RegionRegistry.h"
#include "IPC/FailsafeThread.h"
#include "local/DataSources/FileSystemDataSource.h"
#include "local/DataSources/FileSystemImageSource.h"
#include "local/CommPort.h"
#include "local/DeviceDriver.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Motion/PostureEngine.h"
#include "local/DataSources/SensorStateAccessor.h"

#include <iostream>
#include <iterator>
#include <libxml/tree.h>

#ifndef DISABLE_READLINE
#  include <readline/readline.h>
#  include <readline/history.h>
#  include <dlfcn.h> // to test if we actually have BSD's libedit emulation
#endif

using namespace std;

const float Simulator::avgSpeedupGamma=.99f;

Simulator::MotionMonitorThread * Simulator::motionHookMonitor=NULL;
Simulator* Simulator::theSim=NULL;
std::set<MotionHook*> Simulator::motionHooks;

Simulator::Simulator()
: Process(getID(),getClassName()), plist::PrimitiveListener(), plist::CollectionListener(), MessageQueueStatusThread::StatusListener(),
frameCounter(), cmdThread(),
sensorThread(this,&Simulator::sendSensor), sendSensorSent(false), 
cameraQueue(ipc_setup->registerRegion(Simulator::getCameraQueueID(),sizeof(sim::CameraQueue_t))),
sensorQueue(ipc_setup->registerRegion(Simulator::getSensorQueueID(),sizeof(sim::SensorQueue_t))),
timerWakeup(ipc_setup->registerRegion(Simulator::getTimerWakeupID(),sizeof(sim::TimerWakeup_t))),
motionWakeup(ipc_setup->registerRegion(Simulator::getMotionWakeupID(),sizeof(sim::MotionWakeup_t))),
statusRequest(ipc_setup->registerRegion(Simulator::getStatusRequestID(),sizeof(sim::StatusRequest_t))),
soundmanager(ipc_setup->registerRegion(SoundPlay::getSoundManagerID(),sizeof(SoundManager))),
sounds(ipc_setup->registerRegion(SoundPlay::getSoundPlayID(),sizeof(sim::SoundPlayQueue_t))),
events(ipc_setup->registerRegion(Main::getEventsID(),sizeof(sim::EventQueue_t))),
motionout(ipc_setup->registerRegion(Motion::getMotionOutputID(),sizeof(sim::MotionOutput_t))),
motionoutpids(ipc_setup->registerRegion(Motion::getMotionOutputPIDsID(),sizeof(sim::MotionOutputPIDs_t))),
commandQueue(ipc_setup->registerRegion(Simulator::getCommandQueueID(),sizeof(CommandQueue_t))),
cameraStatus(*cameraQueue), sensorStatus(*sensorQueue), timerStatus(*timerWakeup), motionStatus(*motionWakeup), eventsStatus(), etrans(NULL), commandrecv(NULL), motionrecv(NULL), motionpidsrecv(NULL),
frameTimes(), runSpeed(1), lastTimeScale(0), step(STEP_NONE), waitingSteps(0), curLevel(SharedGlobals::CONSTRUCTING),
activeSensors(), activeSensorSrcs(), activeCameras(), activeCameraSrcs(),
fullspeedWallStart(), fullspeedSimStart(), lastFrameWallStart(), avgWallTime(), avgSimTime(),
simLock()
{
	theSim=this;
	new (&(*cameraQueue)) sim::CameraQueue_t;
	new (&(*sensorQueue)) sim::SensorQueue_t;
	new (&(*timerWakeup)) sim::TimerWakeup_t;
	new (&(*motionWakeup)) sim::MotionWakeup_t;
	new (&(*statusRequest)) sim::StatusRequest_t;
	new (&(*commandQueue)) CommandQueue_t;
	statusRequest->setOverflowPolicy(MessageQueueBase::WAIT);
	commandQueue->setOverflowPolicy(MessageQueueBase::WAIT);
	sndman=&(*soundmanager);
	
	DataSource::setSensorFramerate(&globals->sensors.framerate);
	
	/* Since we do a two stage loading (some stuff in sim.cc launcher, then some command line
	 *  stuff, then the rest of it now that we're loaded), this is a little tricker than it normally would be. */
	// don't remove entries from parse tree, we're going to be adding more and want their values...
	sim::config.setSavePolicy(plist::Collection::UNION);
	// write out the current values for entries we do have, which may have been modified since originally loaded
	sim::config.writeParseTree();
	// add communication ports (could just get away with addEntry()'s, but this function is a little more robust)
	replaceEntry("CommPorts",CommPort::getRegistry(),"Communication portals for use by device drivers");
	// reload from parse tree to get values for these new entries
	sim::config.readParseTree();
	// now add the rest of our entries - we wanted to make sure the CommPorts get set up first so drivers init more easily
	replaceEntry("Drivers",DeviceDriver::getRegistry(),"Settings for device drivers");
	// now we're done adding entries, so if there's anything extra in the file, make note of it
	sim::config.setUnusedWarning(true);
	// reload from parse tree to get values for these new entries
	sim::config.readParseTree();
	// any future saves should be strict and remove those unused values (...maybe?)
	sim::config.setSavePolicy(plist::Collection::SYNC);
	
	motionWakeup->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
	timerWakeup->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
	cameraQueue->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
	sensorQueue->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
	sensorQueue->addMessageFilter(frameCounter);
	
	//handle arguments passed from command line
	list<vector<string> > delayed; // will delay setting data queue sources until after other arguments
	for(unsigned int i=0; i<sim::cmdlineArgs.size(); i++) {
		// this special handling shouldn't really be necessary, but avoids extra resetting and open/closing during initialization
		if(sim::cmdlineArgs[i].find(".Source=")!=string::npos) {
			vector<string> setarg;
			setarg.push_back("set");
			setarg.push_back(sim::cmdlineArgs[i]);
			delayed.push_back(setarg); // delay setting of source until after options like Frozen or Speed have been set
		} else if(sim::cmdlineArgs[i].find(".CommPort=")!=string::npos) {
			vector<string> setarg;
			setarg.push_back("set");
			setarg.push_back(sim::cmdlineArgs[i]);
			delayed.push_front(setarg); // delay setting of commports, but before 'Source's
		} else {
			if(!processCommand(sim::cmdlineArgs[i],false))
				cerr << "Occurred while processing " << sim::cmdlineArgs[i] << endl;
		}
	}
	for(list<vector<string> >::const_iterator it=delayed.begin(); it!=delayed.end(); ++it) {
		//this avoids dropping initial frame(s)
		//if we had set the source before freezing or pausing, might've dropped something between them
		if(!cmdSet(*it))
			cerr << "Occurred while processing " << (*it)[1] << endl;
	}
	
	// if we were supposed to start up paused, reset clock now that we've set the speed
	if(globals->timeScale<=0)
		globals->resetBootTime();
	
	// load startup posture, see also Main and Motion pull defaults from globals->sensorState
	if(globals->motion.startPose.size()>0) {
		PostureEngine pose(globals->motion.startPose);
		for(unsigned int i=0; i<NumOutputs; ++i)
			state->outputs[i]=globals->sensorState.outputs[i]=pose(i).value;
	}
	
	globals->motion.verbose.addPrimitiveListener(this);
	globals->sensors.sources.addCollectionListener(this);
	globals->vision.sources.addCollectionListener(this);
	// we'll start listening to globals->timeScale once we hit run()
	
	cmdThread.start();
	
	if(sim::config.tgtRunlevel>SharedGlobals::RUNNING)
		globals->signalShutdown();
	processRunlevel(SharedGlobals::CONSTRUCTING);
}

Simulator::~Simulator() {
	curLevel=SharedGlobals::DESTRUCTING;
	globals->vision.sources.clear();
	globals->sensors.sources.clear();
	DeviceDriver::getRegistry().clear();
	CommPort::getRegistry().clear();
	DeviceDriver::getRegistry().removeCollectionListener(this);
	globals->motion.verbose.removePrimitiveListener(this);
	globals->vision.sources.removeCollectionListener(this);
	globals->sensors.sources.removeCollectionListener(this);
	globals->sensorState.resourceSync = NULL;
	DataSource::setSensorFramerate(NULL);
	processRunlevel(SharedGlobals::DESTRUCTING);
	//::close(STDIN_FILENO); // seems to prevent proper terminal reset on exit...
	theSim=NULL;
}

static void driversChangedFunctor(Simulator* s) { s->plistCollectionEntriesChanged(DeviceDriver::getRegistry()); }

void Simulator::doStart() {
	curLevel=SharedGlobals::STARTING;
	Process::doStart();
	// this is needed mainly on Mac, as audio callbacks are processed in Simulator group (can't seem to control that)
	sndman->InitAccess(*sounds);
	
	motionHookMonitor = new MotionMonitorThread;
	
	eventsStatus.setMessageQueue(*events);
	if(!sim::config.multiprocess) {
		// don't use our own etrans here, because erouter will delete it for us, don't want a double-delete in our destructor...
		EventTranslator * forwardTrans = new IPCEventTranslator(*events);
		forwardTrans->setTrapEventValue(true);
		erouter->setForwardingAgent(getID(),forwardTrans);
	} else {
		etrans=new IPCEventTranslator(*events);
		MotionManager::setTranslator(etrans); //although Simulator shouldn't use any motions...
		
		// Set up Event Translator to trap and send events to main process
		//send everything over except erouter events
		for(unsigned int i=0; i<EventBase::numEGIDs; i++)
			if(i!=EventBase::erouterEGID)
				erouter->addTrapper(etrans,static_cast<EventBase::EventGeneratorID_t>(i));
	}
	commandrecv = new MessageReceiver(*commandQueue, gotCommand);
	
	// do this now that the settings have been processed, this is when data sources get their registerSource() called
	DeviceDriver::getRegistry().addCollectionListener(this);
	
	// abortable lets us abort and quit if a driver gets hung up
	abortable(&driversChangedFunctor,this);

	processRunlevel(SharedGlobals::STARTING);
}

void Simulator::run() {
	curLevel=SharedGlobals::RUNNING;
	if(sim::config.multiprocess) {
		for(unsigned int i=0; i<ProcessID::NumProcesses; ++i)
			cout << globals->processNames[i] << " pid=" << globals->pids[i] << ";  ";
		cout << endl;
	}
	
	motionrecv = new MessageReceiver(*motionout, gotMotion);
	motionpidsrecv = new MessageReceiver(*motionoutpids, gotMotionPIDs);
	
	if(globals->timeScale!=0)
		runSpeed=globals->timeScale;
	
	DataSource::setNeedsSensor(globals->waitForSensors);
	if(!globals->waitForSensors) {
		resetSpeedMode();
	} else {
		globals->waitForSensors.addPrimitiveListener(this);
		if(globals->sensors.sources.size()==0) {
			cout << "WARNING: Ignoring WaitForSensors configuration flag because Sensors has no sources" << endl;
			globals->signalHaveSensors();
			resetSpeedMode();
		} else if(0==std::count_if(motionHooks.begin(),motionHooks.end(),std::mem_fun(&MotionHook::isConnected))) {
			cout << "WARNING: Ignoring WaitForSensors configuration flag because there are no connected MotionHooks" << endl;
			globals->signalHaveSensors();
			resetSpeedMode();
		} else {
			if(activeSensorSrcs.size()==0) {
				std::cout << "WARNING: WaitForSensors is true, but all Sensors.Sources are invalid (";
				for(plist::ArrayOf<plist::Primitive<std::string> >::const_iterator it=globals->sensors.sources.begin(); it!=globals->sensors.sources.end(); ++it) {
					if(it!=globals->sensors.sources.begin())
						std::cout << ", ";
					std::cout << **it;
				}
				std::cout << ")\n         Instantiate the missing drivers, or set WaitForSensors=false to continue." << std::endl;
			}
			if(globals->timeScale>0) {
				resetSpeedMode();
				abortable(&SharedGlobals::waitSensors,*globals);
			} else {
				for(std::set<DataSource*>::const_iterator it=activeSensorSrcs.begin(); it!=activeSensorSrcs.end(); ++it)
					abortable(&DataSource::advance,**it);
				sendSensorSent=false; // clear the flag to make sure we send since we're not making non-sync calls to sendSensor
				sendSensor(true);
				while(!globals->haveSensors() && !globals->isShutdown()) {
					CallbackThread w4s(&SharedGlobals::waitSensors,*globals,true);
					FailsafeThread fs(w4s,0.5,true);
					if(w4s.join()==Thread::CANCELLED && 0==std::count_if(motionHooks.begin(),motionHooks.end(),std::mem_fun(&MotionHook::isConnected))) {
						cout << "WARNING: Ignoring WaitForSensors configuration flag because there are no longer any connected MotionHooks" << endl;
						globals->signalHaveSensors();
					}
				}
				resetSpeedMode();
			 }
		}
		globals->waitForSensors.removePrimitiveListener(this);
	}
	DataSource::setNeedsSensor(false);

	if(globals->timeScale<0)
		incrementTime();
	globals->timeScale.addPrimitiveListener(this);
	
	if(sim::config.tgtRunlevel==SharedGlobals::RUNNING)
		Process::run();
	
	globals->timeScale.removePrimitiveListener(this);
	if(globals->timeScale<0) {
		motionStatus.removeStatusListener(this);
		timerStatus.removeStatusListener(this);
		sensorStatus.removeStatusListener(this);
		cameraStatus.removeStatusListener(this);
	}
	if(sensorThread.isStarted())
		sensorThread.stop().join();
	if(globals->timeScale!=0)
		setMotionLeavingRealtime(false);
	globals->signalShutdown();
}

void Simulator::plistValueChanged(const plist::PrimitiveBase& pl) {
	MarkScope l(simLock);
	if(&pl==&globals->timeScale) {
		get_time(); // force SharedGlobals to notice the change and update its state
		resetSpeedMode();
		if(globals->timeScale<0)
			incrementTime();
		timerWakeup->sendMessage(NULL);
		motionWakeup->sendMessage(NULL);
	} else if(&pl==&globals->motion.verbose) {
		for(std::set<MotionHook*>::iterator it=motionHooks.begin(); it!=motionHooks.end(); ++it)
			(*it)->setMotionHookVerbose(globals->motion.verbose);
	} else if(&pl==&globals->waitForSensors) {
		cout << "Aborting block on WaitForSensors flag" << endl;
		DataSource::setNeedsSensor(false);
		globals->signalHaveSensors(); // unblock everything waiting on the signal
	} else if(std::count(globals->sensors.sources.begin(),globals->sensors.sources.end(),&pl)) {
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
	} else if(std::count(globals->vision.sources.begin(),globals->vision.sources.end(),&pl)) {
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else {
		cerr << "WARNING: Simulator got a plistValueChanged for an unknown plist primitive";
	}
}
void Simulator::plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& primitive) {
	MarkScope l(simLock);
	if(&col==&DeviceDriver::getRegistry()) {
		if(DeviceDriver * d=dynamic_cast<DeviceDriver*>(&primitive)) {
			if(MotionHook * mh = d->getMotionSink()) {
				motionHookMonitor->motionCheckTime.Set();
				ASSERT(motionHooks.find(mh)==motionHooks.end(),"new DeviceDriver already has active MotionHook?");
				mh->setMotionHookVerbose(globals->motion.verbose);
				if(curLevel==SharedGlobals::RUNNING || curLevel==SharedGlobals::STARTING) {
					motionHookMonitor->curFuncName="motionStarting()"; motionHookMonitor->curHook=mh;
					mh->motionStarting();
					motionHookMonitor->curHook=NULL;
				}
				if(curLevel==SharedGlobals::RUNNING) {
					if(globals->timeScale>0) {
						motionHookMonitor->curFuncName="enteringRealtime()"; motionHookMonitor->curHook=mh;
						mh->enteringRealtime(globals->timeScale);
					} else {
						motionHookMonitor->curFuncName="leavingRealtime()"; motionHookMonitor->curHook=mh;
						mh->leavingRealtime(globals->timeScale<0);
					}
					motionHookMonitor->curHook=NULL;
				}
				motionHooks.insert(mh);
			}
		} else {
			cerr << "WARNING: Simulator got a plistCollectionEntryAdded for an unknown primitive type";
		}
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else if(&col==&globals->sensors.sources) {
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
	} else if(&col==&globals->vision.sources) {
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else {
		cerr << "WARNING: Simulator got a plistCollectionEntryAdded for an unknown plist collection";
	}
}
void Simulator::plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& primitive) {
	MarkScope l(simLock);
	if(&col==&DeviceDriver::getRegistry()) {
		if(DeviceDriver * d=dynamic_cast<DeviceDriver*>(&primitive)) {
			if(MotionHook * mh = d->getMotionSink()) {
				motionHooks.erase(mh);
				motionHookMonitor->motionCheckTime.Set();
				if(curLevel==SharedGlobals::RUNNING && globals->timeScale!=0) {
					motionHookMonitor->curFuncName="leavingRealtime()"; motionHookMonitor->curHook=mh;
					mh->leavingRealtime(false);
					motionHookMonitor->curHook=NULL;
				}
				if(curLevel==SharedGlobals::STARTING || curLevel==SharedGlobals::RUNNING) {
					motionHookMonitor->curFuncName="motionStopping()"; motionHookMonitor->curHook=mh;
					mh->motionStopping();
					motionHookMonitor->curHook=NULL;
				}
			}
		} else {
			cerr << "WARNING: Simulator got a plistCollectionEntryRemoved for an unknown primitive type";
		}
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else if(&col==&globals->sensors.sources) {
		dynamic_cast<plist::PrimitiveBase&>(primitive).removePrimitiveListener(this);
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
	} else if(&col==&globals->vision.sources) {
		dynamic_cast<plist::PrimitiveBase&>(primitive).removePrimitiveListener(this);
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else {
		cerr << "WARNING: Simulator got a plistCollectionEntryRemoved for an unknown plist collection";
	}
}
void Simulator::plistCollectionEntriesChanged(plist::Collection& col) {
	MarkScope l(simLock);
	if(&col==&DeviceDriver::getRegistry()) {
		for(DeviceDriver::registry_t::const_iterator it=DeviceDriver::getRegistry().begin(); it!=DeviceDriver::getRegistry().end(); ++it) {
			if(DeviceDriver * d=DeviceDriver::getRegistry().getInstance(it->first)) {
				if(MotionHook * mh = d->getMotionSink()) {
					if(motionHooks.find(mh)!=motionHooks.end())
						continue; // already activated
					motionHookMonitor->motionCheckTime.Set();
					mh->setMotionHookVerbose(globals->motion.verbose);
					if(curLevel==SharedGlobals::RUNNING || curLevel==SharedGlobals::STARTING) {
						motionHookMonitor->curFuncName="motionStarting()"; motionHookMonitor->curHook=mh;
						mh->motionStarting();
						motionHookMonitor->curHook=NULL;
					}
					if(curLevel==SharedGlobals::RUNNING) {
						if(globals->timeScale>0) {
							motionHookMonitor->curFuncName="enteringRealtime()"; motionHookMonitor->curHook=mh;
							mh->enteringRealtime(globals->timeScale);
						} else {
							motionHookMonitor->curFuncName="leavingRealtime()"; motionHookMonitor->curHook=mh;
							mh->leavingRealtime(globals->timeScale<0);
						}
						motionHookMonitor->curHook=NULL;
					}
					motionHooks.insert(mh);
				}
			} else {
				cerr << "WARNING: In Simulator::plistCollectionEntriesChanged, driver " << it->first << " does not correspond to a known instance" << endl;
			}
		}
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else if(&col==&globals->sensors.sources) {
		updateDataSources(activeSensors, activeSensorSrcs, globals->sensors.sources, &DeviceDriver::getSensorSources);
	} else if(&col==&globals->vision.sources) {
		updateDataSources(activeCameras, activeCameraSrcs, globals->vision.sources, &DeviceDriver::getImageSources);
	} else {
		cerr << "WARNING: Simulator got a plistCollectionEntriesChanged for an unknown plist collection";
	}
}

void Simulator::messagesRead(MessageQueueBase& mq, unsigned int /*n*/) {
	MarkScope l(simLock);
	if(globals->timeScale<0) {
		//clear corresponding bit in waitingSteps
		 if(&mq==&(*cameraQueue)) {
			//cout << "Camera read, ";
			waitingSteps&=~(1<<STEP_CAMERA);
		} else if(&mq==&(*sensorQueue)) {
			//cout << "Sensor read, ";
			waitingSteps&=~(1<<STEP_SENSOR);
		} else if(&mq==&(*timerWakeup)) {
			//cout << "Timer read, ";
			waitingSteps&=~(1<<STEP_TIMER);
		} else if(&mq==&(*motionWakeup)) {
			//cout << "Motion read, ";
			waitingSteps&=~(1<<STEP_MOTION);
		} else if(&mq==&(*events)) {
			//cout << "Main read event queue (" << events->getMessagesUnread() << "remain), ";
			// nothing to do, just waiting for main to catch up before incrementing
		} else {
			cout << "Unknown message base read (either you meant to add some code to Simulator::messagesRead, or why did you bother to register a listener?)" << endl;
		}
		//cout << " waiting " << waitingSteps << " events " << events->getMessagesUnread() << endl;
		
		if(waitingSteps==0 && events->getMessagesUnread()==0) //if that was the last one we were waiting for -- go for the next!
			incrementTime();
	}
}

void Simulator::sendCommand(const std::string& cmd) {
	static unsigned int cmdSN=0;
	char msgname[30];
	snprintf(msgname,30,"SimCommand.%d.%d",ProcessID::getID(),cmdSN++);
	RCRegion * msg = new RCRegion(msgname,cmd.size());
	strcpy(msg->Base(),cmd.c_str());
	SharedObject<CommandQueue_t> commandQ(ipc_setup->registerRegion(Simulator::getCommandQueueID(),sizeof(CommandQueue_t)));
	commandQ->sendMessage(msg,true);
}

void Simulator::setMotionStarting() {
	// don't need lock, this is only called during single-threaded operation (doStart())
	//MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="motionStarting()"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->motionStarting();
	}
	Simulator::motionHookMonitor->curHook=NULL;
}
void Simulator::setMotionStopping() {
	// don't need lock, this is only called during single-threaded operation (doStop())
	//MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="motionStopping()"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->motionStopping();
	}
	Simulator::motionHookMonitor->curHook=NULL;
}
void Simulator::updateMotion(const float outputs[][NumOutputs]) {
	MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="motionCheck(const float[][])"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->motionCheck(outputs);
	}
	Simulator::motionHookMonitor->curHook=NULL;
}
void Simulator::updatePIDs(const std::vector<MotionHook::PIDUpdate>& pids) {
	MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="updatePIDs(PIDUpdate)"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->updatePIDs(pids);
	}
	Simulator::motionHookMonitor->curHook=NULL;
}
void Simulator::setMotionLeavingRealtime(bool isFullSpeed) {
	MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="leavingRealtime()"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->leavingRealtime(isFullSpeed);
	}
	Simulator::motionHookMonitor->curHook=NULL;
	if(theSim!=NULL) { // if actually running in simulator (may be Motion in multiprocess...)
		globals->sensorState.resourceSync = NULL;
		for(std::set<DataSource*>::const_iterator it=theSim->activeCameraSrcs.begin(); it!=theSim->activeCameraSrcs.end(); ++it)
			(*it)->leavingRealtime(isFullSpeed);
		for(std::set<DataSource*>::const_iterator it=theSim->activeSensorSrcs.begin(); it!=theSim->activeSensorSrcs.end(); ++it)
			(*it)->leavingRealtime(isFullSpeed);
	}
}
void Simulator::setMotionEnteringRealtime() {
	MarkScope l(theSim ? dynamic_cast<Resource&>(theSim->simLock) : ::emptyResource);
	Simulator::motionHookMonitor->motionCheckTime.Set();
	Simulator::motionHookMonitor->curFuncName="enteringRealtime()"; 
	for(std::set<MotionHook*>::iterator it=theSim->motionHooks.begin(); it!=theSim->motionHooks.end(); ++it) {
		Simulator::motionHookMonitor->curHook=*it;
		(*it)->enteringRealtime(globals->timeScale);
	}
	Simulator::motionHookMonitor->curHook=NULL;
	if(theSim!=NULL) { // if actually running in simulator (may be Motion in multiprocess...)
		globals->sensorState.resourceSync = syncSensors;
		for(std::set<DataSource*>::const_iterator it=theSim->activeCameraSrcs.begin(); it!=theSim->activeCameraSrcs.end(); ++it)
			(*it)->enteringRealtime(globals->timeScale);
		for(std::set<DataSource*>::const_iterator it=theSim->activeSensorSrcs.begin(); it!=theSim->activeSensorSrcs.end(); ++it)
			(*it)->enteringRealtime(globals->timeScale);
	}
}

const TimeET Simulator::MotionMonitorThread::timeout(1000L);
unsigned int Simulator::MotionMonitorThread::runloop() {
	if(timeout < motionCheckTime.Age()) {
		float blockTime = static_cast<unsigned int>(motionCheckTime.Age().Value()*10+.5)/10.0f;
		if(curHook==NULL) {
			std::cerr << "Looks like " << ProcessID::getIDStr() << " has been stuck for " << blockTime << " seconds, but I can't tell where, try attaching debugger?" << std::endl;
			return 2000000;
		}
		const DeviceDriver * guilty=NULL;
		for(DeviceDriver::registry_t::const_iterator it=DeviceDriver::getRegistry().begin(); it!=DeviceDriver::getRegistry().end(); ++it) {
			if(DeviceDriver * d=DeviceDriver::getRegistry().getInstance(it->first)) {
				if(curHook == d->getMotionSink()) {
					guilty = d;
				}
			}
		}
		if(guilty==NULL) {
			std::cerr << ProcessID::getIDStr() << " has blocked within " << curFuncName << " on an unknown " << " MotionHook with typeid '" << typeid(*curHook).name() << "' @" << curHook << " for " << motionCheckTime.Age() << std::endl;
		} else {
			std::cerr << ProcessID::getIDStr() << " has blocked within " << curFuncName << " on MotionHook with typeid '" << typeid(*curHook).name() << "'";
			if(guilty->getName()==guilty->getClassName())
				std::cerr << " from driver " << guilty->getName();
			else
				std::cerr << " from driver instance " << guilty->getName() << " (type " << guilty->getClassName() << ")";
			std::cerr << " for " << blockTime << " seconds" << std::endl;
		}
	}
	return 2000000;
}


void Simulator::doStop() {
	curLevel=SharedGlobals::STOPPING;
	commandrecv->finish();
	delete commandrecv;
	commandrecv=NULL;
	motionrecv->finish();
	delete motionrecv;
	motionrecv=NULL;
	motionpidsrecv->finish();
	delete motionpidsrecv;
	motionpidsrecv=NULL;
	setMotionStopping();
	delete motionHookMonitor;
	motionHookMonitor=NULL;
	
	if(!sim::config.multiprocess) {
		erouter->setForwardingAgent(getID(),NULL);
		MotionManager::setTranslator(NULL);
	} else {
		erouter->removeTrapper(etrans);
		delete etrans;
		etrans=NULL;
		MotionManager::setTranslator(NULL);
	}
	
	processRunlevel(SharedGlobals::STOPPING);
	Process::doStop();
}


Simulator::CommandThread::~CommandThread() {
	if(isStarted()) {
		interrupt();
		join();
	}
}

#ifndef DISABLE_READLINE
extern int rl_catch_signals; // libedit doesn't export this, but does provide the symbol
#endif

void* Simulator::CommandThread::run() {
	Simulator * simp=dynamic_cast<Simulator*>(Process::getCurrent());
	ASSERTRETVAL(simp!=NULL,"CommandThread not in Simulator!",NULL);
	string prompt; // initially prompt is empty because it will be buried in startup text anyway
	// we'll display another prompt at the end of launch (or if user hits enter)
	while(true) {
		testCancel();
		string line;
#ifndef DISABLE_READLINE
		// Mac OS X ships with BSD libedit emulation, which apparently crashes (as of 10.6.3) if you override rl_getc_function (WTF)
		// Detect this condition:
		{
			// rl_gnu_readline_p is not defined in libedit, and if it were, should be zero
			static bool tested=false, haveGNU;
			if(!tested) {
				void* gnuFlag = dlsym(RTLD_DEFAULT,"rl_gnu_readline_p");
				haveGNU = (gnuFlag!=NULL && *reinterpret_cast<int*>(gnuFlag));
				tested=true; // don't repeat test for every prompt
			}
			if(haveGNU)
				rl_getc_function = ::getc; // the default rl_getc screws up thread cancellation, just use regular getc instead
			// if not using GNU readline, don't need to override rl_getc (the libedit version doesn't eat signals), so can just ignore this
		}
		rl_catch_signals = 0;
		char* readin=readline(prompt.c_str());
		if(readin==NULL) {
			cout << endl;
			simp->cmdQuit(vector<string>());
			break;
		}
		line=readin;
		free(readin);
#else
		cout << prompt << flush;
		getline(cin,line);
		if(!cin) {
			cout << endl;
			simp->cmdQuit(vector<string>());
			break;
		}
#endif
		simp->processCommand(line,true);
		prompt = sim::config.cmdPrompt;
	}
	return NULL;
}

void Simulator::CommandThread::runInitThread(Thread& th) {
	Thread::NoCancelScope nc;
	{
		MarkScope l(initThreadsLock);
		th.start();
		initThreads.insert(&th);
	}
	th.join();
	{
		MarkScope l(initThreadsLock);
		initThreads.erase(&th);
	}
}
void Simulator::CommandThread::abortInitThreads() {
	MarkScope l(initThreadsLock);
	std::for_each(initThreads.begin(),initThreads.end(),std::mem_fun(&Thread::stop));
}



void Simulator::resetSpeedMode() {
	if(globals->timeScale<=0) {
		if(sensorThread.isStarted()) {
			sensorThread.stop();
			//sensorThread.join(); // can't join because runfor/runto pause might be triggered within LoadFileThread's get_time() call
		}
		if(motionHookMonitor->isStarted() && curLevel==SharedGlobals::RUNNING)
			motionHookMonitor->stop();
	}
	if( (lastTimeScale>0 && globals->timeScale<=0) // was realtime, now non-realtime...
		 || (lastTimeScale==0 && globals->timeScale<0) // or was paused, now full-speed...
		 || (lastTimeScale<0 && globals->timeScale==0) ) { // or was full-speed now paused...
		setMotionLeavingRealtime(globals->timeScale<0); // 
	} else if(lastTimeScale<=0 && globals->timeScale>0) {
		setMotionEnteringRealtime();
	}
	if(curLevel==SharedGlobals::RUNNING) {
		if(globals->timeScale>0) {
			if(!sensorThread.isStarted())
				sensorThread.start();
			if(!motionHookMonitor->isStarted())
				motionHookMonitor->start();
		}
	} else {
		if(!motionHookMonitor->isStarted())
			motionHookMonitor->start();
	}
	
	if(globals->timeScale<0) {
		cameraQueue->setOverflowPolicy(MessageQueueBase::WAIT);
		sensorQueue->setOverflowPolicy(MessageQueueBase::WAIT);
		// these status listener calls check to prevent duplicate listeners
		cameraStatus.addStatusListener(this);
		sensorStatus.addStatusListener(this);
		timerStatus.addStatusListener(this);
		motionStatus.addStatusListener(this);
		eventsStatus.addStatusListener(this);
		fullspeedWallStart.Set();
		fullspeedSimStart=globals->simulatorTime;
		lastFrameWallStart.Set();
		avgWallTime=avgSimTime=0;
	} else {
		cameraQueue->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
		sensorQueue->setOverflowPolicy(MessageQueueBase::DROP_OLDEST);
		eventsStatus.removeStatusListener(this);
		motionStatus.removeStatusListener(this);
		timerStatus.removeStatusListener(this);
		sensorStatus.removeStatusListener(this);
		cameraStatus.removeStatusListener(this);
	}
	if(globals->timeScale==0)
		globals->setAutoPauseTime(-1U);
	lastTimeScale=globals->timeScale;
}

void Simulator::replaceEntry(const std::string& name, plist::Dictionary& d, const std::string& comment) {
	plist::Dictionary::const_iterator it = sim::config.findEntry(name);
	if(it==sim::config.end()) {
		sim::config.addEntry(name,d,comment);
	} else {
		d.set(*it->second);
		sim::config.setEntry(name,d);
		sim::config.setComment(name,comment);
	}
}

bool Simulator::sendSensor(bool syncCall) {
	bool sent=false;
	Thread::NoCancelScope nc;
	if(!syncCall)
		sendSensorSent=false;
	if(globals->sensorState.dirty) {
		if(!syncCall)
			return false; // this is counter-intuitive, but we'll reduce latency by waiting for the next update
		if(sendSensorSent)
			return false; // already sent an update for this period, don't exceed requested frame rate
		if(globals->sensors.verbose>=2)
			std::cout << "Sending sensor update at " << get_time() << std::endl;
		++globals->sensorState.frameNumber;
		globals->sensorState.dirty=false;
		sensorQueue->sendMessage(NULL);
		sendSensorSent=sent=true;
	} else if(globals->sensors.heartbeat) {
		unsigned int curTime = get_time();
		unsigned int tgtTime = globals->sensorState.timestamp + static_cast<unsigned int>(1000.f / globals->sensors.framerate + 0.5f);
		//std::cout << "Candidate heartbeat " << tgtTime << " at " << curTime << std::endl;
		if(tgtTime<=curTime) {
			if(globals->sensors.verbose>=3)
				std::cout << "Sending sensor heartbeat at " << curTime << std::endl;
			++globals->sensorState.frameNumber;
			globals->sensorState.timestamp=curTime;
			sensorQueue->sendMessage(NULL);
			sent=true;
		}
	}
	return sent;
}

unsigned int Simulator::nextVisionTime() {
	unsigned int vis=-1U;
	for(std::set<DataSource*>::const_iterator it=activeCameraSrcs.begin(); it!=activeCameraSrcs.end(); ++it) {
		if(!(*it)->getFrozen())
			vis = std::min(vis,(*it)->nextTimestamp());
	}
	return vis;
}

unsigned int Simulator::nextSensorTime() {
	unsigned int tpf = static_cast<unsigned int>(1000/globals->sensors.framerate + .5f);
	return (get_time()/tpf + 1)*tpf;
}

void Simulator::incrementTime() {
	MarkScope l(simLock);
	waitingSteps=getNextFrame();
	if(waitingSteps==0)
		return;
	unsigned int next=*frameTimes.begin();
	if(next>globals->simulatorTime) {
		unsigned int adv=next-globals->simulatorTime;
		avgWallTime=avgWallTime*avgSpeedupGamma + (float)lastFrameWallStart.Age().Value()*(1-avgSpeedupGamma);
		avgSimTime=avgSimTime*avgSpeedupGamma + adv*(1-avgSpeedupGamma);
		lastFrameWallStart.Set();
		//cout << "inc " << (avgSimTime/avgWallTime/1000) << " " << waitingSteps << endl;
		globals->simulatorTime=next;
	}
	if(waitingSteps & (1<<STEP_CAMERA)) {
		bool sent=false;
		for(std::set<DataSource*>::const_iterator it=activeCameraSrcs.begin(); it!=activeCameraSrcs.end(); ++it)
			if((*it)->advance())
				sent=true;
		if(!sent)
			waitingSteps &= ~(1<<STEP_CAMERA); // nothing sent, clear waiting flag
	}
	if(waitingSteps & (1<<STEP_SENSOR)) {
		for(std::set<DataSource*>::const_iterator it=activeSensorSrcs.begin(); it!=activeSensorSrcs.end(); ++it)
			(*it)->advance();
		sendSensorSent=false; // clear the flag to make sure we send since we're not making non-sync calls to sendSensor
		if(!sendSensor(true))
			waitingSteps &= ~(1<<STEP_SENSOR); // nothing sent, clear waiting flag
	}
	if(waitingSteps & (1<<STEP_TIMER))
		timerWakeup->sendMessage(NULL);
	if(waitingSteps & (1<<STEP_MOTION))
		motionWakeup->sendMessage(NULL);
	if(globals->getAutoPauseTime()<=globals->simulatorTime || (1<<step) & waitingSteps) {
		//that was the step we were waiting for, pause sim
		globals->timeScale=0;
		step=STEP_NONE;
		globals->setAutoPauseTime(-1U);
	}
}

unsigned int Simulator::getNextFrame() {
	frameTimes.clear();
	unsigned int vis = nextVisionTime();
	frameTimes.insert(vis);
	unsigned int sen = nextSensorTime();
	frameTimes.insert(sen);
	unsigned int tim=globals->getNextTimer();
	frameTimes.insert(tim);
	unsigned int mot=globals->getNextMotion();
	frameTimes.insert(mot);
	unsigned int next=*frameTimes.begin();
	//cout << "Testing: " << globals->simulatorTime << " => next camera: "<< vis << " next sensor: " << sen << " next timer: " << tim << " next motion: " << mot << " => " << next << endl;
	unsigned int steps=0;
	if(next!=-1U) {
		if(next==vis) {
			steps |= 1<<STEP_CAMERA;
		}
		if(next==sen) {
			steps |= 1<<STEP_SENSOR;
		}
		if(next==tim) {
			steps |= 1<<STEP_TIMER;
		}
		if(next==mot) {
			steps |= 1<<STEP_MOTION;
		}
	}
	return steps;
}

void Simulator::processRunlevel(SharedGlobals::runlevel_t curRunLevel) {
	curLevel=curRunLevel;
	if(sim::config.tgtRunlevel==curLevel && (!globals->isShutdown() || curLevel>SharedGlobals::RUNNING))
		cout << sim::config.cmdPrompt << flush;
	while(sim::config.tgtRunlevel==curLevel && (!globals->isShutdown() || curLevel>SharedGlobals::RUNNING)) {
		usleep(500000); // recheck status every half second
	}
}

bool Simulator::processCommand(const std::string& line, bool addToHistory) {
	vector<string> args;
	vector<unsigned int> offs;
	if(!string_util::parseArgs(line,args,offs)) {
		cerr << "Mismatched quotes" << endl;
		return false;
	}
	if(args.size()==0)
		return true;
#ifndef DISABLE_READLINE
	/*		if(current_history()==NULL)
		cout << "current_history()==NULL" << endl;
	else if(current_history()->line==NULL)
		cout << "line == NULL" << endl;
	else if(line!=current_history()->line)
		cout << "line is different" << endl;
	else {
		cout << "not added" << endl;
		cout << "new line: " << line << endl;
		cout << "old line: " << current_history()->line << endl;
	}
	if(history_get(-1)==NULL)
		cout << "history_get(0)==NULL" << endl;
	else if(history_get(-1)->line==NULL)
		cout << "line 0 == NULL" << endl;
	else {
		cout << "line 0: " << history_get(-1)->line << endl;
		if(line!=history_get(-1)->line)
			cout << "line 0 is different" << endl;
		else
			cout << "0 not added" << endl;
	}
	*/	
	if(addToHistory && (current_history()==NULL || current_history()->line==NULL || line!=current_history()->line))
		add_history(line.c_str());
#endif
	if(args[0]=="shutdown" || args[0]=="quit" || args[0]=="exit") {
		cmdQuit(args);
	} else if(args[0]=="load") {
		cmdLoad(args);
	} else if(args[0]=="save") {
		cmdSave(args);
	} else if(args[0]=="runlevel") {
		cmdRunlevel(args);
	} else if(args[0]=="get_time") {
		cout << "Current time is " << get_time() << endl;
	} else if(args[0]=="print") {
		cmdPrint(args);
	} else if(args[0]=="set") {
		cmdSet(args);
	} else if(args[0]=="runto") {
		cmdRun(args,false);
	} else if(args[0]=="runfor") {
		cmdRun(args,true);
	} else if(args[0]=="run" || args[0]=="r") {
		cmdRun(args);
	} else if(args[0]=="pause" || args[0]=="p") {
		cmdPause(args);
	} else if(args[0]=="help") {
		cmdHelp(args);
	} else if(args[0]=="step") {
		cmdStep(args);
	} else if(args[0]=="status") {
		cmdStatus(args);
	} else if(args[0]=="advance") {
		cmdAdvance(args);
	} else if(args[0]=="freeze") {
		cmdFreeze(true,args);
	} else if(args[0]=="unfreeze") {
		cmdFreeze(false,args);
	} else if(args[0]=="reset") {
		cmdReset(args);
	} else if(args[0]=="new") {
		cmdNew(args);
	} else if(args[0]=="delete") {
		cmdDelete(args);
	} else if(args[0]=="post") {
		cmdPost(args);
	} else if(args[0]=="msg") {
		cmdMsg(args);
	} else {
		unsigned int i;
		for(i=0; i<args.size(); ++i) {
			if(args[i].find("=")!=string::npos) {
				cmdSet(args);
				break;
			}
		}
		if(i==args.size()) {
			cout << "Unknown command '" << args[0] << "'" << endl;
			return false;
		}
	}
	return true;
}

bool Simulator::gotCommand(RCRegion* msg) {
	Simulator * simp=dynamic_cast<Simulator*>(Process::getCurrent());
	ASSERTRETVAL(simp!=NULL,"gotCommand, but not within Simulator process!",true);
	simp->processCommand(msg->Base(),false);
	return true;
}	

bool Simulator::gotMotion(RCRegion* msg) {
#ifdef DEBUG
	Simulator * simp=dynamic_cast<Simulator*>(Process::getCurrent());
	ASSERTRETVAL(simp!=NULL,"gotMotion, but not within Simulator process!",true);
#endif
	updateMotion(reinterpret_cast<float(*)[NumOutputs]>(msg->Base()));
	return true;
}

bool Simulator::gotMotionPIDs(RCRegion* msg) {
#ifdef DEBUG
	Simulator * simp=dynamic_cast<Simulator*>(Process::getCurrent());
	ASSERTRETVAL(simp!=NULL,"gotMotion, but not within Simulator process!",true);
#endif
	std::vector<MotionHook::PIDUpdate> updates;
	unsigned int num = msg->Size()/sizeof(MotionHook::PIDUpdate);
	MotionHook::PIDUpdate * incoming = reinterpret_cast<MotionHook::PIDUpdate*>(msg->Base());
	updates.reserve(num);
	std::copy(incoming,incoming+num,std::back_inserter(updates));
	updatePIDs(updates);
	return true;
}

void Simulator::updateDataSources(std::set<std::string>& active, std::set<DataSource*>& activeSrcs, const plist::ArrayOf<plist::Primitive<std::string> >& requested, getDataSources_t getDataSources) {
	std::set<std::string> reqNames;
	for(plist::ArrayOf<plist::Primitive<std::string> >::const_iterator it=requested.begin(); it!=requested.end(); ++it) {
		(*it)->addPrimitiveListener(this);
		reqNames.insert(**it);
	}
	std::set<std::string> newNames;
	std::set_difference(reqNames.begin(),reqNames.end(),active.begin(),active.end(),std::inserter(newNames,newNames.end()));
	
	// look up the DataSources from the names
	std::set<DataSource*> reqSrcs;
	for(std::set<std::string>::const_iterator it=reqNames.begin(); it!=reqNames.end(); ++it) {
		std::string errStr;
		lookupDataSource(*it, getDataSources, reqSrcs, errStr);
		if(errStr.size()>0) {
			if(newNames.count(*it))
				std::cerr << errStr << std::endl;
		}
	}
	
	// figure out which data sources have been added and which have been removed
	std::set<DataSource*> newSrcs, delSrcs;
	std::set_difference(reqSrcs.begin(),reqSrcs.end(),activeSrcs.begin(),activeSrcs.end(),std::inserter(newSrcs,newSrcs.end()));
	std::set_difference(activeSrcs.begin(),activeSrcs.end(),reqSrcs.begin(),reqSrcs.end(),std::inserter(delSrcs,delSrcs.end()));
	
	if(curLevel==SharedGlobals::RUNNING) {
		if(globals->timeScale!=0) {
			// those which have been removed should be stopped
			for(std::set<DataSource*>::const_iterator it=delSrcs.begin(); it!=delSrcs.end(); ++it)
				(*it)->leavingRealtime(false);
		}
	}
	
	if(curLevel>=SharedGlobals::STARTING) {
		for(std::set<DataSource*>::const_iterator it=delSrcs.begin(); it!=delSrcs.end(); ++it)
			(*it)->deregisterSource();
	}
	if(getDataSources == &DeviceDriver::getImageSources) {
		for(std::set<DataSource*>::const_iterator it=delSrcs.begin(); it!=delSrcs.end(); ++it)
			(*it)->setImageQueue(NULL);
		for(std::set<DataSource*>::const_iterator it=newSrcs.begin(); it!=newSrcs.end(); ++it)
			(*it)->setImageQueue(&*cameraQueue);
	}
	if(curLevel>=SharedGlobals::STARTING) {
		for(std::set<DataSource*>::const_iterator it=newSrcs.begin(); it!=newSrcs.end(); ++it)
			(*it)->registerSource();
	}
	
	if(curLevel==SharedGlobals::RUNNING) {
		// those which are newly added should be started/updated
		if(globals->timeScale<=0) {
			for(std::set<DataSource*>::const_iterator it=newSrcs.begin(); it!=newSrcs.end(); ++it)
				(*it)->leavingRealtime(globals->timeScale<0);
		} else if(globals->timeScale>0) {
			for(std::set<DataSource*>::const_iterator it=newSrcs.begin(); it!=newSrcs.end(); ++it)
				(*it)->enteringRealtime(globals->timeScale);
		}
	}
	
	activeSrcs = reqSrcs;
}

void Simulator::lookupDataSource(const std::string& name, getDataSources_t getDataSources, std::set<DataSource*>& dataSrcs, std::string& errStr) {
	errStr.clear();
	std::string::size_type dot=name.find('.');
	DeviceDriver* dd=DeviceDriver::getRegistry().getInstance(name.substr(0,dot));
	if(dd==NULL) {
		errStr = "Could not find driver named '" + name + "' for data source";
	} else {
		std::map<std::string,DataSource*> dsmap;
		(dd->*getDataSources)(dsmap);
		if(dsmap.size()==0) {
			errStr = "Driver '" + name.substr(0,dot) + "' does not have any data sources";
		} else if(dot==std::string::npos) {
			// add all the data sources
			for(std::map<std::string,DataSource*>::const_iterator ds = dsmap.begin(); ds!=dsmap.end(); ++ds)
				dataSrcs.insert(ds->second);
		} else {
			std::map<std::string,DataSource*>::const_iterator ds=dsmap.find(name.substr(dot+1));
			if(ds!=dsmap.end()) {
				dataSrcs.insert(ds->second);
			} else {
				errStr = "Could not find stream named '" + name.substr(dot+1) + "' in driver '" + name.substr(0,dot) + "'";
			}
		}
	}
}

std::string Simulator::lookupDataSourceName(const DataSource* src) const {
	for(DeviceDriver::registry_t::const_iterator it = DeviceDriver::getRegistry().begin(); it!=DeviceDriver::getRegistry().end(); ++it) {
		DeviceDriver* dd = DeviceDriver::getRegistry().getInstance(it->first);
		std::map<std::string,DataSource*> sources, sensorSources, imageSources;
		dd->getSensorSources(sensorSources);
		dd->getImageSources(imageSources);
		std::merge(sensorSources.begin(),sensorSources.end(), imageSources.begin(),imageSources.end(), std::inserter(sources,sources.end()));
		bool foundNonMatch=false;
		std::string ans;
		for(std::map<std::string,DataSource*>::const_iterator sit=sources.begin(); sit!=sources.end(); ++sit) {
			if(sit->second == src) {
				if(ans.size()==0)
					ans = it->first + "." + sit->first;
				else
					ans += "/" + sit->first;
			} else {
				foundNonMatch=true;
			}
		}
		if(ans.size()>0) {
			// if all source(s) match, then just return the driver name, otherwise return the full name
			return !foundNonMatch ? it->first : ans;
		}
	}
	return std::string();
}


void Simulator::cmdQuit(const std::vector<std::string>& /*args*/) {
	sim::config.tgtRunlevel=SharedGlobals::DESTRUCTED;
	cmdThread.abortInitThreads();
	globals->signalShutdown();
}
void Simulator::cmdLoad(const std::vector<std::string>& args) {
	if(args.size()>1)
		for(unsigned int i=1; i<args.size(); i++) {
			cout << "Loading from " << args[i] << "... " << flush;
			size_t res=sim::config.loadFile(args[i].c_str());
			cout << (res>0 ? "done." : "load failed.") << endl;
		}
	else {
		cout << "Loading from " << sim::config.getLastFile() << "... " << flush;
		size_t res=sim::config.loadFile(sim::config.getLastFile().c_str());
		cout << (res>0 ? "done." : "load failed.") << endl;
	}
}
void Simulator::cmdSave(const std::vector<std::string>& args) {
	if(args.size()>1)
		for(unsigned int i=1; i<args.size(); i++)
			sim::config.saveFile(args[i].c_str());
	else {
		cout << "Saving to " << sim::config.getLastFile() << "... " << flush;
		size_t res=sim::config.saveFile(sim::config.getLastFile().c_str());
		cout << (res>0 ? "done." : "save failed.") << endl;
	}
}
void Simulator::cmdRunlevel(const std::vector<std::string>& args) {
	if(args.size()<=1) {
		sim::config.tgtRunlevel=static_cast<SharedGlobals::runlevel_t>(sim::config.tgtRunlevel+1);
		cout << "Moving to next runlevel: " << SharedGlobals::runlevel_names[sim::config.tgtRunlevel] << endl;
	} else {
		try {
			sim::config.tgtRunlevel=string_util::makeUpper(args[1]);
		} catch(...) {
			cout << "Invalid runlevel specification.  Try one of:\n\t";
			for(unsigned int i=0; i<SharedGlobals::NUM_RUNLEVELS; i++)
				cout << i << ' ' << SharedGlobals::runlevel_names[i] << ", ";
			cout << "\nCurrently at " << SharedGlobals::runlevel_names[curLevel] << endl;
			return;
		}
		if(sim::config.tgtRunlevel<curLevel) {
			sim::config.tgtRunlevel=curLevel;
			cout << "Cannot reduce runlevel, currently at " << curLevel << ' ' << SharedGlobals::runlevel_names[curLevel] << "\n\t";
			for(unsigned int i=0; i<SharedGlobals::NUM_RUNLEVELS; i++)
				cout << i << ' ' << SharedGlobals::runlevel_names[i] << ", ";
			cout << endl;
			return;
		} else if(sim::config.tgtRunlevel==curLevel) {
			cout << "Already at " << curLevel << ' ' << SharedGlobals::runlevel_names[curLevel] << "\n\t";
			for(unsigned int i=0; i<SharedGlobals::NUM_RUNLEVELS; i++)
				cout << i << ' ' << SharedGlobals::runlevel_names[i] << ", ";
			cout << endl;
			return;
		}
	}
	if(sim::config.tgtRunlevel>SharedGlobals::RUNNING && curLevel<=SharedGlobals::RUNNING)
		globals->signalShutdown();
}
bool Simulator::cmdPrint(const std::vector<std::string>& args) {
	if(args.size()==0 || (args[0]=="print" && args.size()==1)) {
		plist::filteredDisplay(cout,sim::config,"^[^.].*",REG_EXTENDED,3);
		return false;
	}
	string arg;
	for(unsigned int i=(args[0]=="print"?1:0); i<args.size(); i++) {
		arg+=args[i];
		if(i!=args.size()-1)
			arg+=" ";
	}
	const string WS_PREFIX="WorldState";
	if(arg.substr(0,WS_PREFIX.size())==WS_PREFIX) {
		return SensorStateAccessor().print(arg);
	}
	plist::ObjectBase* ob=sim::config.resolveEntry(arg);
	if(ob==NULL) {
		cout << "'" << arg << "' is unknown" << endl;
		return false;
	}
	plist::filteredDisplay(cout,*ob,"^[^.].*",REG_EXTENDED,3);
	return true;
}
bool Simulator::cmdSet(const std::vector<std::string>& args) {
	if(args.size()==0 || (args[0]=="set" && args.size()==1)) {
		plist::filteredDisplay(cout,sim::config,"^[^.].*",REG_EXTENDED,3);
		return false;
	}
	string arg;
	for(unsigned int i=(args[0]=="set"?1:0); i<args.size(); i++) {
		arg+=args[i];
		if(i!=args.size()-1)
			arg+=" ";
	}
	const string WS_PREFIX="WorldState";
	if(arg.rfind("=")==string::npos) {
		if(arg.substr(0,WS_PREFIX.size())==WS_PREFIX) {
			return SensorStateAccessor().print(arg);
		}
		plist::ObjectBase* ob=sim::config.resolveEntry(arg);
		if(ob==NULL) {
			cout << "'" << arg << "' is unknown" << endl;
			return false;
		}
		plist::filteredDisplay(cout,*ob,"^[^.].*",REG_EXTENDED,3);
	} else {
		return sim::config.resolveAssignment(arg,std::cerr);
	}
	return false;
}
void Simulator::cmdRun(const std::vector<std::string>& args, bool isRelative) {
	if(args.size()<=1) {
		cout << "runfor/runto requires an argument" << endl;
		return;
	}
	if(isRelative)
		globals->setAutoPauseTime(get_time()+atoi(args[1].c_str()));
	else
		globals->setAutoPauseTime(atoi(args[1].c_str()));
	if(globals->timeScale==0)
		globals->timeScale=runSpeed;
}
void Simulator::cmdRun(const std::vector<std::string>& args) {
	if(args.size()<=1) {
		if(globals->timeScale!=0) {
			cout << "Already running" << endl;
			return;
		}
		globals->timeScale=runSpeed;
	} else {
		float speed=(float)atof(args[1].c_str());
		if(speed!=0)
			runSpeed=speed;
		globals->timeScale=speed;
	}
}
void Simulator::cmdPause(const std::vector<std::string>& args) {
	if(globals->timeScale==0) {
		if(find(args.begin(),args.end(),"quiet")==args.end())
			cout << "Already paused" << endl;
		return;
	}
	runSpeed=globals->timeScale;
	globals->timeScale=0;
}
void Simulator::cmdHelp(const std::vector<std::string>& args) {
	map<string,string> syntax;
	syntax["load"]="[file]";
	syntax["save"]="[file]";
	syntax["runlevel"]="[";
	for(unsigned int i=0; i<SharedGlobals::NUM_RUNLEVELS; i++) {
		stringstream ss;
		ss << i << "|" << SharedGlobals::runlevel_names[i];
		if(i!=SharedGlobals::NUM_RUNLEVELS-1)
			ss << " | ";
		syntax["runlevel"]+=ss.str();
	}
	syntax["runlevel"]+="]";
	syntax["get_time"]="";
	syntax["print"]="[var]";
	syntax["set"]="[var=value]";
	syntax["runto"]="time";
	syntax["runfor"]="time";
	syntax["run"]="[speed]";
	syntax["pause"]="";
	syntax["step"]="[camera|sensor|timer|motion]";
	syntax["status"]="[Main|Motion|SoundPlay|Simulator|all]*";
	syntax["advance"]=syntax["freeze"]=syntax["unfreeze"]="[camera|sensors|all]*";
	syntax["reset"]="[camera|sensors|all]";
	syntax["new"]="<type> [name]";
	syntax["delete"]="name";
	syntax["post"]="<generator> <source> <type> [duration]";
	syntax["msg"]="<string>";
	
	map<string,string> help;
	
	help["load"]="Load HAL configuration from file; if file unspecified, uses last specified file ('hal-$MODEL.plist' by default).\n"
		"Note that these files are human-readable XML (with comments!), and you can remove values to specify only a subset of settings.";
	
	help["save"]="Save HAL configuration to file; if file unspecified, uses last specified file ('hal-$MODEL.plist' by default).\n"
		"Note that these files are human-readable XML (with comments!), and you can remove values to specify only a subset of settings.";
	
	help["runlevel"]="You can specify a runlevel to move to, or if unspecified, the next one.\n"
		"You can only move forward runlevels, not backward.  Usually you'll only need RUNNING, "
		"unless you are debugging startup/shutdown code or the Tekkotsu itself.";
	
	help["get_time"]="Displays the simulator time.";
	
	help["print"]="Displays a configuration entry.  Can also call 'set' without an assignment.";
	
	help["set"]="Sets HAL configuration variables.  Without any arguments, displays all available variables and their current values.\n"
		"Type 'help set <variable>' to get more information about a particular variable.\n"
		"You can also use the set command to assign values for buttons and sensors when not connected to physical hardware.  Type 'print WorldState' to see available keys.";
	
	help["runto"]="Triggers 'run' until the simulator time reaches the specified value and then pauses.";
	
	help["runfor"]="Triggers 'run' until the simulator time has moved by the specified number of milliseconds, then pauses.";
	
	help["run"]="Resets speed to last non-zero value (i.e. value prior to last 'pause'), can override by passing a new value as argument.  Can be abbreviated 'r'.";
	
	help["pause"]="Equivalent to 'set Speed=0'.  Can be abbreviated 'p'.  Stops the flow of time within the simulator.";
	
	help["step"]="Runs at \"full\" speed until the next indicated time frame, or the next available frame if no type is specified.\n"
		"See 'status' for available frames.";
	
	help["status"]="Displays a status report regarding current time, upcoming keyframes, and semaphore usage.  Specify one or more processes to get more in-depth, per-process status reports.";
	
	help["advance"]="Sends the next frame for the specified queue(s) in their listed order (can be listed more than once).\n"
		"Disregards timestamp information, and doesn't advance time, unlike 'step' command.  No arguments and \"all\" is the same as \"sensors camera\".";
	
	help["freeze"]="Equivalent to 'set queue.Frozen=true'.\n"
		"Stops sending frames from the specified queue(s), but still allows time to move (unlike 'pause').  No arguments is the same as \"all\".  See 'advance' and 'unfreeze'.";
	
	help["unfreeze"]="Equivalent to 'set queue.Frozen=false'.\n"
		"Begin sending frames from the specified queue(s) again.  Timestamps for the file listing are offset by the time spent frozen minus frames advanced so the queue(s) will continue from their current position.  No arguments is the same as \"all\".";
	
	help["reset"]="Moves the specified data queue(s) back to the first entry in their list.";
	
	help["new"]="Creates a new driver or communication port instance.\n  Driver types are:";
	set<string> driverNames;
	DeviceDriver::getRegistry().getTypeNames(driverNames);
	for(set<string>::iterator it=driverNames.begin(); it!=driverNames.end(); ++it)
		help["new"]+=" "+*it;
	set<string> commNames;
	CommPort::getRegistry().getTypeNames(commNames);
	help["new"]+="\n  Communication ports types are:";
	for(set<string>::iterator it=commNames.begin(); it!=commNames.end(); ++it)
		help["new"]+=" "+*it;
	
	help["delete"]="Remove an entry from the CommPort or Drivers list";
	
	help["post"]="Posts an event to the Main process, same as Controller's !post command";
	
	help["msg"]="Posts a TextMsgEvent to the Main process, same as Controller's !msg command";
		
	if(args.size()==1) {
		cout << "Available commands: " << endl;
		for(map<string,string>::const_iterator it=help.begin(); it!=help.end(); ++it) {
			cout << '\t' << it->first << " " << syntax[it->first] << endl;
		}
		cout << "type 'help <command>' for more information" << endl;
	} else {
		if(help.find(args[1])==help.end()) {
			cout << "The command '"<< args[1] << "' was not found" << endl;
			return;
		}
		if(args.size()==2) {
			cout << args[1] << " " << syntax[args[1]] << endl;
			cout << help[args[1]] << endl;
		} else {
			if(args[1]=="set") {
				plist::ObjectBase* ob=sim::config.resolveEntry(args[2]);
				if(ob==NULL) {
					cout << "'" << args[2] << "' is unknown" << endl;
					return;
				}
				size_t n=args[2].rfind('.');
				if(n==string::npos)
					cout << sim::config.getComment(args[2]) << endl;
				else {
					ob=sim::config.resolveEntry(args[2].substr(0,n));
					if(const plist::Dictionary * dict=dynamic_cast<const plist::Dictionary*>(ob))
						cout << dict->getComment(args[2].substr(n+1)) << endl;
					else
						cout << "'" << args[2].substr(0,n) << "' is not a dictionary" << endl;
				}
			} else {
				cout << args[1] << " " << syntax[args[1]] << endl;
				cout << help[args[1]] << endl;
			}
		}
	}
}
void Simulator::cmdStep(const std::vector<std::string>& args) {
	if(args.size()<=1) {
		if(globals->timeScale!=0)
			globals->timeScale=0;
		step=STEP_NONE;
		incrementTime();
		return;
	}
	if(args.size()>2) {
		cout << args[0] << " takes 0 or 1 arguments; " << args.size()-1 << " supplied" << endl;
		return;
	}
	if(args[1]=="camera")
		step=STEP_CAMERA;
	else if(args[1]=="sensor" || args[1]=="sensors")
		step=STEP_SENSOR;
	else if(args[1]=="timer")
		step=STEP_TIMER;
	else if(args[1]=="motion")
		step=STEP_MOTION;
	else {
		cout << args[1] << " is not a valid argument for 'step'.  Type 'help step'." << endl;
		return;
	}
	// TODO
	/*
	if(step==STEP_CAMERA && vision.frozen && !vision.heartbeat) {
		cout << "Camera queue is frozen and has no heartbeat, cannot step (use 'advance' instead)" << endl;
		step=STEP_NONE;
	} else if(step==STEP_SENSOR && sensors.frozen && !sensors.heartbeat) {
		cout << "Sensor queue is frozen and has no heartbeat, cannot step (use 'advance' instead)" << endl;
		step=STEP_NONE;
	} else*/ {
		unsigned int steps=getNextFrame();
		if((1<<step) & steps) { // the desired step is the next step -- just increment
			if(globals->timeScale!=0)
				globals->timeScale=0;
			step=STEP_NONE;
			incrementTime();
		} else if(globals->timeScale!=-1)
			globals->timeScale=-1;
	}
}
void Simulator::cmdStatus(const std::vector<std::string>& args) {
	cout << "Speed is " << static_cast<float>(globals->timeScale);
	if(globals->timeScale<0)
		cout << " (full speed mode: avg speed=" << ((globals->simulatorTime-fullspeedSimStart)/fullspeedWallStart.Age().Value()/1000) << "x, "
			<< " current speed=" << (avgSimTime/avgWallTime/1000) << "x)";
	cout << endl;
	cout << "Current time is " << get_time() << endl;
	unsigned int sen=nextSensorTime();
	unsigned int tim=globals->getNextTimer();
	unsigned int mot=globals->getNextMotion();
	cout << "Next camera: ";
	if(activeCameraSrcs.size()==0) cout << "(none)"; else {
		cout << nextVisionTime() << " (";
		for(std::set<DataSource*>::const_iterator it=activeCameraSrcs.begin(); it!=activeCameraSrcs.end(); ++it) {
			if(it!=activeCameraSrcs.begin())
				cout << ' ';
			cout << lookupDataSourceName(*it) << ':' << (*it)->nextName() << '@' << (*it)->nextTimestamp();
			if((*it)->getFrozen())
				cout << "(frozen)";
		}
		cout << ")";
	}
	cout << endl;
	// TODO
	cout << "Next sensor: ";
	if(sen==-1U) cout << "(none)"; else cout << sen;
	/*{
		if(sensors.frozen)
			cout << "Frozen@";
		cout << sensors.getDataSource()->nextName();
		if(!sensors.frozen || sensors.heartbeat) {
			if(sensors.frozen && sensors.heartbeat && sensors.getDataSource()->nextName()!="heartbeat")
				cout << " heartbeat";
			cout << " scheduled at " << sen;
		}
	}*/
	cout << endl;
	cout << "Next timer: ";
	if(tim==-1U) cout << "(none)"; else cout << tim;
	cout << endl;
	cout << "Next motion: ";
	if(mot==-1U) cout << "(none)"; else cout << mot;
	cout << endl;
	unsigned int semUsed=MessageQueueBase::getSemaphoreManager()->used();
	unsigned int semMax=semUsed+MessageQueueBase::getSemaphoreManager()->available();
	cout << "Semaphores used: " << semUsed << "/" << semMax << " (" << ((semUsed*10000+5)/semMax/10)/10.f << "%)" << endl;
	cout << endl;
	if(args.size()>1) {
		if(args[1].substr(0,6)=="region") {
			SemaphoreManager::semid_t sem=statusRequest->addReadStatusListener();
			std::set<std::string> tgts;
			std::copy(args.begin()+2,args.end(),std::inserter(tgts,tgts.end()));
			if(tgts.size()==0)
				tgts.insert("all");
			for(std::set<std::string>::const_iterator it=tgts.begin(); it!=tgts.end(); ++it) {
				RCRegion * region=new RCRegion(it->size()+1);
				strncpy(region->Base(),it->c_str(),region->Size());
				statusRequest->sendMessage(region);
				region->RemoveReference();
			}
			//wait until they're done to put the prompt back up
			if(sem!=statusRequest->getSemaphoreManager()->invalid()) {
				statusRequest->getSemaphoreManager()->lower(sem,tgts.size());
				statusRequest->removeReadStatusListener(sem);
			}
			//check to see if we're included:
			for(std::set<std::string>::const_iterator it=tgts.begin(); it!=tgts.end(); ++it) {
				if(strcasecmp(it->c_str(),getName())==0 || strcasecmp(it->c_str(),"all")==0) {
					statusReport(cout);
					cout << endl;
				}
			}
		} else if(args[1]=="feedback") {
			cout << "Sensor feedback for outputs:\n";
			unsigned int len=10;
			for(unsigned int i=0; i<NumOutputs; ++i)
				if(strlen(outputNames[i])>len)
					len=strlen(outputNames[i]);
			cout << "    " << setw(len+2) << left << "--OUTPUT--" << "--FEEDBACK-SOURCE--\n";
			for(unsigned int i=0; i<NumOutputs; ++i) {
				cout << "    " << setw(len+2) << left << outputNames[i];
				if(globals->sensorState.providedOutputs[i]==0) {
					cout << (globals->motion.feedbackDelay<0 ? "none (Motion.FeedbackDelay is negative)\n" : "motion process (open loop)\n");
				} else if(globals->sensorState.providedOutputs[i]==1) {
					cout << (globals->motion.feedbackDelay>=0 && globals->motion.override ? "motion process override (open loop)\n" : "sensed by device driver\n");
				} else {
					if(globals->motion.feedbackDelay>=0 && globals->motion.override)
						cout << "motion process override (open loop)\n";
					else
						cout << "provided by " << globals->sensorState.providedOutputs[i] << " device drivers (may be conflict)\n";
				}
			}
		}
	}
}
void Simulator::cmdAdvance(const std::vector<std::string>& args) {
	if(curLevel!=SharedGlobals::RUNNING) {
		cout << args[0] << " can only be used in the RUNNING runlevel" << endl;
		return;
	}
	
	// this defines a local functor so we don't need to repeat the hairy STL algo call a bunch of places
	struct { void operator()(const std::set<DataSource*>& src, std::set<DataSource*>& dst) {
			std::remove_copy_if(src.begin(), src.end(), std::inserter(dst,dst.end()), std::not1(std::mem_fun(&DataSource::getFrozen)));
	} } copyUnfrozen;
	
	std::set<DataSource*> camSrcs, senSrcs;
	
	bool isAll=false;
	if(args.size()<=1) {
		// no arguments supplied, advance all frozen queues
		copyUnfrozen(activeCameraSrcs,camSrcs);
		copyUnfrozen(activeSensorSrcs,senSrcs);
		isAll=true;
		
	} else for(unsigned int i=1; i<args.size(); ++i) {
		// process explicitly named data sources
		if(args[i]=="camera" || args[i]=="vision" || args[i]=="image") {
			copyUnfrozen(activeCameraSrcs,camSrcs);
		} else if(args[i]=="sensors") {
			copyUnfrozen(activeSensorSrcs,senSrcs);
		} else {
			std::string errStrC,errStrS;
			
			std::set<DataSource*> tmp;
			lookupDataSource(args[i], &DeviceDriver::getImageSources, tmp, errStrC);
			copyUnfrozen(tmp,camSrcs);
			
			tmp.clear();
			lookupDataSource(args[i], &DeviceDriver::getSensorSources, tmp, errStrS);
			copyUnfrozen(tmp,senSrcs);
			
			if(errStrC.size()>0 && errStrS.size()>0) {
				std::cerr << errStrS << std::endl;
				return;
			}
		}
	}
	
	// do all cameras before all sensors
	// (this is a heuristic that users want the camera before the sensor values, and at least want a reliable ordering)
	for(std::set<DataSource*>::const_iterator it=camSrcs.begin(); it!=camSrcs.end(); ++it) {
		SemaphoreManager::semid_t sem=cameraQueue->addReadStatusListener(); //register read status listener before sending!
		bool sent = (*it)->advance();
		if(!sent && !isAll) // no data in queue and only report empty queue if the queue was explicitly specified
			cout << "No data in " << lookupDataSourceName(*it) << " queue" << endl;
		if(sent)
			cameraQueue->getSemaphoreManager()->lower(sem,true); //block until we know message was read
		cameraQueue->removeReadStatusListener(sem);
	}
	
	// cameras have been processed, now do sensors
	for(std::set<DataSource*>::const_iterator it=senSrcs.begin(); it!=senSrcs.end(); ++it) {
		SemaphoreManager::semid_t sem=sensorQueue->addReadStatusListener(); //register read status listener before sending!
		bool sent = (*it)->advance();
		if(!sent && !isAll) // no data in queue and only report empty queue if the queue was explicitly specified
			cout << "No data in " << lookupDataSourceName(*it) << " queue" << endl;
		if(sent && globals->timeScale==0) {
			ASSERT(!sensorThread.isStarted(), "advance while paused, yet timeScale is non-zero?");
			sendSensorSent=false;
			sent = sendSensor(true); // have to manually pump it out since sensorThread (shouldn't) be running
		}
		if(sent)
			sensorQueue->getSemaphoreManager()->lower(sem,true); //block until we know message was read
		sensorQueue->removeReadStatusListener(sem);
	}
}
void Simulator::cmdFreeze(bool v, const std::vector<std::string>& args) {
	std::set<DataSource*> srcs;
	
	if(args.size()<=1) {
		// no arguments supplied, freeze/unfreeze all frozen queues
		std::merge(activeSensorSrcs.begin(),activeSensorSrcs.end(), activeCameraSrcs.begin(),activeCameraSrcs.end(), std::inserter(srcs,srcs.end()));
		
	} else for(unsigned int i=1; i<args.size(); ++i) {
		if(args[i]=="camera" || args[i]=="vision" || args[i]=="image") {
			srcs.insert(activeCameraSrcs.begin(),activeCameraSrcs.end());
		} else if(args[i]=="sensors") {
			srcs.insert(activeSensorSrcs.begin(),activeSensorSrcs.end());
		} else {
			std::string errStrC,errStrS;
			lookupDataSource(args[i], &DeviceDriver::getImageSources, srcs, errStrC);
			lookupDataSource(args[i], &DeviceDriver::getSensorSources, srcs, errStrS);
			if(errStrC.size()>0 && errStrS.size()>0) {
				std::cerr << errStrS << std::endl;
				return;
			}
		}
	}
	
	for(std::set<DataSource*>::const_iterator it=srcs.begin(); it!=srcs.end(); ++it)
		(*it)->setFrozen(v);
}
void Simulator::cmdReset(const std::vector<std::string>& args) {
	std::set<DataSource*> srcs;
	for(unsigned int i=1; i<args.size(); ++i) {
		if(args[i]=="camera" || args[i]=="vision" || args[i]=="image") {
			srcs.insert(activeCameraSrcs.begin(),activeCameraSrcs.end());
		} else if(args[i]=="sensors") {
			srcs.insert(activeSensorSrcs.begin(),activeSensorSrcs.end());
		} else {
			std::string errStr;
			lookupDataSource(args[i], &DeviceDriver::getSensorSources, srcs, errStr);
			if(errStr.size()>0) {
				lookupDataSource(args[i], &DeviceDriver::getImageSources, srcs, errStr);
				if(errStr.size()>0) {
					std::cerr << errStr << std::endl;
					return;
				}
			}
		}
	}
	
	for(std::set<DataSource*>::const_iterator it=srcs.begin(); it!=srcs.end(); ++it)
		(*it)->reset();
}
void Simulator::cmdNew(const std::vector<std::string>& args) {
	set<string> driverNames;
	DeviceDriver::getRegistry().getTypeNames(driverNames);
	set<string> commNames;
	CommPort::getRegistry().getTypeNames(commNames);
	if(args.size()<2) {
		cerr << "Must specify type to instantiate:\n";
		cerr << "  Communication Ports:";
		for(set<string>::iterator it=commNames.begin(); it!=commNames.end(); ++it)
			cerr << ' ' << *it;
		cerr << endl;
		cerr << "  Device Drivers:";
		for(set<string>::iterator it=driverNames.begin(); it!=driverNames.end(); ++it)
			cerr << ' ' << *it;
		cerr << endl;
		return;
	}
	std::string type = args[1];
	std::string name = (args.size()>2) ? args[2] : args[1];
	try {
		if(driverNames.find(type)!=driverNames.end()) {
			DeviceDriver * d = DeviceDriver::getRegistry().create(type,name);
			if(d==NULL) {
				cerr << "Error instantiating " << type << " (instance with same name already exists?)" << endl;
				return;
			}
		} else if(commNames.find(type)!=commNames.end()) {
			CommPort * c = CommPort::getRegistry().create(type,name);
			if(c==NULL) {
				cerr << "Error instantiating " << type << " (instance with same name already exists?)" << endl;
				return;
			}
		} else {
			cerr << "'" << type << "' is not a valid type for instantiation.  Please choose one of:\n";
			cerr << "  Communication Ports:";
			for(set<string>::iterator it=commNames.begin(); it!=commNames.end(); ++it)
				cerr << ' ' << *it;
			cerr << endl;
			cerr << "  Device Drivers:";
			for(set<string>::iterator it=driverNames.begin(); it!=driverNames.end(); ++it)
				cerr << ' ' << *it;
			cerr << endl;
		}
	} catch(const std::exception& e) {
		cout << "An exception occured during construction of "<<type<<": " << e.what() << endl;
	} catch(...) {
		cout << "An exception occured during construction of "<<type << endl;
	}
}

void Simulator::cmdDelete(const std::vector<std::string>& args) {
	if(args.size()<2) {
		cerr << "Must specify instance to delete:\n";
		cerr << "  Communication Ports:";
		for(CommPort::registry_t::const_iterator it=CommPort::getRegistry().begin(); it!=CommPort::getRegistry().end(); ++it)
			cerr << ' ' << it->first;
		cerr << endl;
		cerr << "  Device Drivers:";
		for(DeviceDriver::registry_t::const_iterator it=DeviceDriver::getRegistry().begin(); it!=DeviceDriver::getRegistry().end(); ++it)
			cerr << ' ' << it->first;
		cerr << endl;
		return;
	}
	if(DeviceDriver::getRegistry().getInstance(args[1])!=NULL) {
		if(!DeviceDriver::getRegistry().destroy(args[1]))
			cerr << "Could not delete driver named '" << args[1] << "'" << endl;
	} else if(CommPort::getRegistry().getInstance(args[1])!=NULL) {
		if(!CommPort::getRegistry().destroy(args[1]))
			cerr << "Could not delete comm port named '" << args[1] << "'" << endl;
	} else {
		// look for a collection entry to remove
		string key=string_util::trim(args[1]);
		plist::ObjectBase* ob=sim::config.resolveEntry(key);
		if(ob==NULL) {
			cerr << "'" << key << "' is unknown" << endl;
		} else {
			string::size_type dotpos = key.rfind(".");
			string parent = key.substr(0,dotpos);
			string entry = key.substr(dotpos+1);
			ob=sim::config.resolveEntry(parent);
			if(ob==NULL) {
				cerr << "Internal error, could not access parent '" << parent << "' of key '" << key << "' for removal" << endl;
			} else if(plist::Collection* col=dynamic_cast<plist::Collection*>(ob)) {
				if((col->getLoadPolicy() & plist::Collection::REMOVALS) != plist::Collection::REMOVALS) {
					cerr << "Collection '" << parent << "' is not dynamically resizeable, cannot remove entry '" << entry << "'" << endl;
				} else if(plist::ArrayBase* arr=dynamic_cast<plist::ArrayBase*>(ob)) {
					size_t i = atoi(entry.c_str());
					arr->removeEntry(i);
				} else if(plist::DictionaryBase* dict=dynamic_cast<plist::DictionaryBase*>(ob)) {
					dict->removeEntry(entry);
				} else {
					cerr << "Internal error, unknown collection type for removing entry '" << entry <<"' from '" << parent << "'" << endl;
				}
			} else {
				cerr << '\'' << parent <<'\'' << " is not a collection, cannot remove entry '" << entry << '\'' << endl;
			}
		}
	}
}

void Simulator::cmdPost(const std::vector<std::string>& args) {
	if(args.size()<4) {
		cerr << "Bad post command, need at least 3 arguments: generator source type [duration]" << endl;
		return;
	}
	//parse generator id -- could be a generator name or a numeric value
	int egid=0;
	for(;egid<EventBase::numEGIDs && args[1]!=EventBase::EventGeneratorNames[egid];egid++) {}
	if(egid==EventBase::numEGIDs) {
		egid=atoi(args[1].c_str());
		if(egid==0 && args[1]!="0") {
			cerr << "Bad event generator '" << args[1] << "'" << endl;
			return;
		}
	}
	//parse source id -- numeric value, unless egid is buttonEGID, in which case we can look up a button name
	//(if you want to add support for other symbolic source types, this is where to do it)
	unsigned int name;
	if(egid==EventBase::buttonEGID) {
		name=capabilities.findButtonOffset(args[2].c_str());
		if(name==-1U) {
			name=atoi(args[2].c_str());
			if(name==0 && args[2]!="0") {
				cerr << "Invalid button name or index '" << args[2] << "'" << endl;
				return;
			}
		}
	} else {
		name=atoi(args[2].c_str());
	}
	//parse type id -- numeric, name, or abbreviated name
	int etid=0;
	for(;etid<EventBase::numETIDs && args[3]!=EventBase::EventTypeNames[etid];etid++) {}
	if(etid==EventBase::numETIDs) {
		etid=0;
		for(;etid<EventBase::numETIDs && args[3]!=EventBase::EventTypeAbbr[etid];etid++) {}
		if(etid==EventBase::numETIDs) {
			etid=atoi(args[3].c_str());
			if(etid==0 && args[3]!="0") {
				cerr << "Bad event type '" << args[3] << "'" << endl;
				return;
			}
		}
	}
	//duration field (optional, have to check args.size())
	int dur=0;
	if(args.size()>4)
		dur=atoi(args[4].c_str());
	//send event!
	SemaphoreManager::semid_t sem=events->addReadStatusListener(); //register read status listener before sending!
	erouter->postEvent((EventBase::EventGeneratorID_t)egid,name,(EventBase::EventTypeID_t)etid,dur);
	events->getSemaphoreManager()->lower(sem,true); //block until we know message was read
	events->removeReadStatusListener(sem);
}

void Simulator::cmdMsg(const std::vector<std::string>& args) {
	SemaphoreManager::semid_t sem=events->addReadStatusListener(); //register read status listener before sending!
	string s;
	if(args.size()>1)
		s=args[1];
	for(size_t i=2; i<args.size(); ++i)
		s.append(" ").append(args[i]);
	erouter->postEvent(TextMsgEvent(s,0));
	events->getSemaphoreManager()->lower(sem,true); //block until we know message was read
	events->removeReadStatusListener(sem);
}


/*! @file
 * @brief 
 * @author ejt (Creator)
 */

