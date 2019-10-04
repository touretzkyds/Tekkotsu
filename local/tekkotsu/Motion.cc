#include "Motion.h"
#include "Main.h"
#include "SoundPlay.h"
#include "Simulator.h"
#include "SimConfig.h"
#include "Shared/Config.h"
#include "MotionExecThread.h"

#include "IPC/RegionRegistry.h"
#include "IPC/MessageReceiver.h"
#include "Motion/Kinematics.h"
#include "Wireless/Wireless.h"
#include "Events/EventRouter.h"
#include "Shared/MarkScope.h"

#include "local/MotionHooks/IPCMotionHook.h"

using namespace std;

Motion::Motion()
	: Process(getID(),getClassName()),
	sounds(ipc_setup->registerRegion(SoundPlay::getSoundPlayID(),sizeof(sim::SoundPlayQueue_t))),
	motions(ipc_setup->registerRegion(getMotionCommandID(),sizeof(sim::MotionCommandQueue_t))),
	motionout(ipc_setup->registerRegion(getMotionOutputID(),sizeof(sim::MotionOutput_t))),
	motionoutpids(ipc_setup->registerRegion(getMotionOutputPIDsID(),sizeof(sim::MotionOutputPIDs_t))),
	events(ipc_setup->registerRegion(Main::getEventsID(),sizeof(sim::EventQueue_t))),
	statusRequest(ipc_setup->registerRegion(Simulator::getStatusRequestID(),sizeof(sim::StatusRequest_t))),
	motionmanager(ipc_setup->registerRegion(getMotionManagerID(),sizeof(MotionManager))),
	soundmanager(ipc_setup->registerRegion(SoundPlay::getSoundManagerID(),sizeof(SoundManager))),
	motionWakeup(ipc_setup->registerRegion(Simulator::getMotionWakeupID(),sizeof(sim::MotionWakeup_t))),
	motionProf(ipc_setup->registerRegion(getMotionProfilerID(),sizeof(motionProfiler_t))),
	etrans(NULL), statusrecv(NULL), wakeuprecv(NULL), motionExec(NULL), motionfwd(NULL),
	wireless_thread(), motionLock()
{
	new (&(*motions)) sim::MotionCommandQueue_t;
	new (&(*motionout)) sim::MotionOutput_t;
	new (&(*motionoutpids)) sim::MotionOutputPIDs_t;
	new (&(*motionmanager)) MotionManager;
	new (&(*motionProf)) motionProfiler_t;
	motman=&(*motionmanager);
	motman->InitAccess(*motions,motionLock);
	sndman=&(*soundmanager);
	::motionProfiler=&(*motionProf);
	
	if(sim::config.multiprocess) {
		//Setup wireless
		ASSERT(wireless==NULL,"global wireless already initialized before Motion?");
		wireless = new Wireless();
		sout=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*12);
		serr=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*4);
		wireless->setDaemon(sout);
		wireless->setDaemon(serr);
		serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);

		//Setup Kinematics
		ASSERT(kine==NULL,"global kine already initialized before Motion?");
		kine=new Kinematics();
	}

	//EventRouter and Config are set up for all processes by main() before fork
}

Motion::~Motion() {
	if(sim::config.multiprocess) {
		delete wireless;
		wireless=NULL;
		delete kine;
		kine=NULL;
	}
}

void Motion::doStart() {
	Process::doStart();
	//These are constructed by other processes, so need to wait
	//until the construction runlevel is complete before we access them
	sndman->InitAccess(*sounds);
	if(!sim::config.multiprocess) {
		// don't use our own etrans here, because erouter will delete it for us, don't want a double-delete in our destructor...
		EventTranslator * forwardTrans = new IPCEventTranslator(*events);
		forwardTrans->setTrapEventValue(true);
		erouter->setForwardingAgent(getID(),forwardTrans);
		MotionManager::setTranslator(forwardTrans);
	} else {
		etrans=new IPCEventTranslator(*events);
		MotionManager::setTranslator(etrans);
		
		// Set up Event Translator to trap and send events to main process
		//send everything over except erouter events
		for(unsigned int i=0; i<EventBase::numEGIDs; i++)
			if(i!=EventBase::erouterEGID)
				erouter->addTrapper(etrans,static_cast<EventBase::EventGeneratorID_t>(i));
		
		wireless->listen(sout, config->motion.console_port);
		wireless->listen(serr, config->motion.stderr_port);
		wireless_thread.start();
		
		Simulator::motionHookMonitor = new Simulator::MotionMonitorThread;
	}

	// initialize output values from initial pose loaded by Simulator constructor
	if(globals->motion.startPose.size()>0) {
		for(unsigned int i=0; i<NumOutputs; ++i)
			motman->setOutput(NULL, i, state->outputs[i]=globals->sensorState.outputs[i]);
	}
	
	statusrecv=new MessageReceiver(*statusRequest,statusReport);
	wakeuprecv=new MessageReceiver(*motionWakeup,gotWakeup,false);

	motionExec=new MotionExecThread(motionLock);
	if(sim::config.multiprocess) {
		Simulator::registerMotionHook(*(motionfwd=new IPCMotionHook(*motionout,*motionoutpids)));
		Simulator::setMotionStarting();
		if(globals->timeScale>0)
			Simulator::setMotionEnteringRealtime();
	}
}


void Motion::run() {
	if(globals->waitForSensors)
		globals->waitSensors();
	
	// might have shutdown triggered while waiting
	// (perhaps device isn't available, user's killing the process...)
	if(globals->isShutdown())
		return; // skip running altogether
	
	motionExec->reset(); //starts the thread if appropriate
	wakeuprecv->start();
	Process::run();
	wakeuprecv->stop();
	wakeuprecv->join();
	if(motionExec->isStarted()) {
		motionExec->stop();
		motionExec->join();
	}
}

void Motion::doStop() {
	Simulator::deregisterMotionHook(*motionfwd);
	delete motionfwd;
	
	wakeuprecv->finish();
	statusrecv->finish();

	delete wakeuprecv;
	wakeuprecv=NULL;
	delete statusrecv;
	statusrecv=NULL;
	
	delete motionExec;
	motionExec=NULL;
	
	if(!sim::config.multiprocess) {
		erouter->setForwardingAgent(getID(),NULL);
		MotionManager::setTranslator(NULL);
	} else {
		if(globals->timeScale>0)
			Simulator::setMotionLeavingRealtime(false);
		Simulator::setMotionStopping();
		
		erouter->removeTrapper(etrans);
		delete etrans;
		etrans=NULL;
		MotionManager::setTranslator(NULL);
		
		wireless_thread.stop();
		wireless_thread.join();
		wireless->setDaemon(sout,false);
		wireless->close(sout);
		sout=NULL;
		wireless->setDaemon(serr,false);
		wireless->close(serr);
		serr=NULL;

		delete Simulator::motionHookMonitor;
		Simulator::motionHookMonitor=NULL;
	}
	motman->RemoveAccess();
	
	Process::doStop();
}

bool Motion::gotWakeup(RCRegion* /*msg*/) {
	Motion * motion=dynamic_cast<Motion*>(Process::getCurrent());
	ASSERTRETVAL(motion!=NULL,"gotWakeup, but not within motion process!",true);
	ASSERTRETVAL(motion->motionExec!=NULL,"motionExec thread is NULL when motion wakeup received",true);
	
	MarkScope l(motion->motionLock);
	if(sim::config.multiprocess) {
		if(globals->timeScale<=0) {
			if(motion->motionExec->isStarted())
				Simulator::setMotionLeavingRealtime(globals->timeScale<0);
			if(Simulator::motionHookMonitor->isStarted())
				Simulator::motionHookMonitor->stop();
		} else {
			if(!motion->motionExec->isStarted())
				Simulator::setMotionEnteringRealtime();
			if(!Simulator::motionHookMonitor->isStarted())
				Simulator::motionHookMonitor->start();
		}
	}
	if(globals->timeScale<=0 && get_time()>=globals->getNextMotion())
		motion->motionExec->poll();
	motion->motionExec->reset(); //reset will set globals->getNextMotion(), so have to do this after the poll itself
	return true;
}

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

