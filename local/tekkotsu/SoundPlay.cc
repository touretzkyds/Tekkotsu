#include "SoundPlay.h"
#include "Simulator.h"
#include "SharedGlobals.h"
#include "Main.h"
#include "IPC/MessageReceiver.h"
#include "IPC/RegionRegistry.h"
#include "Events/EventRouter.h"
#include "SimConfig.h"
#include "Shared/Config.h"

#if !defined(PLATFORM_APERIOS) && !defined(__APPLE__)
#  include "Sound/MaryClient.h"
#endif

SoundPlayThread * SoundPlay::player = NULL;

SoundPlay::SoundPlay()
	: Process(getID(),getClassName()),
		requests(ipc_setup->registerRegion(getSoundPlayID(),sizeof(sim::SoundPlayQueue_t))),
		events(ipc_setup->registerRegion(Main::getEventsID(),sizeof(sim::EventQueue_t))),
		statusRequest(ipc_setup->registerRegion(Simulator::getStatusRequestID(),sizeof(sim::StatusRequest_t))),
		soundmanager(ipc_setup->registerRegion(getSoundManagerID(),sizeof(SoundManager))),
		soundProf(ipc_setup->registerRegion(getSoundProfilerID(),sizeof(soundProfiler_t))),
		etrans(NULL), sndrecv(NULL), statusrecv(NULL)
{
	new (&(*requests)) sim::SoundPlayQueue_t;
	new (&(*soundmanager)) SoundManager;
	new (&(*soundProfiler)) soundProfiler_t;
	sndman=&(*soundmanager);
	::soundProfiler=&(*soundProf);

#if !defined(PLATFORM_APERIOS) && !defined(__APPLE__)
	// Start the Mary server for text to speech
	launchMaryServer();
#endif
}

SoundPlay::~SoundPlay() {}

void SoundPlay::doStart() {
	Process::doStart();
	//These are constructed by other processes, so need to wait
	//until the construction runlevel is complete before we access them
	if(!sim::config.multiprocess) {
		// don't use our own etrans here, because erouter will delete it for us, don't want a double-delete in our destructor...
		EventTranslator * forwardTrans = new IPCEventTranslator(*events);
		forwardTrans->setTrapEventValue(true);
		erouter->setForwardingAgent(getID(),forwardTrans);
	} else {
		etrans=new IPCEventTranslator(*events);
		MotionManager::setTranslator(etrans); //although SoundPlay shouldn't use any motions...

		// Set up Event Translator to trap and send events to main process
		//send everything over except erouter events
		for(unsigned int i=0; i<EventBase::numEGIDs; i++)
			if(i!=EventBase::erouterEGID)
				erouter->addTrapper(etrans,static_cast<EventBase::EventGeneratorID_t>(i));
	}
	
	for(unsigned int i=0; i< ::config->sound.preload.size(); i++)
		sndman->loadFile(::config->sound.preload[i]);
	
#ifdef __APPLE__
	if(!sim::config.multiprocess) // sound output unsupported in multiprocess mode on OS X
		player = new SoundPlayThread;
	else
		std::cerr << "WARNING: sound support disabled on OS X when running in Multiprocess mode" << std::endl;
#else
	player = new SoundPlayThread;
#endif
	if(player!=NULL)
		player->reset();
	sndrecv=new MessageReceiver(*requests,gotSnd);
	statusrecv=new MessageReceiver(*statusRequest,statusReport);
}

void SoundPlay::doStop() {
	sndrecv->finish();
	statusrecv->finish();
	
	delete sndrecv;
	sndrecv=NULL;
	delete statusrecv;
	statusrecv=NULL;
	delete player;
	player=NULL;
	
	sndman->stopPlay();
	
	for(unsigned int i=0; i< ::config->sound.preload.size(); i++)
		sndman->releaseFile(::config->sound.preload[i]);
	
	if(!sim::config.multiprocess) {
		erouter->setForwardingAgent(getID(),NULL);
		MotionManager::setTranslator(NULL);
	} else {
		erouter->removeTrapper(etrans);
		delete etrans;
		etrans=NULL;
		MotionManager::setTranslator(NULL);
	}
	
	Process::doStop();
}

bool SoundPlay::gotSnd(RCRegion* msg) {
	sndman->ProcessMsg(msg);
	if(player!=NULL)
		player->reset();
	return true;
}


/*! @file
 * @brief 
 * @author ejt (Creator)
 */

