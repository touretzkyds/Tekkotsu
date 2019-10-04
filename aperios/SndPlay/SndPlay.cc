//
// Copyright 2002,2003 Sony Corporation 
//
// Permission to use, copy, modify, and redistribute this software for
// non-commercial use is hereby granted.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//

// Additional modifications for Tekkotsu framework

#include <OPENR/OPENRAPI.h>
#include <OPENR/OSyslog.h>
#include <OPENR/core_macro.h>
#include "aperios/SndPlay/entry.h"
#include "SndPlay.h"

#include <iostream>
#include "Sound/SoundManager.h"
#include "Shared/RobotInfo.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"
#include "Events/EventRouter.h"
#include "Shared/Profiler.h"
#include "Shared/MarkScope.h"

SndPlay::SndPlay()
	: active(0), soundManagerMemRgn(NULL), processMapMemRgn(NULL), soundProfilerMemRgn(NULL), etrans(NULL), entryPt(), speakerID(oprimitiveID_UNDEF)
{
	for (unsigned int i = 0; i < SOUND_NUM_BUFFER; i++)
		region[i] = 0;
}

OStatus
SndPlay::DoInit(const OSystemEvent&)
{
	//OSYSDEBUG(("SndPlay::DoInit()\n"));

	NEW_ALL_SUBJECT_AND_OBSERVER;
	REGISTER_ALL_ENTRY;
	SET_ALL_READY_AND_NOTIFY_ENTRY;

	observer[obsReceiveProcessMap]->SetBufCtrlParam(0,1,1);
	observer[obsSoundManagerComm]->SetBufCtrlParam(0,1,SoundManager::MAX_SND+1);
	//+1 to MAX_SND so that we can still get a delete message after we've filled up

	//Set process ID
	ProcessID::setID(ProcessID::SoundProcess);

	//Read config file
	::config = new Config();
	::config->setFileSystemRoot("/ms");
	if(::config->loadFile("config/tekkotsu.xml")==0) {
		if(::config->loadFile("config/tekkotsu.cfg")==0)
			std::cerr << std::endl << " *** ERROR: Could not load configuration file config/tekkotsu.xml *** " << std::endl << std::endl;
		else
			std::cerr << "Successfully imported settings from old-format tekkotsu.cfg" << std::endl;
	}
	
	erouter = new EventRouter;
	etrans=new IPCEventTranslator(*subject[sbjEventTranslatorComm]);
	// only expect to be handling audioEGID, but just in case,
	// subscribe to everything except erouterEGID
	for(unsigned int i=0; i<EventBase::numEGIDs; i++)
		if(i!=EventBase::erouterEGID)
			erouter->addTrapper(etrans,static_cast<EventBase::EventGeneratorID_t>(i));

	//soundManagerMemRgn -> sndman setup
	soundManagerMemRgn = InitRegion(sizeof(SoundManager));
	sndman=new (soundManagerMemRgn->Base()) SoundManager;
	sndman->InitAccess(subject[sbjSoundManagerComm]);
	for(unsigned int i=0; i<config->sound.preload.size(); i++)
		sndman->loadFile(config->sound.preload[i]);
	
	//soundProfilerMemRgn -> soundProfiler setup
	soundProfilerMemRgn = InitRegion(sizeof(soundProfiler_t));
	soundProfiler=new (soundProfilerMemRgn->Base()) soundProfiler_t;
	
	OpenSpeaker();
	NewSoundVectorData();
	SetPowerAndVolume();
	return oSUCCESS;
}

OStatus
SndPlay::DoStart(const OSystemEvent&)
{
	MarkScope ep(entryPt);
  //OSYSDEBUG(("SndPlay::DoStart()\n"));

	ENABLE_ALL_SUBJECT;
	ASSERT_READY_TO_ALL_OBSERVER;
	return oSUCCESS;
}

OStatus
SndPlay::DoStop(const OSystemEvent&)
{
	MarkScope ep(entryPt);
	//OSYSDEBUG(("SndPlay::DoStop()\n"));

	sndman->stopPlay();

	DISABLE_ALL_SUBJECT;
	DEASSERT_READY_TO_ALL_OBSERVER;

	return oSUCCESS;
}

OStatus
SndPlay::DoDestroy(const OSystemEvent&)
{
	MarkScope ep(entryPt);
	for(unsigned int i=0; i<config->sound.preload.size(); i++)
		sndman->releaseFile(config->sound.preload[i]);
	delete etrans;
	etrans=NULL;
	delete erouter;
	if(processMapMemRgn!=NULL) {
		processMapMemRgn->RemoveReference();
		processMapMemRgn=NULL;
	}
	DELETE_ALL_SUBJECT_AND_OBSERVER;
	return oSUCCESS;
}

void
SndPlay::ReadySendSound(const OReadyEvent&)
{
	MarkScope ep(entryPt);
	//	OSYSDEBUG(("SndPlay::Ready()\n"));
	doSendSound();
}

void
SndPlay::ReadyRegisterSoundManager(const OReadyEvent&) {
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		cout << objectName << " Registering SoundManager" << endl;
		subject[sbjRegisterSoundManager]->SetData(soundManagerMemRgn);
		subject[sbjRegisterSoundManager]->NotifyObservers();
	}
}

void
SndPlay::ReadyRegisterProfiler(const OReadyEvent&) {
	MarkScope ep(entryPt);
	static bool is_init=true;
	if(is_init) {
		is_init=false;
		cout << objectName << " Registering soundProfiler" << endl;
		subject[sbjRegisterProfiler]->SetData(soundProfilerMemRgn);
		subject[sbjRegisterProfiler]->NotifyObservers();
	}
}

void
SndPlay::GotSoundMsg(const ONotifyEvent& event) {
	MarkScope ep(entryPt);
	//	OSYSDEBUG(("SndPlay::GotSoundMsg()\n"));
	sndman->ReceivedMsg(event);
	unsigned int curact=sndman->getNumPlaying();
	//	std::cout << "got-active=" << active << " cur=" << curact << std::endl;
	if(active==0 && curact>0) {
		active=curact;
		doSendSound();
	}
	observer[obsSoundManagerComm]->AssertReady();
}

void
SndPlay::GotProcessMap(const ONotifyEvent& event){
	MarkScope ep(entryPt);
	//cout << objectName << "-GOTPROCESSMAP..." << flush;
	//	PROFSECTION("GotMemRegion()",soundProfiler);
	ASSERT(event.NumOfData()==1,"Too many ProcessMaps");
	processMapMemRgn = event.RCData(0);
	processMapMemRgn->AddReference();
	ProcessID::setMap(reinterpret_cast<stacktrace::StackFrame*>(processMapMemRgn->Base()));
	observer[obsReceiveProcessMap]->AssertReady();
	//cout << "done" << endl;
}

void
SndPlay::doSendSound() {
	PROFSECTION("doSendSound()",*soundProfiler);
	//	std::cout << "do-active=" << active << std::endl;
	static unsigned int bufInUse=0;

	active=sndman->getNumPlaying();
	if(active==0) { //if we don't have any active channels, don't send a buffer
		if(bufInUse>0)
			bufInUse--;
		return;
	}

	RCRegion* rgn = FindFreeRegion();
	if(rgn==NULL) //this can happen if a wakeup message comes in right after a sound stopped
		return;
	active=sndman->CopyTo(reinterpret_cast<OSoundVectorData*>(rgn->Base()));
	subject[sbjSpeaker]->SetData(rgn);

	if(bufInUse<SOUND_NUM_BUFFER-1) {
		bufInUse++;
		doSendSound();
	} else //recursive base case
		subject[sbjSpeaker]->NotifyObservers();
}

void
SndPlay::OpenSpeaker()
{
	OStatus result = OPENR::OpenPrimitive(SpeakerLocator, &speakerID);
	if (result != oSUCCESS)
		OSYSLOG1((osyslogERROR, "%s : %s %d","SndPlay::OpenSpeaker()","OPENR::OpenPrimitive() FAILED", result));
}

void
SndPlay::NewSoundVectorData()
{
	OStatus result;
	MemoryRegionID    soundVecDataID;
	OSoundVectorData* soundVecData;

	/*soundUnitSize (bytes/buffer) = sample_rate (samples/sec)
	 *                               * sample_bits (bits/sample)
	 *                               * [1/8] (bytes/bit)
	 *                               * SoundBufferTime (ms/buffer)
	 *                               * [1/1000] (sec/ms)
	 */
	size_t soundUnitSize = config->sound.sample_rate*config->sound.sample_bits/8*SoundBufferTime/1000;

	for (unsigned int i = 0; i < SOUND_NUM_BUFFER; i++) {
		result = OPENR::NewSoundVectorData(1, soundUnitSize,&soundVecDataID, &soundVecData);
		if (result != oSUCCESS) {
			OSYSLOG1((osyslogERROR, "%s : %s %d","SndPlay::NewSoundVectorData()","OPENR::NewSoundVectorData() FAILED", result));
			return;
		}

		soundVecData->SetNumData(1);
		OSoundInfo* sinfo = soundVecData->GetInfo(0);
		sinfo->Set(odataSOUND_VECTOR,speakerID,soundUnitSize);
		sinfo->dataSize      = soundUnitSize;
		sinfo->format        = osoundformatPCM;
		sinfo->channel       = osoundchannelMONO;
		sinfo->samplingRate  = config->sound.sample_rate;
		sinfo->bitsPerSample = config->sound.sample_bits;

		region[i] = new RCRegion(soundVecData->vectorInfo.memRegionID,soundVecData->vectorInfo.offset,(void*)soundVecData,soundVecData->vectorInfo.totalSize);
	}
}

void
SndPlay::SetPowerAndVolume()
{
	OStatus result;

	result = OPENR::ControlPrimitive(speakerID,oprmreqSPEAKER_MUTE_OFF, 0, 0, 0, 0);
	if (result != oSUCCESS)
		OSYSLOG1((osyslogERROR, "%s : %s %d","SndPlay::SetPowerAndVolume()","OPENR::ControlPrimitive(SPEAKER_MUTE_OFF) FAILED", result));

	OPrimitiveControl_SpeakerVolume volume(config->sound.volume);
	result = OPENR::ControlPrimitive(speakerID,oprmreqSPEAKER_SET_VOLUME,&volume, sizeof(volume), 0, 0);
	if (result != oSUCCESS)
		OSYSLOG1((osyslogERROR, "%s : %s %d","SndPlay::SetPowerAndVolume()","OPENR::ControlPrimitive(SPEAKER_SET_VOLUME) FAILED",result));

	if (config->sound.sample_rate == 16000 && config->sound.sample_bits == 16) {
		OPrimitiveControl_SpeakerSoundType soundType(ospksndMONO16K16B);
		result = OPENR::ControlPrimitive(speakerID,oprmreqSPEAKER_SET_SOUND_TYPE,&soundType, sizeof(soundType), 0, 0);
		if (result != oSUCCESS)
			OSYSLOG1((osyslogERROR, "%s : %s %d","SndPlay::SetPowerAndVolume()","OPENR::ControlPrimitive(SPEAKER_SET_SOUND_TYPE) FAILED",result));
	}
}

/*! Will round up size to the nearest page */
RCRegion*
SndPlay::InitRegion(unsigned int size) {
	unsigned int pagesize=4096;
	sError err=GetPageSize(&pagesize);
	if(err!=sSUCCESS)
		std::cerr << "Error "<<err<<" getting page size " << pagesize << std::endl;
	unsigned int pages=(size+pagesize-1)/pagesize;
	return new RCRegion(pages*pagesize);
}

RCRegion*
SndPlay::FindFreeRegion()
{
	for (unsigned int i = 0; i < SOUND_NUM_BUFFER; i++)
		if (region[i]->NumberOfReference() == 1)
			return region[i];
	return 0;
}

/*! @file
 * @brief Implements the SndPlay process (a.k.a. OObject), which is responsible for sending sound buffers to the system to play
 * @author Sony (Creator)
 *
 * This is basically the SoundPlay example from the Sony code, with a few modifications.
 * Here's the license Sony provided with it:
 *
 * Copyright 2002,2003 Sony Corporation 
 *
 * Permission to use, copy, modify, and redistribute this software for
 * non-commercial use is hereby granted.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 */

