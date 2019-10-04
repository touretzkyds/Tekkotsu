#ifndef INCLUDED_SndPlay_h_
#define INCLUDED_SndPlay_h_

#include "aperios/EntryPoint.h"
#include <OPENR/OObject.h>
#include <OPENR/OSubject.h>
#include <OPENR/OObserver.h>
#include "aperios/SndPlay/def.h"

#include "Events/EventTranslator.h"

//! The process (a.k.a. OObject), which is responsible for sending sound buffers to the system to play
/*! This sound process will purposely starve the system of sound buffers when nothing is
 *  playing, both to eliminate needless zeroing of entire buffers, as well as to reduce system
 *  overhead of playing empty buffers.
 *
 *  If you want to know how to play sounds, you should be looking at SoundManager's documentation.
 *  
 *  Basically a slightly modified version of the SoundPlay example code from Sony.  Here's their license:
 *  Copyright 2002,2003 Sony Corporation 
 *
 *  Permission to use, copy, modify, and redistribute this software for
 *  non-commercial use is hereby granted.
 *
 *  This software is provided "as is" without warranty of any kind,
 *  either expressed or implied, including but not limited to the
 *  implied warranties of fitness for a particular purpose.
 */
class SndPlay : public OObject {
 public:
	SndPlay();            //!< constructor
	virtual ~SndPlay() {} //!< destructor

	virtual OStatus DoInit   (const OSystemEvent& event); //!< called by system when time to do init
	virtual OStatus DoStart  (const OSystemEvent& event); //!< called by system when time to start running
	virtual OStatus DoStop   (const OSystemEvent& event); //!< called by system when time to stop running
	virtual OStatus DoDestroy(const OSystemEvent& event); //!< called by system when time to free

	void ReadySendSound(const OReadyEvent& event);            //!< called by system when it's ready for another sound buffer
	void ReadyRegisterSoundManager(const OReadyEvent& event); //!< called by system when observers are ready to receive the SoundManager
	void ReadyRegisterProfiler(const OReadyEvent&); //!< send out sound's profiler

	void GotSoundMsg(const ONotifyEvent& event); //!< called by system when SoundManager has sent itself a message on a different process (either to add or remove sounds from memory)
	void GotProcessMap(const ONotifyEvent& event); //!< called when process map is received from Main

	OSubject*  subject[numOfSubject];   //!< array of subject IDs, used to identify outgoing data
	OObserver* observer[numOfObserver]; //!< array of observer IDs, used to identify what's ready

 private:
	void      doSendSound();                 //!< called to send sound buffer(s) to system
	void      OpenSpeaker();                 //!< initializes speaker
	void      NewSoundVectorData();          //!< sets up sound buffers
	void      SetPowerAndVolume();           //!< sets volume to max
	RCRegion* InitRegion(unsigned int size); //!< inits each buffer
	RCRegion* FindFreeRegion();              //!< finds the first sound buffer which system isn't using (buffers are recycled)
		
	static const size_t SOUND_NUM_BUFFER = 2;                                    //!< number of buffers to use
	
	unsigned int active; //!< number of active sound channels - if it's 0, we'll purposely starve system of sound buffers

	RCRegion*      soundManagerMemRgn; //!< SndPlay creates, Main & Motion receive - Shared region used by SoundManager
	RCRegion*      processMapMemRgn; //!< Main creates, SndPlay receives
	RCRegion*      soundProfilerMemRgn; //!< Sound creates, Main receives
	
	EventTranslator * etrans; //!< will be given all events created by SoundManager to be forwarded to Main
	EntryPoint entryPt; //!< should be used with MarkScope to wrap each entry point from the system, handles marking root stack frame

	OPrimitiveID   speakerID;  //!< ID returned to system after opening SPEAKER_LOCATOR
	RCRegion*      region[SOUND_NUM_BUFFER]; //!< holds references to shared regions holding sound clips

	SndPlay(const SndPlay&); //!< don't call
	SndPlay& operator=(const SndPlay&); //!< don't call
};

/*! @file
 * @brief Describes the SndPlay process (a.k.a. OObject), which is responsible for sending sound buffers to the system to play
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

#endif // SndPlay_h_DEFINED
