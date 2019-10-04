//-*-c++-*-
#ifndef INCLUDED_SoundPlayThread_h_
#define INCLUDED_SoundPlayThread_h_

#include "IPC/CallbackThread.h"

#ifdef __linux__
#include <alsa/asoundlib.h>
#endif

class SoundPlayThread {
public:
	SoundPlayThread() : initSuccess(false),
#ifdef __APPLE__
		context(NULL)
#else
		poller(&SoundPlayThread::poll,*this,TimeET(0L),TimeET(10L),true,CallbackPollThread::STOP_FALSE),
											/*											buf(NULL), bufsize(0), buffersInFlight(0),*/
											lock(), pcm_handle(NULL), buffer_size(0), period_size(0), frame_size(0), buf(NULL)
#endif
	{
		openSystem();
	}
	virtual ~SoundPlayThread() {
		if(initSuccess)
			closeSystem();
	}
	
	void reset();
	
protected:
	void openSystem();
	void closeSystem();
	
	bool initSuccess;
	
#ifdef __APPLE__
	struct SoundPlayContext * context;
#else
	virtual bool poll();
	CallbackPollThread poller;
	/*
	static const long BUFFER_TIME=32;
	static const unsigned int NUM_BUFFERS=2;
	char * buf;
	size_t bufsize;
	unsigned int buffersInFlight;
	*/

	Thread::Lock lock;

	snd_pcm_t * pcm_handle;

	snd_pcm_uframes_t buffer_size;
	snd_pcm_uframes_t period_size;
	unsigned int frame_size;

	char * buf;

#endif
	
private:
	SoundPlayThread(const SoundPlayThread& l); //!< don't call
	SoundPlayThread& operator=(const SoundPlayThread& l); //!< don't call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
