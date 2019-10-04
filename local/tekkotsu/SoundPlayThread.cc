#include "SoundPlayThread.h"
#include "Shared/Config.h"
#include "Sound/SoundManager.h"

#ifdef __linux__
#include <alsa/asoundlib.h>
#endif

/* ******************************************************************/
#ifdef __APPLE__
/* ******************************************************************/


#include <AudioToolbox/AudioToolbox.h>
#include <AudioUnit/AudioUnitProperties.h>

struct SoundPlayContext {
	SoundPlayContext() : outputStarted(false), output(), streamDesc() {}
	bool outputStarted;
	AudioUnit output;
	AudioStreamBasicDescription streamDesc;
};

void SoundPlayThread::reset() {
	if(!initSuccess)
		return;
	if(sndman->getNumPlaying()<=0 && context->outputStarted) {
		context->outputStarted=false;
		if(noErr != AudioOutputUnitStop(context->output)) {
			std::cerr << "WARNING: had error while stopping audio unit" << std::endl;
		}
	} else if(sndman->getNumPlaying()>0 && !context->outputStarted) {
		if(noErr != AudioOutputUnitStart(context->output)) {
			std::cerr << "ERROR: Could not start audio output" << std::endl;
			closeSystem();
			return;
		}
		context->outputStarted=true;
	}
}

static OSStatus audioCallback(void *inRefCon, AudioUnitRenderActionFlags* ioActionFlags, const AudioTimeStamp* inTimeStamp, UInt32 inBusNumber, UInt32 inNumberFrames, AudioBufferList* ioData) {
	SoundPlayContext* context = reinterpret_cast<SoundPlayContext*>(inRefCon);
	//std::cout << sndman->getNumPlaying() << ' ' << context->outputStarted << ' ' << ' ' << inNumberFrames << ' ' << ioData->mNumberBuffers;
	
	for(typeof(ioData->mNumberBuffers) b=0; b<ioData->mNumberBuffers; ++b) {
		AudioBuffer& bufInfo = ioData->mBuffers[b];
		//std::cout << ' ' << bufInfo.mNumberChannels << ' ' << bufInfo.mDataByteSize;
		// beware bufInfo.mNumberChannels!=1...
		sndman->CopyTo(bufInfo.mData,bufInfo.mDataByteSize);
	}
	
	if(sndman->getNumPlaying()<=0 && context->outputStarted) {
		context->outputStarted=false;
		if(noErr != AudioOutputUnitStop(context->output)) {
			std::cerr << "WARNING: had error while stopping audio unit" << std::endl;
		}
	}
	
	//std::cout << '\n';
	return noErr;
}

void SoundPlayThread::openSystem() {
	if(context==NULL)
		context = new SoundPlayContext;
	
	AudioComponentDescription cd;
	cd.componentType = kAudioUnitType_Output;
	cd.componentSubType = kAudioUnitSubType_DefaultOutput;
	cd.componentManufacturer = 0;
	cd.componentFlags = 0;
	cd.componentFlagsMask = 0;
	
	AudioComponent comp = AudioComponentFindNext(NULL, &cd);
	if(comp == NULL) {
		std::cerr << "ERROR: Could not find default audio output" << std::endl;
		return;
	}
	
	AudioUnit& output = context->output;
	if(noErr != AudioComponentInstanceNew(comp, &output)) {
		std::cerr << "ERROR: Could not open audio output component" << std::endl;
		return;
	}
	
	if(noErr != AudioUnitInitialize(output)) {
		std::cerr << "ERROR: Could not initialize audio output" << std::endl;
		AudioComponentInstanceDispose(output);
		return;
	}
	
	AURenderCallbackStruct callbackData;
	callbackData.inputProc = audioCallback;
	callbackData.inputProcRefCon = context;
	
	if(noErr != AudioUnitSetProperty(output, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Global, 0, &callbackData, sizeof(callbackData))) {
		std::cerr << "ERROR: Could not set audio callback" << std::endl;
		closeSystem();
		return;
	}
	
	AudioStreamBasicDescription& streamDesc = context->streamDesc;
	streamDesc.mFormatID = kAudioFormatLinearPCM;
	streamDesc.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger | kLinearPCMFormatFlagIsPacked /* | kLinearPCMFormatFlagIsBigEndian*/;
	streamDesc.mSampleRate = config->sound.sample_rate;
	streamDesc.mBitsPerChannel = config->sound.sample_bits;
	streamDesc.mChannelsPerFrame = 1;
	streamDesc.mBytesPerFrame = streamDesc.mChannelsPerFrame * streamDesc.mBitsPerChannel/8;
	streamDesc.mFramesPerPacket = 1;
	streamDesc.mBytesPerPacket = streamDesc.mFramesPerPacket * streamDesc.mBytesPerFrame;
	
	if(noErr != AudioUnitSetProperty(output, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &streamDesc, sizeof(streamDesc))) {
		std::cerr << "ERROR: Could not set audio format" << std::endl;
		closeSystem();
		return;
	}
	
	UInt32 streamDescSize = sizeof(streamDesc);
	if(noErr != AudioUnitGetProperty( output, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &streamDesc, &streamDescSize)) {
		std::cerr << "ERROR: Could not verify audio format" << std::endl;
		closeSystem();
		return;
	}
	
	initSuccess=true;
}

void SoundPlayThread::closeSystem() {
	initSuccess=false;
	if(context==NULL)
		return;
	
	if(context->outputStarted) {
		if(noErr != AudioOutputUnitStop(context->output)) {
			std::cerr << "WARNING: had error while stopping audio unit" << std::endl;
		}
	}
	
	if(noErr != AudioUnitUninitialize(context->output)) {
		std::cerr << "WARNING: had error while closing audio unit" << std::endl;
	}
	
	if(noErr != AudioComponentInstanceDispose(context->output)) {
		std::cerr << "WARNING: had error while closing audio component" << std::endl;
	}
	
	delete context; context=NULL;
}


/* ******************************************************************/
#else // linux or "other"
/* ******************************************************************/

void SoundPlayThread::reset() {
	//std::cout << "SOUND: reset" << std::endl;

	if(!initSuccess)
		return;

	MarkScope autolock(lock);

	if(sndman->getNumPlaying()<=0 && poller.isStarted()) {
		poller.stop();
		//std::cout << "SOUND: draining" << std::endl;
		if (snd_pcm_drain(pcm_handle) < 0) {
			std::cerr << "SOUND: Error stopping PCM device." << std::endl;
		}
	}
	else if(sndman->getNumPlaying()>0 && !poller.isStarted()) {
		//buffersInFlight=0;
		//std::cout << "SOUND: preparing" << std::endl;
		if (snd_pcm_prepare(pcm_handle) < 0) {
			std::cerr << "SOUND: Error preparing PCM device." << std::endl;
		}
		poller.start();
	}
}

void SoundPlayThread::openSystem()
{
	//std::cout << "SOUND: open" << std::endl;
	MarkScope autolock(lock);

	unsigned int rate = config->sound.sample_rate;
	unsigned int exact_rate;
	int dir;          /* exact_rate == rate --> dir = 0 */
	/* exact_rate < rate  --> dir = -1 */
	/* exact_rate > rate  --> dir = 1 */
	int frame_bits;

	unsigned int period_time = 200000;
	unsigned int buffer_time = 1000000;
	
	snd_pcm_format_t format;
	if (config->sound.sample_bits == 8)
		format = SND_PCM_FORMAT_S8;
	else
		format = SND_PCM_FORMAT_S16_LE;
  
	snd_pcm_hw_params_t *hwparams=NULL; // apparently this does not need a 'free'
	snd_pcm_hw_params_alloca(&hwparams); // (crashes if we try to 'free' afterward)

	if (snd_pcm_open(&pcm_handle, "plughw:0,0", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
		if (snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
			std::cerr << "SOUND: Error opening PCM device 'plughw:0,0' and alternate 'default'" << std::endl;
			goto pcm_close;
		}
	}

	/* Init hwparams with full configuration space */
	if (snd_pcm_hw_params_any(pcm_handle, hwparams) < 0) {
		std::cerr << "SOUND: Can not configure this PCM device." << std::endl;
	}

	// setup basic access
	if (snd_pcm_hw_params_set_access(pcm_handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
		std::cerr << "SOUND: Error setting access." << std::endl;
		goto pcm_close;
	}
  
	if (snd_pcm_hw_params_set_format(pcm_handle, hwparams, format) < 0) {
		std::cerr << "SOUND: Error setting format." << std::endl;
		goto pcm_close;
	}

	/* Set sample rate. If the exact rate is not supported */
	/* by the hardware, use nearest possible rate.         */ 
	exact_rate = rate;
	if (snd_pcm_hw_params_set_rate_near(pcm_handle, hwparams, &exact_rate, 0) < 0) {
		std::cerr << "SOUND: Error setting rate." << std::endl;
		goto pcm_close;
	}
	if (rate != exact_rate) {
		std::cerr << "SOUND: The rate " << rate << " Hz is not supported by your hardware." << std::endl
							<< " ==> Using " << exact_rate << " Hz instead." << std::endl;
	}

	//printf("rate is %d.\n",exact_rate);

	/* Set number of channels */
	if (snd_pcm_hw_params_set_channels(pcm_handle, hwparams, 1) < 0) {
		std::cerr << "SOUND: Error setting channels." << std::endl;
		goto pcm_close;
	}

	// setup period and buffer size
	if (snd_pcm_hw_params_set_period_time_near(pcm_handle, hwparams, &period_time, &dir) < 0) {
		std::cerr << "SOUND: Error setting period time." << std::endl;
		goto pcm_close;
	}

	if (snd_pcm_hw_params_get_period_size(hwparams, &period_size, &dir) < 0) {
		std::cerr << "SOUND: Error getting period size." << std::endl;
		goto pcm_close;
	}

	//std::cout << "SOUND: Period size is " << period_size << std::endl;

	if (snd_pcm_hw_params_set_buffer_time_near(pcm_handle, hwparams, &buffer_time, &dir) < 0) {
		std::cerr << "SOUND: Error getting buffer time." << std::endl;
		goto pcm_close;
	}

	if (snd_pcm_hw_params_get_buffer_size(hwparams, &buffer_size) < 0) {
		std::cerr << "SOUND: Error getting buffer size." << std::endl;
		goto pcm_close;
	}

	//std::cout << "SOUND: Buffer size is " << buffer_size << std::endl;

	/* get the frame size */
	if ((frame_bits = snd_pcm_format_physical_width(format)) < 0) {
		std::cerr << "SOUND: Error getting frame size." << std::endl;
		goto pcm_close;
	}
	
	frame_size = frame_bits / 8;
	//std::cout << "SOUND: Frame size is " << frame_size << ", " << frame_bits << " bits." << std::endl;

	/* Apply HW parameter settings to */
	/* PCM device and prepare device  */
	if (snd_pcm_hw_params(pcm_handle, hwparams) < 0) {
		std::cerr << "SOUND: Error setting HW params." << std::endl;
		goto pcm_close;
	}
	
	initSuccess=true;
	return;

 pcm_close:
	if (pcm_handle) {
		//std::cout << "SOUND: closing" << std::endl;
		if (snd_pcm_close(pcm_handle) < 0) {
			std::cerr << "SOUND: Error closing PCM device." << std::endl;
		}
		pcm_handle = NULL;
	}

	return;
}

void SoundPlayThread::closeSystem() {
	initSuccess=false;
	if(poller.isStarted())
		poller.stop().join();
	delete buf; buf=NULL;

	if (pcm_handle) {
		MarkScope autolock(lock);

		//std::cout << "SOUND: dropping" << std::endl;
		if (snd_pcm_drop(pcm_handle) < 0) {
			std::cerr << "SOUND: Error stopping PCM device." << std::endl;
		}
		//std::cout << "SOUND: closing" << std::endl;
		if (snd_pcm_close(pcm_handle) < 0) {
			std::cerr << "SOUND: Error closing PCM device." << std::endl;
		}
		pcm_handle = NULL;
	}

	//std::cout << "SOUND: close" << std::endl;
}

bool SoundPlayThread::poll() {
	if (!buf) {
		buf = new char[period_size];
	}

	int left = period_size / frame_size;
	char *ptr = buf;

	sndman->CopyTo(buf, period_size);
	
	MarkScope autolock(lock);

	while (left > 0) {
		int wrote = snd_pcm_writei(pcm_handle, ptr, left);
		//std::cout << "SOUND: poll write with ptr " << (int)ptr << " and " << left << " bytes left wrote " << wrote << std::endl;

		if (wrote < 0) {
			if (wrote == -EPIPE) {    /* under-run */
				//std::cout << "SOUND: Buffer underrun." << std::endl;
				if (snd_pcm_prepare(pcm_handle) < 0) {
					std::cerr << "SOUND: Error preparing PCM device." << std::endl;
					return false;
				}
				return true;
			}
		}

		ptr += wrote * frame_size;
		left -= wrote;
	}

	//std::cout << "SoundPlayThread polled " << 0 << ' ' << sndman->getNumPlaying() << std::endl;
	return sndman->getNumPlaying()>0;
}

/* ******************************************************************/
#endif
/* ******************************************************************/


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
