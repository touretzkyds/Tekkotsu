#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_MICROPHONE

#include "MicrophoneServer.h"

#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include "Shared/Config.h"
#include "Wireless/Wireless.h"

#include "Shared/ODataFormats.h"
#ifdef PLATFORM_APERIOS
#  include "OPENR/OPENRAPI.h"
#endif

typedef BehaviorSwitchControl<MicrophoneServer,SingletonFactory<MicrophoneServer> > MicrophoneServerControl;
REGISTER_CONTROL_INSTANCE_OPT(MicrophoneServer,new MicrophoneServerControl("Microphone Server",false),"TekkotsuMon",BEH_NONEXCLUSIVE);

MicrophoneServer* MicrophoneServer::instance = 0;
const char* const MicrophoneServer::MIC_LOCATOR = "PRM:/r1/c1/c2/c3/m1-Mic:M1";

MicrophoneServer& MicrophoneServer::getInstance() {
	if (instance == 0) {
		instance = new MicrophoneServer();
	}
	return *instance;
}

MicrophoneServer::MicrophoneServer()
	: BehaviorBase("Microphone Server"), socket(0) {
		
	addReference();
}

MicrophoneServer::~MicrophoneServer() {
	if (references == 1) {
		instance = 0;
	}
}


void MicrophoneServer::doStart() {
	BehaviorBase::doStart();
	
	if (socket != 0) {
		wireless->setDaemon(socket, false);
		wireless->close(socket);
		socket = 0;
	}
		
	socket = wireless->socket(Socket::SOCK_STREAM, 512, SEND_BUFFER_SIZE);
	wireless->setDaemon(socket, true);
	wireless->listen(socket->sock, config->sound.streaming.mic_port);
	
	erouter->addListener(this, EventBase::micOSndEGID);
}

void MicrophoneServer::doStop() {
	erouter->removeListener(this);
	
	if (socket != 0) {
		wireless->setDaemon(socket, false);
		wireless->close(socket);
		socket = 0;
	}
	
	BehaviorBase::doStop();
}

void MicrophoneServer::doEvent() {
	if (event->getGeneratorID() != EventBase::micOSndEGID) {
		return;
	}
	
	// Got an audio frame from the microphone
	if ((socket == 0) || (!wireless->isConnected(socket->sock))) {
		return;
	}
	
	const DataEvent<const OSoundVectorData*>* e =
	  reinterpret_cast<const DataEvent<const OSoundVectorData*>*>(event);
	OSoundVectorData* data = const_cast<OSoundVectorData *>(e->getData());
	const char* samples = reinterpret_cast<char*>(data->GetData(0));
	const int samplesSize = data->GetInfo(0)->dataSize;
	
	unsigned int sampleRate = config->sound.streaming.mic_sample_rate;
	unsigned int sampleBits = config->sound.streaming.mic_sample_bits;
	bool stereo = config->sound.streaming.mic_stereo;
	
	unsigned int newSamplesSize = GetResampledFrameSize(samplesSize, sampleRate, sampleBits, stereo);
	if (newSamplesSize == 0) {
		return;
	}
	
	const unsigned int headerSize = 8;
	char* buf = (char*) socket->getWriteBuffer(headerSize + newSamplesSize);
	if (buf == 0) {
		// Network not ready, drop this frame
		return;
	}
		
	unsigned int resampledSize = 0;
	resampledSize = ResampleFrame(samples, samplesSize, sampleRate, sampleBits, stereo, buf + headerSize, newSamplesSize);
	if (resampledSize != newSamplesSize) {
		return;
	}
	
	encode(&buf, (unsigned short) (newSamplesSize + headerSize - 4));
	encode(&buf, (unsigned short) 0); // PCM frame
	encode(&buf, (unsigned short) sampleRate);
	encode(&buf, (byte)	 sampleBits);
	encode(&buf, stereo);

	socket->write(headerSize + newSamplesSize);
}

unsigned int MicrophoneServer::GetResampledFrameSize(
	unsigned int samplesSize,
	unsigned int newSampleRate,
	unsigned int newSampleBits,
	bool newStereo) {
	
	if (newSampleRate > 16000) {
		newSampleRate = 16000;
	} else if (newSampleRate < 1) {
		newSampleRate = 1;
	}
	
	if (newSampleBits >= 12) {
		newSampleBits = 16;
	} else {
		newSampleBits = 8;
	}
	
	if ((newSampleRate == 16000) && (newSampleBits == 16) && (newStereo)) {
		// No need to resample
		return samplesSize;
	} else {
		// Resample from 16 kHz 16 bit stereo
		const unsigned int frameCount = samplesSize / 4; 
		const unsigned int newFrameCount = frameCount * newSampleRate / 16000;
		const unsigned int newFrameSize =
		((newSampleBits == 8) ? 1 : 2) * ((newStereo) ? 2 : 1);
		return newFrameCount * newFrameSize;		
	}
}

unsigned int MicrophoneServer::ResampleFrame(
	const char* samples,
	unsigned int samplesSize,
	unsigned int& newSampleRate,
	unsigned int& newSampleBits,
	bool& newStereo,
	void* buf,
	unsigned int bufSize) {
		 
	
	if (newSampleRate > 16000) {
		newSampleRate = 16000;
	} else if (newSampleRate < 1) {
		newSampleRate = 1;
	}
	
	if (newSampleBits >= 12) {
		newSampleBits = 16;
	} else {
		newSampleBits = 8;
	}
			
	if ((newSampleRate == 16000) && (newSampleBits == 16) && (newStereo)) {
		// No need to resample
		if (samplesSize <= bufSize) {
			memcpy(buf, samples, samplesSize);
			return samplesSize;
		} else {
			return 0;
		}
	} else {
		// Resample from 16 kHz 16 bit stereo
		const unsigned int frameCount = samplesSize / 4; 
		const unsigned int newFrameCount = frameCount * newSampleRate / 16000;
		const unsigned int newFrameSize =
			((newSampleBits == 8) ? 1 : 2) * ((newStereo) ? 2 : 1);
		unsigned int newSamplesSize = newFrameCount * newFrameSize;
		if (newSamplesSize > bufSize) {
			return 0;
		}
		
		char* newSamplesChar = (char*) buf;
		short* newSamplesShort = (short*) buf;
		const short* samplesShort = (const short*) samples;
		for (unsigned int newFrame = 0; newFrame < newFrameCount; newFrame++) {
			const int frame = newFrame * 16000 / newSampleRate;
			if (newSampleBits == 8) {
				// 8-bit
				if (newStereo) {
					// 8-bit stereo
					newSamplesChar[newFrame * 2 + 0] = samples[frame * 4 + 1];
					newSamplesChar[newFrame * 2 + 1] = samples[frame * 4 + 3];
				} else {
					// 8-bit mono
					newSamplesChar[newFrame] =
						((int) samplesShort[frame * 2 + 0]
							+ (int) samplesShort[frame * 2 + 1]) >> 9;
				}
			} else {
				// 16-bit
				if (newStereo) {
					// 16-bit stereo
					newSamplesShort[newFrame * 2 + 0] = samplesShort[frame * 2 + 0];
					newSamplesShort[newFrame * 2 + 1] = samplesShort[frame * 2 + 1];
				} else {
					// 16-bit mono
					newSamplesShort[newFrame] =
						((int) samplesShort[frame * 2 + 0]
							+ (int) samplesShort[frame * 2 + 1]) >> 1;
				}
			}
		}
		return newSamplesSize;
	}
}

#ifdef PLATFORM_APERIOS
bool MicrophoneServer::SetMicrophoneUnidirectional(bool unidirectional) {
	OPrimitiveID micID;
	OStatus result = OPENR::OpenPrimitive(MIC_LOCATOR, &micID);
	if (result != oSUCCESS) {
		return false;
	}
	
	result = OPENR::ControlPrimitive(
		micID, ((unidirectional) ? oprmreqMIC_UNI : oprmreqMIC_OMNI),	0, 0, 0, 0);
	if (result != oSUCCESS) {
		return false;
	}
#else
bool MicrophoneServer::SetMicrophoneUnidirectional(bool /*unidirectional*/) {
#endif
	return true;
}

#ifdef PLATFORM_APERIOS
bool MicrophoneServer::SetMicrophoneAlcEnabled(bool enabled) {
	OPrimitiveID micID;
	OStatus result = OPENR::OpenPrimitive(MIC_LOCATOR, &micID);
	if (result != oSUCCESS) {
		return false;
	}
	
	result = OPENR::ControlPrimitive(
		micID, ((enabled) ? oprmreqMIC_ALC_ON : oprmreqMIC_ALC_OFF),	0, 0, 0, 0);
	if (result != oSUCCESS) {
		return false;
	}
#else
bool MicrophoneServer::SetMicrophoneAlcEnabled(bool /*enabled*/) {
#endif	
	return true;
}

#endif
