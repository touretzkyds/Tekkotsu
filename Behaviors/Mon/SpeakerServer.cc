#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_SPEAKERS

#include "SpeakerServer.h"

#include "Shared/Buffer.h"
#include "Shared/Config.h"
#include "Shared/RobotInfo.h"
#include "Wireless/Wireless.h"

typedef BehaviorSwitchControl<SpeakerServer,SingletonFactory<SpeakerServer> > SpeakerServerControl;
REGISTER_CONTROL_INSTANCE_OPT(SpeakerServer,new SpeakerServerControl("Speaker Server",false),"TekkotsuMon",BEH_NONEXCLUSIVE);

SpeakerServer* SpeakerServer::instance = 0;

SpeakerServer& SpeakerServer::getInstance() {
	if (instance == 0) {
		instance = new SpeakerServer();
	}
	return *instance;
}

SpeakerServer::SpeakerServer()
	: BehaviorBase("Speaker Server"), socket(0),
		packet(), frame(0), resampled(0),
		channel(SoundManager::invalid_Play_ID) {
		
	addReference();
}

SpeakerServer::~SpeakerServer() {
	delete frame;
	delete resampled;
	
	if (references == 1) {
		instance = 0;
	}
}


void SpeakerServer::doStart() {
	BehaviorBase::doStart();
	
	if (socket != 0) {
		wireless->setDaemon(socket, false);
		wireless->close(socket);
		socket = 0;
	}
	
	packet.header->SetPosition(0);
	channel = SoundManager::invalid_Play_ID;
		
	socket =
		wireless->socket(Socket::SOCK_STREAM, RECEIVE_BUFFER_SIZE + 8, 512);
	wireless->setReceiver(socket->sock, socket_callback);
	wireless->setDaemon(socket, true);
	wireless->listen(socket->sock, config->sound.streaming.speaker_port);
}

void SpeakerServer::doStop() {
	if (socket != 0) {
		wireless->setDaemon(socket, false);
		wireless->close(socket);
		socket = 0;
	}
	
	sndman->stopPlay(channel);
	channel = SoundManager::invalid_Play_ID;
	
	packet.samples->SetCapacity(0);
	
	delete frame;
	delete resampled;

	BehaviorBase::doStop();
}

int SpeakerServer::socket_callback(char *buf, int size) {
	if (instance == 0) {
		return 0;
	} else {
		return instance->GotSocketData(buf, size);
	}
}

SpeakerServer::Packet::Packet()
	: header(new Buffer(4)), size(0), type(0), skipped(false),
		pcmHeader(new Buffer(4)), sampleRate(0), sampleBits(0),
		samples(new Buffer(0)) {}
		
SpeakerServer::Packet::~Packet() {
	delete header;
	delete pcmHeader;
	delete samples;
}

int SpeakerServer::GotSocketData(char* data, int dataSize) {
	while (dataSize > 0) {
		if (!packet.header->IsFull()) {
			if (!packet.header->Fill(data, dataSize)) {
				break;
			}
			packet.size =
				(unsigned short) GetShort(&packet.header->GetData()[0]);
			packet.type = GetShort(&packet.header->GetData()[2]);
			if ((packet.type == 0) && (packet.size >= 4)) {
				// PCM packet
				packet.pcmHeader->SetPosition(0);
				packet.skipped = false;
			} else {
				packet.skipped = true;
			}
		}
		
		if (packet.skipped) {
			if (packet.size > dataSize) {
				packet.size -= dataSize;
				break;
			} else {
				data += packet.size;
				dataSize -= packet.size;
				// Start reading next packet
				packet.header->SetPosition(0);
				continue;
			}
		}
		
		if (!packet.pcmHeader->IsFull()) {
			if (!packet.pcmHeader->Fill(data, dataSize)) {
				break;
			}
			packet.size -= packet.pcmHeader->GetLimit();
			packet.sampleRate =
				(unsigned short) GetShort(&packet.pcmHeader->GetData()[0]);
			packet.sampleBits = packet.pcmHeader->GetData()[2];
			
			const int resampledSize =
				packet.size *
					((packet.sampleBits == 8) ? 2 : 1)
					* config->sound.sample_rate / packet.sampleRate;
			
			if ((packet.size > MAX_PACKET_SIZE)
						|| (resampledSize > MAX_PACKET_SIZE)) {
				// Too many samples
				packet.skipped = true;
				continue;
			}
			packet.samples->SetCapacity(MAX_PACKET_SIZE);
			packet.samples->SetLimit(packet.size);
			packet.samples->SetPosition(0);
		}
				
		if (!packet.samples->IsFull()) {
			if (!packet.samples->Fill(data, dataSize)) {
				break;
			}
			AddPacket(
				packet.samples->GetData(),
				packet.samples->GetLimit(),
				packet.sampleRate,
				packet.sampleBits);
			// Start reading next packet
			packet.header->SetPosition(0);
			continue;
		}
	}

	return 0;
}

void SpeakerServer::AddPacket(
	const void* samples,
	int samplesSize,
	int sampleRate,
	byte sampleBits) {

	if (samplesSize < 1) {
		return;
	}

	samples =
		ResampleForSpeaker(
			samples, samplesSize, sampleRate, sampleBits, samplesSize);
	if (samples == 0) {
		return;
	}
	
	const int frameSize =
		config->sound.streaming.speaker_frame_length
			* config->sound.sample_rate
			* ((config->sound.sample_bits == 8u) ? 1 : 2)
			/ 1000;
	if (frame == 0) {
		frame = new Buffer(frameSize);
	} else if (frameSize != frame->GetLimit()) {
		if (frame->GetPosition() > frameSize) {
			QueueFrame(frame->GetData(), frame->GetPosition());
			frame->SetPosition(0);
		} else {
			frame->SetLimit(frameSize);
		}
		
		if (frame->GetLimit() < frameSize) {
			if (frame->GetCapacity() < frameSize) {
				frame->SetCapacity(frameSize);
			}
			frame->SetLimit(frameSize);
		}
	}
	
	if (frame->GetLimit() < 1) {
		return;
	}
	
	const char* buf = (const char*) samples;
	while (frame->Fill(buf, samplesSize)) {
		QueueFrame(frame->GetData(), frame->GetPosition());
		frame->SetPosition(0);
	}
}
		
		
void SpeakerServer::QueueFrame(const char* samples, int samplesSize) {
	if (channel != SoundManager::invalid_Play_ID) {
		const int remainingTime = sndman->getRemainTime(channel) - RobotInfo::SoundBufferTime;
		if (remainingTime > (int) config->sound.streaming.speaker_max_delay) {
			// Queue too long
			sndman->stopPlay(channel);
			channel = SoundManager::invalid_Play_ID;
		} else if (remainingTime < 0) {
			// Queue underrun
		} else {
			// Try queueing
			SoundManager::Play_ID newChannel =
				sndman->chainBuffer(channel, samples, (int) samplesSize);
			if (newChannel != SoundManager::invalid_Play_ID) {
				channel = newChannel;
				return;
			}
		}
	}
	
	// Start a new channel
	channel = sndman->playBuffer(samples, samplesSize);
}

const void* SpeakerServer::ResampleForSpeaker(
	const void* samples,
	int samplesSize,
	int sampleRate,
	byte bitsPerSample,
	int& newSamplesSize) {
		
	const int newSampleRate = config->sound.sample_rate;
	const int newBitsPerSample = config->sound.sample_bits;
	
	if ((sampleRate == newSampleRate) && (bitsPerSample == newBitsPerSample)) {
		newSamplesSize = samplesSize;
		return samples;
	}
	
	const int sampleCount = samplesSize / ((bitsPerSample == 16) ? 2 : 1);	
	const int newSampleCount = sampleCount * newSampleRate / sampleRate;
	newSamplesSize = newSampleCount * ((newBitsPerSample == 16) ? 2 : 1);
	
	if (newSampleCount == 0) {
		newSamplesSize = 0;
		return 0;
	}
	
	if (resampled == 0) {
		resampled = new Buffer(newSamplesSize);
	} else if (resampled->GetCapacity() < newSamplesSize) {
		resampled->SetCapacity(newSamplesSize);
	}
	
	// The code below is biased towards upsampling (normal case).
	// It does not average samples during downsampling.
	
	if (bitsPerSample == 16) {
		// 16-bit signed source
		const short* source = (const short*) samples; 
		if (newBitsPerSample == 16) {
			// 16-bit signed source, 16-bit signed destination
			short* dest = (short*) resampled->GetData();
			for (int i = 0; i < newSampleCount; i++) {  
				dest[i] = source[i * sampleRate / newSampleRate];
			}
		} else if (newBitsPerSample == 8) {
			// 16-bit signed source, 8-bit unsigned destination
			byte* dest = (byte*) resampled->GetData();
			for (int i = 0; i < newSampleCount; i++) {  
				dest[i] = ((byte) (source[i * sampleRate / newSampleRate] >> 8)) ^ 0x80;
			}
		} else {
			newSamplesSize = 0;
			return 0;
		}
	} else if (bitsPerSample == 8) {
		// 8-bit unsigned source
		const byte* source = (const byte*) samples;
		if (newBitsPerSample == 8) {
			// 8-bit unsigned source, 8-bit unsigned destination
			byte* dest = (byte*) resampled->GetData();
			for (int i = 0; i < newSampleCount; i++) {  
				dest[i] = source[i * sampleRate / newSampleRate];
			}
		} else if (newBitsPerSample == 16) {
			// 8-bit unsigned source, 16-bit signed destination
			short* dest = (short*) resampled->GetData();
			for (int i = 0; i < newSampleCount; i++) {  
				dest[i] = (source[i * sampleRate / newSampleRate] ^ 0x80) << 8;
			}
		} else {
			newSamplesSize = 0;
			return 0;
		}
	} else {
		newSamplesSize = 0;
		return 0;
	}
	
	return resampled->GetData();
}

#endif
