//-*-c++-*-
#ifndef INCLUDED_SpeakerServer_h_
#define INCLUDED_SpeakerServer_h_

#include "Behaviors/BehaviorBase.h"
#include "Sound/SoundManager.h"

//! Plays streamed audio via the speaker
class SpeakerServer : public BehaviorBase {
	public:
		static SpeakerServer& getInstance(); //!< returns global #instance
		virtual ~SpeakerServer(); //!< destructor
		
		virtual void doStart();
		virtual void doStop();
		
		//! registered by doStart() to be called by networking module with incoming data
		static int socket_callback(char *buf, int size);
	
	private:
		SpeakerServer(); //!< constructor
		SpeakerServer(const SpeakerServer& rhs); //!< don't call
		SpeakerServer& operator=(const SpeakerServer& rhs); //!< don't call
		static SpeakerServer* instance; //!< global instance of the server (only ever want to have one of these)
		
		int GotSocketData(char* data, int dataSize); //!< should be called with new sound data from the network
		class Socket *socket; //!< network communications socket for receiving sound data
		//! returns the next sizeof(short) bytes from @a buf as a short
		static short GetShort(const void* buf) { short result; memcpy(&result, buf, sizeof(short)); return result; }
		
		static const int MAX_PACKET_SIZE = 1024 * 1024; //!< maximum size of sound buffer to send to system
		static const int RECEIVE_BUFFER_SIZE = 2048; //!< maximum network packet size to accept
	
		//! stores information about current sound buffer
		class Packet {
			public:
				Packet(); //!< constructor
				virtual ~Packet(); //!< destructor
				
				class Buffer* header; 
				int size;
				int type;
				bool skipped;
				
				class Buffer* pcmHeader;
				unsigned short sampleRate;
				byte sampleBits;
				
				class Buffer* samples;
			
			private:
				Packet(const Packet& rhs); //!< don't call
				Packet& operator=(const Packet& rhs); //!< don't call
		};
		
		Packet packet; 
		class Buffer* frame;
		class Buffer* resampled;
		
		void AddPacket(
			const void* samples,
			int samplesSize,
			int sampleRate,
			byte sampleBits);
		
		const void* ResampleForSpeaker(
			const void* samples,
			int samplesSize,
			int sampleRate,
			byte bitsPerSample,
			int& newSamplesSize);
				
		void QueueFrame(const char* samples, int samplesSize);
		SoundManager::Play_ID channel;
};
#endif
