//-*-c++-*-
#ifndef INCLUDED_MicrophoneServer_h_
#define INCLUDED_MicrophoneServer_h_

#include "Events/EventBase.h"
#include "Behaviors/BehaviorBase.h"

//! Streams audio from the microphone over the network
class MicrophoneServer : public BehaviorBase {
	public:
		//!enforces singleton status
		static MicrophoneServer& getInstance();
		virtual ~MicrophoneServer(); //!< destructor
		
		virtual void doStart();
		virtual void doStop();
		virtual void doEvent();
		
		//! makes Aperios-specific call to set microphone mode
		static bool SetMicrophoneUnidirectional(bool unidirectional);
		//! makes Aperios-specific call to set microphone mode
		static bool SetMicrophoneAlcEnabled(bool enabled);
		
	private:
		//! max transmission buffer size for #socket
		static const unsigned int SEND_BUFFER_SIZE = 2048 + 16;
	
		MicrophoneServer(); //!< constructor
		MicrophoneServer(const MicrophoneServer& rhs); //!< don't call
		MicrophoneServer& operator=(const MicrophoneServer& rhs); //!< don't call
		static MicrophoneServer* instance; //!< global instance of the server
		
		//! returns size of a "frame" at the given sampling rate and resolution
		unsigned int GetResampledFrameSize(
			unsigned int samplesSize,
			unsigned int newSampleRate,
			unsigned int newSampleBits,
			bool newStereo);
		
		//! performs sampling to a specified rate and resolution, stores into @a newSamples (which you must allocate)
		unsigned int ResampleFrame(
			const char* samples,
			unsigned int samplesSize,
			unsigned int& newSampleRate,
			unsigned int& newSampleBits,
			bool& newStereo,
			void* newSamples,
			unsigned int newSamplesSize);
		
		//! aperios specific identifier for microphone access
		static const char* const MIC_LOCATOR;
		
		//! socket for communication
		class Socket *socket;
		
		//! writes @a value to @a dst and advances @a dst
		template<class T>
		inline static void encode(char **dst, const T& value) {
			memcpy(*dst, &value, sizeof(T));
			(*dst) += sizeof(T);
		}
		
		//! writes @a length bytes from @a src to @a dst
		template<class T>
		inline static void encode(char **dst, const T * src, int num) {
			memcpy(*dst, src, num*sizeof(T));
			(*dst) += num*sizeof(T);
		}
};
#endif
