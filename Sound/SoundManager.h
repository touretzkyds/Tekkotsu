//-*-c++-*-
#ifndef INCLUDED_SoundManager_h_
#define INCLUDED_SoundManager_h_

#include "IPC/RCRegion.h"
#include "Shared/ODataFormats.h"

#include <string>
#include <vector>
#include "IPC/ListMemBuf.h"
#include "IPC/MutexLock.h"
#include "SoundManagerMsg.h"
#include "IPC/ProcessID.h"
#include "Shared/attributes.h"

#ifdef PLATFORM_APERIOS
class OSubject;
class ONotifyEvent;
#else
#  include "IPC/MessageQueue.h"
#endif

//! Provides sound effects and caching services, as well as mixing buffers for the SoundPlay process
/*! Provides easy methods for playing back sounds, either from files
 *  on the memory stick, or from dynamically generated buffers.  You
 *  can chain playback commands so that when one sound finishes,
 *  another picks up automatically.  This might be handy if, say,
 *  someone wants to write an MP3 player ;) The sounds would be too
 *  large to load into memory all at once, but you could load a block
 *  at a time and chain them so it seamlessly moves from one to the
 *  other.
 *  
 *  You can also preload sounds (loadFile()) before playing them (play() / playFile()) so
 *  there's no delay between requesting a sound and having it start playing while it is loaded from disk/memory stick.
 *  Just be sure to release the file (releaseFile()) again
 *  when you're done with it ;)
 *
 *  All functions will attempt to lock the SoundManager.  Remember,
 *  this is running in a shared memory region, accessible by the
 *  SoundPlay process and both the Main and Motion processes (so
 *  MotionCommands can play sounds!)
 *
 *  One could be tempted to draw parallels to the MotionManager, and
 *  envision a system with SoundCommands that are handed over and can
 *  dynamically compute sound buffers as needed.  If you have the time
 *  and inclination, the job's all yours... (Midi players, speech
 *  synthesizer, ...?)
 *
 *  @todo Volume control, variable playback speed, support more wav
 *  file formats (latter two are the same thing if you think about it - need
 *  to be able to resample on the fly)
 *  
 *  @todo Add functions to hand out regions to be filled out to avoid
 *  copying into the buffer.
 *
 *  @see <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/sound.shtml">David Touretzky's "Playing Sounds" Chapter</a>
 */
class SoundManager {
	friend void speechDoneCleanup(long playid);
public:
	//! destructor
	virtual ~SoundManager();

	//!constructor, should only be called by the receiving process (SoundPlay)
	SoundManager();
#ifdef PLATFORM_APERIOS
	//!Each process needs to call this before it can send sounds to the SoundPlay process
	void InitAccess(OSubject* subj);
#else //PLATFORM_LOCAL
	//!Each process (except SoundPlay) needs to call this before it can send sounds to the SoundPlay process
	void InitAccess(MessageQueueBase& sndbufq);
#endif
	
	//!This is used for referring to sound data so you can start playing it or release it
	typedef SoundManagerMsg::Snd_ID Snd_ID;
	static const Snd_ID invalid_Snd_ID=(Snd_ID)-1; //!< for reporting errors
	static const Snd_ID MAX_SND=50; //!< the number of sounds that can be loaded at any given time
	
	//!This is for referring to instances of the play command so you can stop, pause, or monitor progress (later versions will send events upon completion)
	typedef unsigned short Play_ID;
	static const Play_ID invalid_Play_ID=(Play_ID)-1; //!< for reporting errors
	static const Play_ID MAX_PLAY=256; //!< the number of sounds that can be enqueued at the same time (see MixMode_t)

	static const unsigned int MAX_NAME_LEN=128;   //!<maximum length of a path

	//!Used to set the mode for mixing multiple sound channels
	/*!Feel free to add a higher quality mixer if you're an audiophile - I'm pretty new to sound processing*/
	enum MixMode_t {
		// there's some prototype code for a bit-shifting 'Faster' quality level, but it hasn't been finished... 'Fast' is the default for now.
		Fast,    //!< uses real division to maintain volume level, without increasing intermediary precision, which causes low-order bit error in exchange for less CPU usage
		Quality  //!< uses real division to maintain volume level, using an intermediary higher precision buffer for mixing
	};

	//! indicates how to handle channel overflow (trying to play more sounds than maximum number of mixing channels). See #queue_mode
	enum QueueMode_t {
		Enqueue,        //!< newer sounds are played when a channel opens up (when old sound finishes)
		Pause,          //!< newer sounds pause oldest sound, which continues when a channel opens
		Stop,           //!< newer sounds stop oldest sound
		Override,       //!< older sounds have play heads advanced, but don't get mixed until a channel opens
	};

	//!loads a wav file (if it matches Config::sound_config settings - can't do resampling yet)
	/*!Since the SoundManager does the loading, if the same file is being played more than once, only once copy is stored in memory 
	 * @param name can be either a full path, or a partial path relative to Config::sound_config::root
	 * @return ID number for future references (can also use name)
	 * The sound data will be cached until releaseFile() or release() is called a matching number of times*/
	Snd_ID loadFile(std::string const &name);

	//!loads raw samples from a buffer (assumes matches Config::sound_config settings)
	/*!The sound data will be cached until release() is called a matching number of times.\n
	 * This function is useful for dynamic sound sources.  A copy will be made. */
	Snd_ID loadBuffer(const char buf[], unsigned int len);
	
	//!Marks the sound buffer to be released after the last play command completes (or right now if not being played)
	void releaseFile(std::string const &name);

	//!Marks the sound buffer to be released after the last play command completes (or right now if not being played)
	void release(Snd_ID id);
	
	//!play a wav file (if it matches Config::sound_config settings - can't do resampling yet)
	/*!Will do a call to loadFile() first, and then automatically release the sound again when complete.
	 * @param name can be either a full path, or a partial path relative to Config::sound_config::root
	 * @return ID number for future references
	 * The sound data will not be cached after done playing unless a call to loadFile is made*/
	Play_ID playFile(std::string const &name);

	//!loads raw samples from a buffer (assumes buffer matches Config::sound_config settings)
	/*!The sound data will be released after done playing*/
	Play_ID playBuffer(const char buf[], unsigned int len);
	
	//!plays a previously loaded buffer or file
	Play_ID play(Snd_ID id);
	
	//!allows automatic queuing of sounds - good for dynamic sound sources!
	/*!if you chain more than once to the same base, the new buffers are appended
	 * to the end of the chain - the new buffer doesn't replace the current chain
	 * @return @a base - just for convenience of multiple calls*/
	Play_ID chainFile(Play_ID base, std::string const &next);

	//!allows automatic queuing of sounds - good for dynamic sound sources!
	/*!if you chain more than once to the same base, the new buffers are appended
	 * to the end of the chain - the new buffer doesn't replace the current chain
	 * @return @a base - just for convenience of multiple calls*/
	Play_ID chainBuffer(Play_ID base, const char buf[], unsigned int len);

	//!allows automatic queuing of sounds - good for dynamic sound sources!
	/*!if you chain more than once to the same base, the new buffers are appended
	 * to the end of the chain - the new buffer doesn't replace the current chain
	 * @return @a base - just for convenience of multiple calls*/
	Play_ID chain(Play_ID base, Snd_ID next);
	
	//! Speaks its argument using the Mary text-to-speech system to generate a WAV file
	/*! On Aperios or unsupported platforms, simply displays the text on stdout */
	SoundManager::Play_ID speak(const std::string& text, bool showText=true, const std::string& voice="female");

	//!Lets you stop playback of all sounds
	void stopPlay();

	//!Lets you stop playback of a sound
	void stopPlay(Play_ID id);
	
	//!Lets you pause playback
	void pausePlay(Play_ID id);
	
	//!Lets you resume playback
	void resumePlay(Play_ID id);
	
	//!Lets you control the maximum number of channels (currently playing sounds), method for mixing (applies when max_chan>1), and queuing method (for when overflow channels)
	void setMode(unsigned int max_channels, MixMode_t mixer_mode, QueueMode_t queuing_mode);

	//!Gives the time until the sound finishes, in milliseconds.  Subtract 32 to get guarranteed valid time for this ID.
	/*!You should be passing the beginning of a chain to get proper results...\n
	 * May be slightly conservative (will report too small a time) because this
	 * does not account for delay until SoundPlay picks up the message that a
	 * sound has been added.\n
	 * However, it is slighly optimistic (will report too large a time) because
	 * it processes a buffer all at one go, so it could mark the sound as finished
	 * (and cause the ID to go invalid) up to RobotInfo::SoundBufferTime (32 ms)
	 * before the sound finishes.  So subtract SoundBufferTime if you want to be
	 * absolutely sure the ID will still valid. */
	unsigned int getRemainTime(Play_ID id) const;
	
#ifdef PLATFORM_APERIOS
	//!Copies the sound data to the OPENR buffer, ready to be passed to the system, only called by SoundPlay
	/*!@return the number of active sounds */
	unsigned int CopyTo(OSoundVectorData* data);

	//!updates internal data structures on the SoundPlay side - you shouldn't be calling this
	void ReceivedMsg(const ONotifyEvent& event);
#endif

	//!Copies the sound data to the specified memory buffer, ready to be passed to the system
	/*!@return the number of active sounds */
	unsigned int CopyTo(void * dest, size_t destSize);
	
	//!updates internal data structures on the SoundPlay side - you shouldn't be calling this
	void ProcessMsg(RCRegion * rcr);
	
	//! returns number of sounds currently playing
	unsigned int getNumPlaying() { return chanlist.size(); }
	
	//! return the next region serial number -- doesn't actually increment it though, repeated calls will return the same value until the value is actually used
	virtual unsigned int getNextKey() { return sn+1; }
	
protected:
	//!Mixes the channel into the buffer
	void mixChannel(Play_ID channelId, void* buf, size_t size);
	
	//!Mixes the channel into the buffer additively
	/*!If mode is Quality, then the size of the buffer should be double the normal
	* size. */
	void mixChannelAdditively(Play_ID channelId, int bitsPerSample, MixMode_t mode, short scalingFactor, void* buf, size_t size);
	
	//!The intermediate mixer buffer used for Quality mode mixing
	int* mixerBuffer;
	
	//!Size (in bytes) of the intermediate mixer buffer
	size_t mixerBufferSize;

	//!Sets up a shared region to hold a sound - rounds to nearest page size
	RCRegion* initRegion(unsigned int size);

	//!Looks to see if @a name matches any of the sounds in sndlist (converts to absolute path if not already)
	Snd_ID lookupPath(std::string const &name) const;

	//!selects which of the channels are actually to be mixed together, depending on queue_mode
	void selectChannels(std::vector<Play_ID>& mix);

	//!update the offsets of sounds which weren't mixed (when needed depending on queue_mode)
	void updateChannels(const std::vector<Play_ID>& mixs,size_t used);

	//!called when a buffer end is reached, may reset buffer to next in chain, or just stopPlay()
	bool endPlay(Play_ID id);
	
	//!Holds data about the loaded sounds
	struct SoundData {
		SoundData();                             //!<constructor
		RCRegion * rcr;                          //!<shared region - don't need to share among processes, just collect in SoundPlay
		byte* data;                              //!<point to data in region (for convenience, only valid in SoundPlay)
		unsigned int len;                        //!<size of the sound
		unsigned int ref;                        //!<reference counter
		unsigned int sn;                         //!<serial number, allows us to verify that a given message buffer does indeed match this sound, and wasn't delayed in processing
#ifdef __APPLE__
		struct MacSpeechState* macSpeech; //!< if non-NULL, stores text string and OS context for speech request, #data and #rcr will be NULL (no PCM samples)
#endif
		char name[SoundManager::MAX_NAME_LEN];   //!<stores the path to the file, empty if from a buffer
	private:
		SoundData(const SoundData&);             //!< don't call
		SoundData operator=(const SoundData&);   //!< don't call
	};
	//!For convenience
	typedef ListMemBuf<SoundData,MAX_SND,Snd_ID> sndlist_t;
	//!Holds a list of all currently loaded sounds
	sndlist_t sndlist;
	
	//!Holds data about sounds currently being played
	struct PlayState {
		PlayState();            //!<constructor
		Snd_ID snd_id;          //!<index of sound
		unsigned int offset;    //!<position in the sound
		unsigned int cumulative;//!<total time of playing (over queued sounds)
		Play_ID next_id;        //!<lets you queue for continuous sound, or loop
	};
	//!For convenience
	typedef ListMemBuf<PlayState,MAX_PLAY,Play_ID> playlist_t;
	//!Holds a list of all sounds currently enqueued
	playlist_t playlist;
	//!For convenience
	typedef ListMemBuf<Play_ID,MAX_PLAY,Play_ID> chanlist_t;
	//!Holds a list of all currently playing sounds, ordered newest (front) to oldest(back)
	chanlist_t chanlist;
	
	//!Current mixing mode, set by setMode();
	MixMode_t mix_mode;

	//!Current queuing mode, set by setMode();
	QueueMode_t queue_mode;

	//!Current maximum number of sounds to mix together
	unsigned int max_chan;

#ifndef __APPLE__
	//!Prevents multiple processes from accessing at the same time
	mutable MutexLock<ProcessID::NumProcesses> lock;
#else
	//!Prevents multiple processes from accessing at the same time
	mutable struct Lock : public MutexLock<ProcessID::NumProcesses> {
		Lock() : MutexLock<ProcessID::NumProcesses>(), initiatedSpeech(), completedSpeech() {}
		std::vector<struct MacSpeechState*> initiatedSpeech; //!< buffered speech contexts which should be started
		std::vector<struct MacSpeechState*> completedSpeech; //!< buffered speech contexts which need to be freed
		virtual void releaseResource(Data& d);
		//! this buffers launching OS resources until after lock is released so we don't deadlock with OS mutex in CopyTo callback
		void initiated(struct MacSpeechState* mss);
		//! this buffers freeing OS resources until after lock is released so we don't deadlock with OS mutex in CopyTo callback
		void completed(struct MacSpeechState* mss);
	} lock;
#endif
	
	//!A serial number, incremented for each sound which is created
	/*! This is used to verify that a sound message from a process
	 *  refers to a current sound.  If you imaging a pathological
	 *  process, which rapidly creates and releases sounds, it would 
	 *  run through the sndlist ids, possibly before the sound process
	 *  can process the incoming buffers.  So this is used to ensure
	 *  that a given message refers to the current sound, and not one
	 *  that was already released and then reassigned. */
	unsigned int sn;

	//!the size of a SoundManagerMsg, which is prefixed before each region sent/received by SoundManager (rounded up to nearest even word boundary)
	static const unsigned int MSG_SIZE=((sizeof(SoundManagerMsg)-1)/8+1)*8;

#ifdef PLATFORM_APERIOS
	//!Storage of each process's subject object, used to internally transmit sound buffers to SoundPlay
	OSubject * subjs[ProcessID::NumProcesses];
#else //PLATFORM_LOCAL
	//!Storage of each process's attachment of the message queue, used to internally transmit sound buffers to SoundPlay
	MessageQueueBase * subjs[ProcessID::NumProcesses];
#endif

private:
	SoundManager(const SoundManager&);           //!< don't call
	SoundManager operator=(const SoundManager&); //!< don't call
};

//! lets you play a sound from anywhere in your code - just a one liner!
extern SoundManager * sndman;

/*! @file
 * @brief Describes SoundManager, which provides sound effects and caching services, as well as mixing buffers for the SoundPlay process
 * @author ejt (Creator)
 */

#endif
