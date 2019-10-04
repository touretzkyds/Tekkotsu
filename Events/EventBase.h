//-*-c++-*-
#ifndef INCLUDED_EventBase_h
#define INCLUDED_EventBase_h

#include "Shared/XMLLoadSave.h"
#include "Shared/FamilyFactory.h"
#include <string>

//! Forms the basis of communication between modules/behaviors in the framework
/*! 
 *  Events are defined by a 3-tuple:
 *  - The @b generator indicates the class of the creator of the event, and in practice also generally implies the type (i.e. subclass) of the event itself.
 *  - The @b source indicates the instance of the creator of the event, which differentiates when more than one generator is available.
 *  - The @b type indicates the role of the event among other events from the source.
 *
 *  Each of these is represented by an ID, the #EventGeneratorID_t (EGID) and #EventTypeID_t (ETID) are
 *  defined here in EventBase.  Source IDs (SID) are defined separately by each generator, so the
 *  documentation in each entry of EventGeneratorID_t describes how to interpret the source field.
 *
 *  So for example, the button event which results from initially pressing the head button on an ERS-7
 *  robot would be:
 *  <center>(EventBase::buttonEGID, ERS7Info::HeadButOffset, EventBase::activateETID)</center>
 *
 *  While the button is held down, additional #statusETID events are used to report varying pressure values (if the button
 *  is pressure sensitive), and an event with #deactivateETID is sent when the button is released.
 *  Alternatively, an SID value from a vision detector (say #visObjEGID) refers to seeing a particular object, a completely
 *  different domain, values of which may overlap with other generators' source IDs.
 *  Thus, interpreting the source field requires knowing the generator as well.
 *
 *  When the generator doesn't have groups of activity with a 'begin' or 'end', it will use #statusETID
 *  for all of its events.  (in other words, not all generators necessarily have to use activate or deactivate)
 *  For example, sensor updates are continuously occuring, so you will only ever see
 *  <center>(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID)</center>
 *  
 *  The #duration field is also generator specific - some may refer to
 *  the time since the last activation event (e.g. button events)
 *  where as others refer to time since last status (e.g. sensors
 *  updates)
 *
 *  If you want to make a new generator, all you have to do is add a new entry
 *  to the ID list (#EventGeneratorID_t) and then put its name in the 
 *  #EventGeneratorNames[] array.  Alternatively, there is an 'unlicensed spectrum' available under
 *  #unknownEGID.  You can send out events from that generator just
 *  like any other, but it should only be used for quick tests and hacking
 *  around...
 *
 *  The generator ID number is only that -- an ID number.  
 *  Events can be posted to the EventRouter anywhere, anytime,
 *  and do not require anything of the sender.  However, there is an EventGeneratorBase which
 *  can simplify some aspects of behaviors whose sole purpose is processing information
 *  and generating events.
 *
 *  If more information needs to be sent along with the event, the
 *  cleanest solution is to create a subclass of EventBase to hold the
 *  additional information.  For example, you can see the existing
 *  subclasses in the inheritance diagram above.  If you want to use a
 *  quick hack however, you could just pass a pointer to data as the SID if you
 *  don't need that field for something else, or use a DataEvent.
 *
 *  @note All subclasses must override getClassTypeID() and provide
 *  a unique value to allow fast polymorphic serialization for inter-process
 *  communication.  Implementing the load/save functions
 *  ({load,save}BinaryBuffer and {load,save}XML) for your addtional data fields
 *  would also be necessary in order to allow your event to be sent between
 *  processes, or eventually, between robots.
 * 
 *  @see EventRouter for discussion on sending and receiving of events
 *  @see Tutorials:
 *    - <a href="../FirstBehavior2.html">Steps 3, 4, & 5 of Tekkotsu's First Behavior Tutorial</a>
 *    - <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/events.shtml">David Touretzky's Events Chapter</a>
 *    - <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/behaviors.pdf">CMU's Cognitive Robotics course slides</a>
 *    - <a href="../media/TekkotsuQuickReference_ERS7.pdf">ERS-7 Quick Reference Sheet</a>
 */
class EventBase : public XMLLoadSave {
 public:
	//! Lists all possible event generator ids
	/*! An event generator is a abstract source of events, used for listening to and parsing certain classes of events
	 *
	 *  IF YOU ADD AN EVENT GENERATOR, DON'T FORGET TO NAME IT (EventBase::EventGeneratorNames, actual names are in EventBase.cc)*/
	enum EventGeneratorID_t {
		unknownEGID=0,    //!< default EGID, used if you forget to set it, probably not a good idea to use this for anything except errors or testing quick hacks
		aiEGID,           //!< not being used, yet (might use this when AI makes decisions?)
		audioEGID,        //!< Sends an event when a sound starts/ends playback, status events as chained sounds end; SID is SoundManager::Play_ID; duration is playtime
		buttonEGID,       //!< Sends activate event for button down, deactivate for button up.  Status events only for when pressure sensitive buttons' reading changes. (on sensorEGID updates); SIDs are from ButtonOffset_t in the namespace of the target model (e.g. ERS210Info::ButtonOffset_t); duration is button down time
		cameraResolutionEGID, //!< Sends a status event whenever the camera's resolution changes, such as a different camera than predicted by RobotInfo is being used, or a camera is hotswapped.  SID corresponds to the visOFbkEGID for that camera.
		erouterEGID,      //!< Sends activate event on first listener, deactivate on last listener, and status on other listener changes.; SID is the generator ID affected
		estopEGID,        //!< Sends an event when the estop is turned on or off; SID is the MotionManager::MC_ID of the EmergencyStopMC; duration is length of estop activation
		grasperEGID,	//!< Sends events when the Grasper processes a user request
        koduEGID,       //!< Sends events when a kodu event occurs
		locomotionEGID,   //!< Sends events regarding transportation in the world; you can/should assume these will all be LocomotionEvent classes; SID is MotionManager::MC_ID of posting MotionCommand; duration is the time since last velocity change of that MC. (You could generate these for things other than walking...)
		lookoutEGID,      //!< Sends an event when Lookout request is complete; SID is the id for lookout request
		mapbuilderEGID,  //!< Sends a status event when map is completed
		micOSndEGID,      //!< Sends a DataEvent<OSoundVectorData> for every audio buffer received from the system; SID and duration are always 0 (This is generated by the MainObj instantiation of MMCombo)
		micRawEGID,       //!< reserved for future use
		micFFTEGID,       //!< reserved for future use
		micPitchEGID,       //!< Sends a PitchEvent when a particular frequency is detected; SID is a pointer to the PitchDetector, magnitude is product of amplitude and confidence
		mocapEGID,        //!< Sends a MoCapEvent for "motion capture" or GPS type updates from an external localization source.  Used for position/orientation feedback from simulation as well.
		motmanEGID,       //!< Sends events when a MotionCommand is added or removed, SID is is the MotionManager::MC_ID, duration is always 0; individual MotionCommands may throw status events to signal intermediary status
		pilotEGID,        //!< Sends events when position of agent is updated or pilot request is completed
		powerEGID,        //!< Sends events for low power warnings, temperature, etc. see PowerSrcID::PowerSourceID_t
		remoteStateEGID,  //!< Sent when remote state is updated
		runtimeEGID,     //!< Sends an activate upon completion of startup, and a deactivate event upon shutdown; source ID undefined (currently 0)
		sensorEGID,       //!< Sends a status event when new sensor readings are available. see SensorSrcID::SensorSourceID_t
		servoEGID,        //!< Sends notifications of servo errors; the source id is the Tekkotsu output offset
		stateMachineEGID, //!< Sends an event upon entering and leaving a StateNode; SID is pointer to the StateNode; duration is always 0; some state will throw a status event when they have completed their task and are now idling
		stateSignalEGID,  //!< Sends a DataEvent that can be monitored by a SignalTrans for triggering a state transition
		stateTransitionEGID, //!< Sends an event each time a transition is triggered; SID is a pointer to the transition; type is activate at start of firing and deactivate when firing is complete; duration is always 0; activate guaranteed to occur immediately *before* the transition actually occurs
		textmsgEGID,      //!< Sends status events when a text msg is received on console; generated by the Controller, SID is 0 if broadcast from ControllerGUI, 1 if "private" from BehaviorSwitchControl; duration is always 0 (see Controller for more information)
		timerEGID,        //!< Sends timer events; you set timers explicitly, you don't have to listen as well. (See EventRouter::addTimer()) There's no cross-talk, only the listener which requested the timer will receive it; SID is whatever you requested it to be; duration is the time (since boot, in ms) that the timer was supposed to go off; these are always status
		userEGID,        //!< Reserved for user-generated events
		visInterleaveEGID,//!< Sends a FilterBankEvent when new interleaved images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visJPEGEGID,      //!< Sends a FilterBankEvent when JPEG compressed images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visObjEGID,       //!< Sends VisionObjectEvents for objects detected in camera images; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visOFbkEGID,      //!< Sends a DataEvent < OFbkImageVectorData > for every camera image received from the system; SID and duration are always 0 (This is generated by the MainObj instantiation of MMCombo)
		visPNGEGID,      //!< Sends a FilterBankEvent when PNG compressed images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visRawCameraEGID, //!< Sends a FilterBankEvent when new raw camera images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visRawDepthEGID,     //!< Sends FilterBankEvent when new depth images are available
		visRegionEGID,    //!< Sends a SegmentedColorFilterBankEvent when color regions are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visRLEEGID,       //!< Sends a SegmentedColorFilterBankEvent when RLE encoded color segmentated images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		visSegmentEGID,   //!< Sends a SegmentedColorFilterBankEvent when color segmentated images are available; SID is whatever value you gave during setup (typically in StartupBehavior_SetupVision.cc), duration is always 0
		wmVarEGID,        //!< Sends an event when a watched memory is changed; source id is pointer to WMEntry
		worldModelEGID,   //!< not being used, yet (for when objects are detected/lost?)
		numEGIDs          //!< the number of generators available
	};

	//! Holds string versions of each of the generator's names, handy for debugging so you can output the events as readable strings (you'll find this in EventBase.cc since it can't go in the header or we get multiply-defined errors during linking)
	static const char* const EventGeneratorNames[numEGIDs+1];
	
	//! an event type id is used to denote whether it's the first in a sequence (button down), in a sequence (button still down), or last (button up)
	enum EventTypeID_t {
		activateETID,   //!< Start of an event sequence, e.g. button down
		statusETID,     //!< Indicates a value has changed, e.g. new sensor readings
		deactivateETID, //!< Last of a series of events, e.g. button up
		numETIDs        //!< the number of different event types
	};
	
	//! holds string versions of EventTypeID_t
	static const char* const EventTypeNames[numETIDs+1];
	
	//! holds abbreviated string versions of EventTypeID_t
	static const char* const EventTypeAbbr[numETIDs];
	
	//! type used for class ids in the type registry (see getTypeRegistry())
	typedef unsigned int classTypeID_t;
	
	//! type used for the type registry
	typedef FamilyFactory<EventBase,classTypeID_t> registry_t;

	//! returns a FamilyFactory with which you can look up classTypeID_t's to make new instances from serialized data
	static registry_t& getTypeRegistry();
	
	/*! @name Constructors/Destructors */
	//! constructor
	/*! @see EventRouter::postEvent() */
	EventBase(); 
	EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0);
	EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n);
	EventBase(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag);
	virtual ~EventBase() {} //!< destructor

	//! allows a copy to be made of an event, supporting polymorphism
	/*! Must be overridden by all subclasses to allow this to happen
	 * 
	 *  I would like to switch this over to the cloneable interface once
	 *  the compiler gets updated out of the 3.3 branch... see
	 *  Cloneable::clone() for a discussion of the issue and
	 *  implementation notes. */
	virtual EventBase* clone() const { return new EventBase(*this); }
	//@}

	template<typename T>
	operator const T&() const {
	  const T* recastEvent = dynamic_cast<const T*>(this);
	  if ( recastEvent == NULL )
	    std::cerr << "Attempt to dynamic_cast an EventBase to the wrong type\n";
	  return *recastEvent;
	}

	/*! @name Methods */
	virtual const std::string& getName() const { return stim_id; } //!< gets the name of the event - useful for debugging output, see also getDescription()
	virtual EventBase& setName(const std::string& n); //!< sets name to a given string, prevents overwriting by generated names

	virtual float getMagnitude() const { return magnitude; } //!< gets "strength" of event - by default 1 for activate and status events, 0 for deactivate events
	virtual EventBase& setMagnitude(float m) { magnitude=m; return *this; }//!< sets "strength" of event - you may want to override the default values (see getMagnitude())

	virtual unsigned int getTimeStamp() const { return timestamp; } //!< time event was created
	virtual void setTimeStamp(unsigned int t) { timestamp=t; } //!< resets time event was created

	virtual EventGeneratorID_t getGeneratorID() const { return genID; } /*!< @brief gets the generator ID for this event @see EventGeneratorID_t */
	virtual EventBase& setGeneratorID(EventGeneratorID_t gid) { genID=gid; genName(); return *this; } /*!< @brief sets the generator ID for this event @see EventGeneratorID_t */
	
	virtual size_t getSourceID() const { return sourceID; } /*!< @brief gets the source ID for this event @see sourceID */
	virtual EventBase& setSourceID(size_t sid) { sourceID=sid; genName(); return *this; } /*!< @brief sets the source ID for this event @see sourceID */
	
	virtual EventTypeID_t getTypeID() const { return typeID; } /*!< @brief gets the type ID @see EventTypeID_t */
	virtual EventBase& setTypeID(EventTypeID_t tid) { typeID=tid; unsigned int n=strlen(EventTypeAbbr[typeID]); stim_id.replace(stim_id.size()-n-1,n,EventTypeAbbr[typeID]); return *this; } /*!< @brief sets the type ID @see EventTypeID_t */


	virtual int getHostID() const { return hostID; }  //!< ID of the host that generated this event (-1U for localhost)
	virtual EventBase& setHostID(int host) { hostID = host; genName(); return *this; }  //!< sets the ID of the host associated with this event
	
	virtual unsigned int getDuration() const { return duration; } /*!< @brief gets the time since the beginning of this sequence (the timestamp of the activate event) @see duration */
	virtual EventBase& setDuration(unsigned int d) { duration = d; return *this; }/*!< @brief sets the time since the beginning of this sequence (the timestamp of the activate event) @see duration */

	virtual const std::string& resetName() { nameisgen=true; genName(); return stim_id; } //!< resets name to generated form, overwriting any previous name
	virtual bool isCustomName() const { return !nameisgen; } //!< returns true if not using the generated name

	//! generates a description of the event with variable verbosity 
	/*! @param showTypeSpecific should be read by subclasses to add additional information
	 *  @param verbosity can be one of the following values:
	 *    - 0 - Basic: <i>event_name</i> \\t <i>generator_id</i> \\t <i>source_id</i> \\t <i>type_id</i>
	 *    - 1 - Numerics: <i>event_name</i> \\t <i>generator_id</i> \\t <i>source_id</i> \\t <i>type_id</i>
	 *    - 2 - Timing: <i>event_name</i> \\t <i>generator_id</i> \\t <i>source_id</i> \\t <i>type_id</i> \\t <i>duration</i> \\t <i>timestamp</i>
	 *    - 3 and above - Full: <i>event_name</i> \\t <i>generator_id</i> \\t <i>source_id</i> \\t <i>type_id</i> \\t <i>duration</i> \\t <i>timestamp</i> \\t <i>magnitude</i>
	 *  if showTypeSpecific, additional fields will be added after the common fields listed above. */
	virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const; 

	inline bool operator<(const EventBase& e) const { return timestamp<e.timestamp; }

	//! is true if the genID, typeID, and sourceID's all match
	virtual bool operator==(const EventBase& eb) const {
		return (sourceID==eb.sourceID && genID==eb.genID && typeID==eb.typeID);
	}
	//!tests to see if events have the same generator and source IDs
	bool sameGenSource(const EventBase& eb) const { return genID==eb.genID && sourceID==eb.sourceID; }
	
	bool longerThan(const EventBase& eb) const { return duration>eb.duration && *this==eb; } //!< compares event duration and ensures same event generator, source, and type - useful for event masks
	bool shorterThan(const EventBase& eb) const { return duration<eb.duration && *this==eb; }//!< compares event duration and ensures same event generator, source, and type - useful for event masks
	bool equalOrLongerThan(const EventBase& eb) const { return duration>=eb.duration && *this==eb; }//!< compares event duration and ensures same event generator, source, and type - useful for event masks
	bool equalOrShorterThan(const EventBase& eb) const { return duration<=eb.duration && *this==eb; }//!< compares event duration and ensures same event generator, source, and type - useful for event masks
	
	static bool isValidGeneratorID(unsigned int egid) { return egid<numEGIDs; }
	//@}

	//! Useful for serializing events to send between processes
	/*! @name LoadSave interface */

	//! All subclasses should override this and return a unique ID for their class.
	/*! All IDs corresponding to all-capital letters are reserved for future
	 *  framework expansion.  (Thus, user subclasses should contain at least one
	 *  lower-case letter.)  This code can be used when serializing to allow quick
	 *  identification of the class type by the receiver. */
	virtual classTypeID_t getClassTypeID() const { return autoRegisterEventBase; }

	virtual unsigned int getBinSize() const; //!< should return the minimum size needed if using binary format (i.e. not XML)
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len); //!< load from binary format
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const; //!< save to binary format
	virtual void loadXML(xmlNode* node); //!< load from XML format
	virtual void saveXML(xmlNode * node) const; //!< save to XML format
	
	//! no longer need to override this -- will automatically call either loadXML() or loadBinaryBuffer() based on #saveFormat
	/*! tries to be smart so if the load based on the current #saveFormat fails, retries with the alternative format */
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL)  {
		unsigned int test = saveFormat!=BINARY ? XMLLoadSave::loadBuffer(buf,len,filename) : loadBinaryBuffer(buf,len);
		if(test!=0) //if the default didn't work, try the other format too...
			return test;
		return saveFormat!=BINARY ? loadBinaryBuffer(buf,len) : XMLLoadSave::loadBuffer(buf,len,filename);
	}
	//! no longer need to override this -- will automatically call either saveXML() or saveBinaryBuffer() based on #saveFormat
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const { return saveFormat!=BINARY ? XMLLoadSave::saveBuffer(buf,len) : saveBinaryBuffer(buf,len); }

	//! automatically calls either XMLLoadSave::loadFile or LoadSave::loadFile based on #saveFormat
	/*! tries to be smart so if the load based on the current #saveFormat fails, retries with the alternative format */
	virtual unsigned int loadFile(const char* filename) {
		unsigned int test = saveFormat!=BINARY ? XMLLoadSave::loadFile(filename) : LoadSave::loadFile(filename);
		if(test!=0) //if the default didn't work, try the other format too...
			return test;
		return saveFormat!=BINARY ? LoadSave::loadFile(filename) : XMLLoadSave::loadFile(filename);
	}
	//! automatically calls either XMLLoadSave::saveFile or LoadSave::saveFile based on #saveFormat
	virtual unsigned int saveFile(const char* filename) const { return saveFormat!=BINARY ? XMLLoadSave::saveFile(filename) : LoadSave::saveFile(filename); }
	
	//! automatically calls either XMLLoadSave::loadFileStream or LoadSave::loadFileStream based on #saveFormat
	virtual unsigned int loadFileStream(FILE* f, const char* filename=NULL) { return saveFormat!=BINARY ? XMLLoadSave::loadFileStream(f,filename) : LoadSave::loadFileStream(f,filename); }
	//! automatically calls either XMLLoadSave::loadFileStream or LoadSave::loadFileStream based on #saveFormat
	virtual unsigned int saveFileStream(FILE* f) const { return saveFormat!=BINARY ? XMLLoadSave::saveFileStream(f) : LoadSave::saveFileStream(f); }
	
	//! values to pass to setSaveFormat()
	enum SaveFormat {
		BINARY, //!< saves will be in packed binary, loads will try binary first
		XML //!< saves will be in xml, loads will try xml first
	};
	virtual void setSaveFormat(SaveFormat sf) const { saveFormat=sf; } //!< set #saveFormat
	virtual SaveFormat getSaveFormat() const { return saveFormat; } //!< return #saveFormat
	//@}
	
 protected:
	//! converts the first 4 characters of @a str to an unsigned int, should ensure consistent byte ordering across platforms
	static unsigned int makeClassTypeID(const char* str) {
#if LOADSAVE_SWAPBYTES
		unsigned int x;
		byteswap(x,*reinterpret_cast<const unsigned int*>(str));
		return x;
#else
		return *reinterpret_cast<const unsigned int*>(str);
#endif
	}

	std::string stim_id; //!< the name of the event, use the same name consistently or else will be seen as different stimuli
	float magnitude; //!< the current "strength" of the event/stimuli... MAKE SURE this gets set to ZERO IF event is DEACTIVATE
	unsigned int timestamp; //!< the time the event was created - set automatically by constructor

	mutable SaveFormat saveFormat; //!< controls the format used during the next call to saveBuffer() (packed binary or XML)

	bool nameisgen; //!< tracks whether the current name (stim_id) was generated by genName() (true) or setName() (false)
	virtual void genName(); //!< calls setName() with a string version of sourceID, decimal notation

	EventGeneratorID_t genID; //!< generator ID, see EventGeneratorID_t
	EventTypeID_t typeID; //!< type ID, see EventTypeID_t
	size_t sourceID;       /*!< @brief the source ID for this event
													* Source IDs are defined by the generator that made it.  This should
													* give authors flexibility to design their modules without having to
													* worry about ID space collision */
	int hostID;
	unsigned int duration; /*!< @brief the time since this sequence started (like, how long the
													*   button has been pressed); not all generators will set this;
													*   Typically, this would be 0 for activate,
													*   (activate.timestamp-::get_time()) for status and deactivate */

	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterEventBase;
};

/*! @file
 * @brief Describes EventBase, the basic class for sending events around the system
 * @author ejt (Creator)
 */

#endif
