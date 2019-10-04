//-*-c++-*-
#ifndef INCLUDED_SoundManagerMsg_h
#define INCLUDED_SoundManagerMsg_h

class RCRegion;

//! A small header that preceeds data sent by SoundManager between processes
struct SoundManagerMsg {
private:
	friend class SoundManager;

	//! the type to use when referring to Sounds
	typedef unsigned short Snd_ID;

	//! Denotes what type of message this is
	enum MsgType {
		add, //!< indicates a new sound to be played
		del, //!< indicates the sound with #id should be stopped
		wakeup, //!< indicates that it's time to start sending sounds to the system
		unknown //!< initial value for #type until one of the others is assigned
	} type; //!< indicates how this message should be processed
	
	//! The id of the sound this is in reference to
	Snd_ID id;

	//! The RCRegion to free, if it's a deletion
	RCRegion * region;
	
	//! The serial number of the sound this is in reference to, see SoundManager::sn
	unsigned int sn;
	
	//! constructor
	SoundManagerMsg() : type(unknown), id(static_cast<Snd_ID>(-1)), region(NULL), sn(-1U) {}

	//! virtual destructor
	/*! doesn't do anything, but don't remove it, otherwise this would no longer be a virtual base class */
	virtual ~SoundManagerMsg() {}

	//! Accessor for the id number, set by SoundManager
	Snd_ID getID() const { return id; }

	//! Sets up the header as an add message
	void setAdd(Snd_ID sndid, unsigned int sndsn) {
		type=add;
		id=sndid;
		sn=sndsn;
	}

	//! Sets up the header as an erase message
	void setDelete(RCRegion* rcregion) {
		type=del;
		region=rcregion;
	}
	
	//! Sets up the header as a wakeup message
	void setWakeup() {
		type=wakeup;
	}

	SoundManagerMsg(const SoundManagerMsg&); //!< don't call
	SoundManagerMsg operator=(const SoundManagerMsg&); //!< don't call
};

/*! @file
 * @brief Defines SoundManagerMsg, a small header used by SoundManager for sending messages between processes
 * @author ejt (Creator)
 */

#endif // INCLUDED_SoundManagerMsg_h
