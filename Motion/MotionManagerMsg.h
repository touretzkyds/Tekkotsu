//-*-c++-*-
#ifndef INCLUDED_MotionManagerMsg_h
#define INCLUDED_MotionManagerMsg_h

#include "IPC/ProcessID.h"

//! A small header that precedes data sent by MotionManager between processes
/*! Typically this is broadcast to all processes using the MotionManager so
 *  each process has to update its own fields of MotionManager.
 *
 *  One tricky aspect is that with the IPC mechanisms on PLATFORM_LOCAL
 *  the originating process will get an "echo" of the message, whereas
 *  on PLATFORM_APERIOS it's set up so the sender doesn't get an echo
 *  of its own message.  However, this complexity is handled by
 *  MotionManager, not here. */
struct MotionManagerMsg {
	//! the type to use when referring to MotionCommand ID's
	typedef unsigned short MC_ID;

	//! for errors and undefined stuff
	static const MC_ID invalid_MC_ID=static_cast<MC_ID>(-1); 

	//! constructor
	MotionManagerMsg() : type(unknown), creatorPID(ProcessID::getID()), mc_id(invalid_MC_ID) {}

	//! virtual destructor
	/*! doesn't do anything, but don't remove it, otherwise this would no longer be a virtual base class */
	virtual ~MotionManagerMsg() {}

	//! Accessor for the id number, set by MotionManager::addMotion()
	MC_ID getID() const { return mc_id; }

private:
	friend class MotionManager;

	//! Denotes what type of message this is (see #type)
	enum MsgType {
		addMotion,     //!< indicates the msg is actually MotionCommand to be added to the MotionManager
		deleteMotion,  //!< indicates the msg's #mc_id references a MotionCommand to be removed from the MotionManager
		unknown        //!< failsafe default until one of the others is set
	} type; //!< indicates what processing this message requires
	
	//! holds the process that this message was created/sent from
	ProcessID::ProcessID_t creatorPID;
	
	//! The id of the MotionCommand this is in reference to
	MC_ID mc_id;

	//! Sets up the header as an add motion message
	void setAdd(MC_ID id) {
		type=addMotion;
		mc_id=id;
	}

	//! Sets up the header as an erase motion message
	void setDelete(MC_ID id) {
		type=deleteMotion;
		mc_id=id;
	}

	//! resets the #mc_id to #invalid_MC_ID
	void clearID() { mc_id=invalid_MC_ID; }
	
};

/*! @file
 * @brief Defines MotionManagerMsg, a small header used by MotionManager for sending messages between processes
 * @author ejt (Creator)
 */

#endif // INCLUDED_MotionManagerMsg_h
