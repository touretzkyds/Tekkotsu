//-*-c++-*-
#ifndef INCLUDED_CLASSNAME_h_
#define INCLUDED_CLASSNAME_h_

/* Typically, no destructor, copy, or assignment operators are needed.  If
 * you think you do need one, reread the MotionCommand documentation, because
 * you shouldn't be storing pointers in MotionCommands (see IPC/ListMemBuf for
 * a memory pool interface) */

// Replace YOURNAMEHERE, CLASSNAME, and DESCRIPTION as appropriate, and go to town!


#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"

//! DESCRIPTION
class CLASSNAME : public MotionCommand {
public:
	//! constructor
	CLASSNAME() : MotionCommand() {}

	// This is where all the real work is done
	// It will be called by the MotionManager at a high frequency (32ms period)
	virtual int updateOutputs() {
		// For each joint you wish to control, call:
		//   motman->setOutput(this, jointOffset, ...)
		// Do your computations and return as quickly as possible
		// Pre-compute and/or cache values whenever possible.
	}
	
	// Return true if any desired joint values may have changed since the last
	// call to updateOutputs()
	virtual int isDirty() { return true; }
	
	// Return true if this motion still has work to be done -- returning
	// false may allow the MotionManager to delete this motion and send
	// a deactivation event, but only if the motion was added with
	// MotionManager::addPrunableMotion()
	virtual int isAlive() { return true; }
	

protected:
	/* These start and stop functions are called when the motion is
	 * added or removed from the MotionManager.
	 * This may not matter to you -- feel free to delete these functions: */
	virtual void doStart() {
		// <your startup code here>
	}
	virtual void doStop() {
		// <your shutdown code here>
	}	

	// <class members go here>

};

/*! @file
 * @brief Defines CLASSNAME, which DESCRIPTION
 * @author YOURNAMEHERE (Creator)
 */

#endif
