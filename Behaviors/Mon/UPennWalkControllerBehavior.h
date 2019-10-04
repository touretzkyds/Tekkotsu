//-*-c++-*-
#ifndef INCLUDED_UPennWalkControllerBehavior_h_
#define INCLUDED_UPennWalkControllerBehavior_h_

#include <iostream>
#include "Wireless/Wireless.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Motion/UPennWalkMC.h"
#include "Motion/MMAccessor.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Shared/Config.h"
#include "IPC/SharedObject.h"

//! Listens to control commands coming in from the command port for remotely controlling the walk
class UPennWalkControllerBehavior : public BehaviorBase {

 public:	
	//! Points to the one UPennWalkControllerBehavior object that the input
	//! command stream is talking to. A kludge. Dunno how you're gonna
	//! make sure you're not using this uninitialized.
	static UPennWalkControllerBehavior * theOne;
	static int mechacmd_callback(char *buf, int bytes); //!< called by wireless when there's new data

 protected:
	SharedObject<UPennWalkMC> shared_walker; //!< the WalkMC to use
 
 private:
	//!@name Command Bytes
	static const char CMD_fwd  = 'f'; //!< handy symbol for matching incoming communication
	static const char CMD_roto = 'r';
	static const char CMD_side = 's';
	static const char CMD_opt0 = '0';
	static const char CMD_opt1 = '1';
	static const char CMD_opt2 = '2';
	static const char CMD_opt3 = '3';
	static const char CMD_opt4 = '4';
	static const char CMD_opt5 = '5';
	static const char CMD_opt6 = '6';
	static const char CMD_opt7 = '7';
	static const char CMD_opt8 = '8';
	static const char CMD_opt9 = '9';
	//@}

	float dx; //!< Motion parameter
	float dy; //!< Motion parameter
	float da; //!< Motion parameter

	//! The last WCB object that was theOne, so we can restore it
	//! to prominence when we die. This is a nice gesture, but it doesn't
	//! really make sense since we're all using the same port. But just
	//! in case something changes and we don't do that, this mechanism
	//! is in place.
	UPennWalkControllerBehavior *theLastOne;

	//! The input command stream socket
	Socket *cmdsock;

	//! Executes a command. Called by mechacmd_callback.
	void runCommand(unsigned char *command);

	UPennWalkControllerBehavior(const UPennWalkControllerBehavior&); //!< don't call
	UPennWalkControllerBehavior operator=(const UPennWalkControllerBehavior&); //!< don't call

 public:
	//! constructor
	UPennWalkControllerBehavior() :
	  BehaviorBase("UPennWalkControllerBehavior"),
		shared_walker(),
	  dx(0), dy(0), da(0),
	  theLastOne(theOne),
	  cmdsock(NULL)
	{}
	//! destructor
	virtual ~UPennWalkControllerBehavior() { theOne = theLastOne; }

	virtual void doStart();

	virtual void doStop();

	virtual UPennWalkMC * getWalkMC() { return &(*shared_walker); }  //!< returns the WalkMC which [will be|is being] used

	virtual MotionManager::MC_ID getWalkID() { return shared_walker->getID(); } //!< returns the current Walk's MotionCommand ID

	//! The only event we could possibly receive is the stop-if-no-heartbeat timer.
	virtual void doEvent() {
		MMAccessor<UPennWalkMC> walker(getWalkID());
		walker->setTargetVelocity(0,0,0);
	}

	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",*config->main.walkControl_port);
		return std::string("Listens to walk control commands coming in from port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Describes UPennWalkControllerBehavior, listens to control commands coming in from the command port for remotely controlling the walk
 * @author tss (Creator)
 * @author ejt (modifications)
 * @author PA Gov. School for the Sciences 2003 Team Project - Haoqian Chen, Yantian Martin, Jon Stahlman (modifications)
 */

#endif 
