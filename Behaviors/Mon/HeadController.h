//-*-c++-*-
#ifndef INCLUDED_HeadController_h_
#define INCLUDED_HeadController_h_

#include <iostream>
#include "Wireless/Wireless.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Shared/Config.h"

//! Listens to control commands coming in from the command port for remotely controlling the head
/*! The communication protocol is a very simple binary format, shared
 *  with WalkControllerBehavior.  Each command is sent as a 5-byte
 *  group.  The first byte is a command selector, and the following 4
 *  bytes are a floating point argument:
 *
 *  - <@c char: command indicator>
 *  - <@c float: value>
 *  
 *  The valid values for command indicator are given by #CMD_tilt,
 *  #CMD_pan, or #CMD_roll ('t', 'p', or 'r' respectively).
 */  
class HeadController : public BehaviorBase {

 public:	
	//! Points to the one HeadController object that the input
	//! command stream is talking to. A kludge. Dunno how you're gonna
	//! make sure you're not using this uninitialized.
	static HeadController * theOne;
	static int mechacmd_callback(char *buf, int bytes); //!< called by wireless when there's new data

 protected:
	MotionManager::MC_ID head_id;   //!< the HeadPointerMC to use
 
 private:
	//!@name Command Bytes
	static const char CMD_tilt  = 't'; //!< handy symbol for matching incoming communication
	static const char CMD_pan   = 'p';
	static const char CMD_roll  = 'r';
	//@}

	float t; //!< head parameter
	float p; //!< head parameter
	float r; //!< head parameter

	//! The last HPCB object that was theOne, so we can restore it
	//! to prominence when we die. This is a nice gesture, but it doesn't
	//! really make sense since we're all using the same port. But just
	//! in case something changes and we don't do that, this mechanism
	//! is in place.
	HeadController *theLastOne;

	//! The input command stream socket
	Socket *cmdsock;

	//! Executes a command. Called by mechacmd_callback.
	void runCommand(unsigned char *command);

	HeadController(const HeadController&); //!< don't call
	HeadController operator=(const HeadController&); //!< don't call

 public:
	//! constructor
	HeadController() :
	  BehaviorBase("HeadController"),
	  head_id(MotionManager::invalid_MC_ID),
	  t(0), p(0), r(0),
	  theLastOne(theOne),
	  cmdsock(NULL)
	{}
	//! destructor
	virtual ~HeadController() { theOne = theLastOne; }

	virtual void doStart();

	virtual void doStop();

	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",*config->main.headControl_port);
		return std::string("Listens to head control commands coming in from port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Describes HeadController, listens to control commands coming in from the command port for remotely controlling the head
 * @author tss (Creator)
 */

#endif 
