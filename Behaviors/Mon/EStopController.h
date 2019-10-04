//-*-c++-*-
#ifndef INCLUDED_EStopController_h_
#define INCLUDED_EStopController_h_

#include <iostream>
#include "Wireless/Wireless.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Shared/Config.h"
#include "Shared/ProjectInterface.h"

//! Listens to control commands coming in from the command port for remotely controlling the head
class EStopController : public BehaviorBase {

public:	
	//! Points to the one EStopController object that the input command stream is talking to.
	/*! A kludge. Dunno how you're gonna make sure you're not using this uninitialized. */
	static EStopController * theOne;
	static int callback(char *buf, int bytes); //!< called by wireless when there's new data

public:
	//! constructor
	EStopController()
		: BehaviorBase("EStopController"),
			cmdsock(NULL)
	{
		theOne=this;
	}
	//! destructor
	virtual ~EStopController() {
		theOne=NULL;
	}

	virtual void doStart();

	virtual void doStop();

	virtual void doEvent();

	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",*config->main.estopControl_port);
		return std::string("Listens to estop commands coming in from port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }

	virtual void runCommand(const std::string& s); //!< processes a string received from wireless

protected:
	//! The input command stream socket
	Socket *cmdsock;

private:
	EStopController(const EStopController&); //!< don't call
	EStopController operator=(const EStopController&); //!< don't call

};

/*! @file
 * @brief Describes EStopController, listens to control commands coming in from the command port for remotely toggling the estop
 * @author tss (Creator)
 */

#endif 
