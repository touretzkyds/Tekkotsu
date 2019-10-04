#include "EStopController.h"
#include "Motion/EmergencyStopMC.h"
#include "Motion/MotionPtr.h"

REGISTER_BEHAVIOR_MENU_OPT(EStopController,"TekkotsuMon",BEH_START|BEH_NONEXCLUSIVE);

EStopController* EStopController::theOne = NULL;

void EStopController::doStart() {
	// We listen to the estop
	erouter->addListener(this, EventBase::estopEGID);
	// Turn on wireless
	cmdsock=wireless->socket(Socket::SOCK_STREAM, 300, 300);
	wireless->setDaemon(cmdsock,true);
	wireless->setReceiver(cmdsock->sock, callback);
	wireless->listen(cmdsock->sock, config->main.estopControl_port);
}

void EStopController::doStop() {
	// Close socket; turn wireless off
	wireless->setDaemon(cmdsock,false);
	wireless->close(cmdsock);
}

void EStopController::runCommand(const std::string& s) {
	if(s==std::string("start")) {
		ProjectInterface::estop->setStopped(false);
	} else if(s==std::string("stop")) {
		ProjectInterface::estop->setStopped(true);
	} else if(s==std::string("refresh")) {
		if(ProjectInterface::estop->getStopped())
			cmdsock->printf("on\n");
		else
			cmdsock->printf("off\n");
	} else {
		serr->printf("EStopController::runCommand() - bad message: '%s'",s.c_str());
	}
}

void EStopController::doEvent() {
	if(event->getTypeID()==EventBase::activateETID) {
		cmdsock->printf("on\n");
	} else if(event->getTypeID()==EventBase::deactivateETID) {
		cmdsock->printf("off\n");
	}
}

// The command packet reassembly mechanism
int EStopController::callback(char *buf, int bytes) {
	if(EStopController::theOne==NULL)
		return 0;
	static std::string cmd;
	for(int i=0; i<bytes; i++) {
		if(buf[i]=='\n') {
			EStopController::theOne->runCommand(cmd);
			cmd.clear();
		} else
			cmd+=buf[i];
	}
  return 0;
}

/*! @file
 * @brief Implements EStopController, listens to commands coming in from the command port for remotely controlling toggling the estop
 * @author tss (Creator)
 */

