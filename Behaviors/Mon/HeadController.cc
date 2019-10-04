#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_HEAD

#include "HeadController.h"
#include "Behaviors/Controller.h"
#include "Motion/MMAccessor.h"
#include "Motion/HeadPointerMC.h"
#include "Shared/ERS7Info.h"
#include "Shared/ERS210Info.h"
#include "IPC/SharedObject.h"

REGISTER_BEHAVIOR_MENU_OPT(HeadController,"TekkotsuMon",BEH_NONEXCLUSIVE);

HeadController* HeadController::theOne = NULL;

void HeadController::runCommand(unsigned char *command) {
	// Extract the command parameter
	float param;
	unsigned char *paramp = (unsigned char *) &param;

#if defined(BYTE_ORDER) && BYTE_ORDER==BIG_ENDIAN
	paramp[0] = command[4];
	paramp[1] = command[3];
	paramp[2] = command[2];
	paramp[3] = command[1];
#else
	paramp[0] = command[1];
	paramp[1] = command[2];
	paramp[2] = command[3];
	paramp[3] = command[4];
#endif
	
	// Find out what type of command this is
#ifdef TGT_IS_AIBO
	switch(command[0]) {
	case CMD_tilt:
		t = std::abs(param)*outputRanges[HeadOffset+TiltOffset][param>0?MaxRange:MinRange];
		break;
	case CMD_pan:
		p = std::abs(param)*outputRanges[HeadOffset+PanOffset][param>0?MaxRange:MinRange];
		break;
	case CMD_roll:
		r = std::abs(param)*outputRanges[HeadOffset+RollOffset][param>0?MaxRange:MinRange];
		break;
	default:
		std::cout << "MECHA: unknown command " << command[0] << std::endl;
	}
#else
	switch(command[0]) {
		case CMD_tilt: {
			const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::TiltOffset];
			unsigned int i = capabilities.findOutputOffset(n);
			if(i!=-1U)
				t = std::abs(param)*outputRanges[i][param>0?MaxRange:MinRange];
		} break;
		case CMD_pan: {
			const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::PanOffset];
			unsigned int i = capabilities.findOutputOffset(n);
			if(i!=-1U)
				p = std::abs(param)*outputRanges[i][param>0?MaxRange:MinRange];
		} break;
		case CMD_roll: {
			const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::NodOffset];
			unsigned int i = capabilities.findOutputOffset(n);
			if(i!=-1U)
				r = std::abs(param)*outputRanges[i][param>0?MaxRange:MinRange];
			else {
				n = ERS210Info::outputNames[ERS210Info::HeadOffset+ERS210Info::RollOffset];
				i = capabilities.findOutputOffset(n);
				if(i!=-1U)
					r = std::abs(param)*outputRanges[i][param>0?MaxRange:MinRange];
			}
		} break;
		default:
			std::cout << "MECHA: unknown command " << command[0] << std::endl;
	}
#endif

	// If the command was a new motion command, apply the
	// new motion parameters:
	switch(command[0]) {
	case CMD_tilt:
	case CMD_pan:
	case CMD_roll:
		{
			MMAccessor<HeadPointerMC> head(head_id);
#ifdef TGT_HAS_KINECT
			head->setMaxSpeed(0, 1.0f);
			head->setMaxSpeed(1, 1.0f);
#endif
			head->setJoints(t,p,r);
		}
	}
}

void HeadController::doStart() {
	// Behavior startup
	BehaviorBase::doStart();
	// Enable head control
	SharedObject<HeadPointerMC> head;
	head->setHold(false);
	head_id = motman->addPersistentMotion(head);
	// Turn on wireless
	theLastOne=theOne;
	theOne=this;
	cmdsock=wireless->socket(Socket::SOCK_STREAM, 2048, 2048);
	wireless->setReceiver(cmdsock->sock, mechacmd_callback);
	wireless->setDaemon(cmdsock,true);
	wireless->listen(cmdsock->sock, config->main.headControl_port);
	// Open the WalkGUI on the desktop
	Controller::loadGUI("org.tekkotsu.mon.HeadPointGUI","HeadPointGUI",config->main.headControl_port);
}

void HeadController::doStop() {
	// Close the GUI
	Controller::closeGUI("HeadPointGUI");
	// Turn off timers
	erouter->removeListener(this);
	// Close socket; turn wireless off
	wireless->setDaemon(cmdsock,false);
	wireless->close(cmdsock);
	theOne=theLastOne;
	// Disable head pointer
	motman->removeMotion(head_id);
	// Total behavior stop
	BehaviorBase::doStop();
}

// The command packet reassembly mechanism
int HeadController::mechacmd_callback(char *buf, int bytes) {
  static char cb_buf[5];
  static int cb_buf_filled;

  // If there's an incomplete command in the command buffer, fill
  // up as much of the command buffer as we can and then execute it
  // if possible
  if(cb_buf_filled) {
    while((cb_buf_filled < 5) && bytes) {
      cb_buf[cb_buf_filled++] = *buf++;	// copy incoming buffer byte
      --bytes;				// decrement remaining byte ct.
    }
    // did we fill it? if so, execute! and mark buffer empty.
    if(cb_buf_filled == 5) {
      if(HeadController::theOne) HeadController::theOne->runCommand((unsigned char*) cb_buf);
      cb_buf_filled = 0;
    }
  }

  // now execute all complete bytes in the incoming buffer
  while(bytes >= 5) {
    if(HeadController::theOne) HeadController::theOne->runCommand((unsigned char *) buf);
    bytes -= 5;
    buf += 5;
  }

  // finally, store all remaining bytes in the command buffer
  while(bytes) {
    cb_buf[cb_buf_filled++] = *buf++;
    --bytes;
  }

  return 0;
}

#endif

/*! @file
 * @brief Implements HeadController, listens to control commands coming in from the command port for remotely controlling the head
 * @author tss (Creator)
 */

