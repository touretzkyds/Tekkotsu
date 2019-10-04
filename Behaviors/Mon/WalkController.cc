#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK

#include "WalkController.h"
#include "Behaviors/Controller.h"
#include "Sound/SoundManager.h"

REGISTER_BEHAVIOR_MENU_OPT(WalkController,"TekkotsuMon",BEH_NONEXCLUSIVE);

using namespace std;

WalkController* WalkController::theOne = NULL;

void WalkController::runCommand(unsigned char *command) {
	// First, turn off the stop-if-no-heartbeat timer
	erouter->removeTimer(this);

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
	switch(command[0]) {
	case CMD_fwd:
		dx = param;
		break;
	case CMD_roto:
		da = param;
		break;
	case CMD_side:
		dy = param;
		break;
	case CMD_opt0:
		{
			/*			HeadPointerMC *head =
				(HeadPointerMC*)motman->checkoutMotion(head_id);
			head->setJoints(0,0,0);
			motman->checkinMotion(head_id);*/
			break;
		}
	case CMD_opt1:
	case CMD_opt2:
	case CMD_opt3:
	case CMD_opt4:
		cout << "MECHA: hey, reprogram this button!" << endl;
		break;
	case CMD_opt5:
		sndman->playFile("howl.wav");
		break;
	case CMD_opt6:
		sndman->playFile("yap.wav");
		break;
	case CMD_opt7:
		sndman->playFile("whimper.wav");
		break;
	case CMD_opt8:
		sndman->playFile("growl.wav");
		break;
	case CMD_opt9:
		sndman->playFile("barkmed.wav");
		break;
		// The options button commands.
	default:
		cout << "MECHA: unknown command " << command[0] << endl;
	}

	// If the command was a new motion command, apply the
	// new motion parameters:
	switch(command[0]) {
	case CMD_fwd:
	case CMD_roto:
	case CMD_side:
		{
			MMAccessor<WalkMC> walker(getWalkID());
#ifdef TGT_HAS_CMPACKWALK
			float tdx=dx*walker->getCP().max_vel[dx>0?WalkMC::CalibrationParam::forward:WalkMC::CalibrationParam::reverse];
			float tdy=dy*walker->getCP().max_vel[WalkMC::CalibrationParam::strafe];
			float tda=da*walker->getCP().max_vel[WalkMC::CalibrationParam::rotate];
#else
			float tdx=dx*walker->getMaxXVel();
			float tdy=dy*walker->getMaxYVel();
			float tda=da*walker->getMaxAVel();
#endif
			walker->setTargetVelocity(tdx,tdy,tda);
		}
	}

	// Reset the stop-if-no-heartbeat timer -- if we don't
	// hear from the mothership in three seconds, stop immediately.
	erouter->addTimer(this, TIMER_COMM_TIMEOUT, 3000, false);
	erouter->addTimer(this, TIMER_COMM_CHECK, 500, true);
}

void WalkController::doStart() {
	// Behavior startup
	BehaviorBase::doStart();
	erouter->addListener(this, EventBase::runtimeEGID);
	// Enable walker (the MC_ID can be accessed through the shared_walker later)
	motman->addPersistentMotion(shared_walker);
	// Turn on wireless
	theLastOne=theOne;
	theOne=this;
	cmdsock=wireless->socket(Socket::SOCK_STREAM, 2048, 2048);
	sock= cmdsock->sock;
	wireless->setReceiver(sock, mechacmd_callback);
	wireless->setDaemon(sock,true); 
	wireless->listen(sock, config->main.walkControl_port);
	// Open the WalkGUI on the desktop
	Controller::loadGUI("org.tekkotsu.mon.WalkGUI","WalkGUI",config->main.walkControl_port);
}

void WalkController::shutdown() {
	// Turn off timers
	erouter->removeListener(this);
	// Close socket; turn wireless off
	if(sock>0) {
		wireless->setDaemon(sock,false); 
		wireless->close(sock);
		cmdsock=NULL;
		sock=-1;
	}
	theOne=theLastOne;
	// Disable walker
	motman->removeMotion(getWalkID());
}

void WalkController::doStop() {
	shutdown();
	// Close the GUI
	if(!keepGUI)
		Controller::closeGUI("WalkGUI");
	// Total behavior stop
	BehaviorBase::doStop();
}

// The command packet reassembly mechanism
int WalkController::mechacmd_callback(char *buf, int bytes) {
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
      if(WalkController::theOne) WalkController::theOne->runCommand((unsigned char*) cb_buf);
      cb_buf_filled = 0;
    }
  }

  // now execute all complete bytes in the incoming buffer
  while(bytes >= 5) {
    if(WalkController::theOne) WalkController::theOne->runCommand((unsigned char *) buf);
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
 * @brief Implements WalkController, listens to mecha control commands coming in from the command port for remotely controlling the walk
 * @author tss (Creator)
 * @author ejt (modifications)
 * @author PA Gov. School for the Sciences 2003 Team Project - Haoqian Chen, Yantian Martin, Jon Stahlman (modifications)
 */
