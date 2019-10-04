#include "Shared/RobotInfo.h"
#if defined(TGT_IS_AIBO)

#include "UPennWalkControllerBehavior.h"
#include "Behaviors/Controller.h"
#include "Sound/SoundManager.h"

REGISTER_BEHAVIOR_MENU_OPT(UPennWalkControllerBehavior,"TekkotsuMon",BEH_NONEXCLUSIVE);

using namespace std;

UPennWalkControllerBehavior* UPennWalkControllerBehavior::theOne = NULL;

void UPennWalkControllerBehavior::runCommand(unsigned char *command) {
	// First, turn off the stop-if-no-heartbeat timer
	erouter->removeTimer(this);

	// Extract the command parameter
	float param;
	unsigned char *paramp = (unsigned char *) &param;

	paramp[0] = command[1];
	paramp[1] = command[2];
	paramp[2] = command[3];
	paramp[3] = command[4];

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
			MMAccessor<UPennWalkMC> walker(getWalkID());
			float tdx=dx*13;
			float tdy=dy*6.5f;
			float tda=da*.25f;
			walker->setTargetVelocity(tdx,tdy,tda);
		}
	}

	// Reset the stop-if-no-heartbeat timer -- if we don't
	// hear from the mothership in three seconds, stop immediately.
	erouter->addTimer(this, 0, 3000, false);
}

void UPennWalkControllerBehavior::doStart() {
	// Behavior startup
	BehaviorBase::doStart();
	// We listen to timers (but don't need to explicitly tell erouter -- addTimer implies this)
	//erouter->addListener(this, EventBase::timerEGID);
	// Enable walker (the MC_ID can be accessed through the shared_walker later)
	motman->addPersistentMotion(shared_walker);
	// Turn on wireless
	theLastOne=theOne;
	theOne=this;
	cmdsock=wireless->socket(Socket::SOCK_STREAM, 2048, 2048);
	wireless->setReceiver(cmdsock->sock, mechacmd_callback);
	wireless->setDaemon(cmdsock,true); 
	wireless->listen(cmdsock->sock, config->main.walkControl_port);
	// Open the WalkGUI on the desktop
	Controller::loadGUI("org.tekkotsu.mon.WalkGUI","WalkGUI",config->main.walkControl_port);
}

void UPennWalkControllerBehavior::doStop() {
	// Close the GUI
	Controller::closeGUI("WalkGUI");
	// Turn off timers
	erouter->removeListener(this);
	// Close socket; turn wireless off
	wireless->setDaemon(cmdsock,false); 
	wireless->close(cmdsock);
	theOne=theLastOne;
	// Disable walker
	motman->removeMotion(getWalkID());
	// Total behavior stop
	BehaviorBase::doStop();
}

// The command packet reassembly mechanism
int UPennWalkControllerBehavior::mechacmd_callback(char *buf, int bytes) {
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
      if(UPennWalkControllerBehavior::theOne) UPennWalkControllerBehavior::theOne->runCommand((unsigned char*) cb_buf);
      cb_buf_filled = 0;
    }
  }

  // now execute all complete bytes in the incoming buffer
  while(bytes >= 5) {
    if(UPennWalkControllerBehavior::theOne) UPennWalkControllerBehavior::theOne->runCommand((unsigned char *) buf);
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
 * @brief Implements UPennWalkControllerBehavior, listens to mecha control commands coming in from the command port for remotely controlling the walk
 * @author tss (Creator)
 * @author ejt (modifications)
 * @author PA Gov. School for the Sciences 2003 Team Project - Haoqian Chen, Yantian Martin, Jon Stahlman (modifications)
 */

