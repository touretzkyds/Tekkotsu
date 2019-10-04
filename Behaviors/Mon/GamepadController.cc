#include <cassert>

#include "GamepadController.h"
#include "Behaviors/Controller.h"
#include "Shared/Gamepad.h"
#include <stdio.h>

using namespace std;

REGISTER_BEHAVIOR_MENU_OPT(GamepadController,"TekkotsuMon", BEH_START|BEH_NONEXCLUSIVE);

GamepadController* GamepadController::theOne = NULL;

// XXX: This would more accurately be a static assertion
void GamepadController::checkInputMapping(int thisVal, int enumVal) {
	assert((thisVal - X_BUTTON + GamepadSrcID::gamepadXButtonSrcID) == enumVal);
}

void GamepadController::processInput(unsigned char *buf) {
	// Exract the input parameter
	float param;
	unsigned char *paramp = (unsigned char *) &param;


	unsigned char input = buf[0];
	/*
	std::cout << " buf=["
						<< (int)buf[0] << " "
						<< (int)buf[1] << " "
						<< (int)buf[2] << " "
						<< (int)buf[3] << " "
						<< (int)buf[4] << "]" << std::endl;
	*/

#if defined(BYTE_ORDER) && BYTE_ORDER==BIG_ENDIAN
	paramp[0] = buf[1];
	paramp[1] = buf[2];
	paramp[2] = buf[3];
	paramp[3] = buf[4];
#else
	paramp[0] = buf[4];
	paramp[1] = buf[3];
	paramp[2] = buf[2];
	paramp[3] = buf[1];
#endif

	// Check for invalid input
	if (input < X_BUTTON || input > R_BUMPER) {
		if ( input != 0 )
			std::cout << "Invalid gamepad input index'" << (int)buf[0] << "'\n";
		return;
	}

	// There is a direct mapping from input indicator values to sourceids
	int sid = input - X_BUTTON + GamepadSrcID::gamepadXButtonSrcID;
	EventBase::EventTypeID_t tid = EventBase::statusETID;

	// Check whether we should change the event TypeID
	if (input >= X_BUTTON && input <= SELECT_BUTTON) {
		if (param == 0)	tid = EventBase::deactivateETID;
		else tid = EventBase::activateETID;
	}

	// Post the event
	erouter->postEvent(EventBase::buttonEGID, sid, tid, 0, "Gamepad Event", param);
}

void GamepadController::start() {
	// std::cout << "->->->->->->   Gamepad controller start" << std::endl;
	BehaviorBase::start();
	erouter->addListener(this, EventBase::runtimeEGID);

	theLastOne = theOne;
	theOne = this;
	inputsock = wireless->socket(Socket::SOCK_STREAM, 2048, 2048);
	sock = inputsock->sock;
	wireless->setReceiver(sock, input_callback);
	wireless->setDaemon(sock, true);
	wireless->listen(sock, config->main.gamepadControl_port);
}

void GamepadController::shutdown() {
	//Turn off timers
	erouter->removeListener(this);
	if (sock > 0) {
		wireless->setDaemon(sock, false);
		wireless->close(sock);
		inputsock = NULL;
		sock = -1;
	}
	theOne = theLastOne;
}

void GamepadController::stop() {
	// std::cout << "->->->->->->   Gamepad controller stop" << std::endl;
	shutdown();
	BehaviorBase::stop();
}

int GamepadController::input_callback(char* buf, int bytes) {
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
			if (GamepadController::theOne)
				GamepadController::theOne->processInput((unsigned char*) cb_buf);
			cb_buf_filled = 0;
		}
	}

	// now execute all complete bytes in the incoming buffer
	while(bytes >= 5) {
		if (GamepadController::theOne)
			GamepadController::theOne->processInput((unsigned char *) buf);
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

/*! @file
 * @brief Implements GamepadController, listens for gamepad inputs and posts events appropriately
 * @author asf
 * @author med
 */
