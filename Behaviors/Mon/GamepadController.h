//-*-c++-*-
#ifndef INCLUDED_GamepadController_h_
#define INCLUDED_GamepadController_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventBase.h"
#include "Shared/Config.h"
#include "Shared/Gamepad.h"
#include "Wireless/Wireless.h"

//! Listens to Gamepad events coming in from the gamepad port
/*! The communication protocol is a simple, 5-byte group. The first byte
 *  indicates which button/joystick had a state change, and the next 4 bytes
 *  indicate the new state value for that input:
 *
 *  - <@c char: input indicator>
 *  - <@c float: value>
 *
 */
class GamepadController : public BehaviorBase {
	public:
		//! Points to the one GamepadController object that the input stream is
		//! talking to.
		static GamepadController *theOne;
		static int input_callback(char *buf, int bytes); //!< Called when new data comes in over wireless

	protected:
		//! The input indicator values
		enum {
			X_BUTTON=1,
			Y_BUTTON,
			A_BUTTON,
			B_BUTTON,
			START_BUTTON,
			SELECT_BUTTON,
			LJOY_X,
			LJOY_Y,
			RJOY_X,
			RJOY_Y,
			DPAD_X,
			DPAD_Y,
			L_BUMPER,
			R_BUMPER
		};

	private:
		//! The last GamepadController that was theOne.
		GamepadController *theLastOne;

		//! The input stream socket
		Socket *inputsock;
		//! The actual socket
		int sock;

		//! Closes the server socket
		void shutdown();

		//! Processes a received packet
		void processInput(unsigned char* buf);

		void checkInputMapping(int thisVal, int enumVal);

		GamepadController(const GamepadController&); //!< don't call
		GamepadController operator=(const GamepadController&); //!< don't call

	public:
		GamepadController() :
			BehaviorBase("GamepadController"),
			theLastOne(theOne),
			inputsock(NULL), sock(-1)
		{
			//Make sure the assumptions of the class hold true
			//XXX: these should be static assertions
			checkInputMapping(X_BUTTON, GamepadSrcID::gamepadXButtonSrcID);
			checkInputMapping(Y_BUTTON, GamepadSrcID::gamepadYButtonSrcID);
			checkInputMapping(A_BUTTON, GamepadSrcID::gamepadAButtonSrcID);
			checkInputMapping(B_BUTTON, GamepadSrcID::gamepadBButtonSrcID);
			checkInputMapping(START_BUTTON, GamepadSrcID::gamepadStartButtonSrcID);
			checkInputMapping(SELECT_BUTTON, GamepadSrcID::gamepadSelectButtonSrcID);
			checkInputMapping(LJOY_X, GamepadSrcID::gamepadLeftJoyXSrcID);
			checkInputMapping(LJOY_Y, GamepadSrcID::gamepadLeftJoyYSrcID);
			checkInputMapping(RJOY_X, GamepadSrcID::gamepadRightJoyXSrcID);
			checkInputMapping(RJOY_Y, GamepadSrcID::gamepadRightJoyYSrcID);
			checkInputMapping(DPAD_X, GamepadSrcID::gamepadDPadXSrcID);
			checkInputMapping(DPAD_Y, GamepadSrcID::gamepadDPadYSrcID);
			checkInputMapping(L_BUMPER, GamepadSrcID::gamepadLeftBumperSrcID);
			checkInputMapping(R_BUMPER, GamepadSrcID::gamepadRightBumperSrcID);
		}

		virtual ~GamepadController() { theOne = theLastOne; }

		virtual void start();

		virtual void stop();

		virtual void doEvent() {
			if (event->getGeneratorID() == EventBase::runtimeEGID &&
					event->getTypeID() == EventBase::deactivateETID) {
				shutdown();
				BehaviorBase::stop();
			}
		}

		static std::string getClassDescription() {
			char tmp[20];
			sprintf(tmp,"%d",*config->main.gamepadControl_port);
			return std::string("Listens to gamepad inputs coming in from port ")+tmp;
		}
		virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Describes GamepadController, listens to inputs and fires events appropriately
 * @author asf
 * @author med
 */

#endif
