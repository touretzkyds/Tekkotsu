#include <iostream>

#include "KoduState.h"

namespace Kodu {

  void KoduState::update(int generatoregid, int sourceid, float magnitude) {
		// std::cout << "Gamepad event: source=" << sourceid << " magnitude=" << magnitude << std::endl;
    if (generatoregid == EventBase::buttonEGID &&
				IS_GAMEPAD_SID(sourceid)) {
      switch(sourceid) {
			case GamepadSrcID::gamepadLeftJoyXSrcID:
				joystickx = -1*magnitude;
				// std::cout << "joystickx updated to " << joystickx << '\n';
				break;
			case GamepadSrcID::gamepadLeftJoyYSrcID:
				joysticky = -1*magnitude;
				// std::cout << "joysticky updated to " << joysticky << '\n';
				break;
			case GamepadSrcID::gamepadAButtonSrcID:
				abutton = magnitude;
				break;
			case GamepadSrcID::gamepadBButtonSrcID:
				bbutton = magnitude;
				break;
			case GamepadSrcID::gamepadXButtonSrcID:
				xbutton = magnitude;
				break;
			case GamepadSrcID::gamepadYButtonSrcID:
				ybutton = magnitude;
				break;
			case GamepadSrcID::gamepadRightBumperSrcID:
				rightbumper = magnitude;
				break;
			case GamepadSrcID::gamepadLeftBumperSrcID:
				leftbumper = magnitude;
				break;
			default:
				break;
      }
    }
  }

}
