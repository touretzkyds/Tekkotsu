// Kodu Library
#include "Kodu/KoduWorld.h"
#include "Kodu/Primitives/KoduConditionGamepad.h"
#include "Kodu/General/GeneralMacros.h"
#include "Shared/Gamepad.h"
#include "Kodu/General/KoduState.h"

namespace Kodu {

  bool KoduConditionGamepad::evaluate(const KoduWorld& kWorldState) {
		bool result = ( state.joystickx != 0 || state.joysticky != 0 );
		return result;
  }

}
