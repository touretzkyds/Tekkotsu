#include "Shared/RobotInfo.h"
#ifdef TGT_IS_CHIARA

#include "Behaviors/Demos/Chiara/SwingAndSway.h"

REGISTER_BEHAVIOR_MENU(SwingAndSway,DEFAULT_TK_MENU"/Chiara Demos");

MotionPtr<XWalkMC> SwingAndSway::walker;
MotionPtr<HeadPointerMC> SwingAndSway::headpointer;

#endif
