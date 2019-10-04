#include "MotionCommand.h"
#include "MotionManager.h"
#include "Events/EventTranslator.h"
#include "Events/EventRouter.h"
#include "Events/LocomotionEvent.h"
#include "IPC/ProcessID.h"
#include <iostream>

using namespace std;

void MotionCommand::postEvent(const EventBase& event) {
	if(queue==NULL || ProcessID::getID()==ProcessID::MainProcess) {
		erouter->postEvent(event);
	} else {
		queue->encodeEvent(event);
	}
}



/*! @file
 * @brief Empty
 * @author ejt (Creator)
 */

