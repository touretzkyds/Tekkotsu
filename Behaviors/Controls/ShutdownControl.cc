#include "ShutdownControl.h"
#ifdef PLATFORM_APERIOS
#  include <OPENR/OPENRAPI.h>
#else
#  include "Shared/ProjectInterface.h"
#endif

ControlBase * ShutdownControl::doSelect()    {
#ifdef PLATFORM_APERIOS
	OBootCondition bc(0);
	OPENR::Shutdown(bc);
#else
	ProjectInterface::sendCommand("quit");
#endif
	return NULL;
}

/*! @file
 * @brief Implements ShutdownControl, which causes the aibo to shutdown (very short - one function separated out to limit recompile of the OPENR headers)
 * @author ejt (Creator)
 */

