#include "RebootControl.h"
#ifdef PLATFORM_APERIOS
#  include <OPENR/OPENRAPI.h>
#endif

ControlBase * RebootControl::doSelect()    {
#ifdef PLATFORM_APERIOS
	OBootCondition bc(obcbBOOT_TIMER, 0, obcbttRELATIVE);
	OPENR::Shutdown(bc);
#endif
	return NULL;
}

/*! @file
 * @brief Implements RebootControl, which causes the aibo to reboot (very short - one function separated out to limit recompile of the OPENR headers)
 * @author ejt (Creator)
 */

