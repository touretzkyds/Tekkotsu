#include "ControlBase.h"
#include "Shared/Profiler.h"
#include "Wireless/Socket.h"

//! causes the WorldState::mainProfile and WorldState::motionProfile to display reports to #sout
class ProfilerCheckControl : public ControlBase {
public:
	//! Constructor
	ProfilerCheckControl() : ControlBase("Profiler Check","Reports time spent in all of the profiled sections in all processes") {}

	//! Prints a report to sout
	virtual ControlBase * activate(MC_ID, Socket *) {
		sout->printf("~~~ Main: ~~~\n%s\n",mainProfiler==NULL?"Main profile unavailable":mainProfiler->report().c_str());
		sout->printf("~~~ Motion: ~~~\n%s\n",motionProfiler==NULL?"Motion profile unavailable":motionProfiler->report().c_str());
		sout->printf("~~~ Sound: ~~~\n%s\n",soundProfiler==NULL?"Sound profile unavailable":soundProfiler->report().c_str());
		return NULL;
	}
};

REGISTER_CONTROL(ProfilerCheckControl,"Status Reports");

/*! @file
 * @brief Defines ProfilerCheckControl, which causes the WorldState::mainProfile and WorldState::motionProfile to display reports to #sout
 * @author ejt (Creator)
 */
