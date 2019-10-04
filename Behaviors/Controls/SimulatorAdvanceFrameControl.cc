#ifndef PLATFORM_APERIOS		

#include "Behaviors/Controls/NullControl.h"
#include "Shared/ProjectInterface.h"

//! Requests the next camera frame and sensor data, for use when running in simulation
/*! Note that this won't increment the simulator time if triggered while paused... */
class SimulatorAdvanceFrameControl : public NullControl {
public:
	//! default constructor
	SimulatorAdvanceFrameControl()
		: NullControl("Advance Frame","Requests the next camera frame and sensor data, for use when running in simulation")
	{}
	//! constructor which allows a custom name
	SimulatorAdvanceFrameControl(const std::string& n)
		: NullControl(n,"Requests the next camera frame and sensor data, for use when running in simulation")
	{}
	
	virtual ControlBase * activate(MC_ID disp_id, Socket * gui) {
		ProjectInterface::sendCommand("advance");
		return NullControl::activate(disp_id,gui);
	}
	
	virtual std::string getName() const {
		if(canManuallyAdvance())
			return NullControl::getName();
		return "[Auto-Advancing]";
	}
	
	virtual std::string getDescription() const {
		if(canManuallyAdvance())
			return NullControl::getDescription();
		return "Cannot manually advance when in realtime mode, or when AdvanceOnAccess is enabled";
	}
	
protected:
	//! ideally, this should return true only when the simulator is paused or the data source is frozen...
	bool canManuallyAdvance() const { return true; }
};

REGISTER_CONTROL(SimulatorAdvanceFrameControl,"Vision Pipeline");

#endif

/*! @file
 * @brief Defines SimulatorAdvanceFrameControl, which requests the next camera frame and sensor data, for use when running in simulation
 * @author ejt (Creator)
 */
