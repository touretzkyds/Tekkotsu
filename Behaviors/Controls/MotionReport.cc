#include "Behaviors/Controls/NullControl.h"
#include "Motion/MotionManager.h"

//! Triggers MotionManager::motionReport() to display currently active motions and their output usage on stderr
class MotionReport : public NullControl {
public:
	//! default constructor
	MotionReport()
		: NullControl("Motion Report","Triggers MotionManager::motionReport() to display currently active motions and their output usage on stderr")
	{}
	//! constructor which allows a custom name
	MotionReport(const std::string& n)
		: NullControl(n,"Triggers MotionManager::motionReport() to display currently active motions and their output usage on stderr")
	{}
	//! constructor which allows a custom name and tooltip
	MotionReport(const std::string& n, const std::string& d)
		: NullControl(n,d)
	{}

	//! destructor
	virtual ~MotionReport() {}

	virtual ControlBase * activate(MC_ID disp_id, Socket * gui) {
		motman->motionReport();
		return NullControl::activate(disp_id,gui);
	}

private:
	MotionReport(const MotionReport&); //!< you can override, but don't call this...
	MotionReport& operator=(const MotionReport&); //!< you can override, but don't call this...
};

REGISTER_CONTROL(MotionReport,"Status Reports");

/*! @file
 * @brief Defines MotionReport, which DESCRIPTION
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
