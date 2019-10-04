//-*-c++-*-
#ifndef INCLUDED_FreeMemReportControl_h_
#define INCLUDED_FreeMemReportControl_h_

#include "Behaviors/BehaviorBase.h"
#include "ControlBase.h"
#include "Events/EventRouter.h"
#include "BehaviorActivatorControl.h"
#include "ValueEditControl.h"

//! gives reports on free memory size at variable rates, can also warn if free memory drops too low.
class FreeMemReportControl : public BehaviorBase, public ControlBase {
public:
	//!@name Contructors/Destructors
	//!contructor
	FreeMemReportControl() : BehaviorBase("FreeMemReportControl"), ControlBase("Free Memory Report","Reports size of free memory, and monitors for low memory warning"), report_freq(-1U), low_mem(256), monitor_freq(1000), isWarning(false), lastReport() {init();}
	FreeMemReportControl(const std::string& n) : BehaviorBase("FreeMemReportControl"), ControlBase(n,"Reports size of free memory, and monitors for low memory warning"), report_freq(-1U), low_mem(256), monitor_freq(1000),isWarning(false), lastReport() {init();}
	FreeMemReportControl(const std::string& n, const std::string& d) : BehaviorBase("FreeMemReportControl"), ControlBase(n,d), report_freq(-1U), low_mem(256), monitor_freq(1000),isWarning(false), lastReport() {init();}
	virtual ~FreeMemReportControl() { clearSlots(); stop(); } //!< destructor
	//@}

	virtual void doStart() {
		BehaviorBase::doStart();
		resetTimerFreq();
	}

	virtual void doStop() {
		isWarning=false;
		erouter->removeListener(this);
		BehaviorBase::doStop();
	}

	virtual void doEvent();

	virtual std::string getName() const { return (isActive()?"#":"-")+ControlBase::getName(); }

	virtual void refresh();
	
	//! reports size of free memory - if this is below low_mem, also generates a warning
	void report();

	//! returns the size of the free memory
	static size_t freeMem();

	//! resets timer delays
	void resetTimerFreq();

protected:
	//! builds the submenus
	void init() {
		setAutoDelete(false);
		pushSlot(new BehaviorActivatorControl(this));
		pushSlot(new NullControl("Free Mem: unknown"));
		pushSlot(new ValueEditControl<int>("Report Frequency","Controls how often to generate free memory reports (in milliseconds)","Please enter milliseconds to wait between reports (-1 to stop)",&report_freq));
		pushSlot(new ValueEditControl<unsigned int>("Low Memory Threshold","Controls when to start warning about low memory (in KB)","Please enter the new low memory warning threshold (in KB)",&low_mem));
		pushSlot(new ValueEditControl<unsigned int>("Monitor Frequency","Controls how often to check for low memory (in milliseconds)","Please enter milliseconds to wait between low memory checks (-1 to stop)",&monitor_freq));
		lastReport=freeMem();
	}

	int report_freq;  //!< how often to report memory size (in milliseconds - negative turns off, 0 is as often as possible)
	unsigned int low_mem;      //!< threshold to trigger low memory warning (in kilobytes)
	unsigned int monitor_freq; //!< how often to check for low memory (in milliseconds - -1U turns off, 0 is as often as possible)
	bool isWarning;            //!< true we already know we're below threshold
	size_t lastReport; //!< free memory at last report so we can report the difference
};

/*! @file
 * @brief Describes FreeMemReportControl, which gives reports on free memory size at various (configurable) rates
 * @author ejt (object), alokl (core function)
 */

#endif
