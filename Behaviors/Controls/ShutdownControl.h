//-*-c++-*-
#ifndef INCLUDED_ShutdownControl_h_
#define INCLUDED_ShutdownControl_h_

#include "NullControl.h"

//! when activated, this will cause the aibo to shut down
class ShutdownControl : public NullControl {
public:

	ShutdownControl() : NullControl("Shutdown","Turns the Aibo off") {} //!< constructor
	ShutdownControl(const std::string& n) : NullControl(n,"Turns the Aibo off") {} //!< constructor
	ShutdownControl(const std::string& n, const std::string& d) : NullControl(n,d) {} //!< constructor

	virtual ControlBase * activate(MC_ID , Socket * ) { return doSelect(); } //!< calls doSelect()

	//! shuts down
	virtual ControlBase * doSelect();
};

/*! @file
 * @brief Describes ShutdownControl, which initiates the shutdown sequence
 * @author ejt (Creator)
 */

#endif
