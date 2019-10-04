//-*-c++-*-
#ifndef INCLUDED_ValueSetControl_h
#define INCLUDED_ValueSetControl_h

#include "ControlBase.h"

//! Upon activation, this control will set the target pointer to the specified value
template < class T >
class ValueSetControl : public ControlBase {
public:
	/*!@name Constructors/Destructors*/
	//!constructor
	ValueSetControl() : ControlBase(), target(NULL) {}
	ValueSetControl(const std::string& n, T* t) : ControlBase(n), target(t) {}
	ValueSetControl(const std::string& n, T* t, const T& d) : ControlBase(n), target(t), def(d) {}
	ValueSetControl(const ValueSetControl& vsc) : target(vsc.target), def(vsc.def) {} //!<copy constructor
	ValueSetControl operator=(const ValueSetControl& vsc) { ControlBase::operator=(vsc); target=vsc.target; def=vsc.def; return *this; } //!<assignment operator
	virtual ~ValueSetControl() {} //!< destructor
	//@}

	//! assigns #def to object pointed to by #target
	virtual ControlBase * activate(MC_ID display) {
		*target=def;
		if(display!=invalid_MC_ID) {
			//!@todo make the leds flash
		}
		return NULL;
	}

	/*!@name Target
	 * accessors for the target pointer */
	virtual T* getTarget() const { return target; } //!< returns the target pointer
	virtual ValueSetControl& setTarget(T* t) { target=t; return *this; } //!< sets the target pointer - the object pointed to will be overwritten on activate(); returns @c *this
	//@}

	/*!@name Value
	 * accessors for the default value assigned when activated */
	virtual T& getDefault() { return def; } //!< gets reference to default value
	virtual const T& getDefault() const { return def; } //!< gets reference to default value
	virtual ValueSetControl& setDefault(const T& d) { def=d; return *this; } //!< assigns d to the default value (not to the target, yet); returns @c *this
	//@}	

protected:
	T* target; //!< the target that will be set to the default value (#def)
	T def; //!< the value that will be assigned to #target upon a call to activate()
};

/*! @file
 * @brief Defines ValueSetControl class, which will assign a value through a pointer upon activation
 * @author ejt (Creator)
 */

#endif
