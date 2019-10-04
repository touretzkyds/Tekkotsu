//-*-c++-*-
#ifndef INCLUDED_NullControl_h_
#define INCLUDED_NullControl_h_

#include "ControlBase.h"

//! when activated, this will return immediately  (handy for fake items in a menu)
class NullControl : public ControlBase {
public:

	//!Constructor
	explicit NullControl(ControlBase * inputRedirectTgt=NULL) : ControlBase(), inputRedirect(inputRedirectTgt) {}
	//!Constructor
	explicit NullControl(const std::string& n, ControlBase * inputRedirectTgt=NULL) : ControlBase(n), inputRedirect(inputRedirectTgt) {}
	//!Constructor
	NullControl(const std::string& n, const std::string& d, ControlBase * inputRedirectTgt=NULL) : ControlBase(n,d), inputRedirect(inputRedirectTgt) {}

	virtual void setInputRedirect(ControlBase* inputRedirectTgt) { inputRedirect=inputRedirectTgt; } //!< sets #inputRedirect
	virtual ControlBase* getInputRedirect() const { return inputRedirect; } //!< returns #inputRedirect
	
	//@{
	//! returns NULL
	virtual ControlBase * activate(MC_ID , Socket * ) { return NULL; }

	virtual ControlBase * doSelect()    { return NULL; }
	virtual ControlBase * doNextItem()  { return NULL; }
	virtual ControlBase * doPrevItem()  { return NULL; }
	virtual ControlBase * doReadStdIn(const std::string& /*prompt*/=std::string()) { return NULL; }
	//@}

	//! returns NULL unless #inputRedirect is set, in which case it will return inputRedirect->takeInput(msg)
	virtual ControlBase * takeInput(const std::string& msg) { return (inputRedirect==NULL) ? NULL : inputRedirect->takeInput(msg); }

protected:
	//! the target to receiving forwarding of any calls to takeInput()
	/*! this is handy if this instance is some feedback to the user, and any input they
	 *  enter with this control selected only makes sense to be handled by the parent */
	ControlBase* inputRedirect;

private:
	NullControl(const NullControl&); //!< you can override, but don't call this...
	NullControl& operator=(const NullControl&); //!< you can override, but don't call this...
};

/*! @file
 * @brief Defines NullControl, which does absolutely nothing (handy for fake items in a menu)
 * @author ejt (Creator)
 */

#endif
