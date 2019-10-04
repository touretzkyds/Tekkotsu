//-*-c++-*-
#ifndef INCLUDED_ValueEditControl_h
#define INCLUDED_ValueEditControl_h

#include "StringInputControl.h"
#include "Events/EventListener.h"
#include "Events/EventBase.h"
#include "Events/EventRouter.h"
#include "Shared/WorldState.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"
#include "Wireless/Wireless.h"
#include "Behaviors/Controller.h"
#include <vector>
#include <sstream>

//! allows real-time modification of a value through a pointer @todo needs some work to really be useful again
template< class T >
class ValueEditControl : public StringInputControl, public EventListener {
 public:
	//!constructor
	ValueEditControl(const std::string& n, T* t) : StringInputControl(n,"Please enter a new value for "+n), target(t), cur(), copies() {}
	//!constructor
	ValueEditControl(const std::string& n, const std::string& p, T* t) : StringInputControl(n,p), target(t), cur(), copies() {}
	//!constructor
	ValueEditControl(const std::string& n, const std::string& d, const std::string& p, T* t) : StringInputControl(n,d,p), target(t), cur(), copies() {}
	//!copy constructor
	ValueEditControl(const ValueEditControl<T>& vec) : StringInputControl(vec), target(vec.target), cur(vec.cur), copies(vec.copies) {}
	//!assignment operator
	ValueEditControl operator=(const ValueEditControl<T>& vec) { StringInputControl::operator=(vec); target=vec.target; cur=vec.cur; copies=vec.copies; return *this; }
	//!destructor
	virtual ~ValueEditControl() {}

	//!reads in current value from target
	virtual ControlBase * activate(MC_ID display, Socket * gui) {
		cur=*target;
		erouter->removeListener(this);
		return StringInputControl::activate(display,gui);
	}
	//! will increment/decrement the current and then assign it to the target when head buttons pressed
	virtual void processEvent(const EventBase& event) {
		if(Controller::nextItem == event) {
			doNextItem();
			doSelect();
		} else if(Controller::prevItem == event) {
			doPrevItem();
			doSelect();
		} else {
			serr->printf("*** WARNING ValueEditControl got an unasked for event\n");
		}
	}
	
	//! displays current value
	virtual void refresh() {
		//Do console only if GUI is connected
		if(gui_comm!=NULL && wireless->isConnected(gui_comm->sock)) {
			std::stringstream ss;
			ss << getName();
			if(cur!=*target)
				ss << ": " << cur;
			sout->printf("%s\n",ss.str().c_str());
		}

		StringInputControl::refresh();
	}

	//! request to continue receiving events so we can modify the value while running
	virtual void pause() {
		erouter->addListener(this,Controller::nextItem);
		erouter->addListener(this,Controller::prevItem);
		//		erouter->addListener(this,EventBase(EventBase::buttonEGID,ChinButOffset,EventBase::deactivateETID,0));
		StringInputControl::pause();
	}

	//! if the value of the #target!=#cur, assigns the current value to the target and all the #copies
	virtual ControlBase * doSelect()   {
		if(*target!=cur) {
			*target=cur;
			for(typename std::vector<T*>::iterator it=copies.begin(); it!=copies.end(); it++)
				**it=cur;
			//			if(display) {
			//				display->flash(FaceLEDMask,100);
			//				display->clear();
			//			}
			std::stringstream ss;
			ss << getName() << " set to " << *target;
			sout->printf("%s\n",ss.str().c_str());
		}
		return NULL;
	}
	//! adds one to the current value
	virtual ControlBase * doNextItem() {
		cur=(T)(cur+1);
		refresh();
		return this;
	}
	//! subtracts one from the current value
	virtual ControlBase * doPrevItem() {
		cur=(T)(cur-1);
		refresh();
		return this;
	}

	virtual ControlBase * takeInput(const std::string& str) {
		cur = (T)atof(str.c_str());
		StringInputControl::takeInput(str);
		return doSelect();
	}

	/*!@name Target
	 * accessors for the target pointer */
	virtual T* getTarget() const { return target; } //!< returns the target pointer
	virtual ValueEditControl& setTarget(T* t) { target=t; return *this; } //!< sets the target pointer - the object pointed to will be overwritten on activate(); returns @c *this
	//@}

	/*!@name Copies
	 * accessors for the copies vector, so you can assign the same value to several places if you need to */
	virtual std::vector<T*>& getCopies() { return copies; } //!< returns a reference to the vector #copies
	virtual ValueEditControl& addCopy(T* t) { copies.push_back(t); return *this; } //!< copies.push_back(t)
	//@}

	//! shows current value
	virtual std::string getName() const {
		std::stringstream ss;
		ss << StringInputControl::getName() << " (" << *target << ")";
		return ss.str();
	}

 protected:
	T* target; //!< the main target
	T cur; //!< the value to use when set
	std::vector<T*> copies; //!< additional targets
};

/*! @file
 * @brief Defines ValueEditControl class, which will allow modification of a value through a pointer
 * @author ejt (Creator)
 */

#endif
