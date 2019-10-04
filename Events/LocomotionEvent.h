//-*-c++-*-
#ifndef INCLUDED_LocomotionEvent_h_
#define INCLUDED_LocomotionEvent_h_

#include "EventBase.h"
#include <iostream>

//! Gives updates regarding the current movement of the robot through the world
/*! An activate event will be sent when a potential source of motion is created,
 *  and a deactivate when it is destroyed.  Status events will be sent at any
 *  change of direction/speed.
 *
 *  The source ID field will hold the MotionManager::MC_ID of the sending MotionCommand
 */
class LocomotionEvent : public EventBase {
 public:

	//! @name Constructors

	//! constructor
	LocomotionEvent() : EventBase(),x(0),y(0),a(0) {}
	LocomotionEvent(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0) : EventBase(gid,sid,tid,dur),x(0),y(0),a(0) {}
	LocomotionEvent(EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag) : EventBase(gid,sid,tid,dur,n,mag),x(0),y(0),a(0) {}
	virtual EventBase* clone() const { return new LocomotionEvent(*this); }
	//@}

	virtual unsigned int getClassTypeID() const { return autoRegisterLocomotionEvent; }

	//! Allows you to set the new X, Y, and A components
	LocomotionEvent& setXYA(float X, float Y, float A) {
		x=X;
		y=Y;
		a=A;
		return *this;
	}

	virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;

	//!< Returns true if this event indicates motion has stopped (x,y,a are all essentially zero; may not be exactly zero in Mirage)
	virtual bool isStop() const { return fabs(x) < 1e-3 && fabs(y) < 1e-3 && fabs(a) < 1e-3;}

	float x; //!< the new x component (body relative)
	float y; //!< the new y component (body relative)
	float a; //!< the new angular component (body relative)

protected:
	//! causes class type id to automatically be registered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterLocomotionEvent;
};

/*! @file
 * @brief Describes LocomotionEvent, which gives updates regarding the current movement of the robot through the world
 * @author ejt (Creator)
 */

#endif
