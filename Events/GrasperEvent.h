//-*-c++-*-
#ifndef INCLUDED_GrasperEvent_h_
#define INCLUDED_GrasperEvent_h_

#include <iostream>

#include "EventBase.h"
#include "Crew/GrasperRequest.h"

//! Event for reporting the results of a Grasper operation
class GrasperEvent : public EventBase {
public:
	
  bool success;
  std::vector<GrasperRequest::NodeValue_t> path;
  Point suggestedRobotLocation;
  Point suggestedLookAtPoint;
  GrasperRequest::GrasperRequestType_t requestType;
  GrasperRequest::GrasperErrorType_t errorType;

  virtual bool getSuccess() const { return success; }
  virtual GrasperRequest::GrasperRequestType_t getRequestType() const { return requestType; }
  virtual GrasperRequest::GrasperErrorType_t getErrorType() const { return errorType; }
	
  //! Constructors
  GrasperEvent() : EventBase(), success(false), path(), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(), errorType() {}
  //!
  GrasperEvent(bool _success, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0)
    : EventBase(gid,sid,tid,dur), success(_success), path(), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(), errorType() {}
  GrasperEvent(bool _success, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : EventBase(gid,sid,tid,dur,n,mag), success(_success), path(), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(), errorType() {}
  //! 
  GrasperEvent(bool _success, GrasperRequest::GrasperRequestType_t grqt, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0)
    : EventBase(gid,sid,tid,dur), success(_success), path(), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(grqt), errorType() {}
  //!
  //	GrasperEvent(bool _success, GrasperRequest::GrasperRequestType_t grqt, Grasper::GrasperErrorType_t grst, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0)
  //		: EventBase(gid,sid,tid,dur), success(_success), path(NULL), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(grqt), errorType(grst) {}
  //! 
  GrasperEvent(bool _success, GrasperRequest::GrasperRequestType_t grqt, GrasperRequest::GrasperErrorType_t grst, size_t sid)
    : EventBase(EventBase::grasperEGID,sid,EventBase::statusETID,0), success(_success), path(), suggestedRobotLocation(), suggestedLookAtPoint(), requestType(grqt), errorType(grst) {}
	
  virtual EventBase* clone() const { return new GrasperEvent(*this); }
	
  GrasperEvent(const GrasperEvent &other) :
    success(other.success), path(other.path),
    suggestedRobotLocation(other.suggestedRobotLocation),
    suggestedLookAtPoint(other.suggestedLookAtPoint),
    requestType(other.requestType),
    errorType(other.errorType) {}

private:
  GrasperEvent& operator=(const GrasperEvent&);

};

/*
 *  GrasperEvents.h
 *
 *  Created by Glenn Nickens on 7/30/09.
 *
 */

#endif
