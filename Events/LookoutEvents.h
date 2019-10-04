//-*-c++-*-
#ifndef INCLUDED_LookoutEvents_h_
#define INCLUDED_LookoutEvents_h_

#include <iostream>

#include "EventBase.h"
#include "Crew/LookoutRequests.h"
#include "DualCoding/Sketch.h"
#include "Shared/fmatSpatial.h"

//! Abstract base class for all Lookout Events
class LookoutEvent : public EventBase {
public:
  enum LookoutEventType_t { lookAt, sketch, ir, scan, track, search };
  bool success;
  virtual LookoutEventType_t getLookoutEventType() const = 0;
  //{ constructors which take exact same set of arguments as EventBase's and pass them directory to EventBase
  LookoutEvent() : EventBase(), success(false) {}
  LookoutEvent(bool _success, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0)
    : EventBase(gid,sid,tid,dur), success(_success) {}
  LookoutEvent(bool _success, EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : EventBase(gid,sid,tid,dur,n,mag), success(_success) {}
  //}
};

// This event gives access to transformation matrix from joint specified from LookoutRequest to Base Frame
class LookoutPointAtEvent : public LookoutEvent {
public:
  fmat::Transform toBaseMatrix;
  virtual LookoutEventType_t getLookoutEventType() const { return lookAt; }
  LookoutPointAtEvent() : LookoutEvent(), toBaseMatrix() { }
  LookoutPointAtEvent(bool _success, const fmat::Transform& mat) : LookoutEvent(), toBaseMatrix(mat) { success = _success; }
  LookoutPointAtEvent(bool _success, const fmat::Transform& mat, EventGeneratorID_t gid, 
		     size_t sid, EventTypeID_t tid, unsigned int dur=0) 
    : LookoutEvent(_success,gid,sid,tid,dur),toBaseMatrix(mat) {}
  LookoutPointAtEvent(bool _success, const fmat::Transform& mat, EventGeneratorID_t gid, 
		     size_t sid, EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : LookoutEvent(_success,gid,sid,tid,dur,n,mag), toBaseMatrix(mat) {}
  virtual EventBase* clone() const { return new LookoutPointAtEvent(*this); }
  virtual unsigned int getClassTypeID() const { return autoRegisterLookoutPointAtEvent; }
  virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
  virtual unsigned int getBinSize() const;
  virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
  virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
  virtual void loadXML(xmlNode* node);
  virtual void saveXML(xmlNode * node) const;
protected:
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterLookoutPointAtEvent;
};


// Event which gives you access to the sketch stored as a success of StoreImage request
class LookoutSketchEvent : public LookoutPointAtEvent {
protected:
  DualCoding::Sketch<DualCoding::uchar> img; // sketch returned by the Lookout
  
public:
  virtual LookoutEventType_t getLookoutEventType() const { return sketch; }
  LookoutSketchEvent() : LookoutPointAtEvent(), img() {}
  LookoutSketchEvent(bool _success, DualCoding::Sketch<DualCoding::uchar>& _img, const fmat::Transform& mat)
    : LookoutPointAtEvent(_success,mat), img(_img) {}
  LookoutSketchEvent(bool _success, DualCoding::Sketch<DualCoding::uchar>& _img, const fmat::Transform& mat, 
		     EventGeneratorID_t gid, size_t sid, EventTypeID_t tid, unsigned int dur=0) 
    : LookoutPointAtEvent(_success, mat,gid,sid,tid,dur), img(_img) {}
  LookoutSketchEvent(bool _success, DualCoding::Sketch<DualCoding::uchar>& _img, const fmat::Transform& mat, 
		     EventGeneratorID_t gid, size_t sid, 
		     EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : LookoutPointAtEvent(_success,mat,gid,sid,tid,dur,n,mag), img(_img) {}
  //! copy constructor (shallow copy)
  LookoutSketchEvent(const LookoutSketchEvent& other)
    : LookoutPointAtEvent(other), img(other.img) {}
  
  const DualCoding::Sketch<DualCoding::uchar>& getSketch() const { return img; }
  virtual EventBase* clone() const { return new LookoutSketchEvent(*this); }
  //  virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
	
	virtual unsigned int getClassTypeID() const { return autoRegisterLookoutSketchEvent; }
protected:
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterLookoutSketchEvent;
private:
  LookoutSketchEvent& operator=(const LookoutSketchEvent&);
};

class LookoutIREvent : public LookoutPointAtEvent {
public:
  float distance;
  virtual LookoutEventType_t getLookoutEventType() const { return ir; }
  LookoutIREvent() : LookoutPointAtEvent(), distance() {}
  LookoutIREvent(bool _success, float dist, const fmat::Transform& mat) : LookoutPointAtEvent(_success,mat), distance(dist) {}
  LookoutIREvent(bool _success, float dist, const fmat::Transform& mat, EventGeneratorID_t gid, 
		 size_t sid, EventTypeID_t tid, unsigned int dur=0) 
    : LookoutPointAtEvent(_success, mat, gid,sid,tid,dur), distance(dist) {}
  LookoutIREvent(bool _success, float dist, const fmat::Transform& mat, EventGeneratorID_t gid, size_t sid, 
		 EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : LookoutPointAtEvent(_success,mat,gid,sid,tid,dur,n,mag), distance(dist) {}
  virtual EventBase* clone() const { return new LookoutIREvent(*this); }
  virtual unsigned int getClassTypeID() const { return autoRegisterLookoutIREvent; }
  virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
  virtual unsigned int getBinSize() const;
  virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
  virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
  virtual void loadXML(xmlNode* node);
  virtual void saveXML(xmlNode * node) const;
protected:
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterLookoutIREvent;
};

class LookoutScanEvent : public LookoutEvent {
protected:
  //! pointer to tasks implemented during the scan
  std::vector<DualCoding::LookoutScanRequest::Task*> *tasks;
public:
  virtual LookoutEventType_t getLookoutEventType() const { return scan; }
  LookoutScanEvent() : LookoutEvent(), tasks() {}
  LookoutScanEvent(std::vector<DualCoding::LookoutScanRequest::Task*>& _tasks) : LookoutEvent(), tasks(&_tasks) {}
  LookoutScanEvent(std::vector<DualCoding::LookoutScanRequest::Task*>& _tasks, EventGeneratorID_t gid, 
		   size_t sid, EventTypeID_t tid, unsigned int dur=0) 
    : LookoutEvent(true,gid,sid,tid,dur), tasks(&_tasks) {}
  LookoutScanEvent(std::vector<DualCoding::LookoutScanRequest::Task*>& _tasks, EventGeneratorID_t gid, size_t sid, 
		   EventTypeID_t tid, unsigned int dur, const std::string& n, float mag)
    : LookoutEvent(true,gid,sid,tid,dur,n,mag), tasks(&_tasks) {}
  //! copy constructor (shallow copy)
  LookoutScanEvent(const LookoutScanEvent& lose)
    : LookoutEvent(lose), tasks(lose.tasks) {}
  //! assignment operator (shallow copy)
  const LookoutScanEvent& operator=(const LookoutScanEvent& lose) {
    if (this == &lose) return *this;
    LookoutEvent::operator=(lose);
    tasks = lose.tasks;
    return *this;
  }
  virtual EventBase* clone() const { return new LookoutScanEvent(*this); }
  const std::vector<DualCoding::LookoutScanRequest::Task*>& getTasks() const { return *tasks; }
	
	virtual unsigned int getClassTypeID() const { return autoRegisterLookoutScanEvent; }
protected:
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterLookoutScanEvent;
};

#endif
