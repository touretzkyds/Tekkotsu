//-*-c++-*-
#ifndef INCLUDED_LookoutRequests_h_
#define INCLUDED_LookoutRequests_h_

#include "Shared/ProjectInterface.h"
#include "Shared/WorldState.h"

#include "DualCoding/Sketch.h"
#include "DualCoding/Point.h"
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/VRmixin.h"

namespace DualCoding {
  
//! Base class for requests to the Lookout
class LookoutRequestBase {
public:

  enum HeadMotionType_t { 
    noMotion,	//!< use current head position
    pointAt,	//!< move head to specified gaze point
    scan,	//!< scan head along specified path
    track,	//!< move head to track object
    search,	//!< spiral search for a known object
    numHeadMotionTypes
  };

  static const char* const headMotionTypeNames[numHeadMotionTypes];

  HeadMotionType_t getHeadMotionType() const { return headMotionType; }
  void setHeadMotionType(const HeadMotionType_t htype) { headMotionType = htype; }

  enum LookoutResultType_t {
    noResult,	     //!< don't return anything (just move the head)
    imageResult,     //!< take a picture
    distanceResult,  //!< measure distance with IR rangefinder
    interestPoints   //!< collection of interest points (from scanning)
  };

  LookoutResultType_t getResultType() const { return resultType; }
  void setResultType(const LookoutResultType_t rtype) { resultType = rtype; }

  //! Constructor
  LookoutRequestBase(HeadMotionType_t htype=noMotion, LookoutResultType_t rtype=noResult) :
    headMotionType(htype), resultType(rtype), requestID(0) {}

  //! Destructor
  virtual ~LookoutRequestBase() {}

  //! Copy constructor
  LookoutRequestBase(const LookoutRequestBase &req) : 
    headMotionType(req.headMotionType),
    resultType(req.resultType),
    requestID(req.requestID) {}

  HeadMotionType_t  headMotionType;
  LookoutResultType_t resultType;
  unsigned int requestID;   //!< Non-zero value assigned when the request is added to the queue

private:
  LookoutRequestBase& operator=(const LookoutRequestBase&);

public:
  // ------------ Tasks that may be implemented during a scan or track request ----------------
  //! Base class for Lookout tasks; cannot instantiate directly
  class Task {
  public:
    enum TaskType_t { noTask, visObjTask, visRegTask, irTask };

    virtual TaskType_t getTaskType() const = 0;
    virtual Task* clone() const = 0;

    //! Constructor
    Task(AngPi _dTheta) : dTheta(_dTheta), data() {}

    //! Copy constructor
    Task(const Task& t) : dTheta(t.dTheta), data(t.data) {}

    //! Destructor
    virtual ~Task() {}

    AngPi dTheta; //!< angular step size during scan
    std::vector<Point> data; //!< measured data stored here in base frame coordinates
    Task& operator=(const Task&);
  };

  class IRTask : public Task {
  public:
    IRTask(AngPi _dTheta) : Task(_dTheta) {}
    IRTask(const IRTask& t) : Task(t) {}
    virtual TaskType_t getTaskType() const { return irTask; }
    virtual Task* clone() const { return new IRTask(*this); }
  };

  //! Base class for vision tasks, should not be instantiated
  class VisionTask : public Task {
  public:
    virtual TaskType_t getTaskType() const { return noTask; }
    std::set<color_index> index;
    VisionTask(const VisionTask& vt) : Task(vt), index(vt.index) {}
    VisionTask(const std::set<color_index>& _index, AngPi _dTheta)
      : Task(_dTheta), index(_index) {}
    VisionTask(int _index, AngPi _dTheta)
      : Task(_dTheta), index() { index.insert(_index); }
    virtual Task* clone() const { return new VisionTask(*this); }
  };

  //! Uses bult-in object detectors (like pink ball detector) via VisionObjectEvent stream
  class VisionObjectTask : public VisionTask {
  public:
    VisionObjectTask(const std::set<color_index>& sid, AngPi _dTheta=0)
      : VisionTask(sid,_dTheta) {}
    VisionObjectTask(const VisionObjectTask& vot) : VisionTask(vot) {}
    virtual TaskType_t getTaskType() const { return visObjTask; }
    virtual Task* clone() const { return new VisionObjectTask(*this); }
  };

  //! Uses built-in colored region detectors via Region event stream
  class VisionRegionTask : public VisionTask {
  public:
    VisionRegionTask(const std::set<color_index>& colorIndex, AngPi _dTheta=0,
		     unsigned int _minArea=200)
      : VisionTask(colorIndex,_dTheta), minArea(_minArea) {}
    VisionRegionTask(int colorIndex, AngPi _dTheta,
		     unsigned int _minArea=200)
      : VisionTask(colorIndex,_dTheta), minArea(_minArea) {}
    VisionRegionTask(const VisionRegionTask& vrt)
      : VisionTask(vrt), minArea(vrt.minArea) {}
    virtual TaskType_t getTaskType() const { return visRegTask; }
    virtual Task* clone() const { return new VisionRegionTask(*this); }
    unsigned int minArea;
  };

  // ---------------- end of Task classes ----------------

};

//================ LookoutPointRequest ================

//! Take a picture of or measure a distance to a point in space
class LookoutPointRequest : public LookoutRequestBase {
public:
  //! Constructor
  LookoutPointRequest() : 
    LookoutRequestBase(noMotion,imageResult),
#ifdef TGT_HAS_CAMERA
    joint(CameraFrameOffset),
#else
	joint(0),
#endif
    toBaseMatrix(),
    gazePt(), motionSettleTime(1000), 
    numSamples(1), sampleCounter(0), sampleInterval(0),
    image(), sketchFunc(VRmixin::sketchFromSeg)
  {}

  //! Copy constructor
  LookoutPointRequest(const LookoutPointRequest &p)
    : LookoutRequestBase(p), joint(p.joint), toBaseMatrix(p.toBaseMatrix),
      gazePt(p.gazePt), motionSettleTime(p.motionSettleTime),
      numSamples(p.numSamples), sampleCounter(p.sampleCounter), sampleInterval(p.sampleInterval),
      image(p.image), sketchFunc(p.sketchFunc)
  {}

public:
  void setTarget(const Point &target) {
    gazePt = target;
    headMotionType = pointAt;
  }

  unsigned int joint; //!< joint reference frame from which base frame transformation matrix is created, e.g., Camera, IR, etc.
  fmat::Transform toBaseMatrix; //!< transformation matrix from joint frame to base frame
  Point gazePt; //!< point to look at; can be in either egocentric or allocentric reference frame
  unsigned int motionSettleTime;  //!< Time in msec to wait before taking measurements or throwing completion event after head reaches gazePt.
  int numSamples; //!< Number of samples to take; if > 1, return the mode (for pixels) or median (for distance)
  int sampleCounter; //!< Number of samples collected so far
  int sampleInterval; //!< Interval in msec between successive samples
  Sketch<uchar> image; //<! stores image here
  Sketch<uchar> (*sketchFunc)(); //<! function used to generate image

private:
  LookoutPointRequest& operator=(const LookoutPointRequest&); // don't call
};


// ================ LookoutScanRequest ================

class LookoutScanRequest : public LookoutRequestBase {
public:

  std::vector<Task*> tasks;
  float scanSpeed; //!< speed in rad/msec
  ShapeRoot searchArea;
  unsigned int motionSettleTime;

  static const float defSpd;  //!< default scan speed

  //! Constructor
  LookoutScanRequest(float _speed=defSpd): 
    LookoutRequestBase(scan,interestPoints),
    tasks(), scanSpeed(_speed), searchArea(), motionSettleTime(1000) {}

  //! Constructor
  LookoutScanRequest(const Task& _task, float _speed=defSpd) :
    LookoutRequestBase(scan,interestPoints), tasks(), scanSpeed(_speed), searchArea(), motionSettleTime(1000)
  { addTask(_task); }

  //! Copy constructor
  LookoutScanRequest(const LookoutScanRequest& req)
    : LookoutRequestBase(req), tasks(), scanSpeed(req.scanSpeed), searchArea(req.searchArea), motionSettleTime(req.motionSettleTime)
  {
    for (std::vector<Task*>::const_iterator it = req.tasks.begin();
	 it != req.tasks.end(); it++) 
      addTask(**it);
  }

  //! Destructor
  virtual ~LookoutScanRequest();

  void addTask(const Task& t) { tasks.push_back(t.clone()); }

};  // end of LookoutScanRequest



// ================ LookoutTrackRequest ================

// ! Track an object with the head
class LookoutTrackRequest : public LookoutRequestBase {
public:

  friend class Lookout;

  //! Constructor
  LookoutTrackRequest(const ShapeRoot& target=ShapeRoot())
    : LookoutRequestBase(track), targetShape(target),
      minBlobArea(80), exitTest(NULL), cindex(0) {}

  //! Copy constructor
  LookoutTrackRequest(const LookoutTrackRequest& req)
    : LookoutRequestBase(req), targetShape(req.targetShape),
      minBlobArea(req.minBlobArea), exitTest(req.exitTest), cindex(req.cindex) {}

  ShapeRoot targetShape;
  int minBlobArea; //!< Minimum acceptable size of a region
  bool (*exitTest)();  //!< If true, target has been found

  LookoutTrackRequest& operator=(const LookoutTrackRequest&); //!< don't call this

private:
  color_index cindex;
};


// ================ LookoutSearchRequest ================

//! Search for an object and return when found
class LookoutSearchRequest : public LookoutRequestBase {
public:

  friend class Lookout;

  //! Constructor
  LookoutSearchRequest(const ShapeRoot& target=ShapeRoot())
    : LookoutRequestBase(search,imageResult), targetShape(target), 
      minBlobArea(80), exitTest(NULL), image(),
#ifdef TGT_HAS_CAMERA
      joint(CameraFrameOffset),
#else
      joint(0),
#endif
      toBaseMatrix(),
      cindex(0) {}

  //! Copy constructor
  LookoutSearchRequest(const LookoutSearchRequest& req)
    : LookoutRequestBase(req), targetShape(req.targetShape), 
      minBlobArea(req.minBlobArea), exitTest(req.exitTest), 
      image(req.image), joint(req.joint), toBaseMatrix(req.toBaseMatrix),
      cindex(req.cindex) {}

  ShapeRoot targetShape;
  int minBlobArea; //!< Minimum acceptable size of a region
  bool (*exitTest)();  //!< If true, target has been found
  Sketch<uchar> image;  //!< Stores image here
  unsigned int joint; //!< joint reference frame from which base frame transformation matrix is created, e.g., Camera, IR, etc.
  fmat::Transform toBaseMatrix; //!< transformation matrix from joint frame to base frame

private:
  color_index cindex;

  LookoutSearchRequest& operator=(const LookoutSearchRequest&); //!< don't call this
};


} // namespace

#endif
