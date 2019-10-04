//-*-c++-*-
#ifndef _MapBuilder_h_
#define _MapBuilder_h_

#include <queue>

#include "Crew/MapBuilderRequest.h"
#include "Shared/fmatSpatial.h"

#include "DualCoding/Point.h"

#include "DualCoding/BlobData.h"
#include "DualCoding/EllipseData.h"
#include "DualCoding/LineData.h"
#include "DualCoding/SphereData.h"
#include "DualCoding/CylinderData.h"
#include "DualCoding/TargetData.h"
#include "DualCoding/MarkerData.h"

#include "DualCoding/VRmixin.h"
#include "DualCoding/SketchTypes.h"
#include "DualCoding/ShapeSpace.h"
#include "DualCoding/PolygonData.h"

// Note: these are NOT in the DualCoding namespace
class LookoutSketchEvent;
class SiftTekkotsu;

namespace DualCoding {

class SketchSpace;

class MapBuilder : public BehaviorBase {
protected:
  SketchSpace &camSkS;
  ShapeSpace &camShS, &groundShS;
  SketchSpace &localSkS;
  ShapeSpace &localShS;
  SketchSpace &worldSkS;
  ShapeSpace &worldShS;

  const int xres, yres; //!< width and height of camera frame

  PlaneEquation ground_plane; //!< ground plane to which shapes are projected

  static bool retain; //!< if true, VRmixin::stopCrew will not clear MapBuilder structures

public:
  //! Control whether static structures (mapbuilder, sketchGUI sockets, etc.) are retained
  static void setRetain(bool r) { retain = r; }
  static bool isRetained() { return retain; }

  typedef unsigned int MapBuilderVerbosity_t;
  static const MapBuilderVerbosity_t MBVstart = 1<<0;
  static const MapBuilderVerbosity_t MBVevents = 1<<1;
  static const MapBuilderVerbosity_t MBVexecute = 1<<2;
  static const MapBuilderVerbosity_t MBVcomplete = 1<<3;
  static const MapBuilderVerbosity_t MBVdefineGazePoints = 1<<4;
  static const MapBuilderVerbosity_t MBVgazePointQueue = 1<<5;
  static const MapBuilderVerbosity_t MBVnextGazePoint = 1<<6;
  static const MapBuilderVerbosity_t MBVshapeSearch = 1<<7;
  static const MapBuilderVerbosity_t MBVshapesFound = 1<<8;
  static const MapBuilderVerbosity_t MBVgroundPlane = 1<<9;
  static const MapBuilderVerbosity_t MBVprojectionFailed = 1<<10;
  static const MapBuilderVerbosity_t MBVimportShapes = 1<<11;
  static const MapBuilderVerbosity_t MBVnotAdmissible = 1<<12;
  static const MapBuilderVerbosity_t MBVshapeMatch = 1<<13;
  static const MapBuilderVerbosity_t MBVshapesMerge = 1<<14;
  static const MapBuilderVerbosity_t MBVshouldSee = 1<<15;
  static const MapBuilderVerbosity_t MBVdeleteShape = 1<<16;
  static const MapBuilderVerbosity_t MBVsetAgent = 1<<17;
  static const MapBuilderVerbosity_t MBVbadGazePoint = 1<<18;
  static const MapBuilderVerbosity_t MBVskipShape = 1<<19;

private:
  static MapBuilderVerbosity_t verbosity;
public:
  static void setVerbosity(MapBuilderVerbosity_t v) { verbosity = v; }

protected:
  friend class Lookout;
  friend class BiColorMarkerData;   // needs to call getCamBlobs()
  friend class Pilot;
	friend class LineData;
	friend class EllipseData;

  Shape<AgentData> &theAgent; //!< Agent in the world frame
   //!@name Transformation matrices between local and world frames
  //@{
 public:
  fmat::Transform localToWorldMatrix, worldToLocalMatrix;
  //@}

protected:
  std::vector<Point> badGazePoints; //!<  gaze points for which HeadPointerMC.lookAtPoint() returned false

  std::queue<MapBuilderRequest*> requests;
  MapBuilderRequest *curReq;
  unsigned int idCounter;
  float maxDistSq; //!< square of current request's max distance parameter
  float minLinesPerpDistSave; //!< save minLinesPerpDist for LineData::isMatchFor()
  std::map<string,SiftTekkotsu*> siftMatchers;

  unsigned int pointAtID, scanID; //!< ID's for lookout requests
  Point nextGazePoint;

  //! Triggers action to execute the request at the front of the queue
  void executeRequest();
  //! calls exitTest of current request if there is one and returns the result
  bool requestExitTest();
  //! posts completion event and deletes current request, executes next request if there is one
  void requestComplete(); 

#ifdef TGT_IS_AIBO
public:
#endif
  //! Sets agent location and heading, and recomputes local-to-world transformation matrices.  Called by the Pilot.
  void setAgent(const Point &worldLocation, const AngTwoPi worldHeading, bool quiet=false);

public:
  MapBuilder(); //!< Constructor
  virtual ~MapBuilder() {}   //!< Destructor
  virtual void preStart();
  virtual void stop(); 
  virtual std::string getDescription() const { return "MapBuilder"; }
  void printShS(ShapeSpace&) const;
  unsigned int executeRequest(const MapBuilderRequest&, unsigned int *req_id=NULL); //!< Execute a MapBuilder request, and optionally store the request id in a variable; the id will be returned in any case
  void executeRequest(BehaviorBase* requestingBehavior, const MapBuilderRequest &req); //!< Execture a MapBuilder request and store the address of the requesting behavior to use as the source ID in a mapbuilder status event

  MapBuilderRequest* getCurrentRequest() { return curReq; }

  virtual void doEvent();
  void processImage(const LookoutSketchEvent&);

  // Returns true if a ground shape should be seen in the current camera frame
  static bool isPointVisible(const Point &pt, const fmat::Transform& baseToCam, float maxDistanceSq) ;
  static bool isLineVisible(const LineData& ln, const fmat::Transform& baseToCam);
  static bool isShapeVisible(const ShapeRoot &ground_shape, const fmat::Transform& baseToCam, float maxDistanceSq);
  
  
  //! utility functions which may be used by MapBuilderRequest's exit condition and others
  //@{
  const Shape<AgentData>& getAgent() const { return theAgent; }
  
  //! updates the agent location and heading after a relative move
  void moveAgent(coordinate_t const local_dx, coordinate_t const local_dy, coordinate_t const local_dz, AngSignPi dtheta);
  //@}
  
  std::vector<ShapeRoot> getShapes(const ShapeSpace& ShS, int minConf=2) const {
    const std::vector<ShapeRoot> allShapes = ShS.allShapes();
    if (&ShS == &camShS || &ShS == &groundShS || minConf <= 0) 
      return allShapes;
    std::vector<ShapeRoot> nonNoiseShapes;
    for (std::vector<ShapeRoot>::const_iterator it = allShapes.begin();
	 it != allShapes.end(); it++)
      if ((*it)->getConfidence() >= minConf)
	nonNoiseShapes.push_back(*it);
    return nonNoiseShapes;
  }

  void importLocalToWorld();

	ShapeRoot importLocalShapeToWorld(const ShapeRoot &localShape);
  ShapeRoot importWorldToLocal(const ShapeRoot &worldShape);
  template<class T> Shape<T> importWorldToLocal(const Shape<T> &worldShape);

protected:
  //!@name Shape extraction functions
  //@{
  void getCameraShapes(const Sketch<uchar>& camFrame);

  std::vector<Shape<LineData> > 
  getCamLines(const Sketch<uchar>&, const std::set<color_index>& objectColors, 
	      const std::set<color_index>& occluderColors) const;

  std::vector<Shape<EllipseData> > 
  getCamEllipses(const Sketch<uchar>&, const std::set<color_index>& objectColors, 
		 const std::set<color_index>& occluderColors) const;

  void getCamPolygons(const Sketch<uchar>&, const std::set<color_index>& objectColors, 
		      const std::set<color_index>& occluderColors) const;

  std::vector<Shape<LineData> >  
  getCamWalls(const Sketch<uchar>&, unsigned int) const;

  void getCamSpheres(const Sketch<uchar>&, const std::set<color_index>& objectColors, 
		     const std::set<color_index>& occluderColors) const;

  void getCamCylinders(const Sketch<uchar>& camFrame,
		       const std::set<color_index>& colors,
		       const std::map<color_index,coordinate_t>& assumedHeights,
		       const std::map<color_index,int>& minCylinderAreas);

  void getCamBlobs(const Sketch<uchar>& camFrame,
		   const std::set<color_index>& colors,
		   const std::map<color_index,int>& minBlobAreas,
		   const std::map<color_index, BlobData::BlobOrientation_t>& blobOrientations,
		   const std::map<color_index,coordinate_t>& assumedBlobHeights);
  void getCamBlobs(const set<color_index>& colors, int defMinBlobArea=0);
  
  void getCamTargets(const Sketch<uchar> &camFrame, const std::set<color_index>& objectColors,
		     const std::set<color_index>& occluderColors) const;
  
  std::vector<Shape<MarkerData> > 
  getCamMarkers(const Sketch<uchar> &camFrame, const std::set<color_index>& objectColors,
		const std::set<color_index>& occluderColors, const std::set<MarkerType_t>& markerTypes) const;

  void getCamSiftObjects(const Sketch<uchar> &rawY, const std::string &siftDatabasePath, 
			 const std::set<std::string> &siftObjectNames);

  void getCamAprilTags(const Sketch<uchar> &rawY);

	void getCamDominoes(const Sketch<uchar> &camFrame, const std::set<color_index>& objectColors,
										 const std::set<color_index>& secondColors);

	void getCamNaughts(const Sketch<uchar> &camFrame, const std::set<color_index>& objectColors, 
                     const fmat::Column<3>& dimensions) const;

	void getCamCrosses(const Sketch<uchar> &camFrame, const std::set<color_index>& objectColors,
                     const fmat::Column<3>& dimensions) const;

	void getCamAgents(const Sketch<uchar> &camFrame, const Sketch<yuv> &camFrameYUV,
										const std::set<color_index>& objectColors) const;

  //@}

public:
  void newSiftMatcher(const std::string &siftDatabasePath);

  void saveSiftDatabase(const std::string &siftDatabasePath);

  void trainSiftObject(const std::string &siftDatabasePath,
		       const std::string &objectName, const std::string &modelName="model1");

  void trainSiftObject(const std::string &siftDatabasePath, const Sketch<uchar> &sketch,
		       const std::string &objectName, const std::string &modelName="model1");

public: // ** debug **
  // matching shapes between two spaces.
  void matchSrcToDst(ShapeSpace &src, ShapeSpace &dst, std::set<color_index> polygonEdgeColors=std::set<color_index>(),
			    bool mergeSrc=true, bool mergeDst=true);

  //!@name Functions to make requests to the Lookout
  //@{
  void storeImage(bool useNextGazePoint);
  void grabCameraImageAndGo();
  void scanForGazePts();
  //@}

  //! set up initial gazePts either virtually or by scan
  void setInitialGazePts();
  
  void extendLocal(const fmat::Transform& baseToCam);
  void extendWorld(const fmat::Transform& baseToCam);

  //! decrement confidence of shapes which should have been seen according to the baseToCam matrix
  void removeNoise(ShapeSpace&, const fmat::Transform& baseToCam);
  //! erase gaze points which should have been seen according to the baseToCam matrix
  void removeGazePts(std::vector<GazePoint>&, const fmat::Transform& baseToCam);
  
  //! Returns true if it has set up a valid next gaze point in nextGazePoint
  bool determineNextGazePoint();
  //! Returns true if there is a shape which needs be looked at again and is reachable; sets it up as nextGazePoint
  bool determineNextGazePoint(const std::vector<ShapeRoot>&);
  // Returns true if an element of gazePts can be looked at; sets it up as nextGazePoint
  bool determineNextGazePoint(std::vector<GazePoint> &gazePts);
  //! Starts robot moving to the next gaze point
  void moveToNextGazePoint(const bool manualOverride=false);
  void doNextSearch();
  void doNextSearch2();

  // operations in ground shape space 
  bool isBadGazePoint(const Point&) const ;
  void projectToGround(const fmat::Transform& camToBase);
  ShapeRoot projectToLocal(ShapeRoot &shape);
  void filterGroundShapes(const fmat::Transform& baseToCam);

  // calculates ground plane based on ground plane assumption type
  void calculateGroundPlane();

private:
  MapBuilder(const MapBuilder&); //!< never call this
  MapBuilder& operator=(const MapBuilder&);  //!< never call this
};

template<class T> Shape<T> MapBuilder::importWorldToLocal(const Shape<T> &worldShape) {
  ShapeRoot temp(localShS.importShape(worldShape));
  Shape<T> localShape(ShapeRootType(temp,T));
  localShape->applyTransform(worldToLocalMatrix);
  return localShape;
}

//! Utility function for deleting queues of pointers to Lookout or MapBuilder requests
template<typename T> void deleteAll(std::queue<T*> &q) {
  while ( ! q.empty() ) {
    delete q.front();
    q.pop();
  }
}

} // namespace

#endif

