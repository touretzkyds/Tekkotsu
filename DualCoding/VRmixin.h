//-*-c++-*-
#ifndef _VRmixin_h_
#define _VRmixin_h_

#include <string>
#include <iostream>

#include "Behaviors/BehaviorBase.h"
#include "Shared/fmatSpatial.h"
#include "Vision/RawCameraGenerator.h"
#include "Motion/WalkMC.h" // so we can test for TGT_HAS_WALK (Pilot support)

#include "ShapeAgent.h"
#include "BlobData.h"
#include "SketchRoot.h"

#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
class Grasper;
#endif

namespace DualCoding {

class Lookout;
#ifdef TGT_HAS_WALK
class Pilot;
#endif
class SketchDataRoot;
class SketchSpace;
class ShapeRoot;
class MapBuilder;
class ShapeBasedParticleFilter;
class VisualOdometry;//ImageProfileOdometry;
	
typedef unsigned char cmap_t;

//! Mix-in for the BehaviorBase or StateNode class to give access to VisualRoutinesBehavior variables.
class VRmixin {
protected:
  static unsigned int instanceCount; //!< count of VRmixin instances -- when this hits zero, free sketch spaces
  static unsigned int crewCount; //!< count of "crew" (pilot, lookout, map builders) users -- stop these when no one is using them
	
public:
  //! returns reference to the global space instances, call there from global constructors instead of accessing #camSkS, which might not be initialized yet
  static SketchSpace& getCamSkS();
  static SketchSpace& getLocalSkS();
  static SketchSpace& getWorldSkS();
  static ShapeSpace& getGroundShS();

  static SketchSpace& camSkS;      //!< The camera sketch space
  static ShapeSpace& camShS;       //!< The camera shape space
  
  static ShapeSpace& groundShS;    //!< The ground shape space of MapBuilder (MapBuilder::groundShS)
  
  static SketchSpace& localSkS;    //!< The localmap sketch space (LocalMapBuilder::localSkS)
  static ShapeSpace& localShS;     //!< The localmap shape space (LocalMapBuilder::localShS)
  
  static SketchSpace& worldSkS;    //!< The worldmap sketch space (WorldMapBuilder::localSkS)
  static ShapeSpace& worldShS;     //!< The worldmap sketch space (WorldMapBuilder::localShS)
  static Shape<AgentData> theAgent; //!< The robot (usually lives in worldShS)
  
  static MapBuilder* mapBuilder;   //!< the global world mapbuilder instance
  static Lookout* lookout;         //!< the global Lookout instance
#ifdef TGT_HAS_WALK
  static Pilot* pilot;              //!< the global Pilot instance
#endif
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
  static Grasper* grasper;
#endif
	
  static ShapeBasedParticleFilter *particleFilter;   //!< the global particle filter instance
  static bool isWalkingFlag;  //flag indicating if the robot is walking.

private:
  static Socket *camDialogSock;    //!< socket to talk with cam-space sketch viewer
  static Socket *camSketchSock;       //!< socket for transmitting sketch images to cam-space sketch viewer
  static Socket *localDialogSock;  //!< socket to talk with local-space sketch viewer
  static Socket *localSketchSock;     //!< socket for transmitting sketch images to local-space sketch viewer
  static Socket *worldDialogSock;  //!< socket to talk with world-space sketch viewer
  static Socket *worldSketchSock;     //!< socket for transmitting sketch images to world-space sketch viewer
  
public:
  //! Constructor
  VRmixin();

  //! Destructor
  virtual ~VRmixin(void);
  
  static void startCrew(); //!< starts map builders, pilot, and lookout
  static void stopCrew(); //!< stops map builders, pilot, and lookout
	static void requireCrew(const std::string &memberName); //!< checks to make sure Crew is instantiated

  static bool isWalking();

  // serialize the specified Sketch; should use SKETCH encoding later 
  static bool encodeSketch(const SketchDataRoot& image);
  
  //! Import the current color-segmented camera image as a Sketch<uchar>.  Must be called from doEvent().
  static Sketch<uchar> sketchFromSeg();
  
  //! Import channel n image as a Sketch<uchar>.  Must be called from doEvent().
  static Sketch<uchar> sketchFromChannel(const RawCameraGenerator::channel_id_t chan);
  
  //! Import the current y-channel camera image as a Sketch<uchar>a.  Must be called from doEvent().
  static Sketch<uchar> sketchFromRawY();
  
  //! Import the current camera image as a Sketch<yuv>.  Must be called from doEvent().
  static Sketch<yuv> sketchFromYUV();
  
  //! Import the current depth image as a Sketch<usint>.  Must be called from doEvent().
  static Sketch<usint> sketchFromDepth();
  
  //! Import blobs from the current region list as a vector of Shape<BlobData>
  static std::vector<Shape<BlobData> >
  getBlobsFromRegionGenerator(const color_index color, int minarea=25,
			      const BlobData::BlobOrientation_t orient=BlobData::groundplane,
			      const coordinate_t height=0,
			      const int maxblobs=50);
  
  //! processes a single line of input for a Sketch request
  static void processSketchRequest(const std::string &line, 
				   SketchSpace &sketches, 
				   ShapeSpace &shapes);
  
#ifdef TGT_HAS_CAMERA
  //! Project shapes from cam space to ground space, uses CameraFrameOffset and Kinematics::calculateGroundPlane()
  static void projectToGround();
#endif
  // without camera, require they provide the camToBase parameter...
  //! Project shapes from cam space to ground space, will assume Kinematics::calculateGroundPlane() if plane is not specified
  static void projectToGround(const fmat::Transform& camToBase);
  static void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane);
  
  static bool autoRefreshWorldAllowed;
  static bool autoRefreshLocalAllowed;
  static bool autoRefreshCameraAllowed;
  static void refreshSketchWorld();
  static void refreshSketchLocal();
  static void refreshSketchCamera();
  static void autoRefreshSketchWorld();
  static void autoRefreshSketchLocal();
  static void autoRefreshSketchCamera();
  
  //! Used by ShapeSpacePlannerBase::addRobotObstacles to display robot body obstacles at correct point in worldShS
  static Point robotObstaclesPt;

  //! Used by ShapeSpacePlannerBase::addRobotObstacles to display robot body obstacles at correct orientation in worldShS
  static AngTwoPi robotObstaclesOri;

  static std::vector<ShapeRoot> drawShapes; //!< Vector of shapes to be drawn into the RawCam image

  static VisualOdometry *imageOdometry;

private:
  //! used so static member functions can access non-static members; works because there will always be a unique instance of VRmixin
  static VRmixin* theOne;
  
  // dummy functions to satisfy the compiler
  VRmixin (const VRmixin&);	 //!< never call this
  VRmixin& operator=(const VRmixin&); //!< never call this
  
  //! Called whenever data is received on camDialogSocket
  static int camDialogSockCallback(char *buf, int bytes);
  
  //! Called whenever data is received on localDialogSocket
  static int localDialogSockCallback(char *buf, int bytes);
  
  //! Called whenever data is received on worldDialogSocket
  static int worldDialogSockCallback(char *buf, int bytes);
  
  static void dialogCallback(char* buf, int bytes, std::string& incomplete,
			     SketchSpace &SkS, ShapeSpace &ShS);
};
  
} // namespace

#endif
