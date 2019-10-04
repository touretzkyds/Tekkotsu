//-*-c++-*-
#ifndef INCLUDED_MapBuilderRequest_h_
#define INCLUDED_MapBuilderRequest_h_

#include "Behaviors/BehaviorBase.h"
#include "Shared/fmatSpatial.h"
#include "Vision/colors.h"   // for color_index

#include <queue>

#include "DualCoding/BlobData.h"
#include "DualCoding/MarkerData.h"
#include "DualCoding/Point.h"
#include "DualCoding/ShapeTypes.h"

using namespace std;

namespace DualCoding {

class LookoutSearchRequest;

class GazePoint {
	public:
		enum GazePointType_t {
			visible,
			centered
		} type;
		Point point;
		GazePoint(GazePointType_t _type, Point _point) : type(_type), point(_point) {}
		GazePoint() : type(), point() {}
};

class MapBuilderRequest {
  friend class MapBuilder;

public:
  enum MapBuilderRequestType_t { 
    cameraMap, 
    groundMap,
    localMap, 
    worldMap
  };

  typedef unsigned int MapBuilderVerbosity_t;
  static coordinate_t defaultMarkerHeight;

  MapBuilderRequestType_t requestType;  //!< Type of map to build
  std::map<ShapeType_t, std::set<color_index> > objectColors;		//!< For each object type, a set of object color indices
  std::map<ShapeType_t, std::set<color_index> > secondColors;		//!< For two-colored objects (the bar color for dominoes)
  std::map<ShapeType_t, std::set<color_index> > occluderColors;		//!< For each object type, a set of occluder color indices
  float extractorMinLineLength; //!< Minimum length for a line in camera space
  float extractorGapTolerance; //!< Maximum gap tolerated by line extractor
	float minLineLength; //!< Minimum length of a line with valid endpoints, in mm, for isAdmissible() test
	float minRayLength; //!< Minimum length of a line with an invalid endpoint, in mm, for isAdmissible() test
  float minLinesPerpDist; //!< Minimum perpendicular distance to prevent two lines from matching
	float minEllipseSemiMajor; //!< Minimum length of ellipse semimajor axis, for isAdmissible() test
  std::map<color_index, int> minBlobAreas;	//!< Minimum acceptable areas for blobs of specified colors, e.g., req.minBlobAreas[pink_index]=50
  std::map<color_index, BlobData::BlobOrientation_t> blobOrientations; //!< Default orientation for blobs of specified colors
  std::map<color_index, coordinate_t> assumedBlobHeights; //!< Fixed heights for pillar or poster blobs of specified colors
  std::map<color_index, coordinate_t> assumedCylinderHeights; //!< Fixed heights for cylinders of specified colors
  std::set<MarkerType_t> markerTypes; //!< Types of markers to search for
  std::map<MarkerType_t, coordinate_t> markerHeights; //!< Assumed height above ground  in mm for markers of this type
  unsigned int floorColor;
  string siftDatabasePath; //!< Path to the SIFT object database to load
  std::set<std::string> siftObjectNames;  //!< Names of objects to search for; if empty, search for everything
  std::pair<int,int> aprilTagFamily; //!< AprilTag family to use, e.g., Tag16h5 would be std::pair<int,int>(16,5) 
	fmat::Column<3> dominoDimensions; //!< length, width, and height of a domino (in mm)
	fmat::Column<3> naughtDimensions; //!< length, width, and height of a naught (in mm)
	fmat::Column<3> crossDimensions; //!< length, width, and height of a cross (in mm)
  unsigned int motionSettleTime;  //!< Time in msec to wait before taking measurements or throwing completion event after head reaches gazePt.
  int numSamples; //!< Number of camera images to combine, for noise reduction
  int sampleInterval; //!< Interval in msec between successive samples
  float maxDist; //!< Ignore objects farther than this distance
  bool clearCamera; //!< If true (default), clear the camera sketch and shape spaces at start of request.
  bool clearLocal; //!< If true (default), clear the local sketch and shape spaces at start of request.
  bool clearWorld; //!< If true (default is false), clear the world sketch and shape spaces at start of request.
	bool ignoreCamera; //!< Don't clear or import shapes from camera space; just work in local and/or world space
  bool pursueShapes; //!< If true, generate new gaze points as shapes are recognized
  bool immediateRequest; //!< If true, the current camera image is processed immediately and results are available when MapBuilder::executeRequest() returns, so there's no need to set up an event listener.  Only possible if numSamples == 1 and no head motion is required.
  bool manualHeadMotion; //!< If true, waits for !msg MoveHead before moving to next gaze point (for debugging)
  bool rawY; //!< If true, leave an intensity (Y-channel) image in camera space for debugging
  bool removePts; //!< If true, remove pending gaze points if they're visible in current image
  bool doScan; //!< If true, do a continuous scan of the area to find interest points to be examined
  AngPi dTheta; //!< Angular step for scanning
  ShapeRoot searchArea; //!< The area to search, in egocentric or allocentric coords
  std::queue<LookoutSearchRequest*> worldTargets; //!< Queue of search requests for world targets
  void (*userImageProcessing)(); //!< User routine to call after a camera image is received
  void (*userCamProcessing)(); //!< User routine to call after cam space processing
  void (*userGroundProcessing)(); //!< User routine to call after ground space processing
  void (*userLocalProcessing)(); //!< User routine to call after local space processing
  void (*userWorldProcessing)(); //!< User routine to call after world space processing
  bool (*exitTest)(); //!< If true, terminate map building and post completion event
  enum GroundPlaneAssumption_t { onStand, onLegs, onWheel, custom } groundPlaneAssumption;
  PlaneEquation customGroundPlane; //!< User-supplied ground plane
  BehaviorBase* requestingBehavior; //!< The Behavior making this request; used for posting mapbuilder events
  MapBuilderVerbosity_t setVerbosity; //!< Verbosity elements to force on
  MapBuilderVerbosity_t clearVerbosity; //!< Verbosity elements to force off
  fmat::Transform baseTransform; //!< Used by the Pilot when rotating the body to simulate a pan/tilt

private:
  std::vector<GazePoint> gazePts;
  std::vector<fmat::Transform> baseToCamMats;
  unsigned int requestID; //!< Set by mapbuilder when the request is added to its request queue
  MapBuilderVerbosity_t verbosity; //!< Merger of global default verbosity settings and set/clear for this request

public:
  //! Constructor
  MapBuilderRequest(MapBuilderRequestType_t reqtype=cameraMap);

  //! Copy constructor
  MapBuilderRequest(const MapBuilderRequest &req);

  //! Assignment operator
  MapBuilderRequest& operator=(const MapBuilderRequest &req);

  virtual ~MapBuilderRequest() {} //!< Destructor

  MapBuilderRequestType_t getRequestType() const { return requestType; }

  //! Set of all non-zero/non-unsaturated colors (useful for requesting markers of all possible colors)
  std::set<color_index> allColors();

  //! Shortcut for specifying an objectColors entry
  void addObjectColor(ShapeType_t type, std::string const &color_name);
  
  //! Shortcut for specifying an objectColors entry
  void addObjectColor(ShapeType_t type, rgb color);

  //! Shortcut for specifying a secondColors entry
  void addSecondColor(ShapeType_t type, std::string const &color_name);
  
  //! Shortcut for specifying a secondColors entry
  void addSecondColor(ShapeType_t type, rgb color);

  //! Shortcut for specifying all object colors
  void addAllObjectColors(ShapeType_t type);

  //! Shortcut for specifying an occluderColors entry
  void addOccluderColor(ShapeType_t type, std::string const &color_name);
  
  //! Shortcut for specifying a minimum blob area
  void addMinBlobArea(std::string const &color_name, int area);

  //! Shortcut for specifying a minimum blob area for all blob colors
  void addAllMinBlobAreas(int area);

  //! Shortcut for specifying a blob's orientation and (for poster orientations) assumed height
  void addBlobOrientation(std::string const &color_name, BlobData::BlobOrientation_t orient, coordinate_t assumedHeight=0);
  
  //! Shortcut for specifying a blob's orientation and (for poster orientations) assumed height
  void addBlobOrientation(rgb color, BlobData::BlobOrientation_t orient, coordinate_t assumedHeight=0);
  
  //! Shortcut for specifying a cylinder's height
  void addCylinderHeight(std::string const &color_name, coordinate_t assumedHeight=0);
  
  //! Shortcut for specifying a cylinder's height
  void addCylinderHeight(rgb color, coordinate_t assumedHeight=0);

  //! Shortcut for specifying a marker type to look for
  void addMarkerType(MarkerType_t type);

  //! Shortcut for specifying a SIFT object type to look for
  void addSiftObject(string const &name);

  //! Look for shapes that could match this one
  void addAttributes(const ShapeRoot &shape);

  //! Shortcut for requesting AprilTag extraction using the smallest available tag family
  void setAprilTagFamily();

  //! Shortcut for selecting a specific AprilTag family
  void setAprilTagFamily(int bits, int minimumHammingDistance);

	//! Shortcut for requesting agent recognition (seeing other robots)
	void addAgentRecognition();

protected:
  bool validateRequest(); //!< check object types, colors, etc.
  static bool validateColors(const map<ShapeType_t,set<color_index> > &shapes2colors);
  bool validateSift();
  bool validateRequestType();
  static bool validateAdd(ShapeType_t type, std::string const &color_name);

};

} // namespace

#endif
