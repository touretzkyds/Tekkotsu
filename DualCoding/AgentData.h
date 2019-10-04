//-*-c++-*-
#ifndef _AGENTDATA_H_
#define _AGENTDATA_H_

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeEllipse.h"
#include "Shared/Measures.h"    // coordinate_t; AngPi data member
#include "ShapeTypes.h"  // agentDataType
#include "Shared/RobotInfo.h" // bounding box stuff
#include "Sketch.h"

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

//! Representation of the robot on the world map
class AgentData : public BaseData {
private:
  Point center_pt;
  AngTwoPi orientation;
	int hostAddr;

public:
  //! Constructor
  AgentData(ShapeSpace& _space, const Point &c, AngTwoPi orient=0.f);
  
  //! Copy constructor
  AgentData(const AgentData& otherData);
  
  static ShapeType_t getStaticType() { return agentDataType; }

  // =========================================
  // BEGIN FUNCTIONS
  // =========================================
  
  DATASTUFF_H(AgentData);
  
  //! Centroid. (Virtual in BaseData.)
  Point getCentroid() const { return center_pt; }
  
	//! Returns the agent's IP address
	int getHostAddr() const { return hostAddr; }

	//! Sets the agent's IP address
	void setHostAddr(int addr) { hostAddr = addr; }

  //! Returns the bounding box of the agent
  BoundingBox2D getBoundingBox() const;

  //! Match agents based on their parameters.  (Virtual in BaseData.)
  virtual bool isMatchFor(const ShapeRoot& other) const;
  
  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  virtual bool updateParams(const ShapeRoot& other, bool force=false);
  
  virtual void projectToGround(const fmat::Transform& camToBase,
			       const PlaneEquation& groundplane);
  
  //! Transformations. (Virtual in BaseData.)
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);  

  virtual unsigned short getDimension() const { return 3; }  

  AngTwoPi getOrientation() const { return orientation; }
  fmat::Column<3> getBoundingBoxOffset() const {return AgentBoundingBoxBaseFrameOffset;}
  fmat::Column<3> getBoundingBoxHalfDims() const {return AgentBoundingBoxHalfDims;}
  
  void setOrientation(AngTwoPi _orientation); //!< Don't call this; use MapBuilder::setAgent()
  void setCentroidPt(const Point &otherPt) { center_pt.setCoords(otherPt); }  //!< Don't call this; use MapBuilder::setAgent()

  //! Render into a sketch space and return reference.
  virtual Sketch<bool>* render() const;

  AgentData& operator=(const AgentData&); //!< don't call
  
	//! Extraction

	enum RobotSide {
		FRONT,
		BACK,
		LEFT,
		RIGHT
	};

	struct ColorIdTarget {
		int id;
		int y_low;
		int y_high;
		int u_0;
        float u_slope;
		int v_0;
        float v_slope;
	};

	static const int NUM_COLOR_IDS = 4;
	static ColorIdTarget color_ids[NUM_COLOR_IDS];
	static const char *agent_names[NUM_COLOR_IDS];

	static int getColorIdMatch(yuv pixelColor);
	static int getIdFromCenter(const Sketch<yuv> &cam_frame, const Point &a, const Point &b, const Point &c);
	static int getIdFromCorner(const Sketch<yuv> &cam_frame, const Point &vert_a, const Point &vert_b, const Point &loner);

	static bool inline compareCircleSize(const Shape<EllipseData> &a, const Shape<EllipseData> &b) {
		float radius_diff = a->getSemimajor() - b->getSemimajor();
		return fabs(radius_diff) < 3;
	}

	static bool inline compareCircleDist(const Point &a, const Point &b) {
		const int DISTANCE_LIMIT = 200;
		return fabs(a.coordX() - b.coordX()) < DISTANCE_LIMIT;
	}

	static bool circlesVerticallyAligned(const Shape<EllipseData> &a, const Shape<EllipseData> &b);
	static bool circlesHorizontallyAligned(const Shape<EllipseData> &a, const Shape<EllipseData> &b);

	static void findCirclesManual(const Sketch<uchar> &camFrame, std::vector<int> &neighbor_density,
																std::vector<Shape<EllipseData> > &agentIdCircles);
	static void findAgentsBelt(const Sketch<yuv> &camFrameYUV, const std::vector<int> &neighbor_density,
														 const std::vector<Shape<EllipseData> > &agentIdCircles);
	static void filterAgentOverlap();
	static void extractAgents(const Sketch<uchar> &camFrame, const Sketch<yuv> &camFrameYUV, const std::set<color_index> &objcolors);


};

} // namespace

#endif
