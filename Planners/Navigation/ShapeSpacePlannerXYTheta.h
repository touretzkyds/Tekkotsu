//-*-c++-*-
#ifndef _SHAPE_SPACE_PLANNER_XYTHETA_H_
#define _SHAPE_SPACE_PLANNER_XYTHETA_H_

#include "Motion/Kinematics.h"

#include "Planners/RRT/ShapeSpacePlannerBase.h"

class ShapeSpacePlannerXYTheta;

//================ RRTNodeXYTheta ================

class RRTNodeXYTheta : public RRTNodeBase {
public:
  
  struct NodeValue_t {
    float x;						//!< The x-coordinate of this position
    float y;						//!< The y-coordinate of this position
    AngTwoPi theta;			//!< Heading that will take us from parent nodes's position to this position (x,y) 
		AngSignTwoPi turn;	//!< Left or right turn that takes us to this node's desired heading theta
    NodeValue_t() : x(0), y(0), theta(0), turn(0) {}
    NodeValue_t(float _x, float _y, AngTwoPi _theta, AngSignTwoPi _turn=0) :
			x(_x), y(_y), theta(_theta), turn(_turn) {}
  };

  NodeValue_t q;
  
  //! Constructor
  RRTNodeXYTheta(const NodeValue_t &_q, unsigned int _parent) : RRTNodeBase(_parent), q(_q) {};
  
  class CollisionChecker : public ShapeSpaceCollisionCheckerBase<2> {
  public:

    //! Constructor
    CollisionChecker(DualCoding::ShapeSpace & shs,
                     const DualCoding::Shape<DualCoding::PolygonData> &_worldBounds,
                     float _inflation) :
      ShapeSpaceCollisionCheckerBase<2>(shs, _worldBounds, _inflation), body() {
      for (unsigned int i = 0; i < obstacles.size(); i++) {
        if (obstacles[i]->isBodyObstacle())
          body.add(dynamic_cast<PlannerObstacle<2>*>(obstacles[i]->clone()));
      }
    }
      
    HierarchicalObstacle body;
    
    virtual bool collides(const NodeValue_t &qnew, GenericRRTBase::PlannerResult2D* result=NULL);

    std::vector<PlannerObstacle2D*> colliders(const NodeValue_t &q);
  };
  
  static const unsigned int maxInterpolations = 10; //!< Maximum number of interpolation steps in interpolate() when @a truncate is true

  static const AngSignPi turnLimit; //!< Maximum theta difference between adjacent nodes without interpolated collision checking
  
  virtual float distance(const NodeValue_t &target);

  static void generateSample(const NodeValue_t &lower, const NodeValue_t &upper, NodeValue_t &sample);

  static Interp_t interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interpStep,
                              bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool searchingBackwards);

	static bool safeTurn(const NodeValue_t &start, const AngSignPi headingChange, AngSignTwoPi &turn, CollisionChecker *cc);

	//! Interpolation function for potential short-circuiting of segments of a path
  static Interp_t snipInterpolate(const NodeValue_t &start, const NodeValue_t &target, const AngTwoPi nextTheta,
																	const NodeValue_t &interpStep, CollisionChecker *cc,
																	NodeValue_t &reached, AngSignTwoPi &nextTurn);
  
  virtual std::string toString() const;
};

ostream& operator<<(ostream &os, const RRTNodeXYTheta::NodeValue_t q);


//================ ShapeSpacePlannerXYTheta ================

//! Plans a path in a 2D linear space with smooth angle changes (if turnLimit is small), using multiple bounding boxes for collision checking
class ShapeSpacePlannerXYTheta : public GenericRRT<RRTNodeXYTheta, 2> {
public:
  typedef RRTNodeXYTheta NodeType_t;
  typedef NodeType_t::NodeValue_t NodeValue_t;
  
  ShapeSpacePlannerXYTheta(DualCoding::ShapeSpace &shs,
                           const DualCoding::Shape<DualCoding::PolygonData> &worldBounds = Shape<PolygonData>(),
                           float inflation = 0);
  
  virtual ~ShapeSpacePlannerXYTheta() {}
  
  virtual void initialize(const NodeValue_t &start, std::vector<NodeType_t> &treeStartResult,
                          const NodeValue_t &end, std::vector<NodeType_t> &treeEndResult);

	//! Calculate the heading necessary to point the baseOffset at the target.
	/*! This is the slope of the tangent line from the target to a
		circle centered on the baseFrame with radius equal to the y
		component of the baseOffset.
	*/
	AngTwoPi tangentHeading(const NodeValue_t &start, const NodeValue_t &end) const;

  virtual void buildPath(const std::vector<NodeType_t> *treeStart,
												 const std::vector<NodeType_t> *treeEnd, 
												 std::vector<NodeValue_t> &path);
  
	static unsigned int const numDivisions = 18; //!< Number of divisions of the circle to try when targetHeading unspecified

  using GenericRRT<NodeType_t, 2>::planPath;

  //! Plan a robot path from @a startPoint to @a endPoint with optional @a targetHeading at the end
  /*!
    @param startPoint Starting location of the robot
    @param baseOffset Offset from the base frame, e.g., if we want a path that brings the @e gripper (rather than the base) to a particular location
    @param endPoint Desired ending point
    @param initialHeading The robot's heading at the start of the path
    @param targetHeading Desired final heading; if unspecified, the planner generates a collection of possible target headings to try for
    @param maxIterations Maximum number of iterations for the RRT to search before giving up
    @param pathResult Stores the final, smoothed path computed by the planner
    @param treeStartResult Stores the start search tree; only used for debugging or teaching
    @param treeEndResult Stores the end search tree; only used for debugging or teaching
  */
  GenericRRTBase::PlannerResult2D
  planPath(const Point &startPoint,
	   const fmat::Column<3> &_baseOffset,
					 float gateLength,
           const Point &endPoint,
           const AngTwoPi initialHeading,
           const AngTwoPi _targetHeading,
           unsigned int _maxIterations=4000,
           std::vector<NodeValue_t> *pathResult=NULL,
           std::vector<NodeType_t> *treeStartResult=NULL,
           std::vector<NodeType_t> *treeEndResult=NULL);
  
  static void plotPath(const std::vector<NodeValue_t> &path,
											 Shape<GraphicsData> &graphics,
											 rgb color = rgb(0,0,255));

  static void plotTree(const std::vector<NodeType_t> &tree,
											 Shape<GraphicsData> &graphics,
											 rgb color = rgb(0,0,255));

private:
  float targetHeading;  //!< Set by planPath() and used by initialize()
  fmat::Column<3> baseOffset;  //!< Set by planPath() and used by initialize()
	float gateLength; //!< Set by planPath() and used by initialize()
  
};

#endif
