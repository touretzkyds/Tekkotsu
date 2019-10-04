//-*-c++-*-
#ifndef _SHAPE_SPACE_PLANNER_XY_H_
#define _SHAPE_SPACE_PLANNER_XY_H_

#include "Planners/RRT/ShapeSpacePlannerBase.h"

class ShapeSpacePlannerXY;

//================ RRTNodeXY ================

class RRTNodeXY : public RRTNodeBase {
public:
  typedef std::pair<float,float> NodeValue_t;
  typedef GenericRRTBase::PlannerResult<2> PlannerResult;
  NodeValue_t q;
  
  //! Constructor
  RRTNodeXY(const NodeValue_t &_q, unsigned int _parent) : RRTNodeBase(_parent), q(_q) {};
  
  //! Destructor
  virtual ~RRTNodeXY() {}
  
  //! Collision checker to be called by RRT search algorithm
  class CollisionChecker : public ShapeSpaceCollisionCheckerBase<2> {
  public:
    CollisionChecker(ShapeSpace &shs,
                     const Shape<PolygonData> &_worldBounds,
                     float _inflation, float _robotRadius) :
    ShapeSpaceCollisionCheckerBase<2>(shs, _worldBounds, _inflation), robotRadius(_robotRadius) {}
    
    float robotRadius;
    
    virtual bool collides(const NodeValue_t &qnew, PlannerResult* result=NULL) const;
    
  };
  
  static const unsigned int maxInterpolations = 10; //!< Maximum number of interpolation steps in interpolate() when @a truncate is true
  
  virtual float distance(const NodeValue_t &target);
  static void generateSample(const NodeValue_t &lower, const NodeValue_t &upper, NodeValue_t &sample);
  static Interp_t interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interp, 
                              bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool searchingBackwards);
  
  virtual std::string toString() const;
  
};

//================ ShapeSpacePlannerXY ================

//! Plans a path in a 2D linear space, assuming the robot has circular shape
class ShapeSpacePlannerXY : public GenericRRT<RRTNodeXY, 2> {
public:
  typedef RRTNodeXY NodeType_t;
  typedef NodeType_t::NodeValue_t NodeValue_t;
  typedef GenericRRTBase::PlannerResult<2> PlannerResult;
  
  ShapeSpacePlannerXY(DualCoding::ShapeSpace &shs,
                      const DualCoding::Shape<DualCoding::PolygonData> &worldBounds=Shape<PolygonData>(),
                      float inflation=0, float _robotRadius=6*25.4f);
  
  virtual ~ShapeSpacePlannerXY() {}
  
  using GenericRRT<NodeType_t, 2>::planPath;
  PlannerResult
  planPath(const Point &start,
           const Point &end,
           unsigned int _maxIterations=4000,
           std::vector<NodeValue_t> *pathResult=NULL,
           std::vector<NodeType_t> *treeStartResult=NULL,
           std::vector<NodeType_t> *treeEndResult=NULL);
  
  float robotRadius; //!< radius in mm of the CircularObstacle describing the robot
  
  static void plotPath(const std::vector<NodeValue_t> &path,
		       Shape<GraphicsData> &graphics,
		       rgb color = rgb(0,0,255));

  static void plotTree(const std::vector<NodeType_t> &tree,
		       Shape<GraphicsData> &graphics,
		       rgb color = rgb(0,0,255));

};

#endif
