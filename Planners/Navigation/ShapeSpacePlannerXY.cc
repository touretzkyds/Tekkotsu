#include <ostream>

#include "ShapeSpacePlannerXY.h"

//================ RRTNodeXY ================

float RRTNodeXY::distance(const RRTNodeXY::NodeValue_t &target) {
  return (q.first-target.first)*(q.first-target.first) +
  (q.second-target.second)*(q.second-target.second);
}

void RRTNodeXY::generateSample(const RRTNodeXY::NodeValue_t &lower, 
                               const RRTNodeXY::NodeValue_t &upper,
                               RRTNodeXY::NodeValue_t &sample) {
  sample.first = randRange(lower.first, upper.first);
  sample.second = randRange(lower.second, upper.second);
}

RRTNodeBase::Interp_t 
RRTNodeXY::interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interp, 
                       bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool searchBackwards) {
  int xsteps = int(std::fabs(target.first-start.first)/interp.first);
  int ysteps = int(std::fabs(target.second-start.second)/interp.second);
  int numSteps = std::max(xsteps,ysteps);
  float deltaX = (target.first-start.first)/float(numSteps);
  float deltaY = (target.second-start.second)/float(numSteps);
  
  bool truncated = (unsigned int)numSteps > maxInterpolations && truncate;
  if ( truncated )
    numSteps = maxInterpolations;
  
  // Interpolate along the path and check for collisions
  reached = start;
  for (int t = 0; t < numSteps; t++) {
    reached.first += deltaX;
    reached.second += deltaY;
    if ( cc->collides(reached) )
      return COLLISION;
  }
  
  if ( cc->collides(target) )
    return COLLISION;
  else if ( !truncated )
    return REACHED;
  else
    return APPROACHED;
}

std::string RRTNodeXY::toString() const {
  char buff[100];
  sprintf(buff, "%7.2f %7.2f", q.first, q.second);
  return string(buff);
}

bool RRTNodeXY::CollisionChecker::collides(const NodeValue_t &qnew, ShapeSpacePlannerXY::PlannerResult* result) const {
  CircularObstacle robot(qnew.first, qnew.second, robotRadius);
  // treat as collision if outside world bounds
  if ( worldBounds.isValid() && !worldBounds->isInside(Point(qnew.first, qnew.second)) ) {
    if (result) {
      ostringstream os;
      os << VRmixin::theAgent->getName() << "-" << VRmixin::theAgent->getId();
      result->movingObstacle = new CircularObstacle(robot);
      result->movingObstacle->name = os.str();
      ostringstream os2;
      os2 << worldBounds->getName() << "-" << worldBounds->getId();
      result->collidingObstacle = new HierarchicalObstacle;
      result->collidingObstacle->name = os2.str();
    }
    return true;
  }
  
  for (size_t i = 0; i < obstacles.size(); i++)
    if ( !obstacles[i]->isBodyObstacle() && obstacles[i]->collides(robot)) {
      //std::cout << "Collision: " << robot.toString() << " with " << obstacles[i]->toString() << std::endl;
      if (result) {
	ostringstream os;
	os << VRmixin::theAgent->getName() << "-" << VRmixin::theAgent->getId();
        result->movingObstacle = new CircularObstacle(robot);
	result->movingObstacle->name = os.str();
	result->collidingObstacle = dynamic_cast<PlannerObstacle2D*>(obstacles[i]->clone());
      }
      return true;
    }
  return false;
}

//================ ShapeSpacePlannerXY ================

ShapeSpacePlannerXY::ShapeSpacePlannerXY(ShapeSpace &shs, const Shape<PolygonData> &worldBounds,
                                         float inflation, float _robotRadius)
  : GenericRRT<NodeType_t, 2>(new NodeType_t::CollisionChecker(shs, worldBounds, inflation, _robotRadius)),
    robotRadius(_robotRadius) {
  NodeValue_t interp;
  interp.first = interp.second = 25; // step size in mm
  setInterpolation(interp);
}

ShapeSpacePlannerXY::PlannerResult
ShapeSpacePlannerXY::planPath(const Point& start, const Point &end,
                              unsigned int maxIterations,
                              std::vector<NodeValue_t> *pathResult,
                              std::vector<NodeType_t> *treeStartResult,
                              std::vector<NodeType_t> *treeEndResult) {
  NodeValue_t lower, upper;
  if ( cc->getWorldBounds().isValid() ) {
    BoundingBox2D b = cc->getWorldBounds()->getBoundingBox();
    lower.first = b.min[0]; lower.second = b.min[1];
    upper.first = b.max[0]; upper.second = b.max[1];
  } else {
    // combine with obstacles + start and end points
    BoundingBox2D b(cc->getObstacleBoundingBox());
    b.expand(start.coords);
    b.expand(end.coords);
    lower.first =  b.min[0] - 2*robotRadius;
    lower.second = b.min[1] - 2*robotRadius;
    upper.first =  b.max[0] + 2*robotRadius;
    upper.second = b.max[1] + 2*robotRadius;
  }
  std::cout << "World bounds: [" << lower.first << "," << lower.second
	    << "]  to  [" << upper.first << "," << upper.second << "]" << std::endl;
  setLimits(lower,upper);
  
  NodeValue_t startval(start.coordX(), start.coordY());
  NodeValue_t endval(end.coordX(), end.coordY());
  return GenericRRT<NodeType_t, 2>::planPath(startval, endval, maxIterations, 
                                             pathResult, treeStartResult, treeEndResult);
}

void ShapeSpacePlannerXY::plotPath(const std::vector<NodeValue_t> &path,
				   Shape<GraphicsData> &graphics,
				   rgb color) {
  for ( unsigned int i = 1; i < path.size(); i++ )
    graphics->add(new GraphicsData::LineElement("seg", path[i-1], path[i], color));
}

void ShapeSpacePlannerXY::plotTree(const std::vector<NodeType_t> &tree,
                                   Shape<GraphicsData> &graphics,
                                   rgb color) {
  for ( unsigned int i = 1; i < tree.size(); i++ ) // start from 1 because root has no parent
    graphics->add(new GraphicsData::LineElement("branch", tree[tree[i].parent].q, tree[i].q, color));
}

