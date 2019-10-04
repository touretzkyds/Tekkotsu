#include "ShapeSpacePlannerXYTheta.h"
#include "Shared/mathutils.h" // for isnan fix

//================ RRTNodeXYTheta ================

const AngSignPi RRTNodeXYTheta::turnLimit = M_PI/12;  // was M_PI/18;

ostream& operator<<(ostream &os, const RRTNodeXYTheta::NodeValue_t q) {
  os << float(q.turn)*180/M_PI << " deg";
  os << ( (q.turn > 0) ? "<" : (q.turn < 0) ? ">" : ":" );
  os << " [" << q.x << ", " << q.y << "] @ " << float(q.theta)*180/M_PI << " deg.";
  return os;
}

float RRTNodeXYTheta::distance(const NodeValue_t &target) {
  return (q.x-target.x)*(q.x-target.x) +
         (q.y-target.y)*(q.y-target.y);
}

void RRTNodeXYTheta::generateSample(const NodeValue_t &lower, 
                                    const NodeValue_t &upper,
                                    NodeValue_t &sample) {
  sample.x = randRange(lower.x, upper.x);
  sample.y = randRange(lower.y, upper.y);
	sample.theta = std::numeric_limits<float>::quiet_NaN();
}

RRTNodeBase::Interp_t 
RRTNodeXYTheta::interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interpStep,
														bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool searchingBackwards) {
  AngTwoPi newHeading = atan2(target.y - start.y, target.x - start.x) +
		(searchingBackwards ? M_PI : 0);
  AngSignPi headingChange = float(newHeading - start.theta) *
		(searchingBackwards ? -1 : 1);
	AngSignTwoPi turn;
	if ( ! safeTurn(start, headingChange, turn, cc) )
		return COLLISION;

	// If we get here, either the turn from the start heading to the
	// target heading was less than turnLimit, or we found a turn direction that
	// avoided collisions during heading interpolation.
  reached = start;
	reached.theta = newHeading;
	// If searching forward, reached.turn is the turn we will make before going to (x,y).
	// If searching backward, reached.turn is the turn we will make upon reaching node start
	reached.turn = turn;

  float xsteps = std::fabs(target.x - start.x) / interpStep.x;
  float ysteps = std::fabs(target.y - start.y) / interpStep.y;
  float numSteps = std::max(xsteps,ysteps);
	unsigned int safeSteps = max((unsigned int)numSteps, 1u);
	if ( safeSteps < numSteps && (!truncate || safeSteps < maxInterpolations ))
		++safeSteps;
  float deltaX = (target.x - start.x) / safeSteps;
  float deltaY = (target.y - start.y) / safeSteps;
  bool truncated = safeSteps > maxInterpolations && truncate;
  if ( truncated )
    safeSteps = maxInterpolations;
  
  for (unsigned int t = 0; t < safeSteps; t++) {
    reached.x += deltaX;
    reached.y += deltaY;
    if ( cc->collides(reached) )
      return COLLISION;
  }

	if ( truncated )
		return APPROACHED;
	NodeValue_t target2 = target;
	target2.theta = reached.theta;
	if ( cc->collides(target2) )  // last step might not have taken us all the way to target, so check target here		
    	  return COLLISION;
	// If target isn't random but instead comes from the other tree,
	// see if we can safely turn to the target heading.
	if ( ! std::isnan((float)target.theta) ) {
		AngSignTwoPi dummyTurn;
		if ( safeTurn(target2, AngSignPi(target.theta-newHeading), dummyTurn, cc) )
			return REACHED;
	}
	return APPROACHED;
}

bool RRTNodeXYTheta::safeTurn(const NodeValue_t &start, const AngSignPi headingChange, AngSignTwoPi &turn, CollisionChecker* cc) {
	turn = headingChange;
  if ( abs(headingChange) <= turnLimit )
		return true;

  // The turn is substantial, so need to check for collisions while turning 
	AngSignPi deltaHeading = (headingChange > 0 ) ? turnLimit : AngSignPi(-turnLimit);
	int nSteps = abs(float(headingChange) / turnLimit);
	int i = 0;
	NodeValue_t node = start;
	for (; i < nSteps; i++) {
		node.theta += float(deltaHeading);
		if ( cc->collides(node) )
			break; // will try turning the other way
	}
	if ( i == nSteps )
		return true;

	// collision occurrred, so try turning the other way
	turn -= (turn > 0) ? 2*M_PI : -2*M_PI;
	deltaHeading = -deltaHeading;
	nSteps = abs(float(turn) / turnLimit);  // need float so division doesn't return a wrapped AngSignTwoPi
	i = 0;
	node.theta = start.theta;
	for (; i < nSteps; i++) {
		node.theta += float(deltaHeading);
		if ( cc->collides(node) )
			return false;
	}
	return true;
}

RRTNodeBase::Interp_t 
RRTNodeXYTheta::snipInterpolate(const NodeValue_t &start, const NodeValue_t &target, const AngTwoPi nextTheta,
																const NodeValue_t &interpStep, CollisionChecker *cc,
																NodeValue_t &reached, AngSignTwoPi &nextTurn) {
  AngTwoPi newHeading = atan2(target.y - start.y, target.x - start.x);
  AngSignPi headingChange = float(newHeading - start.theta);
  reached = start;
	AngSignTwoPi turn;
	if ( ! safeTurn(reached, headingChange, turn, cc) )
		return COLLISION;
	reached.theta = newHeading;
	reached.turn = turn;

	int xsteps = int(fabs(target.x-start.x)/interpStep.x);
  int ysteps = int(fabs(target.y-start.y)/interpStep.y);
  int numSteps = std::max(xsteps,ysteps);
  float deltaX = (target.x-start.x)/float(numSteps);
  float deltaY = (target.y-start.y)/float(numSteps);
  // Interpolate along the path and check for collisions
  for (int t = 0; t < numSteps; t++) {
    reached.x += deltaX;
    reached.y += deltaY;
    if ( cc->collides(reached) )
      return COLLISION;
  }
  
	reached.x = target.x;
	reached.y = target.y;
  if ( cc->collides(reached) )  // because loop may not have gone all the way to the target
    return COLLISION;
  if ( safeTurn(reached, AngSignPi(nextTheta-newHeading), nextTurn, cc) )
		return REACHED;
	else
		return APPROACHED;
}

std::string RRTNodeXYTheta::toString() const {
  char buff[100];
  sprintf(buff, "%7.2f %7.2f %7.2f", q.x, q.y, (float)(q.theta));
  return string(buff);
}

bool RRTNodeXYTheta::CollisionChecker::collides(const NodeValue_t &qnew, GenericRRTBase::PlannerResult2D* result) {
  // treat as collision if outside world bounds
  if ( worldBounds.isValid() && !worldBounds->isInside(Point(qnew.x, qnew.y)) ) {
    if (result) {
      ostringstream os;
      os << VRmixin::theAgent->getName() << "-" << VRmixin::theAgent->getId();
      result->movingObstacle = new HierarchicalObstacle; // there really is none
      result->movingObstacle->name = os.str();
      ostringstream os2;
      os2 << worldBounds->getName() << "-" << worldBounds->getId();
      result->collidingObstacle = new HierarchicalObstacle;
      result->collidingObstacle->name = os2.str();
    }
    return true;
  }
  
  body.updatePosition(fmat::pack(qnew.x, qnew.y));
  body.updateRotation(fmat::rotation2D(qnew.theta));
  
  for (size_t i = 0; i < obstacles.size(); i++) {
    if ( !obstacles[i]->isBodyObstacle() && obstacles[i]->collides(body)) {
      if (result) {
        ostringstream os;
        os << VRmixin::theAgent->getName() << "-" << VRmixin::theAgent->getId();
        result->movingObstacle = dynamic_cast<PlannerObstacle2D*>(body.clone());
        result->movingObstacle->name = os.str();
        result->collidingObstacle = dynamic_cast<PlannerObstacle2D*>(obstacles[i]->clone());
      }
      // std::cout << "Collision: " << robot.toString() << " with " << obstacles[i]->toString() << std::endl;
      return true;
    }
  }
  return false;
}

std::vector<PlannerObstacle2D*> RRTNodeXYTheta::CollisionChecker::colliders(const NodeValue_t &qnew) {
  body.updatePosition(fmat::pack(qnew.x, qnew.y));
  body.updateRotation(fmat::rotation2D(qnew.theta));
  std::vector<PlannerObstacle2D*> result;
  for (size_t i = 0; i < obstacles.size(); i++)
    if ( !obstacles[i]->isBodyObstacle() && obstacles[i]->collides(body))
      result.push_back(obstacles[i]);
  return result;
}


//================ ShapeSpacePlannerXYTheta ================

ShapeSpacePlannerXYTheta::ShapeSpacePlannerXYTheta(ShapeSpace &shs, const Shape<PolygonData> &worldBounds,
                                                   float inflation) :
  GenericRRT<NodeType_t, 2>(new NodeType_t::CollisionChecker(shs, worldBounds, inflation)),
  targetHeading(), baseOffset(), gateLength()
{
  NodeValue_t interp(25,25,0);  // step size in mm
  setInterpolation(interp);
}

void ShapeSpacePlannerXYTheta::initialize(const NodeValue_t &start, std::vector<NodeType_t> &treeStartResult,
                                          const NodeValue_t &end, std::vector<NodeType_t> &treeEndResult) {
  GenericRRT<NodeType_t, 2>::initialize(start, treeStartResult, end, treeEndResult);
	// To enforce a target heading or base offset, define a gate point
	fmat::Column<3> endpt = fmat::pack(end.x, end.y, 0);
	//cout << "baseoffSet is " << baseOffset << endl;
	if ( gateLength == 0 )
		gateLength = 0.1; // must have a gate
	fmat::Column<3> gateOffset = baseOffset + fmat::pack(gateLength, 0, 0);
	if ( ! std::isnan(targetHeading) ) {
		fmat::Column<3>  baseEndPt = endpt - fmat::rotationZ(targetHeading) * baseOffset;
		treeEndResult[0].q = NodeValue_t(baseEndPt[0], baseEndPt[1], targetHeading);
		fmat::Column<3> offsetEndPt = endpt - fmat::rotationZ(targetHeading) * gateOffset;
		NodeValue_t qnew(offsetEndPt[0], offsetEndPt[1], targetHeading);
		addNode(&treeEndResult, qnew, 0);
		treeEndResult[0].parent = 1;  // index of starting node for nearestNode search
		return;
	}
	// No specific targetHeading, but has baseOffset or gate, so define
	// a ring of offset target points, with one target aligned with the
	// start state heading.
	treeEndResult[0].q.theta = std::numeric_limits<float>::quiet_NaN(); // mark root as a dummy node
	float tanHeading = tangentHeading(start,end);
	// std::cout << "tanHeading = " << tanHeading << " = " << tanHeading*180/M_PI << " deg.\n";
	unsigned int numApproach = 0;
	for (float theta = tanHeading; theta < float(tanHeading)+2*M_PI-(1e-5); theta += 2*M_PI/numDivisions) {
		fmat::Column<3> offsetEndPt = endpt - fmat::rotationZ(theta) * baseOffset;
		NodeValue_t qnew(offsetEndPt[0], offsetEndPt[1], theta);
		if (cc->collides(qnew))
		  continue;
		numApproach++;
		addNode(&treeEndResult, qnew, 0);
  }
	if (!numApproach) 
	  return;
	treeEndResult[0].parent = numApproach+1;  // index of starting node for nearestNode search
	#if defined(TGT_IS_CALLIOPE3A) || defined(TGT_IS_CALLIOPE3) || defined(TGT_IS_CALLIOPE2SP)
	  baseOffset = baseOffset - fmat::pack(250,0,0);
	#endif
	for (unsigned int parentIndex = 1; parentIndex <= numApproach; ++parentIndex) {
		NodeValue_t offsetTarget = treeEndResult[parentIndex].q;
		fmat::Column<3> offsetEndPt = fmat::pack(offsetTarget.x, offsetTarget.y, 0);
		float heading = offsetTarget.theta;
		fmat::Column<3> gatePt = offsetEndPt - fmat::rotationZ(heading)*fmat::pack(gateLength,0,0);
		fmat::Column<3> dist = endpt - gatePt - fmat::rotationZ(heading)*baseOffset;
		NodeValue_t qnew(gatePt[0], gatePt[1], heading);
		if (cc->collides(qnew))
		  continue;
		float xDist = dist[0];
		float xStep = xDist/20;
		float yDist = dist[1];
		float yStep = yDist/20;
		bool noCollision = true;
//		cout << "============================ Coordinate : " << " ( " << gatePt[0] << " , " << gatePt[1] << " ) " << endl;
//		cout << "      ====================== Parameters : xDist is " << xDist << ". xStep is " << xStep << endl; 
		if (abs(endpt[0]-gatePt[0])>=0.01 ) {
		  float slope = (endpt[1]-gatePt[1])/(endpt[0]-gatePt[0]);
		  for (int i = 1; i < 20; i++) {
		    NodeValue_t tmp(gatePt[0]+xStep*i, gatePt[1]+slope*xStep*i, heading);
		    if (cc->collides(tmp)) {
		      noCollision = false;
		      break;
		    }
		  }
		}
		else {
		  for (int i = 1; i < 20; i++) {
		    NodeValue_t tmp(gatePt[0], gatePt[1]+yStep*i, heading);
		    if (cc->collides(tmp)) {
		      noCollision = false;
		      break;
		    }
		  }
		}
		if (!noCollision)
		  continue;
		addNode(&treeEndResult, qnew, parentIndex);
	}
}

AngTwoPi ShapeSpacePlannerXYTheta::tangentHeading(const NodeValue_t &start, const NodeValue_t &end) const {
	// The equations below come from the Wikipedia article on tangent
	// lines between two circles.  We assume the second circle has zero
	// radius.
	fmat::Column<2> c1 = fmat::pack(start.x,start.y), c2 = fmat::pack(end.x,end.y);
	float d = (c2-c1).norm();
  float radius = fabs(baseOffset[1]);
	float R = -radius/d;
	float X = (c2[0] - c1[0]) / d;
	float Y = (c2[1] - c1[1]) / d;
	float rsq = sqrt(1 - R*R);
	float a = R*X - Y*rsq;
	float b = R*Y + X*rsq;
	AngTwoPi theta = atan2(-a,b);
	return theta;
}

void ShapeSpacePlannerXYTheta::buildPath(const std::vector<NodeType_t> *treeStart, const std::vector<NodeType_t> *treeEnd,
                                         std::vector<NodeValue_t> &path) {
  
  // connection point is last value in each tree
  unsigned int n = treeStart->size() - 1;
  path.push_back((*treeStart)[n].q);
  while (n != 0) {
    n = (*treeStart)[n].parent;
    path.push_back((*treeStart)[n].q);
  }
	std::reverse(path.begin(), path.end());
	AngTwoPi lastHeading = path.back().theta;

	// Process the end tree unless it consists of nothing but dummy nodes
	// created due to an unspecified targetHeading.
	// Need to reorder the theta and turn fields since we're reversing the path.
	n = treeEnd->size() - 1;
	AngTwoPi lastTheta = (*treeEnd)[n].q.theta;
	AngSignTwoPi lastTurn = (*treeEnd)[n].q.turn;
	path.push_back((*treeEnd)[n].q);
	path.back().theta = atan2((*treeEnd)[n].q.y, (*treeEnd)[n].q.x);
	path.back().turn = float(AngSignPi(path.back().theta - lastHeading));
	// Note: may need to do a safeTurn check for prevLastTurn, but the
	// path smoother will probably clean this up anyway.
	AngSignTwoPi prevLastTurn = float(AngSignPi(lastTheta - path.back().theta));
	while (n != 0) {
		n = (*treeEnd)[n].parent;
		path.push_back((*treeEnd)[n].q);
		path.back().theta = lastTheta;
		lastTheta = (*treeEnd)[n].q.theta;
		path.back().turn = prevLastTurn;
		prevLastTurn = lastTurn;
		lastTurn = (*treeEnd)[n].q.turn;
	}

	if ( std::isnan(targetHeading) )
		path.pop_back();  // remove dummy root node so we don't try to go there

	/*
  for ( size_t i = 0; i < path.size(); i++ ) {
    Point pt(path[i].x, path[i].y, 0, allocentric);
      cout << "path[" << i << "] = " << pt
	   << " hdg " << float(AngTwoPi(path[i].theta)) * 180/M_PI << " deg."
	   << " (turn " << float(path[i].turn)*180/M_PI << " deg.)" << endl;
  }
	std::cout << "====\n";
	*/

  // Path smoothing: we use two approaches:
	//
	// 1. Pick random pairs of segments and try to snip the intervening
	// segments.  This can potentially snip large sequences quickly.
	//
	// 2. Systematically try every possible excise combination on the
	// segments that remain.

  size_t maxIter = path.size();
	AngSignTwoPi nextTurn;
  
  for (size_t i = 0; i < maxIter; i++) {
    unsigned int a = rand() % (path.size()-1);
		unsigned int b = rand() % (path.size()-1);
    if (a > b)
      std::swap(a,b);
    else if (a == b)
      continue;
		if ( b-a < 2 )
			continue;
    NodeValue_t reached;
		AngTwoPi nextTheta = (b < path.size()-1) ? path[b+1].theta : path[b].theta;
    if ( NodeType_t::snipInterpolate(path[a], path[b], nextTheta, smoothingInterpolationStep,
																		 cc, reached, nextTurn) == RRTNodeBase::REACHED ) {
      path[b] = reached;
			if ( b < path.size()-1 )
				path[b+1].turn = nextTurn;
      path.erase(path.begin()+a+1,path.begin()+b);
    }
  }
  
  for (size_t i = 0; i+2 < path.size()-1; i++) {
    NodeValue_t reached, goodReached;
		AngSignTwoPi goodNextTurn;
    size_t j;
    for (j = i + 2; j < path.size()-1; j++) {
			AngTwoPi nextTheta = (j < path.size()-1) ? path[j+1].theta : path[j].theta;
      if ( NodeType_t::snipInterpolate(path[i], path[j], nextTheta, smoothingInterpolationStep,
																			 cc, reached, nextTurn) == RRTNodeBase::REACHED ) {
				goodReached = reached;
				goodNextTurn = nextTurn;
			}
			else
        break;
    }
    if ( --j >= i + 2 ) {
			path[j] = goodReached;
			if ( j < path.size()-1 )
				path[j+1].turn = goodNextTurn;
      path.erase(path.begin()+i+1, path.begin()+j);
		}
  }

	/*
	// If no targetHeading, make sure we end up pointing toward the destination
	if ( std::isnan(targetHeading) ) {
		NodeValue_t fixHeading = path.back();
		float newTheta = atan2(dest.y-fixHeading.y, dest.x-fixHeading.x);
		fixHeading.turn = float(AngSignPi(newTheta - float(fixHeading.theta)));
		fixHeading.theta = newTheta;
		path.push_back(fixHeading);
	}
*/
}

GenericRRTBase::PlannerResult2D
ShapeSpacePlannerXYTheta::planPath(const Point &startPoint,
																	 const fmat::Column<3> &_baseOffset,
																	 float _gateLength,
																	 const Point &endPoint,
                                   const AngTwoPi initialHeading,
                                   const AngTwoPi _targetHeading,
                                   unsigned int maxIterations,
                                   std::vector<NodeValue_t> *pathResult,
                                   std::vector<NodeType_t> *treeStartResult,
                                   std::vector<NodeType_t> *treeEndResult) {
  NodeValue_t lower, upper;
  if ( cc->getWorldBounds().isValid() ) {
    BoundingBox2D b = cc->getWorldBounds()->getBoundingBox();
    lower.x = b.min[0]; lower.y = b.min[1];
    upper.x = b.max[0]; upper.y = b.max[1];
  } else {
    // combine with obstacles + start and end points
    BoundingBox2D b = cc->getObstacleBoundingBox();
    b.expand(fmat::SubVector<2,const fmat::fmatReal>(startPoint.coords));
    b.expand(fmat::SubVector<2,const fmat::fmatReal>(endPoint.coords));
    fmat::Column<2> robotBounds = cc->getBodyBoundingBox().getDimensions();
    float extra = std::max(robotBounds[0], robotBounds[1]);
    lower.x =  b.min[0] - 2*extra;
    lower.y = b.min[1] - 2*extra;
    upper.x =  b.max[0] + 2*extra;
    upper.y = b.max[1] + 2*extra;
  }
  // std::cout << "World bounds: [" << lower.x << "," << lower.y << "]  to  ["
	// << upper.x << "," << upper.y << "]" << std::endl;
  setLimits(lower,upper);
  
  targetHeading = _targetHeading;
  float safeTargetHeading = std::isnan((float)_targetHeading) ? 0 : targetHeading;
  baseOffset = _baseOffset;
	gateLength = _gateLength;
  
  NodeValue_t startval(startPoint.coordX(), startPoint.coordY(), initialHeading);
  fmat::Column<3> endWithOffset = endPoint.getCoords();
  NodeValue_t endval(endWithOffset[0], endWithOffset[1], AngTwoPi(safeTargetHeading));
  // If no targetHeading specified, endval is a dummy goal node.  The real nodes
  // will be added by the initialize() method that planPath calls.
	return GenericRRT<NodeType_t, 2>::planPath(startval, endval, maxIterations, 
																						 pathResult, treeStartResult, treeEndResult);
}

void ShapeSpacePlannerXYTheta::plotPath(const std::vector<NodeValue_t> &path,
					Shape<GraphicsData> &graphics,
					rgb color) {
  for ( unsigned int i = 1; i < path.size(); i++ )
    graphics->add(new GraphicsData::LineElement("seg",
						std::pair<float,float>(path[i-1].x,path[i-1].y),
						std::pair<float,float>(path[i].x,path[i].y),
						color));
}

void ShapeSpacePlannerXYTheta::plotTree(const std::vector<NodeType_t> &tree,
                                        Shape<GraphicsData> &graphics,
                                        rgb color) {
  for ( unsigned int i = 1; i < tree.size(); i++ ) { // start from 1 because root has no parent
		unsigned int parent = tree[i].parent;
		if ( parent != 0 || !std::isnan((float)tree[parent].q.theta ) )
			graphics->add(new GraphicsData::LineElement
										("branch", 
										 std::pair<float,float>(tree[parent].q.x,
																						tree[parent].q.y),
										 std::pair<float,float>(tree[i].q.x,
																						tree[i].q.y),
										 color));
		else
			graphics->add(new GraphicsData::EllipseElement
										("end",
										 std::pair<float,float>(tree[i].q.x,
																						tree[i].q.y),
										 10, 5, tree[i].q.theta, true, color));
	}
}
