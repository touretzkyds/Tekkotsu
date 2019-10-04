#include "Shared/RobotInfo.h"
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)

#include "Crew/Grasper.h"
#include "DualCoding/ShapeCross.h"
#include "DualCoding/ShapeNaught.h"
#include "DualCoding/VRmixin.h"
#include "Motion/IKSolver.h"
#include "Planners/Navigation/ShapeSpacePlannerXYTheta.h"
#include "Shared/mathutils.h"

using namespace DualCoding;
using namespace mathutils;

//**** These constants are for the Chiara chess gripper.
float openLeftGripperVal = -0.10f;
float closedLeftGripperVal = -0.25f;
float openRightGripperVal = 0.07f;
float closedRightGripperVal = 0.25f;

GrasperRequest* Grasper::curReq = NULL;
Grasper::GrasperVerbosity_t Grasper::verbosity = -1U;

GenericRRTBase::PlannerResult2D
Grasper::planBodyPath(const Point &targetPt, AngTwoPi approachOrientation,
											const fmat::Column<3> &baseOffset, 
											Shape<AgentData> &pose, float radius, bool isFinalApproach = true) {
  //Declare function members for calculation
  Point agentpt = VRmixin::theAgent->getCentroid();				//Define where the robot is currently
  Point vec = targetPt - agentpt;	
  float dist = vec.xyNorm();															//Find the distance between those two points
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);	//Get the bounds of the world
	
  ShapeSpacePlannerXYTheta planner(VRmixin::worldShS, worldBounds, curReq->pilotreq.obstacleInflation);
  GenericRRTBase::PlannerResult2D result;
  std::vector<ShapeSpacePlannerXYTheta::NodeValue_t> pathResult;
  std::vector<ShapeSpacePlannerXYTheta::NodeType_t> *treeStartResult = new std::vector<ShapeSpacePlannerXYTheta::NodeType_t>;
  std::vector<ShapeSpacePlannerXYTheta::NodeType_t> *treeEndResult = new std::vector<ShapeSpacePlannerXYTheta::NodeType_t>;
  Point pIn, pOut, approachPoint;
  ShapeSpacePlannerXYTheta::NodeValue_t finalValue;
	
	//If the robot is outside the desired radius...
  if ( dist >= radius ) {
		//Plan the path to the target and store the result in the variable of the same name
		result = planner.planPath(agentpt, baseOffset, 0, targetPt, 
															VRmixin::theAgent->getOrientation(),
															isFinalApproach ? curReq->targetOrientation : approachOrientation,
															curReq->pilotreq.maxRRTIterations, &pathResult, treeStartResult, treeEndResult);
		//curReq->***Orientation orients the robot and (only by extension) the object to be placed (if we're using a Calliope2SP)
		
    // begin debug---------------------------------------------------------------------------------------------------
    // std::cout << "showing trees size " << treeStartResult->size() << " " << treeEndResult->size() << std::endl;
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
    // end debug-----------------------------------------------------------------------------------------------------
		
    if (result.code != GenericRRTBase::SUCCESS) {
			//If the RRT did not succeed, determine why:
			
      if ( result.code == GenericRRTBase::END_COLLIDES ) {
				VRmixin::robotObstaclesPt = targetPt - Point(baseOffset,allocentric);
				VRmixin::robotObstaclesOri = AngTwoPi(0);
      } else {
			// this means we encountered a START_COLLIDES or MAX_ITER
				VRmixin::robotObstaclesPt = agentpt;
				VRmixin::robotObstaclesOri = VRmixin::theAgent->getOrientation();
      }
      planner.addObstaclesToShapeSpace(VRmixin::worldShS);
      return result;
    }
		
		// If we were successful...
    VRmixin::robotObstaclesPt = agentpt;
    VRmixin::robotObstaclesOri = VRmixin::theAgent->getOrientation();
    /*
    // Find the path segment that crosses the circle
    std::vector<ShapeSpacePlannerXYTheta::NodeValue_t>::const_iterator it = pathResult.end();
    while ( it-- != pathResult.begin() ) {
      pOut = Point(it->first.first, it->first.second);
      float pdist = (pOut - targpt).xyNorm();
      if ( pdist >= radius ) {
	// point pOut is outside the circle; next point will be inside
	++it;
	pIn = Point(it->first.first, it->first.second);
	break;
      }
    }
    */
    finalValue = pathResult.back();
  } else {  // If the dist to the target is < the desired radius
		
    // Must get outside the circle.  Use present heading and back up
    // if necessary.  Start by making a dummy path segment.
    pIn = agentpt;
    float orient = VRmixin::theAgent->getOrientation();
    pOut = Point(pIn.coordX() - 2*radius*cos(orient),
								 pIn.coordY() - 2*radius*sin(orient));
    finalValue = ShapeSpacePlannerXYTheta::
			NodeValue_t(agentpt.coordX() - (2*radius-dist)*cos(orient),
									agentpt.coordY() - (2*radius-dist)*sin(orient),
									orient);
  }
  /*
  // Find the line equation for the path segment
  std::pair<float,float> lineEq = LineData(VRmixin::worldShS,pOut,pIn).lineEquation_mb();
  float m = lineEq.first;
  float b = lineEq.second;
  // Now find the point on the path segment that crosses the circle.
  // This point simultaneously satisfies the line equation y=m*x+b and
  // the constraint x^2+y^2 = r^2.  Solution: replace y with mx+b in
  // the circle equation, yielding a quadratic in x.  Solve for x in
  // terms of the constants m, b, and r.  There can be two solutions.
  float xo = targpt.coordX(), yo = targpt.coordY();
  float root = sqrt(-b*b - 2*b*m*xo + 2*b*yo - m*m*xo*xo + m*m*radius*radius + 2*m*xo*yo + radius*radius - yo*yo);
  float x1 = ( root - b*m + m*yo + xo) / (m*m+1);  // First solution
  float x2 = (-root - b*m + m*yo + xo) / (m*m+1);  // Second solution
  // Choose the correct solution.
  float x;
  if ( dist > radius )
    x = ((x1 >= pOut.coordX() && x1 <= pIn.coordX()) || (x1 >= pIn.coordX() && x1 <= pOut.coordX())) ? x1 : x2;
  else // we're inside the circle
    x = ((agentpt-Point(x1,m*x1+b)).xyNorm() < radius) ? x1 : x2;
  // Create an agent pose at the desired point, and save it in curReq
  float y = m*x + b;
    */
  if ( std::isnan(finalValue.x) ) {
    std::cout << " finalValue.x is nan " << endl;
    finalValue.x = agentpt.coordX();
    finalValue.y = agentpt.coordY();
  }
  if ( std::isnan(float(finalValue.theta)) ) {
    std::cout << " finalValue.theta is nan " << endl;
    finalValue.theta = 0.0;
  }
  Point posePoint(finalValue.x,finalValue.y,0,allocentric);
  //AngTwoPi poseHeading((pIn-pOut).atanYX());  // maintain the segment's heading to assure obstacle clearance
  pose->setCentroidPt(posePoint);
  pose->setOrientation(finalValue.theta);
  pose->setColor(rgb(0,255,0));
  pose->setObstacle(false);
  return result;
}

float Grasper::PlanBodyApproach::getBodyApproachRadius() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  const float radius = 250;
#elif defined(TGT_IS_CALLIOPE5)
  const float radius = 640;
#else
  const float radius = 500;  // a "reasonable" value for unknown robot type
#endif
  return radius;
}

void Grasper::PlanBodyApproach::doStart() {
  if ( ! curReq->allowBodyMotion ) {
    postStateFailure();
    return;
  }
	float bodyRadius = getBodyApproachRadius();
	std::cout << "*** bodyRadius = " << bodyRadius << std::endl;
	fmat::Column<3> baseOffset = fmat::pack(bodyRadius,0,0);
	Point targPoint = curReq->object->getCentroid();
  AngTwoPi orient = curReq->approachOrientation;
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	PostureEngine pe;
	pe.setOutputCmd(ArmBaseOffset,0.0);
	pe.setOutputCmd(ArmShoulderOffset,0.0);
	baseOffset = pe.getPosition(GripperFrameOffset);
#else
	std::cout << "*** Grasper::PlanBodyApproach::doStart doesn't support this robot type." << std::endl;
#endif
	cout << "*** targPoint=" << targPoint << "   orient=" << orient
			 << "   baseOffset=" << baseOffset << "   bodyRadius=" << bodyRadius << endl;
	if ( curReq->object->getType() == cylinderDataType ) {
		baseOffset[0] += ShapeRootTypeConst(curReq->object, CylinderData)->getRadius() + bodyRadius;
		curReq->object->setObstacle(false);  // *** THIS IS WRONG; can cause collision if approachOrientation specified
		// *** Need to rethink path planning to properly take into account the object's radius and grasp point, obstacle inflation,
		// *** minimum required distance for apporach (misnamed as bodyRadius), etc.
	}
	 if ( curReq->object->getType() == naughtDataType ) {
		baseOffset[0] += ShapeRootTypeConst(curReq->object, NaughtData)->getRadius() + bodyRadius;
		curReq->object->setObstacle(false);  // *** THIS IS WRONG; can cause collision if approachOrientation specified
		// *** Need to rethink path planning to properly take into account the object's radius and grasp point, obstacle inflation,
		// *** minimum required distance for apporach (misnamed as bodyRadius), etc.
   } 
	 else if ( curReq->object->getType() == dominoDataType ) {
		orient = std::numeric_limits<float>::quiet_NaN();
		ShapeRootTypeConst(curReq->object, DominoData)->
			computeGraspPoint((DominoData::ObjectFeature)curReq->objectFeature, targPoint, orient);
		baseOffset = fmat::pack(bodyRadius*1.25, 0, 0);  // **** HACK for now
		fmat::Column<3> offsetTarget = targPoint.getCoords() - fmat::rotationZ(orient) * baseOffset;
		cout << "*** targPoint=" << targPoint << "   orient=" << orient 
		     << "   offsetTarget=" << offsetTarget << endl;
		targPoint.setCoords(offsetTarget);
		bodyRadius = 0;
		// we want to put the gripper, not the base, at the offset target point
   }
	 else if ( curReq->object->getType() == crossDataType ) {
		orient = std::numeric_limits<float>::quiet_NaN();
		std::vector<Point> graspPoints = ShapeRootTypeConst(curReq->object, CrossData)->computeGraspPoints();
		targPoint = graspPoints[0];
		orient = (targPoint - curReq->object->getCentroid()).atanYX();
		curReq->object->setObstacle(false);  // *** THIS IS WRONG; can cause collision if approachOrientation specified
		baseOffset = fmat::pack(bodyRadius*1.55, 0, 0);  // **** HACK for now
		fmat::Column<3> offsetTarget = targPoint.getCoords() - fmat::rotationZ(orient) * baseOffset;
		cout << "*** targPoint=" << targPoint << "   orient=" << orient << "   centroid=" << curReq->object->getCentroid()  
		     << "   offsetTarget=" << offsetTarget << endl;
		targPoint.setCoords(offsetTarget);
		bodyRadius = 0;
		// we want to put the gripper, not the base, at the offset target point
	} 
	//Plan the body path
  NEW_SHAPE(approachPose, AgentData, new AgentData(VRmixin::theAgent.getData()));
  GenericRRTBase::PlannerResult2D result =
    VRmixin::grasper->planBodyPath(targPoint, orient, baseOffset, approachPose, bodyRadius, false);

	//Check the result of that planning:
  if ( result.code != GenericRRTBase::SUCCESS ) {
		//If we did not succeed, try to explain why before returning
    switch ( result.code ) {
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Grasper approach planning failed: start state " << result.movingObstacle->toString()
		<< " is in collision with " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Grasper approach planning failed: no collision-free path to " << curReq->object 
		<< " due to " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Grasper approach path planning gave up after " << curReq->pilotreq.maxRRTIterations << " iterations.\n";
      break;
    default: break;
    }
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
    return;
  }
	//If we did succeed, continue with the rest of the request
  curReq->approachPose = approachPose;
  postStateCompletion();
}

void Grasper::DoBodyApproach::doStart() {
  if ( ! curReq->approachPose.isValid() ) {   // should always be valid
    std::cout << "Error: Grasper::ReachBody invoked with no valid approachPose!" << std::endl;
    cancelThisRequest();
    return;
  }
  if ( curReq->pilotreq.targetShape.isValid() )
    pilotreq = curReq->pilotreq;
  else {
    pilotreq = PilotRequest(PilotTypes::goToShape);
    pilotreq.targetShape = curReq->approachPose;
    pilotreq.targetHeading = curReq->approachPose->getOrientation();
  }
}

void Grasper::FindObj::doStart() {
  if ( curReq->mapreq != NULL ) {
    mapreq = *curReq->mapreq;
  }
  else {
    mapreq = MapBuilderRequest(MapBuilderRequest::worldMap);
    mapreq.addAttributes(curReq->object);
    mapreq.addAllMinBlobAreas(1000); // *** stopgap for cylinders (and naughts)
    mapreq.worldTargets.push(new LookoutSearchRequest(curReq->object));
	}
}

void Grasper::PlanArmApproach::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  fmat::Column<3> target;
	if ( curReq->object->getType() == dominoDataType ) {
		Point targPoint;
		AngTwoPi orient;
		ShapeRootTypeConst(curReq->object, DominoData)->
			computeGraspPoint((DominoData::ObjectFeature)curReq->objectFeature, targPoint, orient);
		target = targPoint.getCoords();
	}
	else if ( curReq->object->getType() == crossDataType ) {
		Point targPoint;
		AngTwoPi orient;
		std::vector<Point> graspPoints = ShapeRootTypeConst(curReq->object, CrossData)->
			computeGraspPoints();
		targPoint = graspPoints[0];
		orient = (targPoint - curReq->object->getCentroid()).atanYX();
		target = targPoint.getCoords();
	} 
	else
		target = curReq->object->getCentroid().getCoords();
  // Set shoulder position to 3/4 of the height of the target shape.
  if ( curReq->object->getType() == cylinderDataType ) // grasp cylinders at 75% of their height
    target[2] += ShapeRootTypeConst(curReq->object, CylinderData)->getHeight() * 0.25;
  else if ( curReq->object->getType() == naughtDataType ) // grasp naughts, like cylinders, at 75% of their height
    target[2] += ShapeRootTypeConst(curReq->object, NaughtData)->getHeight() * 0.25;
	else if ( curReq->object->getType() == brickDataType ) // If the object to grasp is a brick-type object
		target[2] += ShapeRootTypeConst(curReq->object, BrickData)->getCentroid().coordZ() * 0.5;
  // cout << "PlanReachArm: target at: " << target << endl; 
  PostureEngine pe;
  pe.solveLinkPosition(target, GripperFrameOffset, fmat::ZERO3);
  //cout << "PlanReachArm: GripperFrame is at " << pe.getPosition(GripperFrameOffset) << endl;
  NodeValue_t armpoint;
#if defined(TGT_IS_CALLIOPE3)
  armpoint[0] = -1.5;  // arm base
#endif
#if defined(TGT_IS_CALLIOPE2)
  armpoint[0] = 0;  // arm base
#endif
  armpoint[1] = pe.getOutputCmd(ArmShoulderOffset).value;
  curReq->approachPath.push_back(armpoint);
  postStateCompletion();
#elif defined(TGT_IS_CALLIOPE5)
  // plan Calliope5 arm reach
  // assume there is a shape in local space that matches our world space object; fail if not
  std::cout << "PlanArmApproach for 5KP" << std::endl;
  ShapeRoot objlocal = find_if(VRmixin::localShS, IsLastMatch(curReq->object));
  if ( ! objlocal.isValid() ) {
    std::cout << "Grasper can' fitnd " << curReq->object << " (last match id " << curReq->object->getLastMatchId()
	      << ") in local shape space!" << std::endl;
    postStateSignal<GraspError>(GrasperRequest::lostObject);
    return;
  }
  objlocal->setObstacle(true);
  std::vector<NodeValue_t> endStates;

  fmat::Column<3> toPtCent = objlocal->getCentroid().getCoords();
  fmat::Column<3> toPtArmRef = toPtCent - kine->getPosition(ArmBaseOffset);
  fmat::Column<3> toPtArmRefNormal = toPtArmRef / toPtArmRef.norm();
  fmat::Column<3> toPtOff = toPtArmRefNormal * 125;  // pick a point 90 mm short of the object
  fmat::Column<3> toPtShort = toPtArmRef - toPtOff;
  IKSolver::Point toPt(kine->getPosition(ArmBaseOffset) + toPtShort);
  VRmixin::grasper->computeGoalStates(toPt,
				      curReq->gripperAngleRangesX,
				      curReq->gripperAngleRangesY,
				      curReq->gripperAngleRangesZ,
				      curReq->angleResolution,
				      endStates, IKSolver::Point(fmat::ZERO3));
  for (uint i = 0; i < endStates.size(); i++) {
    cout << "state " << i << ": ";
	for (uint j = 0; j < 5; j++)
		cout << endStates[i][j] << ", ";
	cout << endl;
  }
  if ( endStates.empty() ) {
	cout << "failed to find end state" << endl;
    postStateSignal<GraspError>(GrasperRequest::pickUpUnreachable);
    return;
  }
  NodeValue_t startSt;
  Grasper::getCurrentState(startSt);
  
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  Grasper::ArmPlanner planner(VRmixin::localShS, worldBounds, curReq->rrtInflation, curReq->effectorOffset);
  std::vector<NodeType_t> *treeStartResult = curReq->displayTree ? new std::vector<NodeType_t> : NULL;
  std::vector<NodeType_t> *treeEndResult = curReq->displayTree ? new std::vector<NodeType_t> : NULL;
  fmat::Transform t = fmat::Transform();
  //t.translation() = VRmixin::theAgent->getCentroid().coords;
  //t.rotation() = fmat::rotationZ(VRmixin::theAgent->getOrientation());
  Grasper::ArmPlanner::PlannerResult result;
  for (uint i = 0; i < endStates.size(); i++) {
		result = planner.planPath(startSt, endStates.front(), curReq->rrtInterpolationStep,
				 t, curReq->rrtMaxIterations,
				 &(curReq->approachPath), treeStartResult, treeEndResult);
	  if (result.code == GenericRRTBase::SUCCESS)
		break;
	  switch ( result.code ) {
		case GenericRRTBase::SUCCESS:
		  break;
		case GenericRRTBase::START_COLLIDES:
		  std::cout << "Arm path planning failed: start state is in collision.\n";
		  break;
		case GenericRRTBase::END_COLLIDES:
		  std::cout << "Arm path planning failed: end state is in collision.\n";
		  break;
		case GenericRRTBase::MAX_ITER:
		  std::cout << "Arm path planning failed: too many iterations.\n";
		  break;
		}
	  if (result.code == GenericRRTBase::START_COLLIDES || result.code == GenericRRTBase::END_COLLIDES) {
		std::cout << "Moving part: " << std::endl << "=================" << std::endl
			  << result.movingObstacle->toString() << std::endl;
		std::cout << "collided with: " << std::endl << "=================" << std::endl
			  << result.collidingObstacle->toString() << std::endl;
		std::cout << "=================" << std::endl;
	}
  }
  switch ( result.code ) {
    case GenericRRTBase::SUCCESS:
      std::cout << "PlanArmApproach succeeded" << std::endl;
      break;
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Arm path planning failed: start state is in collision.\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Arm path planning failed: end state is in collision.\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Arm path planning failed: too many iterations.\n";
      break;
  }
  
  if (result.code == GenericRRTBase::START_COLLIDES || result.code == GenericRRTBase::END_COLLIDES) {
    std::cout << "Obstacle: " << std::endl << "=================" << std::endl
	      << result.movingObstacle->toString() << std::endl;
    std::cout << "collided with: " << std::endl << "=================" << std::endl
	      << result.collidingObstacle->toString() << std::endl;
    std::cout << "=================" << std::endl;
  }
  
  // Display the tree if requested
  if ( curReq->displayTree && result.code != GenericRRTBase::START_COLLIDES && GenericRRTBase::END_COLLIDES ) {
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
  }
  
  // Display the path if requested
  if ( curReq->displayPath && result.code == GenericRRTBase::SUCCESS ) {
    NEW_SHAPE(plannedPath, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotPath(curReq->approachPath, plannedPath, rgb(0,0,255));
  } 
  
  if (result.code != GenericRRTBase::SUCCESS) {
    planner.addObstaclesToShapeSpace(VRmixin::localShS);
    planner.addObstaclesToShapeSpace(VRmixin::worldShS, VRmixin::mapBuilder->localToWorldMatrix);
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
    return;
  }
  // We planned a path to the gate point.  Now add one more step to
  // get to the actual target point.
  KinematicJoint *effector = kine->getKinematicJoint(curReq->effectorOffset)->cloneBranch();
  IKSolver& solver = effector->getIK();
  float const positionMostImportant = 1.0f;
  float const orientationLeastImportant = 0.5f;
  IKSolver::Point goalPos(objlocal->getCentroid().getCoords());
  bool reached = solver.solve(IKSolver::Point(fmat::ZERO3), IKSolver::Rotation(fmat::Quaternion::aboutX(-M_PI/2)),
			      *effector, goalPos, positionMostImportant, IKSolver::Parallel(0,0,1), orientationLeastImportant);
  NodeValue_t goalState;
  Grasper::getCurrentState(goalState,effector);
  curReq->approachPath.push_back(goalState);
  delete effector;
  postStateCompletion();
  return;
#elif defined(TGT_HANDEYE)
  std::vector<NodeValue_t> endStates;
  Point toPt = curReq->object->getCentroid();
  /*
  VRmixin::grasper->computeGoalStates(toPt,
				      curReq->gripperAngleRangesX,
				      curReq->gripperAngleRangesY,
				      curReq->gripperAngleRangesZ,
                                      curReq->angleResolution,
				      endStates, IKSolver::Point(fmat::ZERO3));
  */
  if ( endStates.empty() ) {
    postStateSignal<GraspError>(GrasperRequest::pickUpUnreachable);
    return;
  }

  NodeValue_t startSt;
  Grasper::getCurrentState(startSt);
  
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  Grasper::ArmPlanner planner(VRmixin::worldShS, worldBounds, curReq->rrtInflation, curReq->effectorOffset);
  std::vector<NodeType_t> *treeStartResult = curReq->displayTree ? new std::vector<NodeType_t> : NULL;
  std::vector<NodeType_t> *treeEndResult = curReq->displayTree ? new std::vector<NodeType_t> : NULL;
  fmat::Transform t;
  t.translation() = VRmixin::theAgent->getCentroid().coords;
  t.rotation() = fmat::rotationZ(VRmixin::theAgent->getOrientation());
  Grasper::ArmPlanner::PlannerResult result =
    planner.planPath(startSt, endStates.front(), curReq->rrtInterpolationStep,
		     t, curReq->rrtMaxIterations,
		     &(curReq->approachPath), treeStartResult, treeEndResult);
  
  switch ( result.code ) {
    case GenericRRTBase::SUCCESS:
      std::cout << "Plan Unconstrained succeeded" << std::endl;
      break;
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Arm path planning failed: start state is in collision.\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Arm path planning failed: end state is in collision.\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Arm path planning failed: too many iterations.\n";
      break;
  }
  
  if (result.code == GenericRRTBase::START_COLLIDES || result.code == GenericRRTBase::END_COLLIDES) {
    std::cout << "Obstacle: " << std::endl << "=================" << std::endl
	      << result.movingObstacle->toString() << std::endl;
    std::cout << "collided with: " << std::endl << "=================" << std::endl
	      << result.collidingObstacle->toString() << std::endl;
    std::cout << "=================" << std::endl;
  }
  
  // Display the tree if requested
  if ( curReq->displayTree && result.code != GenericRRTBase::START_COLLIDES && GenericRRTBase::END_COLLIDES ) {
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
  }
  
  // Display the path if requested
  if ( curReq->displayPath && result.code == GenericRRTBase::SUCCESS ) {
    NEW_SHAPE(plannedPath, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotPath(curReq->approachPath, plannedPath, rgb(0,0,255));
  } 
  
  if (result.code != GenericRRTBase::SUCCESS) {
    std::cout << "PlanArmApproach failed" << std::endl;
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
    return;
  }
  postStateCompletion();
#endif
}

void Grasper::DoBodyApproach2::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	// We're at the approach gate but perhaps not pointed quite correctly.
	// Determine how much to rotate the body to point the gripper at the target.
	// Target should have been left in localShS by FindObj
	curReq->object->setObstacle(false);
  ShapeRoot objlocal = find_if(VRmixin::localShS, IsLastMatch(curReq->object));
  if ( ! objlocal.isValid() ) {
    std::cout << "Can't find local shape that matches " << curReq->object
              << " loc=" << curReq->object->getCentroid()
              << "(lastmatch " << curReq->object->getLastMatchId() << ") !" << std::endl;
    SHAPEROOTVEC_ITERATE(VRmixin::localShS, element) {
      std::cout << "   " << element << " loc=" << element->getCentroid() << std::endl;
    } END_ITERATE;
    cancelThisRequest();
    																																																							return;
  }
	fmat::Column<3> objPos = objlocal->getCentroid().getCoords();
	float objDist = objlocal->getCentroid().xyNorm();
	float distsq = objDist * objDist;
	fmat::Column<3> gripperPos = kine->linkToBase(GripperFrameOffset).translation();
/*
	fmat::Column<3> shoulderPos = kine->linkToBase(ArmShoulderOffset).translation();
	fmat::Column<3> armVec = gripperPos - shoulderPos;
	armVec = armVec / armVec.norm() * objDist;
	float pa = (armVec[0]*armVec[0] + armVec[1]*armVec[1]);
	float pb = 2 * (shoulderPos[0]*armVec[0] + shoulderPos[1]*armVec[1]);
	float pc = shoulderPos[0]*shoulderPos[0] + shoulderPos[1]*shoulderPos[1] - distsq;
	float k1 = (-pb + sqrt(pb*pb - 4*pa*pc)) / (2*pa);
	fmat::Column<3> shoulderProj = shoulderPos + k1*armVec;
	cout << "pa=" << pa << " pb=" << pb << " pc=" << pc << " k1=" << k1 << " shoulderProj=" << shoulderProj << endl;
	objPos[2] = shoulderProj[2] = 0;
	float dot = fmat::dotProduct(objPos, shoulderProj) / distsq;
	float theta1 = acos(dot);
	if ( shoulderProj[1] > objPos[1] )
		theta1 *= -1;
	std::cout << "objPos=" << objPos << "  shoulderPos=" << shoulderPos
						<< "  shoulderProj=" << shoulderProj
						<< "  theta1=" << theta1*180/M_PI << " deg." << std::endl;
*/
	fmat::Column<3> ahead = fmat::pack(1,0,0);
	float pa = 1;
	float pb = 2 * gripperPos[0];
	float pc = gripperPos[0]*gripperPos[0] + gripperPos[1]*gripperPos[1] - distsq;
	float k1 = (-pb + sqrt(pb*pb - 4*pa*pc)) / (2*pa);
	fmat::Column<3> gripperProj = gripperPos + k1 * ahead;
	objPos[2] = gripperProj[2] = 0;
	float dot = fmat::dotProduct(objPos, gripperProj) / distsq;
	float theta1 = acos(dot);
	if ( gripperProj[1] > objPos[1] )
		theta1 *= -1;
	std::cout << "objPos=" << objPos << "  gripperPos=" << gripperPos << "  gripperProj=" << gripperProj
		<< "  theta1=" << theta1*180/M_PI << " deg." << std::endl;
  pilotreq.da = theta1;
  pilotreq.turnSpeed = 0.25;
#else
  cancelThisRequest();
#endif
}

void Grasper::DoBodyApproach3::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  ShapeRoot objlocal = find_if(VRmixin::localShS, IsLastMatch(curReq->object));
  float dist = objlocal->getCentroid().xyNorm();
	float radius = 0;
  if ( curReq->object->getType() == cylinderDataType ) // subtract half the cylinder radius (heuristic; should really consider finger length)
		radius = ShapeRootTypeConst(curReq->object, CylinderData)->getRadius();
	else if ( curReq->object->getType() == naughtDataType ) // subtract half the naught radius, like cylinders, (heuristic; should really consider finger length)
		radius = ShapeRootTypeConst(curReq->object, NaughtData)->getRadius();
	else if ( curReq->object->getType() == dominoDataType )
		radius = 30; // *** HACK FOR DOMINOES
	else if ( curReq->object->getType() == crossDataType )
	  radius = ShapeRootTypeConst(curReq->object, CrossData)->getArmSemiLength();
	else if ( curReq->object->getType() == brickDataType ){
		cout << "Trying to approach a brick\n" ;
	}
	float gripperXDisp = kine->linkToBase(GripperFrameOffset).translation()[0];
  pilotreq.dx = dist - gripperXDisp - radius - 10;
  pilotreq.forwardSpeed = 100;
#else
  cancelThisRequest();
#endif
}

void Grasper::DoBodyApproach4::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  ShapeRoot objlocal = find_if(VRmixin::localShS, IsLastMatch(curReq->object));
  float dist = objlocal->getCentroid().xyNorm();
	float radius = 0;
  if ( curReq->object->getType() == cylinderDataType ) // subtract half the cylinder radius (heuristic; should really consider finger length)
		radius = ShapeRootTypeConst(curReq->object, CylinderData)->getRadius();
	else if ( curReq->object->getType() == naughtDataType ) // subtract half the naught radius, like cylinders, (heuristic; should really consider finger length)
		radius = ShapeRootTypeConst(curReq->object, NaughtData)->getRadius();
	else if ( curReq->object->getType() == dominoDataType )
		radius = 30; // *** HACK FOR DOMINOES
	else if ( curReq->object->getType() == crossDataType )
	  radius = ShapeRootTypeConst(curReq->object, CrossData)->getArmSemiLength();
	else if ( curReq->object->getType() == brickDataType ){
		cout << "Trying to approach a brick\n" ;
	}
	float gripperXDisp = kine->linkToBase(GripperFrameOffset).translation()[0];
	//float shoulderXDisp = kine->linkToBase(ArmOffset).translation()[0];
      #if defined(TGT_IS_CALLIOPE2) 
	int const gripperCenterToPalm = 10; // fudge factor for 2SP gripper
      #elif defined(TGT_IS_CALLIOPE3)
	int const gripperCenterToPalm = 40;
      #else 
        int const gripperCenterToPalm = 0;
      #endif
  pilotreq.dx = dist - gripperXDisp - radius + gripperCenterToPalm;
  pilotreq.forwardSpeed = 100;
  postStateCompletion();
#else
  cancelThisRequest();
#endif
}


void Grasper::FingersApproach::doStart() {
  // Should calculate finger distance based on object width.
#if defined(TGT_IS_CALLIOPE2)
  if ( curReq->object->getType() == dominoDataType  || curReq->object->getType() == crossDataType )
    getMC()->openGripper(0.55);
  else
    getMC()->openGripper(0.8);		//Changed as of 4-7-14
  //getMC()->requestGripperLoad(0);
#elif defined(TGT_IS_CALLIOPE3)
  getMC()->openGripper(0.8); // just a guess for now
#elif defined(TGT_IS_CALLIOPE5)
  getMC()->openGripper(0.8f);
#else
  std::cout << "Error: Grasper::FingersApproach undefined for this robot model." << std::endl;
#endif
}
//------------------------------Verify methods-------------------

void Grasper::Verify::VerifyDispatch::doStart() {
	postStateSignal<GrasperRequest::GrasperVerifyStrategy_t>(curReq->verifyStrategy);
}

void Grasper::Verify::GetAprilTag::doStart() {
  mapreq.setAprilTagFamily();
}

void Grasper::Verify::CheckAprilTag::doStart() {
  	Point camcenter(VRmixin::camSkS.getWidth()/2, VRmixin::camSkS.getHeight()/2);
  	float matchTolerance = 0.5;  // tag distance from center must be < 50% of camera width
  	NEW_SHAPEVEC(tags, AprilTagData, select_type<AprilTagData>(VRmixin::camShS));
  	SHAPEVEC_ITERATE(tags, AprilTagData, tag) {
  	  float offcenter = (tag->getCentroid() - camcenter).xyNorm();
  	  if ( offcenter/VRmixin::camSkS.getWidth() < matchTolerance ) {
  	    postParentCompletion();
  	    return;
  	  }
  	} END_ITERATE;
  	std::cout << "Grasper failed to find AprilTag to verify grasp object." << std::endl;
  	postParentSignal<GraspError>(GrasperRequest::badGrasp);
}

void Grasper::Verify::GetDomino::doStart() {
	mapreq.addAttributes(curReq->object);
}

void Grasper::Verify::GetCross::doStart() {
	mapreq.addAttributes(curReq->object);
}

void Grasper::Verify::CheckDomino::doStart() {
	fmat::Column<3> gripperPos;
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	gripperPos = kine->linkToBase(GripperFrameOffset).translation();
#endif
  NEW_SHAPEVEC(dominoes, DominoData, select_type<DominoData>(VRmixin::localShS));
	// ******* SHOULD USE DOMINO GRASP POINT, NOT CENTROID *********
	SHAPEVEC_ITERATE(dominoes, DominoData, domino) {
		Point targPoint;
		AngTwoPi orient;
		domino->computeGraspPoint((DominoData::ObjectFeature)curReq->objectFeature, targPoint, orient);
		fmat::Column<3> diff = targPoint.getCoords() - gripperPos;
		cout << "*** Verify domino grasp point at " << targPoint
				 << ", gripper at " << gripperPos << "  diff=" << diff << endl;
		if ( diff[0] > -25.0 && diff[0] < 0.75*domino->getLength() && fabs(diff[1]) < domino->getWidth() ) {
			postParentCompletion();
			return;
		}
	} END_ITERATE;
	if ( dominoes.empty() )
		cout << "*** Couldn't find a domino in the gripper." << endl;
	else
		cout << "*** Domino found but not within gripper." << endl;
	postParentSignal<GraspError>(GrasperRequest::badGrasp);
}

void Grasper::Verify::CheckCross::doStart() {
	fmat::Column<3> gripperPos;
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	gripperPos = kine->linkToBase(GripperFrameOffset).translation();
#endif
  NEW_SHAPEVEC(crosses, CrossData, select_type<CrossData>(VRmixin::localShS));
	// ******* SHOULD USE CROSS ARM GRASP POINT, NOT CENTROID *********
	SHAPEVEC_ITERATE(crosses, CrossData, cross) {
		Point targPoint;
		AngTwoPi orient;
		std::vector<Point> graspPoints = cross->computeGraspPoints();
		for (unsigned int i = 0; i < 4; i++){
		  targPoint = graspPoints[i];
		  orient = (targPoint - curReq->object->getCentroid()).atanYX();
		  fmat::Column<3> diff = targPoint.getCoords() - gripperPos;
		  cout << "*** Verify cross grasp point at " << targPoint
				   << ", gripper at " << gripperPos << "  diff=" << diff << endl;
		  if ( diff[0] > -25.0 && diff[0] < 0.75*cross->getArmSemiLength() && fabs(diff[1]) < cross->getArmWidth() ) {
			  postParentCompletion();
			  return;
		  }
		}
	} END_ITERATE;
	if ( crosses.empty() )
		cout << "*** Couldn't find a cross in the gripper." << endl;
	else
		cout << "*** Cross found but not within gripper." << endl;
	postParentSignal<GraspError>(GrasperRequest::badGrasp);
}

void Grasper::Verify::CheckGripperLoad::doStart() {
	if (getAncestor<Verify>()->postGrasp == false) {
		//Note: Please update localspace here.
		postParentCompletion();	//If we're calling this before we've tried to grasp an object,
		return;									// we should exit the node, because we have no load to check
	}	
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
	if(state->sensors[GPSXOffset] != 0.0){
		//If this block is true, we're in Mirage and need to ignore this check
		postParentCompletion();
		return;
	}	
#endif
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	std::cout << "*** CheckGripperLoad: pidduty=" << int(state->pidduties[GripperOffset]*1023)
						<< "  gripPressure=" << curReq->gripPressure << std::endl;
  if (int(state->pidduties[GripperOffset]*1023) > curReq->gripPressure) { // getMC()->getDesiredLoad()
		//If we are supposed to have something in our arms, and we have an invalid load, post failure
		postParentSignal<GraspError>(GrasperRequest::badGrasp);
		return;
  }
#endif

	// If the grasp was valid, post completion
	postParentCompletion();
}

void Grasper::Verify::CheckUserVerify::doStart() {
	if ( (*curReq->verifyGraspFunction)(curReq->object) )
		postParentCompletion();
	else
		postParentSignal<GraspError>(GrasperRequest::badGrasp);
}

//-------------------------------------------------------------------

void Grasper::ArmGrasp::doStart() {
  // should compute gripper position based on object widths
	getMC()->setGripperSpeed(0.5);
  getMC()->requestGripperLoad(curReq->gripPressure);	//Changed as of 4-7-2014 to the new method in ArmMC
}

void Grasper::ArmPulse::doStart() {
	if ( activate )
		getMC()->setGripperPulse(2000,125);
	else
		getMC()->clearGripperPulse();
	postStateCompletion();
}

void Grasper::ArmNudge::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  fmat::Column<3> gpos = kine->getPosition(GripperFrameOffset);
  fmat::Column<3> offset = fmat::pack(0, -100, -30);  // lower the gripper so we can see the AprilTag
  fmat::Column<3> newpos = gpos + offset;
  PostureEngine pe;
  pe.solveLinkPosition(newpos, GripperFrameOffset, fmat::ZERO3);
  getMC()->advanceTime(1000);
  getMC()->setOutputCmd(ArmBaseOffset, pe.getOutputCmd(ArmBaseOffset));
	getMC()->setOutputCmd(ArmShoulderOffset, pe.getOutputCmd(ArmShoulderOffset));
#endif
}

void Grasper::ArmRaise::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  fmat::Column<3> gpos = kine->getPosition(GripperFrameOffset);
  fmat::Column<3> offset = fmat::pack(0, 0, 80);  // raise the gripper so we can transport the object
  fmat::Column<3> newpos = gpos + offset;
  PostureEngine pe;
  pe.solveLinkPosition(newpos, GripperFrameOffset, fmat::ZERO3);
  getMC()->advanceTime(1000);
  getMC()->setOutputCmd(ArmBaseOffset, pe.getOutputCmd(ArmBaseOffset));
	getMC()->setOutputCmd(ArmShoulderOffset, pe.getOutputCmd(ArmShoulderOffset));

  getMC()->advanceTime(2000);		//ArmBaseOffset, ArmShoulderOffset
  getMC()->setOutputCmd(ArmBaseOffset, pe.getOutputCmd(ArmBaseOffset));
	getMC()->setOutputCmd(ArmShoulderOffset, pe.getOutputCmd(ArmShoulderOffset));
#endif
}

void Grasper::PlanBodyTransport::doStart() {
  if ( ! curReq->allowBodyMotion ) {
    postStateFailure();
    return;
  }
  NEW_SHAPE(transportPose, AgentData, new AgentData(VRmixin::theAgent.getData()));
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  const float radius = 250;
#elif defined(TGT_CALLIOPE5)
  const float radius = 540;

#else
  const float radius = 1;
#endif
#ifdef TGT_HAS_ARMS
  fmat::Column<3> baseOffset = kine->getPosition(GripperFrameOffset);
#else
  fmat::Column<3> baseOffset = fmat::ZERO3;
#endif
	curReq->targetLocation->setObstacle(false);
  AngTwoPi orient = std::numeric_limits<float>::quiet_NaN();
  GenericRRTBase::PlannerResult2D result = 
    VRmixin::grasper->planBodyPath(curReq->targetLocation->getCentroid(), orient, baseOffset, transportPose, radius, true);
  if ( result.code != GenericRRTBase::SUCCESS ) {
    switch ( result.code ) {
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Grasper transport planning failed: start state " << result.movingObstacle->toString()
		<< " is in collision with " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Grasper transport planning failed: pose at " << curReq->object
		<< " collides with " << result.collidingObstacle->toString() << ".\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Grasper transport planning failed: no path found after "
		<< curReq->pilotreq.maxRRTIterations << " RRT iterations.\n";
      break;
    default: break;
    }
    transportPose.deleteShape();
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
    return;
  }
  curReq->transportPose = transportPose;
  postStateCompletion();
}

void Grasper::DoBodyTransport::doStart() {
  pilotreq.turnSpeed = 0.25;
  pilotreq.targetShape = curReq->transportPose;
  pilotreq.targetHeading = curReq->transportPose->getOrientation();
  pilotreq.allowBackwardMotion = false; // doesn't handle baseOffset properly?  check this.
}

void Grasper::PlanArmDeliver::doStart() {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	//Changed as of 4-28-2014
	
	//Here, we want to lower our object to the ground before we release
	//Like in PlanArmApproach...

	// Sets shoulder position based on the centroid of the target shape -- 
  // we usually want to be at 3/4 of the height of the target shape.
  fmat::Column<3> target = curReq->object->getCentroid().getCoords();
  if ( curReq->object->getType() == cylinderDataType ) // grasp cylinders at 75% of their height
    target[2] += ShapeRootTypeConst(curReq->object, CylinderData)->getHeight() * 0.25;
  else if ( curReq->object->getType() == naughtDataType ) // grasp naughts, like cylinders, at 75% of their height
    target[2] += ShapeRootTypeConst(curReq->object, NaughtData)->getHeight() * 0.25;
	else if ( curReq->object->getType() == brickDataType || // If the object to grasp is a brick-type object, like a domino...
						curReq->object->getType() == dominoDataType  || curReq->object->getType() == crossDataType )
		target[2] += ShapeRootTypeConst(curReq->object, BrickData)->getCentroid().coordZ() * 0.5;
  // cout << "PlanReachArm: target at: " << target << endl; 
  PostureEngine pe;
  pe.solveLinkPosition(target, GripperFrameOffset, fmat::ZERO3);
  //cout << "PlanReachArm: GripperFrame is at " << pe.getPosition(GripperFrameOffset) << endl;
  NodeValue_t armpoint;
#if defined(TGT_IS_CALLIOPE3)
  armpoint[0] = -1.5;  // arm base
#endif
#if defined(TGT_IS_CALLIOPE2)
  armpoint[0] = 0;  // arm base
#endif
  armpoint[1] = pe.getOutputCmd(ArmShoulderOffset).value;
	//This time, however, we set the deliver path instead of the approach path
  curReq->deliverPath.push_back(armpoint);
	
  postStateCompletion();
#elif defined(TGT_IS_CALLIOPE5)
  // plan arm motion to drop off the object
#endif
}

void Grasper::ReleaseArm::doStart() {
    getMC()->openGripper(0.8);
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
    getMC()->setJointValue(ArmShoulderOffset-ArmOffset, 0);
#endif
	// change the object's location to the gripper's location
#ifdef TGT_HAS_ARMS
	fmat::Column<3> gripperPos = VRmixin::mapBuilder->localToWorldMatrix * 
		kine->baseToLocal() * kine->linkToBase(GripperFrameOffset).translation();
#else
	fmat::Column<3> gripperPos = VRmixin::theAgent->getCentroid().coords;
#endif
	Point objPos = curReq->object->getCentroid();
	objPos.setCoords(gripperPos[0], gripperPos[1], objPos.coordZ());
	curReq->object->setPosition(objPos);
}

void Grasper::DoWithdraw::doStart() {
	pilotreq.dx = -100;
}


//================ OLD STUFF FROM HANDEYE ================

void Grasper::PathPlanConstrained::doStart() {
  std::vector<NodeValue_t> endStates;
  
  if (!curReq->targetLocation.isValid()) {
    postStateSignal<GraspError>(GrasperRequest::someError);
    return;
  }
  
  IKSolver::Point toPt(curReq->object->getCentroid().getCoords());
  VRmixin::grasper->computeGoalStates(toPt, curReq->gripperAngleRangesX, curReq->gripperAngleRangesY, curReq->gripperAngleRangesZ,
                                      curReq->angleResolution, endStates, IKSolver::Point(fmat::ZERO3));
  if (endStates.empty()) {
    std::cout << "PLAN - pickUpUnreachable" << std::endl;
    postStateSignal<GraspError>(GrasperRequest::pickUpUnreachable);
    return;
  }
  
  NodeValue_t startSt;
  if (curReq->approachPath.empty())
    Grasper::getCurrentState(startSt);
  else
    startSt = curReq->approachPath.back();
  
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  ArmPlanner planner(VRmixin::worldShS, worldBounds, curReq->rrtInflation,
		     curReq->effectorOffset, curReq->predicate);
  
  std::vector<NodeType_t> *treeStartResult = NULL;
  std::vector<NodeType_t> *treeEndResult = NULL;
  fmat::Transform t;
  t.translation() = VRmixin::theAgent->getCentroid().coords;
  t.rotation() = fmat::rotationZ(VRmixin::theAgent->getOrientation());
	ArmPlanner::PlannerResult result =
	  planner.planPath(startSt, endStates.front(), curReq->rrtInterpolationStep,
			   t, curReq->rrtMaxIterations,
			   &(curReq->approachPath), treeStartResult, treeEndResult);
  
  switch ( result.code ) {
    case GenericRRTBase::SUCCESS:
      std::cout << "Plan Constrained succeeded" << std::endl;
      postStateCompletion();
      return;
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Navigation path planning failed: start state is in collision.\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Navigation path planning failed: end state is in collision.\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Navigation path planning failed: too many iterations.\n";
      break;
  }
  
  if (result.code == GenericRRTBase::START_COLLIDES || result.code == GenericRRTBase::END_COLLIDES) {
    std::cout << "Obstacle: " << std::endl << "=================" << std::endl << result.movingObstacle->toString() << std::endl;
    std::cout << "collided with: " <<std::endl << "=================" << std::endl << result.collidingObstacle->toString() << std::endl;
    std::cout << "=================" << std::endl;
  }
  
  // Display the tree if requested
  if ( curReq->displayTree && result.code != GenericRRTBase::START_COLLIDES && GenericRRTBase::END_COLLIDES ) {
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
  }
  
  // Display the path if requested
  if ( curReq->displayPath && result.code == GenericRRTBase::SUCCESS ) {
    NEW_SHAPE(plannedPath, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotPath(curReq->approachPath, plannedPath, rgb(0,0,255));
  } 
  
  if (result.code == GenericRRTBase::SUCCESS)
    postStateCompletion();
  else {
    std::cout << "PLANNER FAILURE - CONSTRAINED" << std::endl;
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
  }
}

void Grasper::PathPlanToRest::doStart() {
  NodeValue_t startSt;
  NodeValue_t endSt;
  
  switch(curReq->restType) {
    case GrasperRequest::stationary:
      postStateCompletion();
      return;
    case GrasperRequest::settleArm:
    case GrasperRequest::settleBodyAndArm:
      for(unsigned i = 0; i < NumArmJoints; i++)
        endSt[i] = curReq->armRestState[i];
      break;
  }
  
  if (!curReq->deliverPath.empty())
    startSt = curReq->deliverPath.back();
  else if (!curReq->approachPath.empty())
    startSt = curReq->approachPath.back();
  else
    Grasper::getCurrentState(startSt);
  
  GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
  ArmPlanner planner(VRmixin::worldShS, worldBounds, curReq->rrtInflation, curReq->effectorOffset, NULL);
  
  std::vector<NodeType_t> *treeStartResult = NULL;
  std::vector<NodeType_t> *treeEndResult = NULL;
  fmat::Transform t;
  t.translation() = VRmixin::theAgent->getCentroid().coords;
  t.rotation() = fmat::rotationZ(VRmixin::theAgent->getOrientation());
  ArmPlanner::PlannerResult result =
    planner.planPath(startSt, endSt, curReq->rrtInterpolationStep,
		     t, curReq->rrtMaxIterations,
		     &(curReq->releasePath), treeStartResult, treeEndResult);
  switch ( result.code ) {
    case GenericRRTBase::SUCCESS:
      std::cout << "Plan disengage succeeded" << std::endl;
      postStateCompletion();
      return;
    case GenericRRTBase::START_COLLIDES:
      std::cout << "Navigation path planning failed: start state is in collision.\n";
      break;
    case GenericRRTBase::END_COLLIDES:
      std::cout << "Navigation path planning failed: end state is in collision.\n";
      break;
    case GenericRRTBase::MAX_ITER:
      std::cout << "Navigation path planning failed: too many iterations.\n";
      break;
  }
  
  if (result.code == GenericRRTBase::START_COLLIDES || result.code == GenericRRTBase::END_COLLIDES) {
    std::cout << "Obstacle: " << std::endl << "=================" << std::endl << result.movingObstacle->toString() << std::endl;
    std::cout << "collided with: " <<std::endl << "=================" << std::endl << result.collidingObstacle->toString() << std::endl;
    std::cout << "=================" << std::endl;
  }
  
  // Display the tree if requested
  if ( curReq->displayTree && result.code != GenericRRTBase::START_COLLIDES && GenericRRTBase::END_COLLIDES ) {
    NEW_SHAPE(plannerTrees, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotTree(*treeStartResult, plannerTrees, rgb(0,0,0));
    planner.plotTree(*treeEndResult, plannerTrees, rgb(0,192,0));
    delete treeStartResult;
    delete treeEndResult;
  }
  
  // Display the path if requested
  if ( curReq->displayPath && result.code == GenericRRTBase::SUCCESS ) {
    NEW_SHAPE(plannedPath, GraphicsData, new GraphicsData(VRmixin::worldShS));
    planner.plotPath(curReq->releasePath, plannedPath, rgb(0,0,255));
  } 
  
  if (result.code == GenericRRTBase::SUCCESS)
    postStateCompletion();
  else {
    std::cout << "PLANNER FAILURE - ARM_REST" << std::endl;
    postStateSignal<GraspError>(GrasperRequest::noGraspPath);
  }
}

void Grasper::MoveArm::doStart() {
  switch(pa) {
    case GrasperRequest::doApproach:
      if ( curReq->verbosity & GVexecutePath )
        std::cout << "Grasper: executing arm reach path" << std::endl;
      executeMove(curReq->approachPath);
      break;
    case GrasperRequest::doDeliver:
      if ( curReq->verbosity & GVexecutePath )
        std::cout << "Grasper: moving along constrained path" << std::endl;
      executeMove(curReq->deliverPath);
      break;
    case GrasperRequest::doRelease:
      if ( curReq->verbosity & GVexecutePath )
        std::cout << "Grasper: executing arm release path" << std::endl;
      executeMove(curReq->releasePath);
      break;
    case GrasperRequest::noPath:
      break;
  }
}

void Grasper::MoveArm::executeMove(const std::vector<NodeValue_t>& path) {
#ifdef TGT_HAS_ARMS
  MMAccessor<DynamicMotionSequence> move_acc = getMC();
  move_acc->clear();
  move_acc->setTime(1000);
  for(unsigned int p = 0; p < path.size(); p++) {
    for(unsigned int j = 0; j < numPlannerJoints; j++) {
      if (kine->getKinematicJoint(ArmOffset+j)->isMobile() == false)
        continue;
      move_acc->setOutputCmd(ArmOffset+j, (float)path[p][j]);
      //cout << "MoveArm path[" << p << "][" << j << "] = " << path[p][j] << endl;
    }
    if (p < path.size()-1) {
      NodeValue_t confDif;
      for(unsigned i = 0; i < numPlannerJoints; i++)
        confDif[i] = path[p][i] - path[p+1][i];
      // std::cout << "advTime = " << advTime(confDif) << std::endl;
      move_acc->advanceTime( advTime(confDif) );
    }
  }
  
  move_acc->play();
#endif
}

void Grasper::SetJoint::moveJoint(float value) {
  unsigned int offset = 10;
  if(jointName == "ARM:shldr")
    offset = 0;
  else if(jointName == "ARM:elbow")
    offset = 1;
  else if(jointName == "ARM:wristYaw")
    offset = 2;
  else if(RobotInfo::NumArmJoints > 3) {
    if(jointName == "ARM:wristPitch")
      offset = 3;
    else if(jointName == "ARM:wristRoll" || jointName == "ARM:gripperLeft")
      offset = 4;
    else if(jointName == "ARM:gripper" || jointName == "ARM:gripperRight")
      offset = 5;
    else {
      postStateCompletion();
      return;
    }
  }
  else {
    postStateCompletion();
    return;
  }
  if ( (curReq ? curReq->verbosity : verbosity) & GVsetJoint )
    cout << "Setting " << jointName << " to " << value << endl;
  getMC()->takeSnapshot();
  getMC()->setMaxSpeed(offset, speed); 	 
  getMC()->setJointValue(offset, value);
}

void Grasper::getCurrentState(NodeValue_t &current, KinematicJoint* endEffector) {
  const KinematicJoint* joint = !endEffector ? kine->getKinematicJoint(curReq->effectorOffset) : endEffector;
  for (unsigned int j = numPlannerJoints; j > 0; j--) {
    while (joint->isMobile() == false) joint = joint->getParent();
    current[j-1] = joint->getQ();
    joint = joint->getParent();
  }
}

void Grasper::computeGoalStates(IKSolver::Point &toPt,
				std::vector<std::pair<float, float> > &rangesX,
                                std::vector<std::pair<float, float> > &rangesY,
				std::vector<std::pair<float, float> > &rangesZ,
                                float resolution, std::vector<NodeValue_t>& goals, const IKSolver::Point &offset) {
  // std::cout << "computGoalStates " << toPt << " resolution=" << resolution << std::endl;
  KinematicJoint* effector = kine->getKinematicJoint(curReq->effectorOffset)->cloneBranch();
  goals.clear();
  if (resolution == 0)
    resolution = M_PI/2;
  
  if (rangesX.empty())
    rangesX.push_back(pair<float, float>(0, 2*M_PI));
  if (rangesY.empty())
    rangesY.push_back(pair<float, float>(0, 2*M_PI));
  if (rangesZ.empty())
    rangesZ.push_back(pair<float, float>(0, 2*M_PI));
  
  int factorX = 0;
  bool keepGoing;
  do {
    keepGoing = false;
    for(unsigned rX = 0; rX < rangesX.size(); rX++) {
      float thetaX = resolution * factorX;
      float midX = (rangesX[rX].first + rangesX[rX].second)/2;
      if (thetaX > rangesX[rX].second - midX)
        continue;

      int factorY = 0;
      for (unsigned rY = 0; rY < rangesY.size(); rY++) {
        float thetaY = resolution * factorY;
        float midY = (rangesY[rY].first + rangesY[rY].second)/2;
        if (thetaY > rangesY[rY].second - midY)
          continue;

        int factorZ = 0;
        for (unsigned rZ = 0; rZ < rangesZ.size(); rZ++) {
          float thetaZ = resolution * factorZ;
          float midZ = (rangesZ[rZ].first + rangesZ[rZ].second)/2;
          if (thetaZ > rangesZ[rZ].second - midZ)
            continue;
          
          keepGoing = true;
          fmat::Quaternion oriPlus = fmat::Quaternion::fromMatrix(fmat::rotationZ(midZ + thetaZ) * 
								  fmat::rotationY(midY + thetaY) *
								  fmat::rotationX(midX + thetaX));
	  checkGoalCandidate(offset, IKSolver::Rotation(oriPlus), effector, toPt, goals);
	  if ( factorX > 0 ||  factorY > 0 || factorZ > 0 ) {
	    fmat::Quaternion oriMinus = fmat::Quaternion::fromMatrix(fmat::rotationZ(midZ - thetaZ) *
								     fmat::rotationY(midY - thetaY) * 
								     fmat::rotationX(midX - thetaX));
	    checkGoalCandidate(offset, IKSolver::Rotation(oriMinus), effector, toPt, goals);
	  }

          if ( goals.size() >= curReq->maxNumberOfAngles )
            keepGoing = false;
          factorZ++;
        } // Z
        factorY++;
      } // Y
      factorX++;
    } // X
  } while (keepGoing);
  if ( curReq->verbosity & GVcomputeGoals )
    std::cout << "Grasper found " << goals.size() << " potential goal states." << std::endl;
}

void Grasper::checkGoalCandidate(const IKSolver::Point &offset, const IKSolver::Rotation &ori, 
				 KinematicJoint *effector, const IKSolver::Point &position, std::vector<NodeValue_t>& goals) {
  IKSolver& solver = effector->getIK();
  float const positionMostImportant = 1.0f;
  float const orientationLeastImportant = 0.5f;
  bool reached = solver.solve(offset, IKSolver::Rotation(fmat::Quaternion::aboutX(-M_PI/2)), *effector,
			      position, positionMostImportant, IKSolver::Parallel(0,0,1)/*ori*/, orientationLeastImportant);
  std::cout << "checkGoalCandidate: reached = " << reached << std::endl;
  if (true||reached) {
    NodeValue_t endSt;
    KinematicJoint *joint = effector;
    for (int j = numPlannerJoints-1; j >= 0; j--) {
      while (joint->isMobile() == false)
	joint = joint->getParent();
      endSt[j] = joint->getQ();
      joint = joint->getParent();
    }
    switch ( curReq->graspStrategy ) {
    case GrasperRequest::unconstrainedGrasp:
      break;
    case GrasperRequest::sideGrasp:
#ifdef TGT_IS_CALLIOPE5
      //      endSt[WristRotateOffset-ArmOffset] = M_PI/2 * (endSt[WristRotateOffset-ArmOffset] > 0 ? 1 : -1 );
      std:: cout << "wristrot " << WristRotateOffset << " value " << endSt[WristRotateOffset-ArmOffset] << std::endl;
#endif
      break;
    case GrasperRequest::overheadGrasp:
#ifdef TGT_IS_CALLIOPE5
      // Set wrist joint to complement elbow and shoulder joints,
      // making fingers perpendicular to the ground to the extent this
      // is compatible with the joint limits.
      // *** THIS IS WRONG *** It changes the position of GripperFrame; not a legal solution.
      float wristDesired = -M_PI - endSt[ArmShoulderOffset-ArmOffset] - endSt[ArmElbowOffset-ArmOffset];
      float wristAchievable = std::max(wristDesired, outputRanges[ArmWristOffset][0]);
      endSt[ArmWristOffset-ArmOffset] = wristAchievable;
#endif
      break;
    }
    goals.push_back(endSt);
  }
}

unsigned int Grasper::executeRequest(const GrasperRequest& req, BehaviorBase* requestingBehavior) {
	VRmixin::requireCrew("Grasper");
  GrasperRequest *newReq = new GrasperRequest(req);
  newReq->requestingBehavior = requestingBehavior;
  unsigned int reqID = ++idCounter;
  newReq->requestID = reqID;
  GraspError e = newReq->validateRequest();
  if ( e != GrasperRequest::noError ) {
    std::cout << "*** Grasper received invalid grasp request, id=" << reqID << std::endl;
    GrasperEvent myEvent(false, newReq->requestType, e, (size_t)newReq->requestingBehavior);
    erouter->postEvent(myEvent);
  }
  else {
    requests.push(newReq);
    executeRequest();
  }
  return reqID;
}

void Grasper::executeRequest() {
  if ( curReq != NULL )
    return;
  else if ( requests.empty() )
    return;
  curReq = requests.front();
  requests.pop();
  
  curReq->verbosity = (verbosity & ~curReq->clearVerbosity) | curReq->setVerbosity;
  if ( curReq->verbosity & GVexecuteRequest ) {
    std::cout << "Grasper now executing request " << curReq->requestID;
		if ( curReq->object.isValid() )
			std::cout << " object=" << curReq->object;
		if ( curReq->targetLocation.isValid() )
			std::cout << " targetLocation=" << curReq->targetLocation
								<< " : " << curReq->targetLocation->getCentroid();
		std::cout << std::endl;
  }
  /*
  if (curReq->object.isValid() && curReq->object->getSpace().getRefFrameType() == VRmixin::worldShS.getRefFrameType())
    curReq->object = VRmixin::mapBuilder->importWorldToLocal(curReq->object);
  
  if (curReq->targetLocation.isValid() && curReq->targetLocation->getSpace().getRefFrameType() == VRmixin::worldShS.getRefFrameType())
    curReq->targetLocation = VRmixin::mapBuilder->importWorldToLocal(curReq->targetLocation);
  */
  {
    GET_SHAPE(approachPose, AgentData, VRmixin::worldShS);
    GET_SHAPE(transportPose, AgentData, VRmixin::worldShS);
    GET_SHAPE(withdrawPose, AgentData, VRmixin::worldShS);
    GET_SHAPE(plannerTrees, GraphicsData, VRmixin::worldShS);
    approachPose.deleteShape();
    transportPose.deleteShape();
    withdrawPose.deleteShape();
		plannerTrees.deleteShape();
  }

  dispatch();
}

void Grasper::dispatch() {
  startmain_->start();
  /*
  switch( curReq->requestType ) {
    case GrasperRequest::reach:
      reachRoot->start();
      break;
    case GrasperRequest::computeReach:
      if ( curReq->populateEventPathWith == GrasperRequest::noPath )
        curReq->populateEventPathWith = GrasperRequest::moveFree;
      computeReachRoot->start();
      break;
    case GrasperRequest::grasp:
      graspRoot->start();
      break;
    case GrasperRequest::checkGraspable:
      checkRoot->start();
      break;
    case GrasperRequest::release:
      releaseRoot->start();
      break;
    case GrasperRequest::touch:
      touchRoot->start();
      break;
    case GrasperRequest::moveTo:
      moveRoot->start();
      break;
    case GrasperRequest::checkMovable:
      checkRoot->start();
      break;
    case GrasperRequest::rest:
      restRoot->start();
      break;
    case GrasperRequest::checkRestable:
      checkRoot->start();
      break;
    case GrasperRequest::sweep:
      std::cout << "Grasper functionality " << curReq->requestType << " not implemented"<< std::endl;
      break;
    default:
      std::cout << "Unidentified requestType" << std::endl;
      break;
  }
  */
}

void Grasper::doStop() {
  std::cout << "Stopping Grasper sub-machines" << std::endl;
  startmain_->stop();
  motman->removeMotion(armId);
  armId = MotionManager::invalid_MC_ID;
}

#endif // TGT_HAS_ARMS
