#include "ShapeSpacePlannerBase.h"

template <>
void ShapeSpaceCollisionCheckerBase<2>::addRobotObstacles(const KinematicJoint& j) {
  for (KinematicJoint::branch_iterator it = j.getBranches().begin(); it != j.getBranches().end(); ++it)
    addRobotObstacles(**it);

  // for 2D obstacles, to avoid projection geometry, we'll try modeling all obstacles as rectangles.
  RectangularObstacle* rect = NULL;
  if (j.collisionModel == "Cube" || j.collisionModel == "Cylinder" || j.collisionModel == "Sphere") {
    rect = new RectangularObstacle;
		j.getOwnBB2D(j.getFullT(), *rect);
    rect->setBodyObstacle();
    rect->name = j.outputOffset.get();
    if ( rect->name.size() == 0 )
      rect->name = j.model.get();
    obstacles.push_back(rect);
  }
  for (KinematicJoint::component_iterator it = j.components.begin(); it != j.components.end(); ++it) {
    // for 2D obstacles, to avoid projection geometry, we'll try modeling all obstacles as rectangles.
    if ((*it)->collisionModel == "Cube" || (*it)->collisionModel == "Cylinder" || (*it)->collisionModel == "Sphere") {
      rect = new RectangularObstacle;
      (*it)->getOwnBB2D(j.getFullT(), *rect);
      rect->setBodyObstacle();
      rect->name = (*it)->model.get();
      if ( rect->name.size() == 0 )
				rect->name = j.outputOffset.get() + "-component";
      obstacles.push_back(rect);
    }
  }
}

template <>
void ShapeSpaceCollisionCheckerBase<3>::createBodyObstacle(const LinkComponent& j, const fmat::Transform& robotT) {
  // std::cout << "ShapeSpacePlannerBase::createBodyObstacle '" << j.collisionModel << "' for '" << j.model << "'" << std::endl;
  fmat::Transform t;
  j.getCollisionModelTransform(t);
  t = robotT * t;
  fmat::Column<3> scale;
  j.collisionModelScale.exportTo(scale);
	
  // create respective obstacle
  if (j.collisionModel == "Cube") {
    BoxObstacle* box = new BoxObstacle;
    j.getOwnBB3D(robotT, *box);
    box->setBodyObstacle();
    box->name = j.model.get();
    obstacles.push_back(box);
  }
  else if (j.collisionModel == "Cylinder") {
    fmat::Matrix<3,3> rot = t.rotation();
    fmat::Column<3> pos = t.translation();
    CylindricalObstacle* c = new CylindricalObstacle(pos, rot, std::max(scale[0], scale[1])/2, scale[2]/2);
    c->setBodyObstacle();
    c->name = j.model.get();
    obstacles.push_back(c);
  }
  else if (j.collisionModel == "Sphere") {
    fmat::Matrix<3,3> rot = t.rotation();
    EllipsoidObstacle* e = new EllipsoidObstacle(t.translation(), rot, scale);
    e->setBodyObstacle();
    e->name = j.model.get();
    obstacles.push_back(e);
  }
  else if ( j.collisionModel.size() > 0 )
    std::cout << "ShapeSpacePlannerBase::createBodyObstacle can't handle collision model '" 
	      << j.collisionModel << "' for model '" << j.model << "'" << std::endl;
}

template <>
void ShapeSpaceCollisionCheckerBase<3>::addRobotObstacles(const KinematicJoint& j) {
  // **************** THIS IS A STUB FUNCTION : MOST OF THE CODE IS MISSING ****************
  for (KinematicJoint::branch_iterator it = j.getBranches().begin(); it != j.getBranches().end(); ++it) {
    addRobotObstacles(**it);
  }
  createBodyObstacle(j, j.getFullT());
  for (KinematicJoint::component_iterator it = j.components.begin(); it != j.components.end(); ++it)
    createBodyObstacle(**it, j.getFullT());
}

template <>
void ShapeSpaceCollisionCheckerBase<2>::addDisplayRobotObstacles(const KinematicJoint& j) {
  for (KinematicJoint::branch_iterator it = j.getBranches().begin(); it != j.getBranches().end(); ++it) {
    addDisplayRobotObstacles(**it);
  }
	
  fmat::Transform worldT;
  DualCoding::Point p = DualCoding::VRmixin::robotObstaclesPt;
  worldT.translation() = fmat::pack(p.coordX(),p.coordY(),p.coordZ());
  worldT.rotation() = fmat::rotationZ(DualCoding::VRmixin::robotObstaclesOri);
  //std::cout << "Display robot obstacles, use pos: " << p << ", ori: " << DualCoding::VRmixin::robotObstaclesOri << std::endl;

  // for 2D obstacles, to avoid projection geometry, we'll try modeling all obstacles as rectangles.
  RectangularObstacle* rect = NULL;
  if (j.collisionModel == "Cube" || j.collisionModel == "Cylinder" || j.collisionModel == "Sphere") {
    rect = new RectangularObstacle;
    j.getOwnBB2D(worldT * j.getFullT(), *rect);
    rect->setBodyObstacle();
    rect->name = j.outputOffset.get();
    if ( rect->name.size() == 0 )
      rect->name = j.model.get();
    displayRobotObstacles.push_back(rect);
  }
  for (KinematicJoint::component_iterator it = j.components.begin(); it != j.components.end(); ++it) {
    // for 2D obstacles, to avoid projection geometry, we'll try modeling all obstacles as rectangles.
    if ((*it)->collisionModel == "Cube" || (*it)->collisionModel == "Cylinder" || (*it)->collisionModel == "Sphere") {
      rect = new RectangularObstacle;
      (*it)->getOwnBB2D(worldT * j.getFullT(), *rect);
      rect->setBodyObstacle();
      rect->name = (*it)->model.get();
      if ( rect->name.size() == 0 )
	rect->name = j.outputOffset.get() + "-component";
      displayRobotObstacles.push_back(rect);
    }
  }
}

template <>
void ShapeSpaceCollisionCheckerBase<3>::addDisplayRobotObstacles(const KinematicJoint& j) {
  for (KinematicJoint::branch_iterator it = j.getBranches().begin(); it != j.getBranches().end(); ++it)
    addDisplayRobotObstacles(**it);
}

template <>
void ShapeSpaceCollisionCheckerBase<2>::addObstaclesToShapeSpace(ShapeSpace &shs, const fmat::Transform &t) {
  {
    GET_SHAPE(plannerObstacles, GraphicsData, shs);
    plannerObstacles.deleteShape();
  }
  NEW_SHAPE(plannerObstacles, GraphicsData, new GraphicsData(shs));
  typedef std::pair<float,float> xyPair;
  rgb obstacleColor(200, 200, 0);
  for (unsigned int i = 0; i < displayWorldObstacles.size(); i++) {
    { RectangularObstacle *r = dynamic_cast<RectangularObstacle*>(displayWorldObstacles[i]);
      if (r) {
	std::vector<xyPair> pts;
	for (int j = 0; j < RectangularObstacle::NUM_CORNERS; j++) {
	  fmat::Column<2> p = r->getCorner(static_cast<RectangularObstacle::CornerOrder>(j));
	  pts.push_back(xyPair(p[0],p[1]));
	}
	plannerObstacles->add(new GraphicsData::PolygonElement("plannerObstacle-"+r->name, pts, true, obstacleColor));
	continue;
      }
    }
    { CircularObstacle *c = dynamic_cast<CircularObstacle*>(displayWorldObstacles[i]);
      if (c) {
	fmat::Column<2> center = c->getCenter();
	float radius = c->getRadius();
	plannerObstacles->add(new GraphicsData::EllipseElement("plannerObstacle-"+c->name, xyPair(center[0],center[1]), 
							       radius, radius, 0, false, 
							       obstacleColor));
	continue;
      }
    }
    { EllipticalObstacle *e = dynamic_cast<EllipticalObstacle*>(displayWorldObstacles[i]);
      if (e) {
	fmat::Column<2> c = e->getCenter();
	fmat::Column<2> vec = e->focus1 - c;
	AngTwoPi orientation = AngTwoPi(std::atan2(vec[1],vec[0]));
	plannerObstacles->add(new GraphicsData::EllipseElement("plannerObstacle-"+e->name, xyPair(c[0],c[1]), 
							       e->semimajor, e->semiminor, orientation, false, 
							       obstacleColor));
	continue;
      }
    }
    { ConvexPolyObstacle *cp = dynamic_cast<ConvexPolyObstacle*>(displayWorldObstacles[i]);
      if ( cp ) {
	std::vector<xyPair> pts;
	pts.reserve(cp->getPoints().size());
	for (size_t p = 0; p < cp->getPoints().size(); p++)
	  pts[p] = xyPair(cp->getPoints()[p][0], cp->getPoints()[p][1]);
	plannerObstacles->add(new GraphicsData::PolygonElement("plannerObstacle-"+cp->name, pts, true, obstacleColor));
	continue;
      }
    }
    std::cout << "Unhandled obstacle type " << *displayWorldObstacles[i] << std::endl;
  }
  
  // now add the robot's components
  addDisplayRobotObstacles(*(kine->getKinematicJoint(BaseFrameOffset)));
  for (unsigned int i = 0; i < displayRobotObstacles.size(); i++) {
    { EllipticalObstacle *e = dynamic_cast<EllipticalObstacle*>(displayRobotObstacles[i]);
      if (e) {
	fmat::Column<2> c = e->getCenter();
	fmat::Column<2> vec = e->focus1 - c;
	AngTwoPi orientation = AngTwoPi(std::atan2(vec[1],vec[0]));
	plannerObstacles->
	  add(new GraphicsData::EllipseElement("plannerObstacle-"+e->name, 
					       xyPair(c[0],c[1]), e->semimajor, e->semiminor, 
					       orientation, false, rgb(150, 0, 255)));
	continue;
      }
    }
    
    { RectangularObstacle *r = dynamic_cast<RectangularObstacle*>(displayRobotObstacles[i]);
      if (r) {
	std::vector<xyPair> pts;
	for (int j = 0; j < RectangularObstacle::NUM_CORNERS; j++) {
	  fmat::Column<2> p = r->getCorner(static_cast<RectangularObstacle::CornerOrder>(j));
	  pts.push_back(xyPair(p[0],p[1]));
	}
	plannerObstacles->add(new GraphicsData::PolygonElement("plannerObstacle-"+r->name, pts, true, rgb(150, 0, 255)));
	continue;
      }
    }
  }
  plannerObstacles->setObstacle(false);
}

template <>
void ShapeSpaceCollisionCheckerBase<3>::addObstaclesToShapeSpace(ShapeSpace &shs, const fmat::Transform &t) {
  {
    GET_SHAPE(plannerObstacles, GraphicsData, shs);
    plannerObstacles.deleteShape();
  }
  NEW_SHAPE(plannerObstacles, GraphicsData, new GraphicsData(shs));
  rgb obstacleColor(200, 200, 0);
  typedef std::pair<float,float> xyPair;
  for (unsigned int i = 0; i < displayWorldObstacles.size(); i++) {
    { CylindricalObstacle *c = dynamic_cast<CylindricalObstacle*>(displayWorldObstacles[i]);
      if (c) {
	fmat::Column<3> center = t * c->getCenter();
	float radius = c->getRadius();
	plannerObstacles->add(new GraphicsData::EllipseElement("plannerObstacle-"+c->name, xyPair(center[0],center[1]), 
							       radius, radius, 0, false, 
							       obstacleColor));
	continue;
      }
    }
    std::cout << "Unhandled obstacle type " << *displayWorldObstacles[i] << std::endl;
  }
}
