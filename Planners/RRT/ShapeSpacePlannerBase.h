//-*-c++-*-
#ifndef _SHAPE_SPACE_PLANNER_BASE_H_
#define _SHAPE_SPACE_PLANNER_BASE_H_

#include "Motion/KinematicJoint.h"
#include "Motion/Kinematics.h"
#include "Planners/PlannerObstacles.h"
#include "DualCoding/DualCoding.h"
#include "DualCoding/ShapeCylinder.h"
#include "GenericRRT.h"

//================ ShapeSpaceCollisionCheckerBase ================

//! Base class for doing collision checking in world shape space
template <size_t N>
class ShapeSpaceCollisionCheckerBase {
protected:
  //! world bounds, must be closed to be used
  DualCoding::Shape<DualCoding::PolygonData> worldBounds;
  
  float const inflation;  //!< Amount in mm to add to obstacle bounding shape
  
  //! world or local map obstacles
  std::vector<PlannerObstacle<N>*> obstacles;
  
  //! world map obstacles for display
  std::vector<PlannerObstacle<N>*> displayWorldObstacles;

  //! robot obstacles for display
  std::vector<PlannerObstacle<N>*> displayRobotObstacles;
  
  //! Model the robot as a set of PlannerObstacles, based on its kinematic structure.
  void addRobotObstacles(const KinematicJoint &j);

  void createBodyObstacle(const LinkComponent& j, const fmat::Transform& robotT);
  
  void addDisplayRobotObstacles(const KinematicJoint &j);
  
public:
  ShapeSpaceCollisionCheckerBase(ShapeSpace &shs,
                                 const Shape<PolygonData> &_worldBounds,
                                 float _inflation);
  
  virtual ~ShapeSpaceCollisionCheckerBase();
  
  const Shape<PolygonData> getWorldBounds() const { return worldBounds; }
  
  BoundingBox<N> getObstacleBoundingBox() const;
  BoundingBox<N> getBodyBoundingBox() const;
  
  //! Debugging tool to make obstacles visible
  void addObstaclesToShapeSpace(DualCoding::ShapeSpace & shs, const fmat::Transform &t=fmat::Transform());
};

template <size_t N>
BoundingBox<N> ShapeSpaceCollisionCheckerBase<N>::getObstacleBoundingBox() const {
  if (obstacles.empty()) return BoundingBox<N>();
  
  BoundingBox<N> bounds = obstacles.front()->getBoundingBox();
  
  for (unsigned int i = 0; i < obstacles.size(); i++) {
    if (!obstacles[i]->isBodyObstacle())
      bounds.expand(obstacles[i]->getBoundingBox());
  }
  return bounds;
}

template <size_t N>
BoundingBox<N> ShapeSpaceCollisionCheckerBase<N>::getBodyBoundingBox() const {
  if (obstacles.empty()) return BoundingBox<N>();
  
  BoundingBox<N> bounds = obstacles.front()->getBoundingBox();
  
  for (unsigned int i = 0; i < obstacles.size(); i++) {
    if (obstacles[i]->isBodyObstacle())
      bounds.expand(obstacles[i]->getBoundingBox());
  }
  return bounds;
}

template <size_t N>
ShapeSpaceCollisionCheckerBase<N>::ShapeSpaceCollisionCheckerBase(ShapeSpace &shs,
                                                                  const Shape<PolygonData> &_worldBounds,
                                                                  float _inflation) :
	worldBounds(_worldBounds), inflation(_inflation), obstacles(),
	displayWorldObstacles(), displayRobotObstacles() {
  SHAPEROOTVEC_ITERATE(shs, s) {
    if ( s->getId() != DualCoding::VRmixin::theAgent->getId() && s->isObstacle() )
      PlannerObstacle<N>::convertShapeToPlannerObstacle(s, inflation, obstacles);
  } END_ITERATE;
  displayWorldObstacles = obstacles;
  
  addRobotObstacles(*(kine->getKinematicJoint(BaseFrameOffset)));
  
  DualCoding::Point location = VRmixin::theAgent->getCentroid();
}

template <size_t N>
ShapeSpaceCollisionCheckerBase<N>::~ShapeSpaceCollisionCheckerBase() {
  for (unsigned int i = 0; i < obstacles.size(); i++)
    delete obstacles[i];
  // displayWorldObstacles[] points to a subset of obstacles[] so
  // we've already deleted its elements
  for (unsigned int i = 0; i < displayRobotObstacles.size(); i++)
    delete displayRobotObstacles[i];
}

template <>
void ShapeSpaceCollisionCheckerBase<2>::addRobotObstacles(const KinematicJoint& j);

template <>
void ShapeSpaceCollisionCheckerBase<3>::addRobotObstacles(const KinematicJoint& j);

template <>
void ShapeSpaceCollisionCheckerBase<3>::createBodyObstacle(const LinkComponent& j, const fmat::Transform& robotT);

template <>
void ShapeSpaceCollisionCheckerBase<2>::addDisplayRobotObstacles(const KinematicJoint& j);

template <>
void ShapeSpaceCollisionCheckerBase<3>::addDisplayRobotObstacles(const KinematicJoint& j);

template <>
void ShapeSpaceCollisionCheckerBase<2>::addObstaclesToShapeSpace(ShapeSpace &shs, const fmat::Transform &t);

template <>
void ShapeSpaceCollisionCheckerBase<3>::addObstaclesToShapeSpace(ShapeSpace &shs, const fmat::Transform &t);

#endif
