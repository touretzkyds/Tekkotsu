//-*-c++-*-
#if defined(TGT_HAS_ARMS) || defined(TGT_MANTIS)

#ifndef _SHAPE_SPACE_PLANNER_3DR_H_
#define _SHAPE_SPACE_PLANNER_3DR_H_

#include "Motion/Kinematics.h"
#include "Planners/RRT/ShapeSpacePlannerBase.h"

//================ RRTNode3DR ================

template <size_t N>
class ShapeSpacePlanner3DR;

template <size_t N>
class RRTNode3DR : public RRTNodeBase {
public:
	
  class NodeValueWrapper {
  private:
    AngSignPi angles[(N==0)?1:N]; // funny array size to help older compilers that error on zero-sized arrays
  public:
    NodeValueWrapper() : angles() {}
		AngSignPi& operator[](int i)       { return angles[i]; }
		AngSignPi  operator[](int i) const { return angles[i]; }
  };
	
  typedef NodeValueWrapper NodeValue_t;
  typedef typename GenericRRTBase::PlannerResult<3> PlannerResult;
  NodeValue_t q;
	
  //! Constructor
  RRTNode3DR(const NodeValue_t &_q, unsigned int _parent) : RRTNodeBase(_parent), q(_q) {};

	//! Copy constructor
	RRTNode3DR(const RRTNode3DR &other) : RRTNodeBase(other), q(other.q) {}

	//! Assignment operator
	RRTNode3DR& operator=(const RRTNode3DR &other) {
		RRTNodeBase::operator=(other);
		q = other.q;
		return *this;
	}
	
	class CollisionChecker : public ShapeSpaceCollisionCheckerBase<3> {
  protected:
    KinematicJoint* rootJ;
  public:
    fmat::Transform worldT;
  protected:
		
		//! Coarse box-type obstacle representing a joint and all its components.
		/*! If this obstacle does not collide, we needn't look any
			further.  If it does collide, then we have to look at the individual
			components and their actual shapes to check if a collision really occurred.
		 */
    class JointObstacle {
    public:
      BoxObstacle obstacle;
      KinematicJoint* joint;
      bool valid;
			
      JointObstacle() : obstacle(), joint(), valid() {}
			
			JointObstacle(KinematicJoint* _joint) : obstacle(), joint(_joint), valid(false) {
				obstacle.name = joint->outputOffset.get();
			}
			
			JointObstacle(const BoxObstacle& _obstacle, KinematicJoint* _joint, bool _valid) :
				obstacle(_obstacle), joint(_joint), valid(_valid) {}
			
			virtual ~JointObstacle() {}
			
	    JointObstacle(const JointObstacle& other) : obstacle(other.obstacle), joint(other.joint), valid(other.valid) {}

			JointObstacle& operator=(const JointObstacle& other) {
				obstacle = other.obstacle;
				joint = other.joint;
				valid = other.valid;
				return *this;
			}
			
			bool collides(const JointObstacle& other) const { if (!other.valid) return false; return false; }

			bool collides(const PlannerObstacle3D& obs) const { return obs.collides(obstacle); }
			
			virtual bool setupObstacle(const fmat::Transform& t, bool includeChildren);
    };
		
		//! An obstacle representing the actual shape of one robot component.
		class LinkObstacle : public JointObstacle {
    public:
      LinkComponent* link;
			
      LinkObstacle() : JointObstacle(), link() {}
			
			LinkObstacle(const JointObstacle& jObs) : JointObstacle(jObs.joint), link() {
				this->obstacle.name += "Component";
			}
			
			virtual ~LinkObstacle() {}
			
			void setLink(LinkComponent* _link) { link = _link; }
			
			LinkObstacle(const LinkObstacle& other) : JointObstacle(other), link(other.link) {}

			LinkObstacle& operator=(const LinkObstacle& other) {
				JointObstacle::operator=(other);
	      link = other.link;
	      return *this;
			}
			
			virtual bool setupObstacle(const fmat::Transform& t, bool includeChildren); 
    };
		
  public:
    CollisionChecker(DualCoding::ShapeSpace & shs,
                     const DualCoding::Shape<DualCoding::PolygonData> &_worldBounds,
                     float _inflation,
                     unsigned int effectorOffset) :
      ShapeSpaceCollisionCheckerBase<3>(shs, _worldBounds, _inflation), rootJ(), worldT() {
      rootJ = kine->getKinematicJoint(effectorOffset)->cloneBranch();
      for (unsigned int i = 0; i < N; i++)
				rootJ = rootJ->getNextMobileAncestor();
      
      /* For the 3D arm planner, we have an arm that
       * has the potential to go beneath the ground.
       * Therefore, we add an obstacle that models
       * a large cube, 5m in length, width, and height
       * directly beneath the robot's position.
       * This ensures that no link in the arm will travel underground,
       * and potentially cause the robot to lift
       * itself off of the ground, which can be problematic. */
      const int size = 2500;
      fmat::Column<3> pos = worldT.translation();
      pos[2] -= float(2500);
      fmat::Matrix<3,3> rot = worldT.rotation();
      BoxObstacle* bo = new BoxObstacle(pos, fmat::pack(size,size,size), rot);
      bo->name = "GroundObstacle";
      obstacles.push_back(bo);
    }
		
		CollisionChecker(const CollisionChecker& other) : rootJ(other.rootJ) {}
	
		CollisionChecker operator=(const CollisionChecker& other) {
			rootJ = other.rootJ;
			return *this;
		}
	
		virtual bool collides(const NodeValue_t &qnew, PlannerResult* result=NULL) const;
	
		template<class T, class U>
	  static bool checkComponent(std::vector<T>& full, const U& newObs, T& collisionObs);

  };

  static const unsigned int maxInterpolations = 100; //!< Maximum number of interpolation steps in interpolate() when @a truncate is true
	
  virtual float distance(const NodeValue_t &target);
  static void generateSample(const NodeValue_t &lower, const NodeValue_t &upper, NodeValue_t &sample);
  static Interp_t interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interp, 
															bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool fromOtherTree);
	
  virtual std::string toString() const;
};

template<size_t N>
float RRTNode3DR<N>::distance(const NodeValue_t &target) {
  float result = 0;
  for (size_t i = 0; i < N; i++) {
    result += target[i]*target[i];
  }
  return result;
}

template<size_t N>
void RRTNode3DR<N>::generateSample(const NodeValue_t &lower, 
                                   const NodeValue_t &upper,
                                   NodeValue_t &sample) {
  for (size_t i = 0; i < N; i++) {
    sample[i] = randRange(lower[i], upper[i]);
  }
}

template<size_t N>
RRTNodeBase::Interp_t RRTNode3DR<N>::interpolate(const NodeValue_t &start, const NodeValue_t &target, const NodeValue_t &interp, 
                                                 bool truncate, CollisionChecker *cc, NodeValue_t &reached, bool fromOtherTree) {
  NodeValue_t delta;
  int numSteps = 0;
  for (size_t i = 0; i < N; i++) {
    int steps = int(fabs(target[i]-start[i])/interp[i]);
    if (steps > numSteps)
      numSteps = steps;
  }
  for (size_t i = 0; i < N; i++) {
    delta[i] = (target[i]-start[i])/float(numSteps);
  }
	
  bool truncated = (unsigned int)numSteps > maxInterpolations && truncate;
  if ( truncated )
    numSteps = maxInterpolations;
	
  // Interpolate along the path and check for collisions
  reached = start;
  for (int t = 0; t < numSteps; t++) {
    for (unsigned int i = 0; i < N; i++) {
      reached[i] += delta[i];
    }
    if ( cc->collides(reached) )
      return COLLISION;
  }
	
  if ( !truncated )
    return REACHED;
  else
    return APPROACHED;
}

template<size_t N>
std::string RRTNode3DR<N>::toString() const {
  char buff[100];
  string result;
  for (size_t i = 0; i < N; i++) {
    sprintf(buff, "%7.2f ", float(q[i]));
    result.append(result);
  }
  return result;
}

template<size_t N>
bool RRTNode3DR<N>::CollisionChecker::collides(const NodeValue_t &qnew, RRTNode3DR<N>::PlannerResult* result) const {
  // set joint angles to test the NodeValue
  KinematicJoint* joint = rootJ;
  for (unsigned int i = 0; i < N; i++) {
    if (!joint) { // just in case...
      std::cout << "*** ERROR: Collision checker cannot find arm joints. Assuming collision." << std::endl;
      return true;
    }
		
    joint->setQ(qnew[i]);
    joint = joint->nextJoint();
    while (joint != NULL && joint->isMobile() == false) joint = joint->nextJoint();
  }
	
  LinkObstacle collisionObs;  //!< stores obstacle found by check component
	
  // create and assign array of joints
  std::vector<JointObstacle> obs;
  for (joint = rootJ; joint != NULL; joint = joint->nextJoint()) {
    obs.push_back(JointObstacle(joint));
    for (KinematicJoint::branch_iterator it = joint->getBranches().begin(); it != joint->getBranches().end(); ++it)
      obs.push_back(JointObstacle(*it));
  }
	
  for (unsigned int i = 0; i < obs.size(); i++) {
    // if the joint has no boundingbox at all, skip it
    if (!obs[i].setupObstacle(worldT,true))
      continue;
		
    // test if new joint obstacle collides with any other joint obstacle
    std::vector<int> collidingJObs; collidingJObs.reserve((i == 0) ? 0 : i-1);
    for (int j = 0; j < (int)i-1; j++) { // i-1 so that we don't check adjacent joint collision
      if (obs[j].valid && obs[i].collides(obs[j])) {
				collidingJObs.push_back(j);
      }
    }
		
    // test if new joint obstacle collides with any planner obstacle
    std::vector<PlannerObstacle3D*> collidingPObs; collidingPObs.reserve(obstacles.size());
    for (unsigned int k = 0; k < obstacles.size(); k++) {
      if ((i != 0 || !(obstacles[k]->isBodyObstacle())) && obs[i].collides(*(obstacles[k]))) {
				collidingPObs.push_back(obstacles[k]);
      }
    }
		
    // if there are no collisions at all, skip to the next joint
    if (collidingJObs.empty() && collidingPObs.empty())
      continue;
		
    /* Assuming there is any collision, we now test each LinkComponent,
     * so we need to assign a vector of components, including the
     * joint's own obstacle */
    JointObstacle jointOwnObs(obs[i].joint);
    jointOwnObs.setupObstacle(worldT, false); // false flag specifies we're using the LinkComponent BB, (without children)
		
    std::vector<LinkObstacle> componentObs(obs[i].joint->components.size(), LinkObstacle(obs[i]));
    for (unsigned int c = 0; c < obs[i].joint->components.size(); c++) {
      componentObs[c].setLink(&(obs[i].joint->components[c]));
      componentObs[c].setupObstacle(worldT, false);
    }
		
    // if there is a collision with a joint obstacle, check the sub-components to confirm
    for (unsigned int jObs = 0; jObs < collidingJObs.size(); jObs++) {
      unsigned int numComponents = obs[collidingJObs[jObs]].joint->components.size();
			
      // if there aren't any subcomponents in either, then there must be a collision
      if (componentObs.size() == 0 && numComponents == 0) {
				if (result) {
					result->movingObstacle = new BoxObstacle(obs[i].obstacle);
					result->collidingObstacle = new BoxObstacle(obs[collidingJObs[jObs]].obstacle);
				}
				std::cout << "*** debug A\n";
				return true;
      }
			
      // set the second joint's components
      // but whenever one is added, check for collision with previous
      JointObstacle collidingOwnObs(obs[collidingJObs[jObs]]);
      collidingOwnObs.setupObstacle(worldT, false);
			
      // check if both joints' own obstacles collide
      if (collidingOwnObs.collides(jointOwnObs)) {
				if (result) {
					result->movingObstacle = new BoxObstacle(collidingOwnObs.obstacle);
					result->collidingObstacle = new BoxObstacle(jointOwnObs.obstacle);
				}
				std::cout << "*** debug B\n";
				return true;
      }
      // check if the colliding joint's own obstacle collides with joint's components
      if (checkComponent(componentObs, collidingOwnObs, collisionObs)) {
				if (result) {
					result->movingObstacle = new BoxObstacle(collidingOwnObs.obstacle);
					result->collidingObstacle = new BoxObstacle(collisionObs.obstacle);
				}
				std::cout << "*** debug C\n";
				return true;
      }
			
      // check if any of the colliding joint's components collides with the joint's own obstacle
      LinkObstacle componentOb(obs[collidingJObs[jObs]]);
      for (unsigned int c = 0; i < numComponents; c++) {
				componentOb.setLink(&(obs[collidingJObs[jObs]].joint->components[c]));
				componentOb.setupObstacle(worldT, false);
				if (componentOb.collides(jointOwnObs)) {
					if (result) {
						result->movingObstacle = new BoxObstacle(componentOb.obstacle);
						result->collidingObstacle = new BoxObstacle(jointOwnObs.obstacle);
					}
					std::cout << "*** debug D\n";
					return true;
				}
				
				// check if any of both joints' components collide
				if (checkComponent(componentObs, componentOb, collisionObs)) {
					if (result) {
						result->movingObstacle = new BoxObstacle(componentOb.obstacle);
						result->collidingObstacle = new BoxObstacle(collisionObs.obstacle);
					}
					std::cout << "*** debug E\n";
					return true;
				}
      }
    }
		
    // if there is a collision with a planner obstacle, check the subcomponents to confirm
    for (unsigned int p = 0; p < collidingPObs.size(); p++) {
      if ((/*i != 0 || */ !(collidingPObs[p]->isBodyObstacle())) && jointOwnObs.collides(*(collidingPObs[p]))) {
				if (result) {
					result->movingObstacle = new BoxObstacle(jointOwnObs.obstacle);
					result->collidingObstacle = collidingPObs[p]->clone();
				}
				std::cout << "*** debug F\n";
				return true;
      }
      if ((/* i != 0 || */ !(collidingPObs[p]->isBodyObstacle())) && checkComponent(componentObs, *(collidingPObs[p]), collisionObs)) {
				if (result) {
					result->movingObstacle = new BoxObstacle(collisionObs.obstacle);
					result->collidingObstacle = collidingPObs[p]->clone();
				}
				std::cout << "*** debug G\n";
				i = *(int*)0; // *** debug
				return true;
      }
    }
  }
	
  // no collision detected
  return false;
}

template<size_t N>
template<class T, class U>
bool RRTNode3DR<N>::CollisionChecker::checkComponent(std::vector<T>& full, const U& newObs, T& collisionObs) {
  for (unsigned int i = 0; i < full.size(); i++) {
    if (full[i].collides(newObs)) {
      collisionObs = full[i];
      return true;
    }
  }
  return false;
}

template<size_t N>
bool RRTNode3DR<N>::CollisionChecker::JointObstacle::setupObstacle(const fmat::Transform& t, bool includeChildren) {
  if (joint == NULL) {
    std::cout << "***ERROR Joint value is null in CollisionChecker." << std::endl;
    valid = false;
  }
  else {
    if (includeChildren)
      valid = joint->getBB3D(t * joint->getFullT(), obstacle);
    else
      valid = joint->getOwnBB3D(t * joint->getFullT(), obstacle);
  }
  return valid;
}

template<size_t N>
bool RRTNode3DR<N>::CollisionChecker::LinkObstacle::setupObstacle(const fmat::Transform& t, bool includeChildren) {
  if (this->joint == NULL || link == NULL) {
    std::cout << "***ERROR Joint/link value is null in CollisionChecker." << std::endl;
    this->valid = false;
  }
  else {
    if (includeChildren)
      this->valid = link->getBB3D(t * this->joint->getFullT(), this->obstacle);
    else
      this->valid = link->getOwnBB3D(t * this->joint->getFullT(), this->obstacle);
  }
  return this->valid;
}

//================ ShapeSpacePlanner3DR ================

//! Plans a path in a n-dimensional angular space, uses forward kinematics for collision testing
template <size_t N>
class ShapeSpacePlanner3DR : public GenericRRT<RRTNode3DR<N>, 3> {
public:
  typedef RRTNode3DR<N> NodeType_t;
  typedef typename NodeType_t::NodeValue_t NodeValue_t;
  typedef typename GenericRRTBase::PlannerResult<3> PlannerResult;
	
  ShapeSpacePlanner3DR(DualCoding::ShapeSpace &shs,
											 const DualCoding::Shape<DualCoding::PolygonData> &worldBounds,
											 float inflation,
											 unsigned int effectorOffset,
											 AdmissibilityPredicate<NodeType_t> *predicate=NULL);
	
  ShapeSpacePlanner3DR(const ShapeSpacePlanner3DR& other) : RRTNode3DR<N>(other) {
    for (size_t i = 0; i < N; i++)
      joints[i] = other.joints[i];
    worldT = other.worldT;
  }
	
		ShapeSpacePlanner3DR operator=(const ShapeSpacePlanner3DR& other) {
			for (size_t i = 0; i < N; i++)
				joints[i] = other.joints[i];
			worldT = other.worldT;
			return *this;
		}
	
  KinematicJoint* joints[N];
  fmat::Transform worldT;
	
  virtual ~ShapeSpacePlanner3DR() {}
	
  using GenericRRT<NodeType_t, 3>::planPath;
  PlannerResult
	planPath(NodeValue_t start,
					 NodeValue_t end,
					 NodeValue_t interpolationStep,
					 const fmat::Transform& _worldT,
					 unsigned int _maxIterations=40000,
					 std::vector<NodeValue_t> *pathResult=NULL,
					 std::vector<NodeType_t> *treeStartResult=NULL,
					 std::vector<NodeType_t> *treeEndResult=NULL);
	
  //! Populates a Shape<GraphicsData> with BoundingBoxes
  void plotTree(const std::vector<NodeType_t> &tree,
								Shape<GraphicsData> &graphics,
								rgb color=rgb(255,0,0));
	
  //! Populates a Shape<GraphicsData> with BoundingBoxes
  void plotPath(const std::vector<NodeValue_t> &tree,
								Shape<GraphicsData> &graphics,
								rgb color=rgb(255,0,0));
	
  //! Returns BoundingBox of each link, for plotTree()
  void getBoxes(std::vector<std::vector<std::pair<float,float> > >& boxes, const KinematicJoint& joint);
	
};

template<size_t N>
ShapeSpacePlanner3DR<N>::ShapeSpacePlanner3DR(DualCoding::ShapeSpace &shs,
                                              const DualCoding::Shape<DualCoding::PolygonData> &worldBounds,
                                              float inflation,
                                              unsigned int effectorOffset,
                                              AdmissibilityPredicate<NodeType_t> *_predicate) :
  GenericRRT<NodeType_t, 3>::GenericRRT(new typename NodeType_t::CollisionChecker(shs, worldBounds, inflation, effectorOffset), _predicate),
  worldT() {
  KinematicJoint* joint = kine->getKinematicJoint(effectorOffset)->cloneBranch();
  if (joint->isMobile() == false) joint = joint->getNextMobileAncestor();
  NodeValue_t lower, upper;
	
  for (int i = (int)N-1; i >= 0; i--) {
    lower[i] = joint->qmin;
    upper[i] = joint->qmax;
    joints[i] = joint;
    joint = joint->getNextMobileAncestor();
  }
	
  this->setLimits(lower, upper);
}

template<size_t N>
typename ShapeSpacePlanner3DR<N>::PlannerResult
ShapeSpacePlanner3DR<N>::planPath(NodeValue_t start,
                                  NodeValue_t end,
                                  NodeValue_t interpolationStep,
                                  const fmat::Transform& _worldT,
                                  unsigned int maxIterations,
                                  std::vector<NodeValue_t> *pathResult,
                                  std::vector<NodeType_t> *treeStartResult,
                                  std::vector<NodeType_t> *treeEndResult) {
  this->setInterpolation(interpolationStep);
  this->worldT = _worldT;
  this->cc->worldT = _worldT;
  return GenericRRT<NodeType_t, 3>::planPath(start, end, maxIterations,
																						 pathResult, treeStartResult, treeEndResult);
}

template<size_t N>
void ShapeSpacePlanner3DR<N>::plotTree(const std::vector<NodeType_t> &tree,
                                       Shape<GraphicsData> &graphics,
                                       rgb color) {
  for ( unsigned int i = 0; i < tree.size(); i++ ) {
    // set joint values
    for (size_t j = 0; j < N; j++)
      joints[j]->setQ(tree[i].q[j]);
		
    // for now, just plot the location of the Gripper
    KinematicJoint* j = joints[N-1];
		
    while (j->nextJoint() != NULL) {
      j = j->nextJoint();
    }
		
    fmat::Column<3> pos = j->getWorldPosition();
		
    graphics->add(new GraphicsData::CircleElement("start", DualCoding::GraphicsData::xyPair(pos[0],pos[1]), 5, true, color));
		
    /*// get bounding box of every link
      std::vector<std::vector<std::pair<float,float> > > boxes;
      getBoxes(boxes, *joints[0]);*/
		
    /*// add to element
      for (size_t b = 0; b < boxes.size(); b++) {
      graphics->add(new GraphicsData::PolygonElement(boxes[b], true, color));
      }*/
  }
}

template<size_t N>
void ShapeSpacePlanner3DR<N>::plotPath(const std::vector<NodeValue_t> &path,
                                       Shape<GraphicsData> &graphics,
                                       rgb color) {
  for ( unsigned int i = 0; i < path.size(); i++ ) {
    // set joint values
    for (size_t j = 0; j < N; j++)
      joints[j]->setQ(path[i][j]);
		
    // set robot transformation in WorldSpace (converts from BaseFrame -> "WorldFrame")
    DualCoding::Point p = VRmixin::theAgent->getCentroid();
    worldT.translation() = -fmat::pack(p.coordX(),p.coordY(),p.coordZ());
    worldT.rotation() = fmat::rotationZ(-VRmixin::theAgent->getOrientation());
		
    // set first and last to different colors
    rgb theColor;
    if (i == 0) theColor = rgb(255,0,0);
    else if (i == path.size()-1) theColor = rgb(0,255,0);
    else theColor = color;
        
    KinematicJoint * j = joints[0];

    // testing method: boundingbox using the width, height, length values of the BoxObstacle object of each joint and their components
        
    while (j->nextJoint() != NULL)
      {
				BoxObstacle bo;
				float w, h, l;
				fmat::Quaternion q;
				fmat::Column<3> centroid;
				j->getOwnBB3D(worldT * j->getFullT(), bo);
				if (bo.getExtents() != fmat::Column<3>()) {
					// set points
					centroid = bo.getCenter();
					w = bo.getWidth();
					h = bo.getHeight();
					l = bo.getLength();
					q = fmat::Quaternion::fromMatrix(bo.getOrientation());
					graphics->add(new GraphicsData::BoundingBoxElement(j->outputOffset.get(), q, centroid, l, w, h));
                
					BoxObstacle boc;
					float cw, ch, cl;
					fmat::Quaternion cq;
					fmat::Column<3> ccentroid;
					for (KinematicJoint::component_iterator it = j->components.begin(); it != j->components.end(); ++it) {
						(*it)->getOwnBB3D(worldT * j->getFullT(), boc);
						ccentroid = boc.getCenter();
						cw = boc.getWidth();
						ch = boc.getHeight();
						cl = boc.getLength()/2;
						cq = fmat::Quaternion::fromMatrix(boc.getOrientation());
						graphics->add(new GraphicsData::BoundingBoxElement("component", cq, ccentroid, cl, cw, ch));
					}
				}
				j = j->nextJoint();
      }
    // Previous method: for now, just plot the location of the Gripper
    /* KinematicJoint* j = joints[N-1]; */
		
    /* while (j->nextJoint() != NULL) { */
    /*     j = j->nextJoint(); */
    /* } */
		
    /* fmat::Column<3> pos = j->getWorldPosition(); */
		
    /* graphics->add(new GraphicsData::CircleElement(DualCoding::GraphicsData::xyPair(pos[0],pos[1]), 5, true, theColor)); */
		
    /*// get bounding box of every link
      std::vector<std::vector<std::pair<float,float> > > boxes;
      getBoxes(boxes, *joints[0]);*/
		
    /*// add to element
      for (size_t b = 0; b < boxes.size(); b++) {
      graphics->add(new GraphicsData::PolygonElement(boxes[b], true, theColor));
      }*/
  }
}
template<size_t N>
void ShapeSpacePlanner3DR<N>::getBoxes(std::vector<std::vector<std::pair<float,float> > >& boxes, const KinematicJoint& joint) {
  // we don't have to go through all branches, because we already cloned the branch
  if (joint.nextJoint() != NULL)
    getBoxes(boxes, *joint.nextJoint());
	
  // get the joint's BB (in obstacle format)
  std::vector<std::pair<float,float> > points(BoxObstacle::NUM_CORNERS);
  BoxObstacle bo;
  joint.getOwnBB3D(worldT * joint.getFullT(), bo);
	
  if (bo.getExtents() != fmat::Column<3>()) {
    // set points
    for (int p = 0; p < BoxObstacle::NUM_CORNERS; p++) {
      fmat::Column<3> corner = bo.getCorner(static_cast<BoxObstacle::CornerOrder>(p));
      points[p] = std::pair<float,float>(corner[0], corner[1]);
    }
    boxes.push_back(points);
  }
	
  // do the same for each link
  for (KinematicJoint::component_iterator it = joint.components.begin(); it != joint.components.end(); ++it) {
    // set points
    (*it)->getOwnBB3D(worldT * joint.getFullT(), bo);
    for (int p = 0; p < BoxObstacle::NUM_CORNERS; p++) {
      fmat::Column<3> corner = bo.getCorner(static_cast<BoxObstacle::CornerOrder>(p));
      points[p] = std::pair<float,float>(corner[0], corner[1]);
    }
    boxes.push_back(points);
  }
}

#endif

#endif // TGT_HAS_ARMS
