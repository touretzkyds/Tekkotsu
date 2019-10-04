//-*-c++-*-
#ifndef INCLUDED_PhysicsWorld_h_
#define INCLUDED_PhysicsWorld_h_

#include "Shared/plist.h"
#include "Motion/KinematicJoint.h"
#include <map>
#include <set>

class PhysicsBody;
class MotorController;
class btPoint2PointConstraint;

//! description of PhysicsWorld
class PhysicsWorld : public virtual plist::Dictionary {
	friend class PhysicsBody;
	friend class PhysicsState;
public:
	PhysicsWorld()
		: plist::Dictionary(), gravity(-9.80665f), spaceScale(0.01f), massScale(1), solverIter(250),
		erp(0.25f), splitImpulse(false), linearDamping(0), angularDamping(0), collisionMargin(1),
		collisionConfiguration(NULL), dispatcher(NULL), broadphase(NULL), solver(NULL), dynamicsWorld(NULL), filter(NULL),
		bodies(), motorControllers(),
		gravityListener(gravity,*this,&PhysicsWorld::updateGravity,false),
		spaceScaleListener(spaceScale,*this,&PhysicsWorld::updateGravity,false),
		solverIterListener(solverIter,*this,&PhysicsWorld::updateSolverInfo,false),
		erpListener(erp,*this,&PhysicsWorld::updateSolverInfo,false)
	{
		addEntry("Gravity",gravity,"Controls the force of gravity along z vector in meters per second. (default -9.80665)");
		addEntry("SpaceScale",spaceScale,"Controls a spatial scaling factor for simulation stability, applied to millimeter scale used for graphics. (default 0.01, i.e. 10cm internal base unit for physics)");
		addEntry("MassScale",massScale,"Controls an inertial scaling factor for simulation stability, applied to kilogram scale (default 1, i.e. internal base unit of 1 kilogram for physics)");
		addEntry("SolverIterations",solverIter,"Maximum number of iterations for the constraint solver.\nIncreasing this will resolve 'breaking' constraints, such as servos under load");
		addEntry("ERP",erp,"Error resolution proportion, analgous to gradient descent step size.\nLarger values can help reduce solver iterations, but can cause jitter/instability if too high");
		addEntry("SplitImpulse",splitImpulse,"Causes the physics engine to solve for linear and angular constraints independently.");
		addEntry("LinearDamping",linearDamping,"The amount of damping to apply on translational motion.\nThis is a default value applied at object creation, run-time modification won't affect current objects.");
		addEntry("AngularDamping",angularDamping,"The amount of damping to apply on rotational motion.\nThis is a default value applied at object creation, run-time modification won't affect current objects.");
		addEntry("CollisionMargin",collisionMargin,"The collision margin (in mm) to apply to shapes to handle minor perterbations more efficiently.\nThis is a default value applied at object creation.");
		setLoadSavePolicy(FIXED,SYNC);
		initWorld();
	}
	~PhysicsWorld();
	
	plist::Primitive<float> gravity;
	plist::Primitive<float> spaceScale;
	plist::Primitive<float> massScale;
	plist::Primitive<unsigned int> solverIter;
	plist::Primitive<float> erp;
	plist::Primitive<bool> splitImpulse;
	plist::Primitive<float> linearDamping;
	plist::Primitive<float> angularDamping;
	plist::Primitive<float> collisionMargin;
	
	size_t step(float t);
	size_t step(float t, float freq) { return step(t, static_cast<unsigned int>(t/freq)+1, freq); }
	size_t step(float t, unsigned int max, float freq);
	
	PhysicsBody* getBody(const LinkComponent& info);
	class btDiscreteDynamicsWorld& getWorld() { return *dynamicsWorld; }
	
	typedef btPoint2PointConstraint PointConstraint;
	PointConstraint* addPointConstraint(PhysicsBody* body, const fmat::Column<3>& worldPt);
	void removeConstraint(PointConstraint* cn);
	
	typedef std::map<LinkComponent*, PhysicsBody*>::const_iterator body_iterator;
	body_iterator begin() const { return bodies.begin(); }
	body_iterator end() const { return bodies.end(); }
	
	void updateControllers(float t); //!< called by physics internal step callback
	
protected:
	class btDefaultCollisionConfiguration* collisionConfiguration;
	class btCollisionDispatcher* dispatcher;
	class btBroadphaseInterface* broadphase;
	class btSequentialImpulseConstraintSolver* solver;
	class btDiscreteDynamicsWorld* dynamicsWorld;
	
	class CollisionFilter* filter;
	
	void initWorld();
	void updateGravity();
	void updateSolverInfo();
	
	std::map<LinkComponent*, PhysicsBody*> bodies;
	std::set<MotorController*> motorControllers;
	
	// these must be initialized last so they will only be called at end of initialization, and cleared before destruction
	plist::PrimitiveCallbackMember<PhysicsWorld> gravityListener;
	plist::PrimitiveCallbackMember<PhysicsWorld> spaceScaleListener;
	plist::PrimitiveCallbackMember<PhysicsWorld> solverIterListener;
	plist::PrimitiveCallbackMember<PhysicsWorld> erpListener;
	
private:
	PhysicsWorld(const PhysicsWorld&);
	PhysicsWorld& operator=(const PhysicsWorld&);
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
