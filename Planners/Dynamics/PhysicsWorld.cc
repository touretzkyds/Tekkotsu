#ifdef HAVE_BULLET

#include "PhysicsWorld.h"
#include "PhysicsBody.h"
#include "MotorControllers.h"
#include "Shared/debuget.h"

#ifdef __APPLE__
#include <BulletDynamics/btBulletDynamicsCommon.h>
#else
#include <btBulletDynamicsCommon.h>
#endif
#if BT_BULLET_VERSION < 280
#  include <BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h>
#endif

using namespace std;

static void ticCallback(btDynamicsWorld *world, btScalar timeStep) {
	static_cast<PhysicsWorld*>(world->getWorldUserInfo())->updateControllers(timeStep);
}

//! this is apparently not needed due to using the 'true' argument to addConstraint to disable collisions between linked objects...
struct CollisionFilter : public btOverlapFilterCallback {
	// return true when pairs need collision
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const {
		if( (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask)==0 || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask)==0)
			return false;
		
		btRigidBody* rb0 = btRigidBody::upcast(static_cast<btCollisionObject*>(proxy0->m_clientObject));
		if(rb0==NULL)
			return true;
		bool filter = rb0->checkCollideWithOverride(static_cast<btCollisionObject*>(proxy1->m_clientObject));
		/*std::string name0 = static_cast<PhysicsBody*>(rb0->getUserPointer())->getPath();
		std::string name1 = static_cast<PhysicsBody*>(static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer())->getPath();
		std::cout << "filter " << name0 << ' ' << name1 << " = " << filter << std::endl;*/
		return filter;
	}
};

PhysicsWorld::~PhysicsWorld() {
	ASSERT(bodies.size()==0,"PhysicsWorld still has bodies at destruction");
	delete dynamicsWorld; dynamicsWorld=NULL;
	delete solver; solver=NULL;
	delete broadphase; broadphase=NULL;
	delete dispatcher; dispatcher=NULL;
	delete collisionConfiguration; collisionConfiguration=NULL;
	//delete filter; filter=NULL;
}

void PhysicsWorld::initWorld() {
	//collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();
	collisionConfiguration->setConvexConvexMultipointIterations(4,4);
	
	//use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	// eliminates jitter of small cubes on the ground plane
#  if BT_BULLET_VERSION >= 280
	// this function was introduced in 2.79, but initially broken, wait for 2.80
	collisionConfiguration->setPlaneConvexMultipointIterations();
#  else
	// do setPlaneConvexMultipointIterations() manually:
	typedef btConvexPlaneCollisionAlgorithm::CreateFunc CF;
	CF* cpCF = (CF*)collisionConfiguration->getCollisionAlgorithmCreateFunc(BOX_SHAPE_PROXYTYPE,STATIC_PLANE_PROXYTYPE);
	cpCF->m_numPerturbationIterations = 3;
	cpCF->m_minimumPointsPerturbationThreshold = 3;
	CF* pcCF = (CF*)collisionConfiguration->getCollisionAlgorithmCreateFunc(STATIC_PLANE_PROXYTYPE,BOX_SHAPE_PROXYTYPE);
	pcCF->m_numPerturbationIterations = 3;
	pcCF->m_minimumPointsPerturbationThreshold = 3;
#  endif

	//make sure to disable the special box-box collision detector, replace it by the generic convex version
	dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE, collisionConfiguration->getCollisionAlgorithmCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE,CONVEX_HULL_SHAPE_PROXYTYPE));

	/*
	 //the maximum size of the collision world. Make sure objects stay within these boundaries
	 //Don't make the world AABB size too large, it will harm simulation quality and performance
	 btVector3 worldAabbMin(-1000,-1000,-1000);
	 btVector3 worldAabbMax(1000,1000,1000);
	 int	maxProxies = 1024;
	 broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);*/
	broadphase = new btDbvtBroadphase();
	
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;
	
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	dynamicsWorld->setInternalTickCallback(ticCallback,this,true);
	
	// this is apparently not needed due to using the 'true' argument to addConstraint to disable collisions between linked objects...
	//filter = new CollisionFilter;
	//dynamicsWorld->getPairCache()->setOverlapFilterCallback(filter);
	
	updateGravity();
	updateSolverInfo();
}

void PhysicsWorld::updateGravity() {
	dynamicsWorld->setGravity(btVector3(0,0,gravity*1000*spaceScale));
}

void PhysicsWorld::updateSolverInfo() {
	dynamicsWorld->getSolverInfo().m_erp=erp;
	dynamicsWorld->getSolverInfo().m_numIterations=solverIter;
	dynamicsWorld->getSolverInfo().m_splitImpulse=splitImpulse;
	
	//dynamicsWorld->getSolverInfo().m_linearSlop=1*spaceScale;
	//dynamicsWorld->getSolverInfo().m_damping=50;
	//dynamicsWorld->getSolverInfo().m_erp2=0.5f;
	//dynamicsWorld->getSolverInfo().m_maxErrorReduction=50;
	//dynamicsWorld->getSolverInfo().m_globalCfm=0.01f;
	//dynamicsWorld->getSolverInfo().m_friction=.3;
	//dynamicsWorld->getSolverInfo().m_restitution=0;
}

void PhysicsWorld::updateControllers(float t) {
	for(std::set<MotorController*>::const_iterator it=motorControllers.begin(); it!=motorControllers.end(); ++it)
		(*it)->updateControllerResponse(t);
}

size_t PhysicsWorld::step(float t) {
	return dynamicsWorld->stepSimulation(t, 0, t);
}

size_t PhysicsWorld::step(float t, unsigned int max, float freq) {
	return dynamicsWorld->stepSimulation(t, max, freq);
}


PhysicsBody* PhysicsWorld::getBody(const LinkComponent& info) {
	std::map<LinkComponent*, PhysicsBody*>::const_iterator it = bodies.find(const_cast<LinkComponent*>(&info));
	if(it!=bodies.end())
		return it->second;
	return NULL;
}

PhysicsWorld::PointConstraint* PhysicsWorld::addPointConstraint(PhysicsBody* body, const fmat::Column<3>& worldPt) {
	PointConstraint * cn = new PointConstraint(*body->body,(worldPt*spaceScale).exportTo<btVector3>() - body->centerOffset);
	dynamicsWorld->addConstraint(cn, true);
	return cn;
}

void PhysicsWorld::removeConstraint(PhysicsWorld::PointConstraint* cn) {
	dynamicsWorld->removeConstraint(cn);
}

#endif // HAVE_BULLET

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
