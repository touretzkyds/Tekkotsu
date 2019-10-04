#ifdef HAVE_BULLET

#include "PhysicsBody.h"
#include "PhysicsWorld.h"
#include "MotorControllers.h"
#include "Shared/ReferenceCounter.h"
#include "Shared/debuget.h"

#ifdef HAVE_OGRE
#  ifdef __APPLE__
#    include <Ogre/OgreMesh.h>
#    include <Ogre/OgreSubMesh.h>
#  else
#    include <OGRE/OgreMesh.h>
#    include <OGRE/OgreSubMesh.h>
#  endif
#endif

#ifdef __APPLE__
#include <BulletCollision/btBulletCollisionCommon.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>
#else
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#endif

#include <stdexcept>

using namespace std; 

std::ostream& operator<<(std::ostream& os, const btTransform& t) {
	fmat::Matrix<4,4> ft;
	for(unsigned int i=0; i<3; ++i)
		for(unsigned int j=0; j<3; ++j)
			ft(i,j) = t.getBasis()[i][j];
	(fmat::SubVector<3>)ft.column(3) = t.getOrigin();
	ft(3,3)=1;
	return os << ft;
}

std::ostream& operator<<(std::ostream& os, const btVector3& v) {
	return os << fmat::SubVector<3>(const_cast<float*>(&v[0]));
}


struct CollisionData : public ReferenceCounter {
#ifdef HAVE_OGRE
	CollisionData(const std::string& meshName, Ogre::MeshPtr mesh);
#endif
	~CollisionData() {
		delete vertices; vertices=NULL;
		delete indices; indices=NULL;
		PhysicsBody::collisionData.erase(name);
	}
	std::string name;
	float * vertices;
	unsigned int * indices;
};

std::map<std::string,CollisionData*> PhysicsBody::collisionData;


PhysicsBody::~PhysicsBody() {
	//std::cout << "Destructing " << getPath() << std::endl;
	// unregister with parent
	if(parentBody!=NULL)
		parentBody->children.erase(this);
	// give children to parent
	for(std::set<PhysicsBody*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
		(*it)->parentBody = parentBody;
		if(parentBody!=NULL)
			parentBody->children.insert(*it);
	}
	ASSERT(children.size()==0 || parentBody!=NULL, "deleting PhysicsBody with children but no parent");
	children.clear();
	clear();
	parentBody=NULL;
	std::map<LinkComponent*, PhysicsBody*>::iterator it = world.bodies.find(&link);
	if(it==world.bodies.end()) {
		std::cerr << "WARNING PhysicsBody: body was not found in world for removal" << std::endl;
	} else {
		world.bodies.erase(it);
	}
}

std::string PhysicsBody::getPath() const {
	stringstream ss;
	if(const KinematicJoint* kj = dynamic_cast<const KinematicJoint*>(&link)) {
		ss << kj->getPath() << ":" << getModel();
	} else {
		ss << "Free-" << getModel();
	}
	ss << "@" << &link;
	return ss.str();
}

void PhysicsBody::getCurrent(fmat::Column<3>& pos, fmat::Quaternion& quat) const {
	ASSERTRET(body!=NULL,"PhysicsData::getCurrent called with NULL body");
	//std::cout << " Get current " << kj.outputOffset.get() << ":\n" << body->getWorldTransform() << '\n' << centerOffset << std::endl;
	const btTransform& t = body->getWorldTransform();
	//std::cout << getPath() <<'\n'<< t << std::endl;
	pos.importFrom(t * -centerOffset / world.spaceScale);
	//std::cout << fmat::transpose(fmat::SubMatrix<4,3>(t.getBasis()[0])) << std::endl;
	btQuaternion q = t.getRotation();
	quat = fmat::Quaternion(q.w(),q.x(),q.y(),q.z());
}

bool PhysicsBody::hasJoint() const { return jointInterface!=NULL; }

float PhysicsBody::getJoint() const { return jointInterface->getValue(0); }

void PhysicsBody::teleport(const fmat::Column<3>& pos) {
	btTransform cur = body->getWorldTransform();
	float sc = world.spaceScale;
	cur.setOrigin(btVector3(pos[0]*sc,pos[1]*sc,pos[2]*sc));
	teleport(cur);
}

void PhysicsBody::teleport(const fmat::Quaternion& quat) {
	btTransform cur = body->getWorldTransform();
	cur.setOrigin(cur.getOrigin() - quatRotate(cur.getRotation(),centerOffset));
	cur.setRotation(btQuaternion(quat.getX(),quat.getY(),quat.getZ(),quat.getW()));
	teleport(cur);
}

void PhysicsBody::teleport(const fmat::Column<3>& pos, const fmat::Quaternion& quat) {
	float sc = world.spaceScale;
	btTransform cur(btQuaternion(quat.getX(),quat.getY(),quat.getZ(),quat.getW()),btVector3(pos[0]*sc,pos[1]*sc,pos[2]*sc));
	teleport(cur);
}

void PhysicsBody::teleport(const btTransform& tr) {
	std::vector<std::pair<PhysicsBody*,btTransform> > cts; // will child transforms relative to current to reset at end
	//const btTransform mt = body->getWorldTransform().inverse(); // my transform, inverted
	for(std::set<PhysicsBody*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
		btTransform cur = (*it)->body->getWorldTransform();
		cur.setOrigin(cur.getOrigin() - quatRotate(cur.getRotation(),(*it)->centerOffset));
		cts.push_back(std::make_pair(*it,cur));
	}
	clear();
	lastTransform = tr;
	build();
	for(std::vector<std::pair<PhysicsBody*,btTransform> >::const_iterator it=cts.begin(); it!=cts.end(); ++it)
		it->first->teleport(it->second);
}

void PhysicsBody::setVelocity(const fmat::Column<3>& linear, const fmat::Column<3>& angular) {
	float sc = world.spaceScale;
	setVelocity(btVector3(linear[0]*sc,linear[1]*sc,linear[2]*sc), btVector3(angular[0],angular[1],angular[2]));
}
void PhysicsBody::setVelocity(const btVector3& linear, const btVector3& angular) {
	assert(!isnan(linear[0]));
	assert(!isnan(angular[0]));
	// these values are at link origin, but bullet applies these at the center of mass, so need to offset linear velocity by radius of center of mass
	const btTransform& t = body->getWorldTransform();
	//std::cout << getPath() << " curvel " << body->getLinearVelocity() << " setvel " << linear << " + " << angular.cross(t.getBasis() * centerOffset) << std::endl;
	body->activate();
	body->setLinearVelocity(linear + angular.cross(t.getBasis() * centerOffset));
	body->setAngularVelocity(angular);
	for(std::set<PhysicsBody*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
		// similarly offset linear velocity for children
		//const btTransform& ct = (*it)->body->getWorldTransform();
		(*it)->setVelocity(linear,angular); // + angular.cross(ct.getOrigin() - t.getOrigin())
	}
}

void PhysicsBody::init() {
	//std::cout << "Creating PhysicsBody for " << getPath() << std::endl;
	if(parentBody!=NULL) {
		// see if parent has any child bodies that are actually this body's children
		for(std::set<PhysicsBody*>::const_iterator it=parentBody->children.begin(); it!=parentBody->children.end(); ++it) {
			if(isChild(link,(*it)->link)) {
				children.insert(*it);
			}
		}
		// steal our children back from parent
		for(std::set<PhysicsBody*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
			(*it)->parentBody=this;
			parentBody->children.erase(*it);
		}
		// register with parent
		parentBody->children.insert(this);
	}
	ASSERTIF(world.bodies.find(&link)==world.bodies.end(),"WARNING PhysicsBody: body already added to world!") {
		world.bodies[&link] = this;
	}
	build();
}

void PhysicsBody::build() {
	//std::cout << "Building PhysicsBody for " << getPath() << std::endl;
	if(body==NULL) {
		ASSERT(compound==NULL,"have compound for collision shape but no body?");
		compound = new btCompoundShape;
		//compound->setMargin(0);  // we'll set margins on the individual sub-shapes, compound defaults to 0 margin anyway
	} else {
		ASSERT(compound!=NULL,"have body but no compound for collision shape?");
	}
	
	fmat::Column<3> com;
	link.sumLinkCenterOfMass(com,totalMass);
	(com * world.spaceScale).exportTo(centerOffset);
	totalMass*=world.massScale;
	
	inertia.setValue(0,0,0);
	
	//if(const KinematicJoint* kj = dynamic_cast<const KinematicJoint*>(&link))
	buildLink(link, fmat::Transform::identity());
	//else addComponent(link, fmat::Transform::identity());
	//compound->setMargin(6*world.spaceScale);

	//std::cout << "built inertia for " << getPath() << " is " << inertia << std::endl;
	
	btVector3 localInertia;
	compound->calculateLocalInertia(totalMass,localInertia);
	inertia+=localInertia;
	
	//std::cout << "local inertia for " << getPath() << " is " << localInertia << std::endl;
	//std::cout << "total inertia for " << getPath() << " is " << inertia << std::endl;

	ASSERT(totalMass==0 || compound->getNumChildShapes()>0,"PhysicsBody created with mass but no collision shape")
	
	if(body!=NULL) {
		//std::cout << "Reusing body for " << getPath() << std::endl;
		// can't do resetTransform() as currently written... reusing body implies rebuild, need to update children for change in transform,
		// and even then the teleport within bullet is a bad idea, essentially have to remove and reconstruct children from scratch to handle constraints (?)
		//resetTransform();
		
		// This seems to work in order enable the PhysicsBody::teleport() functionality, but kind of questionable.
		world.dynamicsWorld->removeRigidBody(body);
		resetTransform();
		world.dynamicsWorld->addRigidBody(body);

	} else {
		//std::cout << "Body construction for " << getPath() << std::endl;
		//std::cout << "MIRROR btRigidBodyConstructionInfo(" << totalMass << "," << inertia <<")" << std::endl;
		//btRigidBody::btRigidBodyConstructionInfo rbInfo(totalMass,new btDefaultMotionState,compound,inertia);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(totalMass,NULL,compound,inertia);
		/*rbInfo.m_linearDamping = 0.1;
		rbInfo.m_angularDamping = 0.1;
		rbInfo.m_additionalDamping=true;
		rbInfo.m_additionalDampingFactor=.4;
		rbInfo.m_additionalAngularDampingFactor=.4;
		rbInfo.m_additionalAngularDampingThresholdSqr=.5;*/
		body = new btRigidBody(rbInfo);
		body->setDamping(world.linearDamping, world.angularDamping);
		body->setUserPointer(this);
		//body->setMassProps(totalMass,btVector3(inertia[0],inertia[1],inertia[2]));
		
		resetTransform();
		world.dynamicsWorld->addRigidBody(body);
	}
	
	// Set up constraints
	if(parentBody==NULL) {
		// no-op, root body doesn't have constraint
	} else {
		btTransform pT;
		const fmat::Transform fm = link.getT(parentBody->link);
		pT.setBasis(btMatrix3x3(fm(0,0),fm(0,1),fm(0,2), fm(1,0),fm(1,1),fm(1,2), fm(2,0),fm(2,1),fm(2,2)));
		pT.setOrigin(btVector3(fm(0,3),fm(1,3),fm(2,3)) * world.spaceScale - parentBody->centerOffset);
		//pT.setOrigin(pT * centerOffset);
		btTransform lT;
		lT.setBasis(btMatrix3x3::getIdentity());
		lT.setOrigin(-centerOffset);
		if(qpos!=NULL) {
			btMatrix3x3& rot = lT.getBasis();
			rot[0][0] = rot[1][1] = std::cos(*qpos);
			rot[0][1] = -(rot[1][0] = std::sin(*qpos));
		}

		//std::cout << "CENTER " << parentBody->centerOffset << '\n' << pT << std::endl;
		
		ASSERT(joint==NULL,"PhysicsBody already has joint!?");
		joint = new btGeneric6DofConstraint(*parentBody->body,*body,pT,lT,true);
		
		//const float ERP=0.95f, SOFT=0.65f;
		const float ERP=0.f, SOFT=0.0f;
		joint->getTranslationalLimitMotor()->m_restitution=0;
		joint->getTranslationalLimitMotor()->m_limitSoftness=SOFT;
		for(unsigned int i=0; i<3; ++i) {
			joint->getRotationalLimitMotor(i)->m_limitSoftness=SOFT;
#if BT_BULLET_VERSION<276
			joint->getRotationalLimitMotor(i)->m_ERP=ERP;
#else
			joint->getTranslationalLimitMotor()->m_stopERP[i] = ERP;
			joint->getTranslationalLimitMotor()->m_stopCFM[i] = SOFT;
			joint->getTranslationalLimitMotor()->m_normalCFM[i] = SOFT;
			joint->getRotationalLimitMotor(i)->m_stopERP = ERP;
			joint->getRotationalLimitMotor(i)->m_stopCFM = SOFT;
			joint->getRotationalLimitMotor(i)->m_normalCFM = SOFT;
#endif
		}
		if(qpos==NULL) {
			// static constraint
			/*joint->setLinearLowerLimit(btVector3(0,0,0));
			joint->setLinearUpperLimit(btVector3(0,0,0));*/
			joint->setAngularLowerLimit(btVector3(0,0,0));
			joint->setAngularUpperLimit(btVector3(0,0,0));
		} else if(link.controllerInfo.velocity) {
			// constraint controlled by velocity
			joint->setAngularLowerLimit(btVector3(0,0,1));
			joint->setAngularUpperLimit(btVector3(0,0,-1));
			jointInterface = new GenericAngularMotorInterface(*joint);
			float scale=1;
			const fmat::Column<3> aabbdim = link.getAABB().getDimensions();
			if(aabbdim[0]!=aabbdim[1])
				std::cerr << "non-symmetric wheel boundaries?  what's the radius?" << std::endl;
			if(aabbdim[0]<=0) {
				std::cerr << "wheel diameter is non-positive" << std::endl;
			} else {
				scale = 2/aabbdim[0];
			}
			jointController = new LinearMotorController(*qpos,*jointInterface,scale);
		} else {
			// constraint controlled by position
			joint->setAngularLowerLimit(btVector3(0,0,-link.qmax));
			joint->setAngularUpperLimit(btVector3(0,0,-link.qmin));
			for(unsigned int i=0; i<3; ++i) {
				joint->getRotationalLimitMotor(i)->m_maxMotorForce=8000;
				joint->getRotationalLimitMotor(i)->m_maxLimitForce=8000;
			}
			if(link.controllerInfo.forceControl) {
				jointInterface = new GenericAngularMotorInterface(*joint);
				PIDMotorController * tmp = new PIDMotorController(*qpos,*jointInterface);
				tmp->p = 10;
				tmp->i = 0;
				tmp->d = 0;
				jointController = tmp;
				
				/*jointController->p = 150;
				 jointController->i = 0;
				 jointController->d = 0;
				 jointController->derrGamma = 0.5;
				 jointController->maxResponse=-1;
				 jointController->minResponse=15;
				 jointController->punch=5;
				 jointController->friction=0.5;
				 //jointController->linearff=1;*/
			} else {
				jointInterface = new GenericAngularPositionInterface(*joint);
				jointController = new LinearMotorController(*qpos,*jointInterface);
			}
		}

		//btHingeConstraint* hinge = new btHingeConstraint(*parentBody->body,*body,pT,lT,true);
		
		/*jointInterface = new HingeAngularMotorInterface(*hinge);
		DirectController * pid = new DirectController(*qpos,*jointInterface);*/
		
		/*jointInterface = new HingeAngularMotorInterface(*hinge);
		PIDController * pid = new PIDController(*qpos,*jointInterface);
		pid->p=.75;
		pid->i=10;
		pid->d = 0;
		pid->minResponse=0; pid->punch=0; pid->friction=0;*/
		
		/*jointInterface = new HingeVelocityInterface(*hinge);
		PIDController * pid = new PIDController(*qpos,*jointInterface);
		pid->p=.02;
		pid->i=.25;
		pid->d = 0.001;
		pid->minResponse=0; pid->punch=0; pid->friction=0;*/
		
		/*jointInterface = new HingePositionInterface(*hinge);
		PIDController * pid = new PIDController(*qpos,*jointInterface);
		pid->p = 90;
		pid->i = 40;
		pid->d = 0.2;
		pid->minResponse=10; pid->punch=0; pid->friction=70;*/
		
		/*jointInterface = new HingePositionInterface(*hinge);
		PIDController * pid = new PIDController(*qpos,*jointInterface);
		pid->p = 300;//200000;
		pid->i = 0;
		pid->d = 0;//20000;
		pid->minResponse=10; pid->punch=0; pid->friction=2;//20000;*/
		
		if(jointController!=NULL) {
			jointController->set(link.controllerInfo);
			world.motorControllers.insert(jointController);
		}
		if(joint!=NULL)
			world.dynamicsWorld->addConstraint(joint, true);
		/*if(jointController!=NULL)
			jointController->updateControllerResponse(0);*/
		//setVelocity(btVector3(0,0,0), btVector3(0,0,0));
	}
		
	//body->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(centerOffset[0],centerOffset[1],centerOffset[2])));
	
	//setQ(kj.getQ(),findParentBody()); // updates body position
	
	//std::cout << kj.getPath() << " set body at " << fmat::SubVector<3>(body->getWorldTransform().getOrigin()) << std::endl;
	// todo offset geom -m.c
	//dGeomSetOffsetPosition(box, centerOffset[0], centerOffset[1], centerOffset[2]);
	
	//updateJoint();
	updateFrictionForce();
	updateAnistropicFriction();
}

void PhysicsBody::buildLink(const KinematicJoint& kj, const fmat::Transform& tr) {
	addComponent(kj,tr);
	for(KinematicJoint::component_iterator it=kj.components.begin(); it!=kj.components.end(); ++it) {
		addComponent(**it,tr);
	}
	for(KinematicJoint::branch_iterator it=kj.getBranches().begin(); it!=kj.getBranches().end(); ++it) {
		if(!(*it)->hasMass()) {
			buildLink(**it,tr * (*it)->getTq());
		}
	}
}

void PhysicsBody::addComponent(const LinkComponent& comp, const fmat::Transform& tr) {
	if(comp.collisionModel.size()==0) {
		if(comp.mass>0) {
			// todo: should use bb or something instead of point mass (what if CoM at origin?  no inertia?)
			float m = comp.mass * world.massScale;
			fmat::Column<3> com;
			com.importFrom(comp.centerOfMass);
			com = tr * com * world.spaceScale;
			btVector3 localInertia = btVector3(com[0],com[1],com[2]) - centerOffset;
			inertia += btVector3(localInertia[0]*localInertia[0]*m, localInertia[1]*localInertia[1]*m, localInertia[2]*localInertia[2]*m);
		}
		return;
	}
	//std::cout << "Adding " << getModel() << " on " << getPath() << std::endl;
	const fmat::Column<3> dim = fmat::Column<3>().importFrom(comp.collisionModelScale);
	const fmat::Column<3> pos = fmat::Column<3>().importFrom(comp.collisionModelOffset);
	fmat::Quaternion rot = fmat::Quaternion::fromAxis(comp.collisionModelRotation);
	btVector3 dim_2, origin;
	(dim/2 * world.spaceScale).exportTo(dim_2);
	(tr * pos * world.spaceScale).exportTo(origin);
	rot = fmat::Quaternion::fromMatrix(tr.rotation()) * rot;
	if(comp.collisionModel=="Cube") {
		btCollisionShape * collide = new btBoxShape(dim_2);
		collide->setMargin(world.collisionMargin*world.spaceScale);
		const btTransform btr(btQuaternion(rot.getX(),rot.getY(),rot.getZ(),rot.getW()), origin - centerOffset);
		compound->addChildShape(btr,collide);
		//std::cout << kj.getPath() << " set BB " << dim << std::endl;
	} else if(comp.collisionModel=="Sphere") {
		btCollisionShape * collide=NULL;
		if(dim[0]==dim[1] && dim[0]==dim[2]) {
			collide = new btSphereShape(dim_2[0]);
		} else {
			btVector3 center(0,0,0);
			btScalar radi=1;
			collide = new btMultiSphereShape(&center,&radi,1);
			collide->setLocalScaling(dim_2);
		}
		collide->setMargin(world.collisionMargin*world.spaceScale);
		const btTransform btr(btQuaternion(rot.getX(),rot.getY(),rot.getZ(),rot.getW()), origin - centerOffset);
		compound->addChildShape(btr,collide);
	} else if(comp.collisionModel=="Cylinder") {
		btCollisionShape * collide = new btCylinderShapeZ(dim_2);
		collide->setMargin(world.collisionMargin*world.spaceScale);
		const btTransform btr(btQuaternion(rot.getX(),rot.getY(),rot.getZ(),rot.getW()), origin - centerOffset);
		compound->addChildShape(btr,collide);
	} else if(comp.collisionModel=="Plane") {
		fmat::Column<3> normal = rot * fmat::UNIT3_Z;
		fmat::fmatReal dist = fmat::dotProduct(normal,pos) * world.spaceScale;
		btCollisionShape * collide = new btStaticPlaneShape(normal.exportTo<btVector3>(),dist);
		//collide->setMargin(world.collisionMargin*world.spaceScale); // 0 by default, maybe all static shapes should be 0...?
		compound->addChildShape(btTransform::getIdentity(),collide);
	} else {
#ifdef HAVE_OGRE
		/*
		CollisionData*& cd = collisionData[collisionDataName = comp.collisionModel];
		if(cd==NULL)
			cd = new CollisionData(comp.collisionModel,Client::loadMesh(comp.collisionModel));
		cd->addReference();
		 */
		// TODO add collision data to compound
		//collide->setMargin(2*world.spaceScale);
		//collide->recalculateLocalAabb(); // need manual recalc aabb on bullet collision shape after set margin
#endif
	}
}

void PhysicsBody::resetTransform() {
	btTransform pT_w;
	if(parentBody==NULL) {
		pT_w.setBasis(lastTransform.getBasis());
		pT_w.setOrigin(lastTransform.getOrigin());
	} else {
		pT_w = parentBody->body->getWorldTransform();
		const btVector3& parentOffset = parentBody->centerOffset;
		pT_w.setOrigin(pT_w * -parentOffset);
	}
	//std::cout << getPath() << " pT_w is\n" << pT_w << std::endl;
	
	if(qpos!=NULL)
		link.tryQ(*qpos);
	btTransform kjT_p;
	if(parentBody==NULL) {
		if(KinematicJoint* kj = dynamic_cast<KinematicJoint*>(&link)) {
			const fmat::Transform fm = kj->getFullT();
			kjT_p.setBasis(btMatrix3x3(fm(0,0),fm(0,1),fm(0,2), fm(1,0),fm(1,1),fm(1,2), fm(2,0),fm(2,1),fm(2,2)));
			kjT_p.setOrigin(btVector3(fm(0,3),fm(1,3),fm(2,3)) * world.spaceScale);
		} else {
			kjT_p = btTransform::getIdentity();
		}
	} else {
		KinematicJoint& parentKJ = dynamic_cast<KinematicJoint&>(parentBody->link);
		const fmat::Transform fm = dynamic_cast<KinematicJoint&>(link).getT(parentKJ);
		kjT_p.setBasis(btMatrix3x3(fm(0,0),fm(0,1),fm(0,2), fm(1,0),fm(1,1),fm(1,2), fm(2,0),fm(2,1),fm(2,2)));
		kjT_p.setOrigin(btVector3(fm(0,3),fm(1,3),fm(2,3)) * world.spaceScale);
	}
	//std::cout << getPath() << " kjT_p is\n" << kjT_p << std::endl;
	const btTransform kjT_w = pT_w * kjT_p;
	const btVector3 b_w = kjT_w * centerOffset;
	
	body->setWorldTransform(btTransform(kjT_w.getBasis(),b_w));
	/*if(string_util::beginsWith(getPath(),"/BaseFrame:Chiara/Body2")) {
		std::cout << getPath() << " set body at " << body->getWorldTransform().getOrigin() << std::endl;
		stacktrace::displayCurrentStackTrace();
	}*/
}	

void PhysicsBody::clear() {
	//std::cout << "Clearing " << getPath() << std::endl;
	if(body!=NULL) {
		ASSERT( (jointController==NULL && jointInterface==NULL) || (jointController!=NULL && jointInterface!=NULL), "Have some joint constraint objects, but not all?");
		if(jointController!=NULL) {
			world.motorControllers.erase(jointController);
			delete jointController; jointController=NULL;
		}
		if(jointInterface!=NULL) {
			delete jointInterface; jointInterface=NULL;
		}
		if(joint!=NULL) {
			world.dynamicsWorld->removeConstraint(joint);
			delete joint; joint=NULL;
		}
		lastTransform = body->getWorldTransform();
		lastTransform.setOrigin(lastTransform.getOrigin() - quatRotate(lastTransform.getRotation(),centerOffset));
		
		for(int i=compound->getNumChildShapes()-1; i>=0; --i) {
			delete compound->getChildShape(i);
			compound->removeChildShapeByIndex(i);
		}
		if(collisionDataName.size()>0)
			collisionData[collisionDataName]->removeReference();
		collisionDataName.clear();
		
		if(children.size()>0) {
			//std::cout << "Leaving " << getPath() << " for reuse (and not to break child constraints)" << std::endl;
		} else {
			world.dynamicsWorld->removeRigidBody(body);
			delete body; body=NULL;
			delete compound; compound = NULL;
		}

		/*
		if(kj.getParent()!=NULL)
			client.kjInfos[kj.getParent()]->physic.updateMass();
		for(KinematicJoint::branch_iterator it=kj.getBranches().begin(); it!=kj.getBranches().end(); ++it) {
			client.kjInfos[*it]->physic.updateBody(false);
		}
		 */
	} else {
		ASSERT(jointController==NULL,"have jointController but no body?");
		ASSERT(jointInterface==NULL,"have jointInterface but no body?");
		ASSERT(joint==NULL,"have joint but no body?");
		ASSERT(compound->getNumChildShapes()==0,"have collision shape but no body?");
		ASSERT(collisionDataName.size()>0,"have collisionDataName but no body?");
	}
}

void PhysicsBody::updateFrictionForce() {
	body->setFriction(link.frictionForce);
}

void PhysicsBody::updateAnistropicFriction() {
	body->setAnisotropicFriction(btVector3(link.anistropicFrictionRatio[0],link.anistropicFrictionRatio[1],link.anistropicFrictionRatio[2]));
}

bool PhysicsBody::isChild(const LinkComponent& pb, const LinkComponent& child) {
	if(const KinematicJoint* kj = dynamic_cast<const KinematicJoint*> (&pb)) {
		if(std::find(kj->components.begin(),kj->components.end(),&child) != kj->components.end())
			return true;
		for(KinematicJoint::branch_iterator it=kj->getBranches().begin(); it!=kj->getBranches().end(); ++it)
			if(isChild(**it,child))
				return true;
	}
	return false;
}



PhysicsBody::ComponentListener::~ComponentListener() {
	if(KinematicJoint* kj = dynamic_cast<KinematicJoint*>(&comp)) {
		kj->removeBranchListener(this);
		kj->components.removeCollectionListener(this);
	}
	for(std::map<LinkComponent*, ComponentListener*>::iterator it=subComps.begin(); it!=subComps.end(); ++it)
		delete it->second;
	subComps.clear();
	freeBody();
}

std::string PhysicsBody::ComponentListener::getPath() const {
	stringstream ss;
	if(const KinematicJoint* kj = dynamic_cast<const KinematicJoint*>(&comp)) {
		ss << kj->getPath() << ":" << getModel();
	} else if(parent!=NULL) {
		ss << parent->getPath() << "/" << getModel();
	} else {
		ss << "Free-" << getModel();
	}
	ss << "@" << &comp;
	return ss.str();
}

void PhysicsBody::ComponentListener::init() {
	//std::cout << "Creating listener for " << getPath() << std::endl;
	//if(parent==NULL)
	//	allocBody();
	if(KinematicJoint* kj = dynamic_cast<KinematicJoint*>(&comp)) {
		//std::cout << kj << ' ' << body << ' ' << kj->hasMass() << ' ' << hasCollisionShape(*kj) << std::endl;
		bool hasMass = kj->hasMass();
		bool hasCollision = hasCollisionShape(*kj);
		kj->components.addCollectionListener(this);
		kj->addBranchListener(this);
		
		// if mass and shape, valid independent body; if root and shape, it's a static obstacle; otherwise wait for a sub-branch to be added
		if(body==NULL && ( (hasMass && hasCollision) || (parent==NULL && hasCollision) ) )
			allocBody();

		plistCollectionEntriesChanged(kj->components);
	}
}

void PhysicsBody::ComponentListener::allocBody() {
	ASSERTRET(body==NULL,"attempted to allocate a new body without freeing previous")
	btTransform tr(btTransform::getIdentity());
	if(bodyLocation!=NULL)
		tr.getOrigin() = btVector3((*bodyLocation)[0],(*bodyLocation)[1],(*bodyLocation)[2]) * world.spaceScale;
	if(bodyOrientation!=NULL) {
		fmat::Quaternion quat = fmat::Quaternion::fromAxis(*bodyOrientation);
		tr.setRotation(btQuaternion(quat.getX(),quat.getY(),quat.getZ(),quat.getW()));
	}
	PhysicsBody* bodyParent=NULL;
	if(parent!=NULL)
		bodyParent = &parent->findBody();
	KinematicJoint& kj = dynamic_cast<KinematicJoint&>(comp);
	plist::Primitive<float>* pos = (kj.outputOffset==plist::OutputSelector::UNUSED) ? NULL : &positions[kj.outputOffset.get()];
	ASSERT(body==NULL,"creating new body, already had one!?");
	body = new PhysicsBody(world, kj, pos, tr, bodyParent);
	updateListeners();
}

void PhysicsBody::ComponentListener::freeBody() {
	if(body!=NULL) {
		delete collisionModelScaleListener; collisionModelScaleListener=NULL;
		delete collisionModelRotationListener; collisionModelRotationListener=NULL;
		delete collisionModelOffsetListener; collisionModelOffsetListener=NULL;
		delete centerOfMassListener; centerOfMassListener=NULL;
		delete body; body=NULL;
	}
}

PhysicsBody& PhysicsBody::ComponentListener::findBody() {
	if(body!=NULL)
		return *body;
	if(parent==NULL) {
		//throw std::runtime_error("Could not find body for ComponentListener");
		allocBody();
		return *body;
	}
	return parent->findBody();
}

PhysicsBody::ComponentListener& PhysicsBody::ComponentListener::findLinkRoot() {
	if(dynamic_cast<KinematicJoint*>(&comp) || parent==NULL)
		return *this;
	ASSERT(dynamic_cast<KinematicJoint*>(&parent->comp),"Parent is not a kinematic joint?")
	return *parent;
}

void PhysicsBody::ComponentListener::updateListeners() {
	delete collisionModelScaleListener; collisionModelScaleListener=NULL;
	delete collisionModelRotationListener; collisionModelRotationListener=NULL;
	delete collisionModelOffsetListener; collisionModelOffsetListener=NULL;
	delete centerOfMassListener; centerOfMassListener=NULL;
	PhysicsBody& b = findBody();
	if(comp.collisionModel=="Cube") {
		collisionModelScaleListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelScale,b,&PhysicsBody::rebuild, false);
		collisionModelRotationListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelRotation,b,&PhysicsBody::rebuild, false);
		collisionModelOffsetListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelOffset,b,&PhysicsBody::rebuild, false);
	}
	if(comp.mass!=0)
		centerOfMassListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.centerOfMass,b,&PhysicsBody::rebuild, false);
	for(std::map<LinkComponent*, ComponentListener*>::iterator it=subComps.begin(); it!=subComps.end(); ++it)
		if(it->second->body==NULL)
			it->second->updateListeners();
}

void PhysicsBody::ComponentListener::plistCollectionEntryAdded(plist::Collection& /*col*/, plist::ObjectBase& entry) {
	LinkComponent& lc = dynamic_cast<LinkComponent&>(entry);
	ComponentListener * listener = new ComponentListener(world,lc,positions,this);
	subComps[&lc] = listener;
	if(listener->body==NULL && (lc.hasMass() || hasCollisionShape(lc)))
		rebuildBody();
}

void PhysicsBody::ComponentListener::plistCollectionEntryRemoved(plist::Collection& /*col*/, plist::ObjectBase& entry) {
	LinkComponent& lc = dynamic_cast<LinkComponent&>(entry);
	bool willBeMissed = (lc.hasMass() || hasCollisionShape(lc));
	std::map<LinkComponent*, ComponentListener*>::iterator it=subComps.find(&lc);
	if(it==subComps.end()) {
		std::cerr << "ERROR: PhysicsBody::LinkComponentListener could not find entry in component list for removed component" << std::endl;
	} else {
		delete it->second;
		subComps.erase(it);
	}
	if(willBeMissed)
		rebuildBody();
}

void PhysicsBody::ComponentListener::plistCollectionEntriesChanged(plist::Collection& /*col*/) {
	size_t oldSize = subComps.size();
	for(std::map<LinkComponent*, ComponentListener*>::iterator it=subComps.begin(); it!=subComps.end(); ++it)
		delete it->second;
	subComps.clear();
	KinematicJoint& kj = dynamic_cast<KinematicJoint&>(comp);
	for(KinematicJoint::component_iterator it=kj.components.begin(); it!=kj.components.end(); ++it) {
		ComponentListener * cl = new ComponentListener(world,**it,positions,this);
		subComps[*it] = cl; // finish construction before adding to map so we don't have intermediary NULLs
	}
	for(KinematicJoint::branch_iterator it=kj.getBranches().begin(); it!=kj.getBranches().end(); ++it) {
		ComponentListener * cl = new ComponentListener(world,**it,positions,this);
		subComps[*it] = cl; // finish construction before adding to map so we don't have intermediary NULLs
	}
	if(oldSize!=0 || (subComps.size()!=0 && comp.hasMass() && hasCollisionShape(comp)))
		rebuildBody();
}


void PhysicsBody::ComponentListener::kinematicJointBranchAdded(KinematicJoint& kjparent, KinematicJoint& branch) {
	plistCollectionEntryAdded(kjparent, branch);
}

void PhysicsBody::ComponentListener::kinematicJointBranchRemoved(KinematicJoint& kjparent, KinematicJoint& branch) {
	plistCollectionEntryRemoved(kjparent, branch);
}

void PhysicsBody::ComponentListener::kinematicJointReconfigured(KinematicJoint& /*joint*/) {}


void PhysicsBody::ComponentListener::updateMass() {
	if(comp.mass==0) {
		// if mass is zero, center of mass doesn't matter
		delete centerOfMassListener;
		centerOfMassListener=NULL;
		// check that we haven't lost the body
		if(parent!=NULL) {
			// on a sub-branch, must have both mass and shape to keep independent body
			// (the root of an object can be zero mass or no shape -- indicates its base is bolted to ground (or otherwise not subject to simulation)
			ComponentListener& l = findLinkRoot();
			if(l.body!=NULL) {
				if(!l.comp.hasMass() || !hasCollisionShape(l.comp)) {
					l.freeBody();
				}
			}
		}
	} else {
		if(centerOfMassListener==NULL)
			centerOfMassListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.centerOfMass,findBody(),&PhysicsBody::rebuild, false);
		// check that we might need a new body
		if(parent!=NULL) {
			// on a sub-branch (root of an object always has a body)
			ComponentListener& l = findLinkRoot();
			if(l.body==NULL) {
				if(l.comp.hasMass() && hasCollisionShape(l.comp)) {
					rebuildBody(); // first trigger rebuild on the body we're splitting from
					l.allocBody(); // now make our own body
					return; // just created a new body, don't fall through to setup stuff below
				}
			}
		}
	}
	rebuildBody();
}

void PhysicsBody::ComponentListener::updateModel() {
	if(comp.collisionModel.size()==0) {
		// if collisionModel is blank, these don't matter
		delete collisionModelScaleListener; collisionModelScaleListener=NULL;
		delete collisionModelRotationListener; collisionModelRotationListener=NULL;
		delete collisionModelOffsetListener; collisionModelOffsetListener=NULL;
		// check that we haven't lost the body
		if(parent!=NULL) {
			// on a sub-branch, must have both mass and shape to keep independent body
			// (the root of an object can be zero mass or no shape -- indicates its base is bolted to ground (or otherwise not subject to simulation)
			ComponentListener& l = findLinkRoot();
			if(l.body!=NULL) {
				if(!l.comp.hasMass() || !hasCollisionShape(l.comp)) {
					l.freeBody();
				}
			}
		}
	} else {
		// check that we might need a new body
		if(parent!=NULL) {
			// on a sub-branch (root of an object always has a body)
			ComponentListener& l = findLinkRoot();
			if(l.body==NULL) {
				if(l.comp.hasMass() && hasCollisionShape(l.comp)) {
					rebuildBody(); // first trigger rebuild on the body we're splitting from
					l.allocBody(); // now make our own body
					return; // just created a new body, don't fall through to setup stuff below
				}
			}
		}
		if(collisionModelOffsetListener==NULL) {
			PhysicsBody& b = findBody();
			collisionModelScaleListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelScale,b,&PhysicsBody::rebuild, false);
			collisionModelRotationListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelRotation,b,&PhysicsBody::rebuild, false);
			collisionModelOffsetListener = new plist::CollectionCallbackMember<PhysicsBody>(comp.collisionModelOffset,b,&PhysicsBody::rebuild, false);
		}
	}
	rebuildBody();
}
	
bool PhysicsBody::ComponentListener::hasCollisionShape(const LinkComponent& c) {
	if(c.collisionModel.size()>0)
		return true;
	if(const KinematicJoint* kj = dynamic_cast<const KinematicJoint*>(&c)) {
		for(KinematicJoint::component_iterator it=kj->components.begin(); it!=kj->components.end(); ++it) {
			if((*it)->collisionModel.size()>0)
				return true;
		}
		for(KinematicJoint::branch_iterator it=kj->getBranches().begin(); it!=kj->getBranches().end(); ++it) {
			if(!(*it)->hasMass()) {
				if(hasCollisionShape(**it))
					return true;
			}
		}
	}
	return false;
}



#ifdef HAVE_OGRE
CollisionData::CollisionData(const std::string& meshName, Ogre::MeshPtr mesh)
	: name(meshName), vertices(NULL), indices(NULL) 
{
	// Calculate how many vertices and indices we're going to need
	bool added_shared = false;
	size_t vertex_count=0, index_count=0;
	for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i) {
		Ogre::SubMesh* submesh = mesh->getSubMesh(i);
		// We only need to add the shared vertices once
		if(submesh->useSharedVertices) {
			if( !added_shared ) {
				vertex_count += mesh->sharedVertexData->vertexCount;
				added_shared = true;
			}
		} else {
			vertex_count += submesh->vertexData->vertexCount;
		}
		// Add the indices
		index_count += submesh->indexData->indexCount;
	}
	
	// Allocate space for the vertices and indices
	
	vertices = new float[vertex_count*3];
	indices = new unsigned int[index_count];
	
	// Run through the submeshes again, adding the data into the arrays
	added_shared = false;
	size_t current_offset = 0;
	size_t shared_offset = 0;
	size_t next_offset = 0;
	size_t index_offset = 0;
	for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i) {
		Ogre::SubMesh* submesh = mesh->getSubMesh(i);
		
		Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
		
		if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared)) {
			if(submesh->useSharedVertices) {
				added_shared = true;
				shared_offset = current_offset;
			}
			
			const Ogre::VertexElement* posElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
			Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
			unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
			
			float* pReal;
			for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
			{
				posElem->baseVertexPointerToElement(vertex, &pReal);
				vertices[(current_offset + j)*3 + 0] = pReal[0]/1000;
				vertices[(current_offset + j)*3 + 1] = pReal[1]/1000;
				vertices[(current_offset + j)*3 + 2] = pReal[2]/1000;
				/*Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
				 vertices[current_offset + j] = (orient * (pt * scale)) + position;*/
				//std::cout << "Vertex " << j << ": " << fmat::SubVector<3>(&vertices[(current_offset + j)*3]) << std::endl;
			}
			
			vbuf->unlock();
			next_offset += vertex_data->vertexCount;
		}
		
		
		Ogre::IndexData* index_data = submesh->indexData;
		const size_t indexCnt = index_data->indexCount;
		Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
		
		bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
		
		unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
		unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);
		
		size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
		
		if ( use32bitindexes ) {
			for ( size_t k = 0; k < indexCnt; k+=3) {
				indices[index_offset++] = pLong[k+0] + static_cast<unsigned long>(offset);
				indices[index_offset++] = pLong[k+1] + static_cast<unsigned long>(offset);
				indices[index_offset++] = pLong[k+2] + static_cast<unsigned long>(offset);
				//std::cout << "Triangle " << k/3 << ": " << fmat::SubVector<3,dTriIndex>(&indices[index_offset-3]) << std::endl;
			}
		} else {
			for ( size_t k = 0; k < indexCnt; k+=3) {
				indices[index_offset++] = static_cast<unsigned long>(pShort[k+0]) + static_cast<unsigned long>(offset);
				indices[index_offset++] = static_cast<unsigned long>(pShort[k+1]) + static_cast<unsigned long>(offset);
				indices[index_offset++] = static_cast<unsigned long>(pShort[k+2]) + static_cast<unsigned long>(offset);
				//std::cout << "Triangle " << k/3 << ": " << fmat::SubVector<3,dTriIndex>(&indices[index_offset-3]) << std::endl;
			}
		}
		
		ibuf->unlock();
		current_offset = next_offset;
	}
	
	std::cout << "Vertices: " << vertex_count << std::endl;
	for(unsigned int i=0; i<vertex_count; ++i)
		std::cout << vertices[i*3+0] << ' ' << vertices[i*3+1] << ' ' << vertices[i*3+2] << std::endl;
	std::cout << "Indices: " << index_count << std::endl;
	for(unsigned int i=0; i<index_count/3; ++i)
		std::cout << indices[i*3+0] << ' ' << indices[i*3+1] << ' ' << indices[i*3+2] << std::endl;
}
#endif

#endif // HAVE_BULLET

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
