#include "Client.h"
#include "CameraInfo.h"
#include "ImageBuffer.h"
#include "EnvConfig.h"
#include "Physics.h"
#include "UIController.h"
#include "GraphicsInfo.h"
#include "CommThread.h"

#include "Shared/debuget.h"
#include "Shared/FamilyFactory.h"
#include "Shared/newmat/newmatap.h"

#include <algorithm>
#include <numeric>
#include <functional>
#include <cmath>

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#ifdef __APPLE__
#  include <Carbon/Carbon.h>
#  include <Ogre/Ogre.h>
#else
#  include <OGRE/Ogre.h>
#endif

using namespace std; 

std::map<std::string,Client*> Client::clients;

Client& Client::getClient(const std::string& clientID) {
	std::map<std::string,Client*>::iterator it=clients.find(clientID);
	if(it!=clients.end())
		return *it->second;
	throw std::runtime_error("Client does not exist");
}

Client& Client::getIncomingClient(const std::string& clientID) {
	std::map<std::string,Client*>::iterator it=clients.find(clientID);
	if(it!=clients.end())
		return *it->second;
	std::cout << "Creating client " << clientID << std::endl;
	Client* cl = new Client(clientID);
	cl->addReference();
	clients.insert(std::pair<std::string,Client*>(clientID,cl));
	return *cl;
}

void Client::removeClient(const std::string& clientID) {
	std::map<std::string,Client*>::iterator it=clients.find(clientID);
	if(it==clients.end())
		return;
	if(!it->second->removeReference())
		std::cerr << "WARNING: removed client '" << it->first <<"', but was not last reference" << std::endl;
}

void Client::removeAllClients() {
	// removing last reference will erase from clients
	// we could just use while(!clients.empty()) and clients.front(), but nicer to respect external references
	// so instead, copy clients out so we can remove one time each
	std::vector<Client*> tmpcl;
	tmpcl.reserve(clients.size());
	for(std::map<std::string,Client*>::iterator it=clients.begin(); it!=clients.end(); ++it)
		tmpcl.push_back(it->second);
	std::for_each(tmpcl.begin(),tmpcl.end(),std::mem_fun(&Client::removeReference));
	// if there are any external references, display a warning
	for(std::map<std::string,Client*>::iterator it=clients.begin(); it!=clients.end(); ++it)
		std::cerr << "WARNING: removing all clients, '" << it->first <<"' was not last reference" << std::endl;
}

void Client::renderCameras() {
	for(std::map<std::string,Client*>::const_iterator it=clients.begin(); it!=clients.end(); ++it) {
		MarkScope autolock(it->second->lock);
		plist::ArrayOf<CameraInfo>& cams = it->second->cameras;
		for_each(cams.begin(), cams.end(), mem_fun(&CameraInfo::renderImage));
	}
}

void Client::updateGraphics() {
	for(std::map<std::string,Client*>::const_iterator it=clients.begin(); it!=clients.end(); ++it) {
		Client& cl = *it->second;
		if(cl.physicsListener==NULL)
			continue; // has not been activated yet
		MarkScope autolock(cl.lock);
		// check updateContacts() docs as to when it is used...
		if( ( cl.fixedContactPoints.size()>0 && !cl.model.hasMass() )
		   || ( cl.wheels.size()>0 && (!cl.model.hasMass() || !cl.physicsWheels) ) )
			cl.updateContacts();
		cl.updateGraphics(cl.model);
		/*if(PhysicsBody * body = Physics::singleton().getBody(cl.model)) {
			fmat::Column<3> pos;
			fmat::Quaternion quat;
			body->getCurrent(pos,quat);
			if(it->first=="RobotID-1")
				std::cout << pos << ' ' << quat << std::endl;
		}*/
		cl.updateSensors();
	}
}

void Client::removeSensorListenerEverywhere(SensorListener* listener) {
	for(std::map<std::string,Client*>::const_iterator it=clients.begin(); it!=clients.end(); ++it) {
		if(it->second->sensorListeners.erase(listener)) {
			std::cerr << "ERROR found leaked listener for " << it->first << std::endl;
		}
	}
}

//! Receives contact points for a specified body via dynamics world's contactTest() call via Client::updateSensors, runs KinematicJoint::SensorContact tests
struct ContactSensorCallback : public btCollisionWorld::ContactResultCallback {
	//! constructor, sets sensor value to 0 (will be reset if contact is found in addSingleResult())
	ContactSensorCallback(SensorContact& sensor_, btRigidBody& body_, const fmat::Transform* trans_)
		: btCollisionWorld::ContactResultCallback(), sensor(sensor_), body(body_), trans(trans_) { sensor.value=0; }

	SensorContact& sensor; //!< the sensor being processed
	btRigidBody& body; //!< the body the sensor is monitoring
	const fmat::Transform* trans; //!< an optional transform, in case the sensor is relative to a particular CollisionShape that doesn't have its own body
	
	//! don't consider collisions where the bodies are joined by a constraint
	virtual bool needsCollision(btBroadphaseProxy* proxy) const {
		return body.checkCollideWithOverride(static_cast<btCollisionObject*>(proxy->m_clientObject));
	}
	
	//! test each contact to see if it falls in the sensor's parameters; if so set sensor value to 1
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int /*partId0*/,int /*index0*/,
		const btCollisionObject* colObj1,int /*partId1*/,int /*index1*/)
	{
		fmat::Column<3> pt;
		const btVector3& off = static_cast<PhysicsBody*>(body.getUserPointer())->getCenterOffset();
		//std::cout << "Collide " << static_cast<PhysicsBody*>(colObj0->getUserPointer())->getPath() << ' ' << static_cast<PhysicsBody*>(colObj1->getUserPointer())->getPath() << std::endl;
		if(colObj0==&body) {
			pt.importFrom(cp.m_localPointA+off);
		} else {
			ASSERTRETVAL(colObj1==&body,"body does not match either collision object",0);
			pt.importFrom(cp.m_localPointB+off);
		}
		pt /= Physics::singleton().spaceScale;
		if(trans!=NULL)
			pt = (*trans) * pt;
		//std::cout << pt << std::endl;
		if(sensor.testPoint(pt[0],pt[1],pt[2]))
			sensor.value=1;
		return 0;
	}
};

void Client::updateSensors() {
	if(sensors.size()==0
	   && EnvConfig::singleton().mocapPos==EnvConfig::Environment::MOCAP_NONE
	   && EnvConfig::singleton().mocapOri==EnvConfig::Environment::MOCAP_NONE)
		return;
	for(std::map<JointGraphicsInfo*, SensorInfo*>::const_iterator it=sensors.begin(); it!=sensors.end(); ++it) {
		if(SensorRangeFinder* rf = dynamic_cast<SensorRangeFinder*>(it->second)) {
			// this seems to have a bit of latency:
			Ogre::Matrix4 m = it->first->getJointNode()._getFullTransform();
			Ogre::Vector3 from = m*Ogre::Vector3(0,0,rf->minRange) * Physics::getSpaceScale();
			Ogre::Vector3 to = m*Ogre::Vector3(0,0,rf->maxRange) * Physics::getSpaceScale(); 
			btVector3 btFrom(from.x,from.y,from.z);
			btVector3 btTo(to.x,to.y,to.z);
			/*fmat::Column<3> tpos;
			fmat::Quaternion tquat;
			Physics::singleton().getBody(it->first->getKJ())->getCurrent(tpos,tquat);
			btQuaternion r(tquat.getX(),tquat.getY(),tquat.getZ(),tquat.getW());
			btVector3 bodyOff(tpos[0],tpos[1],tpos[2]);
			btVector3 btFrom = r*btVector3(0,0,rf->minRange * Physics::getSpaceScale()) + bodyOff;
			btVector3 btTo = r*btVector3(0,0,rf->maxRange * Physics::getSpaceScale()) + bodyOff;*/
			btCollisionWorld::ClosestRayResultCallback rayCallback(btFrom,btTo);
			Physics::singleton().getWorld().rayTest(btFrom, btTo, rayCallback);
			if(!rayCallback.hasHit()) {
				rf->value = rf->maxRange;
				it->first->updateRangeFinderNodes(false,*rf);
				//std::cout << "From " << btFrom[0] <<','<<btFrom[1]<<','<<btFrom[2] << " To " << btTo[0] <<','<<btTo[1]<<','<<btTo[2] << " no hit" << std::endl;
			} else {
				btVector3 btHit = rayCallback.m_hitPointWorld;
				rf->value = (btHit-btFrom).length()/Physics::getSpaceScale() + rf->minRange;
				it->first->updateRangeFinderNodes(true,*rf);
				if(rf->value < rf->saturationRange)
					rf->value = rf->saturationRange;
				//std::cout << "From " << btFrom[0] <<','<<btFrom[1]<<','<<btFrom[2] << " To " << btTo[0] <<','<<btTo[1]<<','<<btTo[2] << " Hit " << btHit[0] << ',' << btHit[1] << ',' << btHit[2] << " dist " << (btHit-btFrom).length() << std::endl;
			}
		} else if(SensorContact* c = dynamic_cast<SensorContact*>(it->second)) {
			PhysicsBody * pbody;
			const KinematicJoint * kj, &orig=it->first->getKJ();
			findBody(orig, pbody, kj);
			if(kj==NULL) {
				std::cerr << "WARNING: body not found for contact sensor" << std::endl;
				continue;
			}
			fmat::Transform trans;
			fmat::Transform* transptr;
			if(kj==&orig) {
				transptr=NULL; // no transform necessary
			} else {
				trans = kj->getT(orig);
				transptr = &trans;
			}
			btRigidBody& body = pbody->getBody();
			ContactSensorCallback callback(*c,body,transptr);
			Physics::singleton().getWorld().contactTest(&body,callback);
		} else if(SensorFeedback* fb = dynamic_cast<SensorFeedback*>(it->second)) {
			const KinematicJoint& kj = it->first->getKJ();
			PhysicsBody * pbody = Physics::singleton().getBody(kj);
			if(pbody==NULL || !pbody->hasJoint()) {
				fb->value = kj.getQ();
			} else {
				fb->value = pbody->getJoint();
			}
		} else if (GPSSensor* g = dynamic_cast<GPSSensor*>(it->second)) {
			Ogre::Matrix4 m = it->first->getJointNode()._getFullTransform();
			g->curX = m[0][3];
			g->curY = m[1][3];
			g->curZ = m[2][3];
			g->curHeading = std::atan2(m[1][0], m[0][0]);
		} else if (OdometrySensor* o = dynamic_cast<OdometrySensor*>(it->second)) {
			Ogre::Matrix4 m = it->first->getJointNode()._getFullTransform();
			Ogre::Vector3 displacements(m[0][3] - o->lastX, m[1][3] - o->lastY, m[2][3] - o->lastZ);
			Ogre::Matrix3 rotation;
			m.extract3x3Matrix(rotation);
			rotation = rotation.Transpose();
			displacements = rotation * displacements;		 
			float heading = std::atan2(m[1][0], m[0][0])*180/(float)M_PI;
			float differenceInAngle = heading - o->lastHeading;
			if (differenceInAngle > 180)
				differenceInAngle -= 360;
			if (differenceInAngle < -180)
				differenceInAngle += 360;
			o->lastX = m[0][3];
			o->lastY = m[1][3];
			o->lastZ = m[2][3];
			o->lastHeading = heading;
			o->deltaX = displacements[0];
			o->deltaY = displacements[1];
			o->deltaZ = displacements[2];
			o->deltaHeading = differenceInAngle;
			o->cumX += displacements[0];
			o->cumY += displacements[1];
			o->cumZ += displacements[2];
			o->cumHeading += differenceInAngle;
			// simulate encoder values if requesteed
			if ( o->wheelBase * o->wheelCircumference > 0 ) {
				float avgTicks = displacements[0] / o->wheelCircumference * o->encoderTicksPerRev;
				float diffTicks = (differenceInAngle*M_PI/180) * o->wheelBase
					/ o->wheelCircumference * o->encoderTicksPerRev;
				o->leftEncoder += avgTicks - diffTicks/2;
				o->rightEncoder += avgTicks + diffTicks/2;
			}
		} else {
			std::cerr << __FILE__ << ":" << __LINE__ << "Unknown sensor type " << it->second->sensorType << std::endl;
		}
	}
	
	switch(EnvConfig::singleton().mocapPos) {
		case EnvConfig::Environment::MOCAP_ALL: {
			for(std::map<KinematicJoint*, JointGraphicsInfo*>::const_iterator it=kjInfos.begin(); it!=kjInfos.end(); ++it) {
				if(it->first->outputOffset!=plist::OutputSelector::UNUSED) {
					sensorValues.framePositions[it->first->outputOffset.get()].importFrom(it->second->getJointNode()._getDerivedPosition());
				}
			}
		} break; // already includes everything, don't fall through
			
		case EnvConfig::Environment::MOCAP_LEAVES: {
			recordLeafFrames(model,true);
		} // purposely fall through to include root
			
		case EnvConfig::Environment::MOCAP_ROOT: {
			Ogre::Vector3 p = kjInfos[&model]->getJointNode()._getDerivedPosition();
			sensorValues.framePositions[model.outputOffset.get()].importFrom(p);
		} break;
			
		case EnvConfig::Environment::MOCAP_NONE: {
			sensorValues.framePositions.clear();
		} break;
	}
	
	switch(EnvConfig::singleton().mocapOri) {
		case EnvConfig::Environment::MOCAP_ALL: {
			for(std::map<KinematicJoint*, JointGraphicsInfo*>::const_iterator it=kjInfos.begin(); it!=kjInfos.end(); ++it) {
				if(it->first->outputOffset!=plist::OutputSelector::UNUSED) {
					Ogre::Quaternion q = it->second->getJointNode()._getDerivedOrientation();
					if(q.w<0)
						sensorValues.frameOrientations[it->first->outputOffset.get()].set(-q.x,-q.y,-q.z);
					else
						sensorValues.frameOrientations[it->first->outputOffset.get()].set(q.x,q.y,q.z);
				}
			}
		} break; // already includes everything, don't fall through
			
		case EnvConfig::Environment::MOCAP_LEAVES: {
			recordLeafFrames(model,false);
		} // purposely fall through to include root
			
		case EnvConfig::Environment::MOCAP_ROOT: {
			Ogre::Quaternion q = kjInfos[&model]->getJointNode()._getDerivedOrientation();
			if(q.w<0)
				sensorValues.frameOrientations[model.outputOffset.get()].set(-q.x,-q.y,-q.z);
			else
				sensorValues.frameOrientations[model.outputOffset.get()].set(q.x,q.y,q.z);
		} break;
			
		case EnvConfig::Environment::MOCAP_NONE: {
			sensorValues.frameOrientations.clear();
		} break;
	}
	
	for(std::set<SensorListener*>::const_iterator it=sensorListeners.begin(); it!=sensorListeners.end(); ++it)
		(*it)->sensorsUpdated(sensorValues);
}
void Client::recordLeafFrames(KinematicJoint& kj, bool pos) {
	const std::set<KinematicJoint*>& branches = kj.getBranches();
	if(branches.size()>0) {
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			recordLeafFrames(**it,pos);
	} else {
		if(pos) {
			sensorValues.framePositions[kj.outputOffset.get()].importFrom(kjInfos[&kj]->getJointNode()._getDerivedPosition());
		} else {
			Ogre::Quaternion q = kjInfos[&kj]->getJointNode()._getDerivedOrientation();
			if(q.w<0)
				sensorValues.frameOrientations[kj.outputOffset.get()].set(-q.x,-q.y,-q.z);
			else
				sensorValues.frameOrientations[kj.outputOffset.get()].set(q.x,q.y,q.z);
		}
	}
}


void Client::init() {
	addEntry("ID",robotID);
	addEntry("Persist",persist);
	addEntry("Location",location);
	addEntry("Orientation",orientation);
	addEntry("Model",model);
	addEntry("Positions",positions);
	addEntry("PhysicsWheels",physicsWheels);
	addEntry("Cameras",cameras);
	addEntry("FixedPoints",fixedPoints);
	robotID.addPrimitiveListener(this);
	positions.setLoadSavePolicy(UNION,SYNC);
	cameras.setLoadSavePolicy(SYNC,SYNC);
	cameras.addCollectionListener(this);
	physicsWheels.addPrimitiveListener(this);
	setLoadSavePolicy(FIXED,SYNC);
	sensorValues.setSaveCondensed(true);
}

void Client::activate() {
	for(unsigned int i=0; i<location.size(); ++i)
		location[i].addPrimitiveListener(this);
	for(unsigned int i=0; i<orientation.size(); ++i)
		orientation[i].addPrimitiveListener(this);
	if(physicsListener==NULL) {
		// construct physics before calling kJBA so graphics infos can access physics stuff during their initialization
		// (otherwise need to do the JointGraphicsInfo::rebuildCenterOfMass() thing for every kjInfos->second, see "Model" case in loadXMLNode())
		physicsListener = new PhysicsBody::ComponentListener(Physics::singleton(),model,positions,&location,&orientation);
		kinematicJointBranchAdded(model,model);
		kjInfos[&model]->rebuildCenterOfMass(); // root missed physics, so rebuild now that it's available
		// implied by kinematicJointBranchAdded:  kjInfos[&model] = new JointGraphicsInfo(*this, model, *ogreSceneMgr->getRootSceneNode());
		ui->clientConnected(*this);
	}
}

bool Client::hasListeners() {
	if(!sensorListeners.empty())
		return true;
	for(plist::ArrayOf<CameraInfo>::const_iterator it=cameras.begin(); it!=cameras.end(); ++it)
		if((*it)->getNumImageListeners()>0)
			return true;
	return false;
}

bool Client::removeReference() {
	if(getReferences()==1)
		clients.erase(robotID);
	return ReferenceCounter::removeReference();
}

Client::~Client() {
	lock.lock();
	delete physicsListener;
	physicsListener = NULL;
	CommThread::dropClient(robotID);
	ui->clientDisconnecting(*this);
	kinematicJointBranchRemoved(model,model);
	// implied by kinematicJointBranchRemoved call:
	//model.clearBranches();
	//delete kjInfos[&model];
	//kjInfos.erase(&model);
	//model.removeBranchListener(this);
	for(std::map<KinematicJoint*,JointGraphicsInfo*>::const_iterator it=kjInfos.begin(); it!=kjInfos.end(); ++it)
		std::cerr << "Leftover KinematicJointInfo: " << it->first->outputOffset.get() << " - " << it->first->model << std::endl;
	cameras.removeCollectionListener(this);
	robotID.removePrimitiveListener(this);
	for(unsigned int i=0; i<fixedVis.size(); ++i)
		ogreSceneMgr->destroySceneNode(fixedVis[i]);
	fixedVis.clear();
}

Ogre::MeshPtr Client::loadMesh(const std::string& mesh) {
	// prefabs are size 100 instead of 1, and kind of low res... not really anything special, might as well use our own meshes
	/*if(mesh=="Cube" || mesh=="cube") {
	 return Ogre::MeshManager::getSingleton().load("Prefab_Cube",Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
	 } else if(mesh=="Sphere" || mesh=="sphere") {
	 return Ogre::MeshManager::getSingleton().load("Prefab_Sphere",Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
	 } else if(mesh=="Square" || mesh=="square") {
	 return Ogre::MeshManager::getSingleton().load("Prefab_Plane",Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
	 } else*/ {
		 return Ogre::MeshManager::getSingleton().load(mesh+".mesh", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	 }
}

Ogre::Entity* Client::createEntity(const std::string& identifier, const std::string& mesh) const {
	static unsigned int modelCount=0;
	std::stringstream ss;
	ss << robotID << "-" << identifier << "/" << modelCount++;
	// prefabs are size 100 instead of 1, and kind of low res... not really anything special, might as well use our own meshes
	/*if(mesh=="Cube" || mesh=="cube") {
		return ogreSceneMgr->createEntity(ss.str(),Ogre::SceneManager::PT_CUBE);
	} else if(mesh=="Sphere" || mesh=="sphere") {
		return ogreSceneMgr->createEntity(ss.str(),Ogre::SceneManager::PT_SPHERE);
	} else if(mesh=="Square" || mesh=="square") {
		return ogreSceneMgr->createEntity(ss.str(),Ogre::SceneManager::PT_PLANE);
	} else*/ {
		return ogreSceneMgr->createEntity(ss.str(),mesh+".mesh");
	}
}

bool Client::loadXMLNode(const std::string& name, xmlNode* val, const std::string& comment) {
	MarkScope autolock(lock);
	if(name=="FixedPoints" || name=="Location" || name=="Orientation") {
		// there might also have been changes to joint positions, apply these under the old contacts first
		// check updateContacts() docs as to when it is used...
		if( ( fixedContactPoints.size()>0 && !model.hasMass() )
		   || ( wheels.size()>0 && (!model.hasMass() || !physicsWheels) ) )
			updateContacts(); // implies updateKJTargets(model) if fixedPoints.size()>0
	}
	try {
		if(!physicsWheels && name=="Positions") {
			// Redirect wheel values to wheelSpeeds for updateContacts to use, but not motor controllers
			plist::DictionaryOf<plist::Primitive<float> > pos;
			pos.loadXML(val);
			for(plist::DictionaryOf<plist::Primitive<float> >::const_iterator it=pos.begin(); it!=pos.end(); ++it) {
				unsigned int offset = capabilities.getOutputOffset(it->first); // won't fail, TGT_DYNAMIC creates a new entry
				std::map<unsigned int, KinematicJoint*>::const_iterator kj = childMap.find(offset);
				if(kj==childMap.end()) {
					// is normal, there are joints without kinematics, e.g. currently LEDs
					//std::cerr << "Client " << robotID << " could not find joint " << it->first << " for position " << it->second << std::endl;
				} else {
					std::set<KinematicJoint*>::const_iterator w = wheels.find(kj->second);
					if(w!=wheels.end()) {
						wheelSpeeds[*w] = *it->second;
					} else {
						positions[it->first] = *it->second;
					}
				}
			}
			return true;
		}
		if(!plist::Dictionary::loadXMLNode(name,val,comment))
			return false;
	} catch(const std::exception& ex) {
		std::cerr << "An exception occurred loading '" << name << "' field: " << ex.what() << std::endl;
		return false;
	} catch(...) {
		std::cerr << "An unknown exception occurred loading '" << name << "'" << std::endl;
		return false;
	}
	if(name=="Model") {
		// have to reprocess inertiaNodes because of listener race between graphics and physics during initialization
		//std::cout << "REBUILDING CENTER OF MASS " << kjInfos.size() << std::endl;
		for(std::map<KinematicJoint*, JointGraphicsInfo*>::const_iterator it = kjInfos.begin(); it!=kjInfos.end(); ++it)
			it->second->rebuildCenterOfMass();
	} else if(name=="FixedPoints" || (fixedContactPoints.size()>0 && (name=="Location" || name=="Orientation"))) {
		// either the body was teleported, or the fixed points themselves changed... either way, update fixed points
		bool hasMass = model.hasMass();
		const Ogre::SceneNode& jointNode = kjInfos[&model]->getJointNode();
		
		if(hasMass) {
			//updateKJPositions(model); // was not already done in updateContacts call above
			fixedInitialPosition = 0;//jointNode.getPosition();
			fixedInitialOrientation = Ogre::Quaternion(); //jointNode.getOrientation();
			
			for(unsigned int i=0; i<fixedConstraints.size(); ++i)
				Physics::singleton().removeConstraint(fixedConstraints[i]);
			fixedConstraints.resize(fixedPoints.size());
			fixedContactPoints.resize(fixedPoints.size());
			for(unsigned int i=0; i<fixedConstraints.size(); ++i) {
				PhysicsBody * body;
				const KinematicJoint * kj, &orig=*childMap[fixedPoints[i].frame];
				findBody(orig, body, kj);
				if(kj==NULL) {
					std::cerr << "WARNING: body not found for fixed point with mass" << std::endl;
					continue;
				}
				fmat::Column<3> pt;
				fixedPoints[i].point.exportTo(pt);
				if(&orig!=kj)
					pt = orig.getT(*kj) * pt;
				fixedConstraints[i] = Physics::singleton().addPointConstraint(body, pt);
				
				fmat::Column<3> bPos;
				fmat::Quaternion bOri;
				body->getCurrent(bPos,bOri);
				fixedContactPoints[i] = bOri*pt + bPos;
			}
		} else {
			if(fixedVis.size()==0 && fixedPoints.size()>0)
				updateKJTargets(model); // was not already done in updateContacts call above
			calcFixedPoints(fixedContactPoints);
			filterPoints(fixedContactPoints);
			
			//std::cout << "fixed points ("<<fixedContactPoints.size()<<")"/* updated at " << fixedInitialPosition << " " << fixedInitialOrientation*/ << std::endl;
			fmat::Column<3> p;
			p.importFrom(jointNode.getPosition());
			fmat::Quaternion q = fmat::Quaternion::from(jointNode.getOrientation());
			std::vector<fmat::Column<3> > worldContactPoints, projectedContactPoints;
			worldContactPoints.reserve(fixedContactPoints.size());
			projectedContactPoints.reserve(fixedContactPoints.size());
			//std::cout << "ORIG POS " << p << " ORI " << q << " tol=" << (q.norm()-1) << std::endl;
			for(unsigned int i=0; i<fixedContactPoints.size(); ++i) {
				worldContactPoints.push_back(q*fixedContactPoints[i] + p);
				// leave z at 0, assumes flat world plane... project down to collision instead?
				float zval = 0;
				/* // except not quite 0 to avoid matrix degenerency in fitTransform's Ogre::Matrix3 SVD call, which currently isn't handled correctly
				 float r = random() / float(1<<30) - 1;
				std::cout << "rand " << r << std::endl;
				float zval = r * .5; */
				projectedContactPoints.push_back(fmat::pack( worldContactPoints[i][0], worldContactPoints[i][1], zval ));
				//std::cout << i << "  Body " << fixedContactPoints[i] << std::endl;
				//std::cout << i << " World " << worldContactPoints.back() << std::endl;
				//std::cout << i << "  Proj " << projectedContactPoints.back() << std::endl;
			}
			float totProjErr=0;
			for(unsigned int i=0; i<fixedContactPoints.size(); ++i)
				totProjErr += (worldContactPoints[i] - projectedContactPoints[i]).norm();
			//std::cout << "Projection error is " << totProjErr << std::endl;
			const float TOTAL_PROJ_ERR_TOL = 1.0f;
			if(totProjErr < TOTAL_PROJ_ERR_TOL) {
				// close enough, just use current position
				fixedInitialPosition = jointNode.getPosition();
				fixedInitialOrientation = jointNode.getOrientation();
			} else {
				// significantly out of place, re-project to ground
				Ogre::Vector3 projPosOff;
				Ogre::Quaternion projOriOff(1,0,0,0);
				fitTransform(projectedContactPoints,worldContactPoints, projPosOff,projOriOff);
				fixedInitialPosition = projOriOff * jointNode.getPosition() + projPosOff;
				fixedInitialOrientation = projOriOff * jointNode.getOrientation();
				fixedInitialOrientation.normalise();
				
				//std::cout << "PROJECTION ERR POS " << projPosOff << " ORI " << projOriOff << " tol=" << (std::sqrt(projOriOff.Norm())-1) << std::endl;
				//std::cout << "PROJECTED POS " << fixedInitialPosition << " ORI " << fixedInitialOrientation << " tol=" << (std::sqrt(fixedInitialOrientation.Norm())-1) << std::endl;
				/*for(unsigned int i=0; i<fixedContactPoints.size(); ++i) {
				 Ogre::Vector3 fcp;
				 fixedContactPoints[i].exportTo(fcp);
				 fmat::Column<3> newpt;
				 newpt.importFrom(fixedInitialOrientation*fcp + fixedInitialPosition);
				 std::cout << i << " RePrj " << newpt << "   Err " << newpt - projectedContactPoints[i] << std::endl;
				 }*/
			}
		}
		
		unsigned int oldSize = fixedVis.size();
		for(unsigned int i=fixedPoints.size(); i<fixedVis.size(); ++i)
			ogreSceneMgr->destroySceneNode(fixedVis[i]);
		fixedVis.resize(fixedPoints.size());
		Ogre::Vector3 fcp;
		for(unsigned int i=0; i<oldSize && i<fixedVis.size(); ++i) {
			fixedContactPoints[i].exportTo(fcp);
			Ogre::Vector3 fp = (fixedInitialOrientation * fcp) + fixedInitialPosition;
			fixedVis[i]->setPosition(fp);
		}
		for(unsigned int i=oldSize; i<fixedVis.size(); ++i) {
			fixedContactPoints[i].exportTo(fcp);
			Ogre::Vector3 fp = (fixedInitialOrientation * fcp) + fixedInitialPosition;
			fixedVis[i] = ogreSceneMgr->getRootSceneNode()->createChildSceneNode(fp);
			fixedVis[i]->setScale(8,8,30);
			stringstream ss;
			ss << "FixedPoint" << i;
			Ogre::Entity * ent = createEntity(ss.str(), "Sphere");
			if(ent==NULL)
				throw std::bad_alloc();
			fixedVis[i]->attachObject(ent);
			ent->setMaterialName("Pink");
		}		
	}
	return true;
}

void Client::updateKJTargets(KinematicJoint& kj) {
	if(kj.outputOffset!=plist::OutputSelector::UNUSED) {
		kj.setQ(positions[kj.outputOffset.get()]);
		//std::cout << "set kj " << kj.outputOffset.get() << " to " << kj.getQ() << std::endl;
	}
	for(KinematicJoint::branch_iterator it = kj.getBranches().begin(); it!=kj.getBranches().end(); ++it)
		updateKJTargets(**it);
}

void Client::updateKJPositions(KinematicJoint& kj) {
	if(kj.outputOffset!=plist::OutputSelector::UNUSED) {
		if(kj.controllerInfo.velocity) {
			kj.setQ(positions[kj.outputOffset.get()]);
		} else {
			PhysicsBody * body = Physics::singleton().getBody(kj);
			if(body==NULL)
				kj.setQ(positions[kj.outputOffset.get()]);
			else if(body->hasJoint()) // avoid base frame (jointInterface is NULL)
				kj.setQ(body->getJoint());
		}
		//std::cout << "set kj " << kj.outputOffset.get() << " to " << kj.getQ() << std::endl;
	}
	for(KinematicJoint::branch_iterator it = kj.getBranches().begin(); it!=kj.getBranches().end(); ++it)
		updateKJPositions(**it);
}

void Client::updateContacts() {
	MarkScope autolock(lock);
	updateKJPositions(model);
	Ogre::SceneNode& jointNode = kjInfos[&model]->getJointNode();
	PhysicsBody * body = Physics::singleton().getBody(model);
	
	fmat::Column<3> pNext;
	fmat::Quaternion qNext;
	
	if(fixedContactPoints.size()>0) {
		std::vector<fmat::Column<3> > contacts;
		calcFixedPoints(contacts);
		filterPoints(contacts);
		//for(unsigned int i=0; i<contacts.size(); ++i)
		//	std::cout << i << "  Move " << contacts[i] << std::endl;
		Ogre::Vector3 nodeOff;
		Ogre::Quaternion nodeRot(1,0,0,0);
		fitTransform(fixedContactPoints, contacts, nodeOff, nodeRot);
		
		pNext.importFrom(fixedInitialPosition + fixedInitialOrientation*nodeOff);
		qNext.importFrom(nodeRot * fixedInitialOrientation);
		/*jointNode.setPosition(fixedInitialPosition);
		jointNode.setOrientation(fixedInitialOrientation);
		jointNode.translate(nodeOff, Ogre::Node::TS_LOCAL);
		jointNode.rotate(nodeRot, Ogre::Node::TS_LOCAL);*/
		//std::cout << "OFFSET POS " << nodeOff << " ORI " << nodeRot << " tol=" << (std::sqrt(nodeRot.Norm())-1) << std::endl;
		//std::cout << "FINAL " << poff << " " << qoff << std::endl;
		//std::cout << "CURRENT POS " << jointNode.getPosition() << " ORI " << jointNode.getOrientation() << " tol=" << (std::sqrt(jointNode.getOrientation().Norm())-1) << std::endl;
	}
	
	if(wheels.size()>0) {
		fmat::Column<3> poff;
		float aoff=0;
		
		fmat::Column<3> cof;
		for(std::set<KinematicJoint*>::const_iterator it=wheels.begin(); it!=wheels.end(); ++it)
			cof += (*it)->getFullT().translation();
		
		for(std::set<KinematicJoint*>::const_iterator it=wheels.begin(); it!=wheels.end(); ) {
			const KinematicJoint& kj = **it;
			std::map<const KinematicJoint*,float>::const_iterator wsit = wheelSpeeds.find(&kj);
			// updateContacts may still be called when robot has no mass, forces non-physics mode, use kj.getQ()
			// ASSERTRET(wsit!=wheelSpeeds.end(), "updateContacts, non-physics wheel motion, but wheel speed not found");
			const float wheelVel = (wsit!=wheelSpeeds.end()) ? wsit->second : kj.getQ();
			fmat::Transform t = kj.getFullT();
			fmat::Column<3> dir = fmat::pack(t(1,2),-t(0,2),0.f);
			fmat::fmatReal dm=dir.norm();
			if(dm<std::numeric_limits<float>::epsilon()*10) {
				std::cerr << "WARNING: kinematic joint " << kj.getPath() << " is marked as wheel, but axle is vertical, ignoring it" << std::endl;
				poff=aoff=0;
				wheels.erase(it);
				it=wheels.begin();
				continue; // wheel is on its side, does not contribute
			}
			dir/=dm;
			poff += dir * wheelVel;

			fmat::Column<3> pos = t.translation();
			fmat::Column<3> curCof = (cof - pos) / (wheels.size()-1);
			fmat::Column<3> norm = (pos - curCof);
			norm -= fmat::dotProduct(dir,norm) * dir;
			float base = norm.norm();
			const float MAX_ROT = 2; // limit crazy rotational speeds in case of short base
			float aInc = (base*MAX_ROT > std::abs(wheelVel) ) ? wheelVel / base : 0; // send limit to zero because centrally applied instead of inf. rotation
			if(norm[0]*dir[1] > norm[1]*dir[0]) // i.e. if z component of crossProduct(norm, dir) > 0
				aoff += aInc;
			else
				aoff -= aInc;
			/*std::cout << "Wheel " << kj.getPath() << " @ " << wheelVel << ":" << std::endl;
			std::cout << "\tpos " << pos << std::endl;
			std::cout << "\tdir " << dir << std::endl;
			std::cout << "\tcurCof " << curCof << std::endl;
			std::cout << "\tnorm " << norm << std::endl;
			std::cout << "\tbase " << base << " (" << ((norm[0]*dir[1] > norm[1]*dir[0]) ? '+' : '-') << ")" << std::endl;*/
			
			++it;
		}
		float t = std::min<float>(wheelTime.Age().Value(), 0.5f) * EnvConfig::singleton().timeScale;
		wheelTime.Set();
		
		poff *= t / wheels.size();  // average: two parallel wheels go forward, they all move equal distance
		aoff *= t; // summation: if two wheels go opposite direction, rotate twice as much as individually
		
		//std::cout << "t " << t << " poff " << poff << " aoff " << aoff << " wheels " << wheels.size() << std::endl;
		
		fmat::Column<3> pCur;
		fmat::Quaternion qCur;
		if(body!=NULL) {
			// jointNode will be set from body in updateGraphics, so only update body position
			body->getCurrent(pCur,qCur);
		} else {
			// no body, so update jointNode directly
			pCur.importFrom(jointNode.getPosition());
			qCur.importFrom(jointNode.getOrientation());
		}
		
		if(poff.sumSq()!=0) {
			if(aoff!=0) {
				fmat::Quaternion qoff_2=fmat::Quaternion::aboutZ(aoff/2);
				qCur = qoff_2 * qCur;
				pCur += qCur * poff;
				qCur = qoff_2 * qCur;
			} else {
				pCur += qCur * poff;
			}
		} else if(aoff!=0) {
			qCur *= fmat::Quaternion::aboutZ(aoff);
		}
		
		if(fixedContactPoints.size()==0) {
			pNext = pCur;
			qNext = qCur;
		} else {
			// average offsets in case we have both wheels and legs (strange but could happen...)
			pNext = (pNext+pCur) / 2;
			fmat::Column<4> tmp = qNext.exportTo<fmat::Column<4> >() + qCur.exportTo<fmat::Column<4> >();
			qNext.importFrom(tmp/tmp.norm());
		}
	}
	
	if(body!=NULL) {
		// jointNode will be set from body in updateGraphics, so only update body position
		// no mass (or we wouldn't be in updateContacts()), so use teleport to move
		//std::cout << "teleport to " << pNext << " @ " << qNext << std::endl;
		body->teleport(pNext,qNext);
	} else {
		// no body, so update jointNode directly
		jointNode.setPosition(pNext.exportTo<Ogre::Vector3>());
		jointNode.setOrientation(qNext.exportTo<Ogre::Quaternion>());
	}
}

void Client::findBody(const KinematicJoint& src, PhysicsBody*& body, const KinematicJoint*& link) {
	link=&src;
	body = Physics::singleton().getBody(*link);
	while(body==NULL) {
		link=link->getParent();
		if(link==NULL)
			return;
		body = Physics::singleton().getBody(*link);
	}
}

void Client::calcFixedPoints(std::vector<fmat::Column<3> >& points) {
	points.resize(fixedPoints.size());
	fmat::Column<3> fp;
	for(unsigned int i=0; i<fixedPoints.size(); ++i) {
		fp.importFrom(fixedPoints[i].point);
		KinematicJoint* fpkj = childMap[fixedPoints[i].frame];
		if(fpkj==NULL) {
			std::cerr << "ERROR: unknown reference frame for contact point '" << fixedPoints[i].frame.get() << "'.  Check synchronization between RobotInfo.h namespace and .kin file." << std::endl;
			continue;
		}
		points[i] = fpkj->getFullT() * fp;
	}
}
void Client::filterPoints(std::vector<fmat::Column<3> >& points) {
	const fmat::Column<3>::storage_t TOL = std::numeric_limits<fmat::Column<3>::storage_t>::epsilon()*10;
	switch (points.size()) {
		case 0:
		case 1:
			return; // can't go wrong
			
		case 2: // check that points aren't coincident
			if((points[0]-points[1]).sumSq() < TOL)
				points.resize(1);
			break;
			
		case 3: // check that points aren't colinear: length of cross product of two edges should be non-zero
			if( fmat::crossProduct(points[1]-points[0], points[2]-points[0]).sumSq() < TOL ) {
				// all three are colinear, pick furthest two
				unsigned int drop=2;
				fmat::Column<3>::storage_t d, bestD = (points[1]-points[0]).sumSq();
				d = (points[2]-points[0]).sumSq();
				if( d > bestD ) {
					bestD = d;
					drop = 1;
				}
				d = (points[2]-points[1]).sumSq();
				if( d > bestD ) {
					bestD = d;
					drop = 0;
				}
				if(bestD < TOL) {
					// really only one point
					points.resize(1);
				} else {
					points[drop]=points[2];
					points.resize(2);
				}
			}
			break;
			
		default: 
			// hack no longer needed, but left here for reference:
			/*{
			// this is a hack due to only handling 3 points in fitTransform... check that first three are valid, otherwise keep substituting other points
			// should check rank/span if doing this properly
			std::vector<fmat::Column<3> > testPoints(points.begin(),points.begin()+3);
			filterPoints(testPoints);
			for(unsigned int i=3; testPoints.size()<3 && i<points.size(); ++i) {
				testPoints.push_back(points[i]);
			}
			points = testPoints;
		}*/ break;
	}
}

/*std::ostream& operator<<(std::ostream& os, const Ogre::Matrix3& m) {
	for(size_t r=0; r<3; ++r) {
		for(size_t c=0; c<3; ++c)
			os << m[r][c] << ' ';
		os << '\n';
	}
	return os;
}*/

void Client::fitTransform(const std::vector<fmat::Column<3> >& orig, const std::vector<fmat::Column<3> >& cur, Ogre::Vector3& posOff, Ogre::Quaternion& oriOff) {
	switch(cur.size()) {
		case 0:
			return;
			
		case 1:
			(orig[0] - cur[0]).exportTo(posOff);
			break;
			
		case 2: {
			using namespace fmat;
			Column<3> fixedCenter = (orig[0] + orig[1])/2;
			Column<3> contactCenter = (cur[0] + cur[1])/2;
			
			Column<3> off1 = orig[0] - fixedCenter;
			Column<3> off2 = cur[0] - contactCenter;
			
			fmatReal m = off1.sumSq() * off2.sumSq();
			if(m < std::numeric_limits<fmatReal>::epsilon()) {
				oriOff = Ogre::Quaternion::IDENTITY;
				(fixedCenter - contactCenter).exportTo(posOff);
			} else {
				Column<3> axis = fmat::crossProduct(off2,off1);
				
				// straightforward implementation...
				/*fmatReal cangle = fmat::dotProduct(off1,off2) / std::sqrt(m);
				if(cangle>1) cangle=1; else if(cangle<-1) cangle=-1; // clip to [-1,1] in case of numerical slop
				Quaternion q = Quaternion::fromAxisAngle(axis,std::acos(cangle)); 
				*/
				
				// but since the dot product is already cos scaled, and the cross product is sin scaled, can avoid calling further trig operations...
				fmatReal ws = std::sqrt(m) + dotProduct(off1,off2); // = |a||b| + |a||b|cos(θ) = |a||b| (1 + cos(θ)) = |a||b| 2cos²(θ/2)
				Quaternion q(ws, axis[0], axis[1], axis[2]); // |axis| = |a||b|sin(θ) = |a||b| 2sin(θ/2)cos(θ/2)
				// normalization divisor will be √( (2|a||b|)² · (cos⁴(θ/2) + sin²(θ/2)cos²(θ/2)) ) = √( (2|a||b|)² · cos²(θ/2) ) = 2|a||b|cos(θ/2)
				q.normalize(); // thus w = cos(θ/2), and |x,y,z| = sin(θ/2)... tada the desired quaternion :)
				
				q.exportTo(oriOff);
				(fixedCenter - q*contactCenter).exportTo(posOff); // f = Tc, f = Ac+a, f - Ac = a
			}
			
		} break;
		
		case 3: {
			Ogre::Vector3 fixedCenter, fixedX, fixedY, fixedZ;
			(orig[0] + orig[1] + orig[2]).exportTo(fixedCenter) /= 3;
			orig[0].exportTo(fixedX) -= fixedCenter;
			orig[1].exportTo(fixedY) -= fixedCenter;
			fixedX.normalise();
			fixedY.normalise();
			fixedZ = fixedX.crossProduct(fixedY);
			fixedZ.normalise(); // X and Y are not orthogonal, thus this would not already be unit length (even though X and Y are normalised)
			
			Ogre::Vector3 contactCenter, contactX, contactY, contactZ;
			(cur[0] + cur[1] + cur[2]).exportTo(contactCenter) /= 3;
			cur[0].exportTo(contactX) -= contactCenter;
			cur[1].exportTo(contactY) -= contactCenter;
			contactX.normalise();
			contactY.normalise();
			contactZ = contactX.crossProduct(contactY);
			contactZ.normalise(); // X and Y are not orthogonal, thus this would not already be unit length (even though X and Y are normalised)
			
			// The 'X' and 'Y' axis we have so far are not orthogonal, and although Quaternion constructor does
			//   some fitting of its own to handle this possibility, it's more stable to pick an orthogonal basis in a consistent manner:
			// Set X axis to average of X and Y, then Y is determined by cross product of previous Z and the new X
			(fixedX = fixedX+fixedY).normalise();
			fixedY = fixedZ.crossProduct(fixedX);
			(contactX = contactX+contactY).normalise();
			contactY = contactZ.crossProduct(contactX);
			
			Ogre::Quaternion fixedQ(fixedX,fixedY,fixedZ), contactQ(contactX,contactY,contactZ);
			oriOff = fixedQ * contactQ.UnitInverse(); // F = AC, FC' = A
			
			posOff = fixedCenter - (oriOff * contactCenter); // f = Tc, f = Ac+a, f - Ac = a
		} break;
			
		default: {
			
#define USE_NEWMAT_SVD
			
#ifdef USE_OGRE_SVD
			// This version based on Ogre3D math should be faster than the NEWMAT version below,
			// but unfortunately the Matrix3::SingularValueDecomposition implementation is unstable,
			// returns bad decompositions when the source matrix is low rank, or nearly so, which happens a lot.
			
			// find centroid for fixed and contacts, then shift centroid to origin
			Ogre::Vector3 fixedCenter;
			std::accumulate(orig.begin(), orig.end(), fmat::Column<3>()).exportTo(fixedCenter);
			fixedCenter/=orig.size();
			std::vector<Ogre::Vector3> fixedc(orig.size());
			for(unsigned int i=0; i<orig.size(); ++i)
				orig[i].exportTo(fixedc[i]) -= fixedCenter;
			
			Ogre::Vector3 contactCenter;
			std::accumulate(cur.begin(), cur.end(), fmat::Column<3>()).exportTo(contactCenter);
			contactCenter/=cur.size();
			std::vector<Ogre::Vector3> contactc(cur.size());
			for(unsigned int i=0; i<cur.size(); ++i)
				cur[i].exportTo(contactc[i]) -= contactCenter;
			
			std::cout << "FIXED:\n";
			for(unsigned int r=0; r<3; ++r) {
				for(unsigned int i=0; i<fixedc.size(); ++i)
					std::cout << fixedc[i][r] << ' ';
				std::cout << '\n';
			}
			std::cout << "CONTACT:\n";
			for(unsigned int r=0; r<3; ++r) {
				for(unsigned int i=0; i<contactc.size(); ++i)
					std::cout << contactc[i][r] << ' ';
				std::cout << '\n';
			}
			
			// now compute ABᵀ
			Ogre::Vector3 ABtc0(0.f), ABtc1(0.f), ABtc2(0.f);
			for(unsigned int i=0; i<fixedc.size(); ++i) {
				ABtc0 += fixedc[i] * contactc[i][0];
				ABtc1 += fixedc[i] * contactc[i][1];
				ABtc2 += fixedc[i] * contactc[i][2];
			}
			
			Ogre::Matrix3 oABt;
			oABt.SetColumn(0,ABtc0);
			oABt.SetColumn(1,ABtc1);
			oABt.SetColumn(2,ABtc2);
			std::cout << "ABt:\n" << oABt << std::endl;
			
			Ogre::Matrix3 oU, oV;
			Ogre::Vector3 oS;
			oABt.SingularValueDecomposition(oU, oS, oV);
			
			std::cout << "Ogre DIAGONAL: " << oS[0] << ' ' << oS[1] << ' ' << oS[2] << '\n'
			 << "Ogre U:\n" << fmat::Matrix<3,3>().importFromArray(oU) << '\n'
			 << "Ogre V:\n" << fmat::Matrix<3,3>().importFromArray(oV) << std::endl;
			
			Ogre::Matrix3 oR = oU*oV;
			
			std::cout << "OGRE DET " << oR.Determinant() << '\n';
			if(oR.Determinant()<0) {
				//std::cout << "OGRE REFLECTION?" << std::endl;
				oU.SetColumn(2,-oU.GetColumn(2));
				oR = oU*oV;
			}
			
			std::cout << "Ogre R:\n" << fmat::Matrix<3,3>().importFromArray(oR) << std::endl;
						
			oriOff.FromRotationMatrix(oR);
			oriOff.normalise();
			std::cout << "Ogre q: " << oriOff << std::endl;
			posOff = fixedCenter - (oR * contactCenter); // f = Tc, f = Ac+a, f - Ac = a
			
#elif defined(USE_NEWMAT_SVD)

			// find centroid for fixed and contacts, then shift centroid to origin
			fmat::Column<3> fixedCenter = std::accumulate(orig.begin(), orig.end(), fmat::Column<3>());
			fixedCenter/=orig.size();
			std::vector<fmat::Column<3> > fixedc;
			fixedc.reserve(orig.size());
			for(std::vector<fmat::Column<3> >::const_iterator it=orig.begin(); it!=orig.end(); ++it)
				fixedc.push_back(*it - fixedCenter);
			
			fmat::Column<3> contactCenter = std::accumulate(cur.begin(), cur.end(), fmat::Column<3>());
			contactCenter/=cur.size();
			std::vector<fmat::Column<3> > contactc;
			contactc.reserve(cur.size());
			for(std::vector<fmat::Column<3> >::const_iterator it=cur.begin(); it!=cur.end(); ++it)
				contactc.push_back(*it - contactCenter);
			
			// now compute ABᵀ
			fmat::Matrix<3,3> ABt;
			for(unsigned int i=0; i<fixedc.size(); ++i) {
				ABt.column(0) += fixedc[i] * contactc[i][0];
				ABt.column(1) += fixedc[i] * contactc[i][1];
				ABt.column(2) += fixedc[i] * contactc[i][2];
			}
			//std::cout << "ABt:\n" << ABt << std::endl;
			
			// copy to NEWMAT matrix for SVD processing
			NEWMAT::Matrix nmABt(3,3);
			ABt.exportToNewmat(nmABt);
			
			NEWMAT::DiagonalMatrix nmQ;
			NEWMAT::Matrix nmU,nmV;
			NEWMAT::SVD(nmABt,nmQ,nmU,nmV);
			nmV=nmV.t();
			//std::cout << "NM DIAGONAL: " << nmQ(1) << ' ' << nmQ(2) << ' ' << nmQ(3) << '\n'
			// << "NM U:\n" << fmat::Matrix<3,3>().importFromNewmat(nmU) << '\n'
			// << "NM V:\n" << fmat::Matrix<3,3>().importFromNewmat(nmV) << std::endl;
			
			fmat::Matrix<3,3> U, V;
			U.importFromNewmat(nmU);
			V.importFromNewmat(nmV);
			
			fmat::Matrix<3,3> R = U * V;

			// check det(R) to flip in case of reflection
			//std::cout << "NM DET " << fmat::det(R) << '\n';
			if( fmat::det(R) < 0 ) {
				//std::cout << "NM REFLECTION?" << std::endl;
				U.column(2) = -U.column(2);
				R = U * V;
			}
			
			//std::cout << R << std::endl;
			fmat::Quaternion q = fmat::Quaternion::fromMatrix(R);
			//std::cout << q << " angle " << fmat::quatAngle(q) << '\n';

			q.exportTo(oriOff);
			(fixedCenter - (R * contactCenter)).exportTo(posOff); // f = Tc, f = Ac+a, f - Ac = a
#endif
		} break;
	}
}

void Client::locationUpdated() {
	PhysicsBody * body = Physics::singleton().getBody(model);
	if(body!=NULL) {
		// update body, graphics will be updated from that
		body->teleport(fmat::pack(location[0],location[1],location[2]));
	}
	// set graphics directly (this would be updated from physics body, but do it now in case of contacts updated with non-physics walk)
	Ogre::SceneNode& rootNode = kjInfos[&model]->getJointNode();
	Ogre::Vector3 pos = rootNode._getDerivedPosition();
	Ogre::Vector3 off = Ogre::Vector3(location[0],location[1],location[2])-pos;
	rootNode.translate(off, Ogre::Node::TS_WORLD);
}

void Client::orientationUpdated() {
	PhysicsBody * body = Physics::singleton().getBody(model);
	if(body!=NULL) {
		// update body, graphics will be updated from that
		body->teleport(fmat::Quaternion::fromAxis(orientation));
	}
	// set graphics directly (this would be updated from physics body, but do it now in case of contacts updated with non-physics walk)
	Ogre::SceneNode& rootNode = kjInfos[&model]->getJointNode();
	Ogre::Quaternion cur = rootNode._getDerivedOrientation();
	fmat::Quaternion q = fmat::Quaternion::fromAxis(orientation);
	Ogre::Quaternion off = cur.UnitInverse() * q.exportTo<Ogre::Quaternion>();
	rootNode.rotate(off, Ogre::Node::TS_LOCAL);
}

void Client::plistValueTouched(const plist::PrimitiveBase& pl) {
	if(&pl==&location[0] || &pl==&location[1] || &pl==&location[2]) {
		locationUpdated();
	} else if(&pl==&orientation[0] || &pl==&orientation[1] || &pl==&orientation[2]) {
		orientationUpdated();
	}
}

void Client::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&robotID) {
		std::cerr << "WARNING: Client's robot ID value was changed! (from " << robotID.getPreviousValue() << " to " << robotID << ")" << std::endl;
	} else if(&pl==&location[0] || &pl==&location[1] || &pl==&location[2]) {
		locationUpdated();
	} else if(&pl==&orientation[0] || &pl==&orientation[1] || &pl==&orientation[2]) {
		orientationUpdated();
	} else if(&pl==&physicsWheels) {
		if(!physicsWheels) {
			// Disabling physics, disable wheel rotation otherwise we get "double" motion from both the wheels and Mirage's own updates
			// A previous approach was to disable wheel friction but allow them to rotate, however this allows the robot to 'slide' from any external forces
			for(std::set<KinematicJoint*>::const_iterator it=wheels.begin(); it!=wheels.end(); ++it) {
				// future positions will be ignored via loadXML until physicsWheels is restored
				wheelSpeeds[*it] = positions[(*it)->outputOffset.get()];
				if((*it)->outputOffset!=plist::OutputSelector::UNUSED) {
					positions[(*it)->outputOffset.get()] = 0;
				}
			}
			wheelTime.Set();
		} else {
			// Re-enabling physics, restore wheel rotation
			for(std::set<KinematicJoint*>::const_iterator it=wheels.begin(); it!=wheels.end(); ++it) {
				if((*it)->outputOffset!=plist::OutputSelector::UNUSED) {
					std::map<const KinematicJoint*,float>::const_iterator w = wheelSpeeds.find(*it);
					ASSERTIF(w != wheelSpeeds.end(),"Re-enabling physicsWheels, could not find speed value for wheel " << (*it)->outputOffset) {
						positions[(*it)->outputOffset.get()] = w->second;
					}
				}
			}
		}
	} else {
		std::cerr << "Unknown plist value changed!" << std::endl;
	}
}



void Client::kinematicJointBranchAdded(KinematicJoint& parent, KinematicJoint& branch) {
	//std::cout << &branch << "'"<<branch.outputOffset.get()<<"' added to '"<<parent.outputOffset.get()<<"'" << std::endl;
	Ogre::SceneNode& parentNode = (&branch==&parent) ? *ogreSceneMgr->getRootSceneNode() : kjInfos[&parent]->getJointNode();
	kjInfos[&branch] = new JointGraphicsInfo(*this,branch,parentNode);
	branch.addBranchListener(this);
	if(branch.outputOffset!=plist::OutputSelector::UNUSED)
		childMap[branch.outputOffset]=&branch;
	const std::string& name = branch.outputOffset.get();
	bool hasFeedback = (sensorValues.outputs.findEntry(name)!=sensorValues.outputs.end());
	for(plist::ArrayOf<SensorInfo>::const_iterator sit = branch.sensorInfo.begin(); sit!=branch.sensorInfo.end(); ++sit) {
		bool valid=true;
		if(SensorRangeFinder* rf = dynamic_cast<SensorRangeFinder*>(*sit))
			kjInfos[&branch]->createRangeFinderNodes(*rf);
		else if(SensorFeedback* fb = dynamic_cast<SensorFeedback*>(*sit)) {
			fb->sensorName = name;
			if(hasFeedback) {
				std::cerr << "WARNING: found explicit SensorFeedback for joint with pre-existing (automatically created?) feedback instance" << std::endl;
				valid=false;
			}
			hasFeedback=true;
		}
		if(valid) {
			sensors.insert(std::make_pair(kjInfos[&branch],*sit));
			(*sit)->declareValues(sensorValues);
			std::cout << "Created " << (*sit)->sensorType << " on " << branch.outputOffset.get() << std::endl;
		}
	}
	if(!hasFeedback && branch.outputOffset!=plist::OutputSelector::UNUSED && branch.isMobile()) {
		SensorFeedback* fb = new SensorFeedback;
		branch.sensorInfo.addEntry(fb);
		fb->sensorName = name;
		sensors.insert(std::make_pair(kjInfos[&branch],fb));
		fb->declareValues(sensorValues);
	}
	if(branch.controllerInfo.velocity) {
		wheels.insert(&branch); // todo should add a listener in case this is modified?
		if(!physicsWheels) {
			if(branch.outputOffset!=plist::OutputSelector::UNUSED) {
				wheelSpeeds[&branch] = positions[name];
				positions[name] = 0;
			} else {
				wheelSpeeds[&branch] = 0;
			}
		}
	}
	for(KinematicJoint::branch_iterator it=branch.getBranches().begin(); it!=branch.getBranches().end(); ++it)
		kinematicJointBranchAdded(branch, **it);
}

void Client::kinematicJointBranchRemoved(KinematicJoint& parent, KinematicJoint& branch) {
	wheels.erase(&branch);
	branch.clearBranches(); // will trigger recursive calls, so the rest of this function will execute on children first
	//std::cout << "Destroying " << branch.outputOffset.get() << std::endl;
	if(branch.outputOffset!=plist::OutputSelector::UNUSED) {
		const std::string& name = branch.outputOffset.get();
		positions.removeEntry(name);
		childMap.erase(branch.outputOffset);
	}
	if(sensors.find(kjInfos[&branch])!=sensors.end()) {
		std::pair<sensors_t::iterator,sensors_t::iterator> sens = sensors.equal_range(kjInfos[&branch]);
		for(sensors_t::iterator it=sens.first; it!=sens.second; ++it) {
			//std::cout << "Removed " << it->second->sensorType << " sensor on " << branch.outputOffset.get() << std::endl;
			it->second->reclaimValues(sensorValues);
		}
		sensors.erase(sens.first,sens.second);
	}
	delete kjInfos[&branch];
	kjInfos.erase(&branch);
}

void Client::kinematicJointReconfigured(KinematicJoint& joint) {
	/*
	Ogre::SceneNode* node = nodes[&joint];
	if(&joint==&model)
		node->setPosition(location[0],location[1],location[2]);
	else
		node->setPosition(0,0,0);
	node->resetOrientation();
	node->roll(Ogre::Radian(joint.theta));
	node->translate(joint.r,0,joint.d,Ogre::Node::TS_LOCAL);
	node->pitch(Ogre::Radian(joint.alpha));
	switch(joint.jointType) {
		case KinematicJoint::REVOLUTE: {
			node->roll(Ogre::Radian(joint.qOffset));
		} break;
		case KinematicJoint::PRISMATIC: {
			node->translate(0,0,joint.qOffset,Ogre::Node::TS_LOCAL);
		} break;
	}
	 */
}


void Client::plistCollectionEntryAdded(Collection& col, ObjectBase& entry) {
	if(&col==&cameras) {
		dynamic_cast<CameraInfo&>(entry).setParent(this);
	} else {
		std::cerr << "unknown collection entry added" << std::endl;
	}
}
void Client::plistCollectionEntryRemoved(Collection& col, ObjectBase& entry) {
	if(&col==&cameras) {
		dynamic_cast<CameraInfo&>(entry).setParent(NULL);
	} else {
		std::cerr << "unknown collection entry removed" << std::endl;
	}
}
void Client::plistCollectionEntriesChanged(Collection& col) {
	if(&col==&cameras) {
		for(plist::ArrayOf<CameraInfo>::const_iterator it=cameras.begin(); it!=cameras.end(); ++it)
			(*it)->setParent(this);
	} else {
		std::cerr << "unknown collection changed" << std::endl;
	}
}

void Client::updateGraphics(KinematicJoint& kj) {
	std::map<KinematicJoint*, JointGraphicsInfo*>::const_iterator tmp = kjInfos.find(&kj);
#ifdef DEBUG
	if(tmp==kjInfos.end()) {
		std::cerr << "No kjInfo entry for " << kj.getPath() << std::endl;
		return;
	}
	if(tmp->second==NULL) {
		std::cerr << "kjInfo[" << kj.getPath() << "] is NULL" << std::endl;
		return;
	}
#endif
	JointGraphicsInfo& info = *tmp->second;
	if(const PhysicsBody * body = Physics::singleton().getBody(kj)) {
		fmat::Column<3> tpos;
		fmat::Quaternion tquat;
		body->getCurrent(tpos,tquat);
		
		Ogre::Vector3 pos;
		tpos.exportTo(pos);
		Ogre::Quaternion ori(1,0,0,0);
		tquat.exportTo(ori);
		
		Ogre::Quaternion parentOri = info.getJointNode().getParent()->_getDerivedOrientation();
		ori = parentOri.UnitInverse() * ori;
		info.getJointNode().setOrientation(ori);
		info.getJointNode()._update(false, false);
		
		// Ogre::Vector3 parentPos = info.getJointNode().getParent()->_getDerivedPosition();
		pos = info.getJointNode().getParent()->_getFullTransform().inverseAffine() * pos;
		info.getJointNode().setPosition(pos);
		
		//std::cout << "pos in parent " << pos << ", parent @ " << parentPos << " / " << parentOri << std::endl;
		//std::cout << "now ori in parent " << ori << std::endl;
		
		info.getJointNode()._update(true, false);
		
	} else if(kj.outputOffset!=plist::OutputSelector::UNUSED && kj.isMobile()) {
		if(fixedPoints.size()>0) {
			// already did updateKJPositions, can't tell what's dirty
			info.updateJointNode();
		} else {
			float pos = positions[kj.outputOffset.get()];
			if(kj.getQ()!=pos) {
				kj.setQ(pos);
				info.updateJointNode();
			}
		}
	}
	
	for(KinematicJoint::branch_iterator it=kj.getBranches().begin(); it!=kj.getBranches().end(); ++it)
		updateGraphics(**it);
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
