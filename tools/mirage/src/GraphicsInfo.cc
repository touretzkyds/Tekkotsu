#include "GraphicsInfo.h"
#include "Client.h"
#include "EnvConfig.h"
#include "MirageFrameListener.h"

#include "Shared/debuget.h"

#ifdef __APPLE__
#  include <Carbon/Carbon.h>
#  include <Ogre/Ogre.h>
#else
#  include <OGRE/Ogre.h>
#endif

using namespace std;

GraphicsInfo* GraphicsInfo::userSelection=NULL;

void GraphicsInfo::cleanup() {
	frameListener->graphicsInfoDestructed(*this);
	Ogre::SceneNode** nodes[] = { &inertiaNode, &centerOfMassNode, &bbNode, &rfBeamNode, &rfHitNode };
	for(size_t i=0; i<sizeof(nodes)/sizeof(Ogre::SceneNode**); ++i) {
		Ogre::SceneNode*& node = *nodes[i];
		if(node==NULL)
			continue;
		node->removeAndDestroyAllChildren();
		ogreSceneMgr->destroySceneNode(node);
		node=NULL;
	}
	if(modelEntity!=NULL) {
		delete scaleListener; scaleListener=NULL;
		delete rotationListener; rotationListener=NULL;
		delete offsetListener; offsetListener=NULL;
		Ogre::SceneNode * modelNode = modelEntity->getParentSceneNode();
		modelNode->removeAndDestroyAllChildren();
		ogreSceneMgr->destroySceneNode(modelNode);
		modelEntity=NULL;
	}
	if(collisionModelEntity!=NULL) {
		delete collisionScaleListener; collisionScaleListener=NULL;
		delete collisionRotationListener; collisionRotationListener=NULL;
		delete collisionOffsetListener; collisionOffsetListener=NULL;
		Ogre::SceneNode * modelNode = collisionModelEntity->getParentSceneNode();
		modelNode->removeAndDestroyAllChildren();
		ogreSceneMgr->destroySceneNode(modelNode);
		collisionModelEntity=NULL;
	}
}

void GraphicsInfo::select() {
	unselect();
	if(bbNode!=NULL)
		bbNode->getAttachedObject(0)->addVisibilityFlags(GRAPHICS_MODEL_MASK | COLLISION_MODEL_MASK);
	userSelection=this;
}
void GraphicsInfo::unselect() {
	if(userSelection!=NULL && userSelection->bbNode!=NULL)
		userSelection->bbNode->getAttachedObject(0)->removeVisibilityFlags(GRAPHICS_MODEL_MASK | COLLISION_MODEL_MASK);
	userSelection=NULL;
}

void GraphicsInfo::createRangeFinderNodes(const SensorRangeFinder& rf) {
	Ogre::Entity * ent = createEntity("rfBeam", "RangeFinderBeam");
	ent->setMaterialName("Pink");
	ent->setVisibilityFlags(COLLISION_MODEL_MASK);
	ent->setCastShadows(false);
	rfBeamNode = jointNode->createChildSceneNode();
	rfBeamNode->attachObject(ent);
	rfBeamNode->setPosition(0, 0, rf.minRange);
	updateRangeFinderNodes(false,rf);
}

void GraphicsInfo::updateRangeFinderNodes(bool hit, const SensorRangeFinder& rf) {
	//std::cout << (hit?"hit ":"miss ") << rf.value << std::endl;
	rfBeamNode->setScale(1, 1, rf.value-rf.minRange);
	if(hit) {
		if(rfHitNode==NULL) {
			Ogre::Entity * ent = createEntity("rfHit", "RangeFinderHit");
			ent->setMaterialName("Green");
			ent->setVisibilityFlags(COLLISION_MODEL_MASK);
			ent->setCastShadows(false);
			rfHitNode = jointNode->createChildSceneNode();
			rfHitNode->attachObject(ent);
		}
		rfHitNode->setPosition(0, 0, rf.value);
		// being cutsey, animate hit target
		rfHitNode->rotate(Ogre::Quaternion(Ogre::Degree(2),Ogre::Vector3(0,0,1)), Ogre::Node::TS_LOCAL);
		TimeET t;
		float st = (float)std::sin(t.Value()*2);
		float ct = (float)std::cos(t.Value()*2);
		float xs = (st*st*st+2)/2;
		float ys = (ct*ct*ct+2)/2;
		rfHitNode->setScale(xs,ys,1);
	} else {
		if(rfHitNode!=NULL) {
			ogreSceneMgr->destroySceneNode(rfHitNode);
			rfHitNode=NULL;
		}
	}
}

const KinematicJoint& GraphicsInfo::parentKJ() const {
	return dynamic_cast<const KinematicJoint&>(parent.link);
}

Ogre::Entity* GraphicsInfo::createEntity(const std::string& identifier, const std::string& mesh) {
	const std::string parentName = parentKJ().outputOffset.get();
	Ogre::Entity *ent = client.createEntity(identifier.size()==0?parentName:parentName+"-"+identifier,mesh);
	ent->setUserAny(Ogre::Any(this));
	return ent;
}

void GraphicsInfo::updateBB() {
	BoundingBox3D bb;
	bb = link.getAABB();
	if(!bb.empty()) {
		if(bbNode==NULL) {
			Ogre::Entity *ent = createEntity("bb", "Cube");
			if(ent==NULL)
				throw std::bad_alloc();
			ent->setMaterialName("BoundingBox");
			ent->setVisibilityFlags(BOUNDING_BOX_MASK);
			ent->setCastShadows(false);
			bbNode = jointNode->createChildSceneNode();
			if(bbNode==NULL)
				throw std::bad_alloc();
			bbNode->attachObject(ent);
		}
		fmat::Column<3> dims = bb.getDimensions();
		bbNode->setScale(dims[0],dims[1],dims[2]);
		bbNode->setPosition(bb.min[0]+dims[0]/2,bb.min[1]+dims[1]/2,bb.min[2]+dims[2]/2);
	} else if(bbNode!=NULL) {
		ogreSceneMgr->destroySceneNode(bbNode);
		bbNode=NULL;
	}
	if(&parent!=this)
		parent.updateBB();
}

void GraphicsInfo::updateModel() {
	Ogre::SceneNode * modelNode = NULL;
	if(modelEntity!=NULL) {
		modelNode = modelEntity->getParentSceneNode();
#if OGRE_VERSION_MAJOR==1 && OGRE_VERSION_MINOR<7
		modelEntity->detatchFromParent();
#else
		modelEntity->detachFromParent();
#endif
		ogreSceneMgr->destroyEntity(modelEntity);
		modelEntity=NULL;
	}
	if(link.model.size()>0 && link.model!="CollisionModel") {
		try {
			modelEntity = createEntity("", link.model);
			if(modelEntity==NULL)
				throw std::bad_alloc();
		} catch(const std::exception& ex) {
			std::cerr << "An exception occurred loading mesh '" << link.model << "' for " << parentKJ().getPath() << ": " << ex.what() << std::endl;
			return;
		} catch(...) {
			std::cerr << "An unknown exception occurred loading mesh '" << link.model << "' for " << parentKJ().getPath() << std::endl;
			return;
		}
		if(modelNode!=NULL) {
			modelNode->attachObject(modelEntity);
		} else {
			modelNode = jointNode->createChildSceneNode();
			modelNode->attachObject(modelEntity);
			scaleListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.modelScale,*this,&GraphicsInfo::updateModelScale);
			rotationListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.modelRotation,*this,&GraphicsInfo::updateModelRotation);
			offsetListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.modelOffset,*this,&GraphicsInfo::updateModelOffset);
		}
		if(link.material.size()>0)
			modelEntity->setMaterialName(link.material);
	}
	if(modelEntity==NULL && modelNode!=NULL) {
		modelNode->removeAndDestroyAllChildren();
		ogreSceneMgr->destroySceneNode(modelNode);
		delete scaleListener; scaleListener=NULL;
		delete rotationListener; rotationListener = NULL;
		delete offsetListener; offsetListener = NULL;
	}
	updateVisibility();
}

void GraphicsInfo::updateCollisionModel() {
	Ogre::SceneNode * modelNode = NULL;
	if(collisionModelEntity!=NULL) {
		modelNode = collisionModelEntity->getParentSceneNode();
#if OGRE_VERSION_MAJOR==1 && OGRE_VERSION_MINOR<7
		collisionModelEntity->detatchFromParent();
#else
		collisionModelEntity->detachFromParent();
#endif
		ogreSceneMgr->destroyEntity(collisionModelEntity);
		collisionModelEntity=NULL;
	}
	if(link.collisionModel.size()>0) {
		try {
			collisionModelEntity = createEntity("-hit", link.collisionModel);
			if(collisionModelEntity==NULL)
				throw std::bad_alloc();
		} catch(const std::exception& ex) {
			std::cerr << "An exception occurred loading mesh '" << link.collisionModel << "' for " << parentKJ().getPath() << ": " << ex.what() << std::endl;
			return;
		} catch(...) {
			std::cerr << "An unknown exception occurred loading mesh '" << link.collisionModel << "' for " << parentKJ().getPath() << std::endl;
			return;
		}
		if(modelNode!=NULL) {
			modelNode->attachObject(collisionModelEntity);
		} else {
			modelNode = jointNode->createChildSceneNode();
			modelNode->attachObject(collisionModelEntity);
			collisionScaleListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.collisionModelScale,*this,&GraphicsInfo::updateCollisionModelScale);
			collisionRotationListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.collisionModelRotation,*this,&GraphicsInfo::updateCollisionModelRotation);
			collisionOffsetListener = new plist::CollectionCallbackMember<GraphicsInfo>(link.collisionModelOffset,*this,&GraphicsInfo::updateCollisionModelOffset);
		}
		if(link.material.size()>0)
			collisionModelEntity->setMaterialName(link.material);
		else
			collisionModelEntity->setMaterialName("CollisionShape");
	}
	if(collisionModelEntity==NULL && modelNode!=NULL) {
		modelNode->removeAndDestroyAllChildren();
		ogreSceneMgr->destroySceneNode(modelNode);
		delete collisionScaleListener; collisionScaleListener=NULL;
		delete collisionRotationListener; collisionRotationListener=NULL;
		delete collisionOffsetListener; collisionOffsetListener=NULL;
	}
	updateVisibility();
}

void GraphicsInfo::updateCenterOfMass() {
	if(link.mass<=0 || EnvConfig::singleton().massDisplayScale==0) {
		if(centerOfMassNode!=NULL) {
			centerOfMassNode->removeAndDestroyAllChildren();
			ogreSceneMgr->destroySceneNode(centerOfMassNode);
			centerOfMassNode=NULL;
		}
	} else {
		if(centerOfMassNode==NULL) {
			centerOfMassNode = jointNode->createChildSceneNode();
			if(centerOfMassNode==NULL)
				throw std::bad_alloc();
		}
		centerOfMassNode->setPosition(link.centerOfMass.construct<Ogre::Vector3>());
		float dispMassSc = std::sqrt(link.mass) * EnvConfig::singleton().massDisplayScale;
		centerOfMassNode->setScale(Ogre::Vector3(dispMassSc));
		
		Ogre::Entity *ent = createEntity("com", "Sphere");
		if(ent==NULL)
			throw std::bad_alloc();
		ent->setMaterialName("Yellow");
		ent->setVisibilityFlags(COLLISION_MODEL_MASK);
		ent->setCastShadows(false);
		centerOfMassNode->attachObject(ent);
	}
	
	PhysicsBody * body = Physics::singleton().getBody(link);
	if(body==NULL || EnvConfig::singleton().inertiaDisplayScale==0) {
		if(inertiaNode!=NULL) {
			inertiaNode->removeAndDestroyAllChildren();
			ogreSceneMgr->destroySceneNode(inertiaNode);
			inertiaNode=NULL;
		}
	} else {
		if(inertiaNode==NULL) {
			inertiaNode = jointNode->createChildSceneNode();
			if(inertiaNode==NULL)
				throw std::bad_alloc();
		}
		inertiaNode->setPosition(link.centerOfMass.construct<Ogre::Vector3>());
		float unitSc = Physics::getSpaceScale() * Physics::getSpaceScale() * Physics::getMassScale();
		fmat::fmatReal (*sqrtReal)(fmat::fmatReal) = std::sqrt; // select correct overloaded sqrt
		fmat::Column<3> inertiaSc = (body->getInertia() / unitSc).map(sqrtReal) * EnvConfig::singleton().inertiaDisplayScale;
		inertiaNode->setScale(inertiaSc.exportTo<Ogre::Vector3>());
		
		Ogre::Entity *ent = createEntity("com", "Cube");
		if(ent==NULL)
			throw std::bad_alloc();
		ent->setMaterialName("Pink");
		ent->setVisibilityFlags(COLLISION_MODEL_MASK);
		ent->setCastShadows(false);
		inertiaNode->attachObject(ent);
	}
}

void GraphicsInfo::updateMaterial() {
	if(link.material.size()>0) {
		if(modelEntity!=NULL)
			modelEntity->setMaterialName(link.material);
		if(collisionModelEntity!=NULL)
			collisionModelEntity->setMaterialName(link.material);
	} else {
		if(modelEntity!=NULL)
			updateModel();
		if(collisionModelEntity!=NULL)
			collisionModelEntity->setMaterialName("CollisionShape");
	}
}

void GraphicsInfo::updateModelRotation() {
	Ogre::SceneNode * modelNode = modelEntity->getParentSceneNode();
	modelNode->resetOrientation();
	Ogre::Vector3 qaxis(link.modelRotation[0],link.modelRotation[1],link.modelRotation[2]);
	float sx_2 = qaxis.length();
	if(sx_2>std::numeric_limits<float>::epsilon()) {
		float x = (sx_2>1) ? (float)M_PI : (2 * std::asin(sx_2));
		modelNode->rotate(qaxis/sx_2,Ogre::Radian(x));
	}
}

void GraphicsInfo::updateModelScale() {
	Ogre::SceneNode * modelNode = modelEntity->getParentSceneNode();
	modelNode->setScale(link.modelScale[0], link.modelScale[1], link.modelScale[2]);
}

void GraphicsInfo::updateModelOffset() {
	Ogre::SceneNode * modelNode = modelEntity->getParentSceneNode();
	modelNode->setPosition(link.modelOffset[0], link.modelOffset[1], link.modelOffset[2]);
}

void GraphicsInfo::updateCollisionModelRotation() {
	Ogre::SceneNode * modelNode = collisionModelEntity->getParentSceneNode();
	modelNode->resetOrientation();
	Ogre::Vector3 qaxis(link.collisionModelRotation[0],link.collisionModelRotation[1],link.collisionModelRotation[2]);
	float sx_2 = qaxis.length();
	if(sx_2>std::numeric_limits<float>::epsilon()) {
		float x = (sx_2>1) ? (float)M_PI : (2 * std::asin(sx_2));
		modelNode->rotate(qaxis/sx_2,Ogre::Radian(x));
	}
	updateBB();
}

void GraphicsInfo::updateCollisionModelScale() {
	Ogre::SceneNode * modelNode = collisionModelEntity->getParentSceneNode();
	modelNode->setScale(link.collisionModelScale[0], link.collisionModelScale[1], link.collisionModelScale[2]);
	updateBB();
}

void GraphicsInfo::updateCollisionModelOffset() {
	Ogre::SceneNode * modelNode = collisionModelEntity->getParentSceneNode();
	modelNode->setPosition(link.collisionModelOffset[0], link.collisionModelOffset[1], link.collisionModelOffset[2]);
	updateBB();
}

void GraphicsInfo::updateVisibility() {
	if(link.model=="CollisionModel") {
		ASSERT(modelEntity==NULL,"updateVisibity(): model is CollisionModel, but still have modelEntity?");
		if(collisionModelEntity!=NULL) {
			collisionModelEntity->setVisibilityFlags(link.visible ? (ROBOT_CAMERA_MASK | GRAPHICS_MODEL_MASK | COLLISION_MODEL_MASK) : (GRAPHICS_MODEL_MASK | COLLISION_MODEL_MASK));
			collisionModelEntity->setCastShadows(link.visible);
		}
	} else {
		if(modelEntity!=NULL) {
			modelEntity->setVisibilityFlags(link.visible ? (ROBOT_CAMERA_MASK | GRAPHICS_MODEL_MASK) : (GRAPHICS_MODEL_MASK));
			modelEntity->setCastShadows(link.visible);
		}
		if(collisionModelEntity!=NULL) {
			collisionModelEntity->setVisibilityFlags(COLLISION_MODEL_MASK);
			collisionModelEntity->setCastShadows(false);
		}
	}
}



JointGraphicsInfo::JointGraphicsInfo(const Client& cl, const KinematicJoint& kjSrc, Ogre::SceneNode& parentNode)
	: GraphicsInfo(cl, kjSrc, *this, parentNode.createChildSceneNode()), components()
{
	if(&link==&client.model)
		jointNode->setPosition(client.location[0],client.location[1],client.location[2]);
	else
		jointNode->setPosition(0,0,0);
	const KinematicJoint& kj = parentKJ();	 
	jointNode->resetOrientation();	 
	jointNode->roll(Ogre::Radian(kj.theta));	 
	jointNode->translate(kj.r,0,kj.d,Ogre::Node::TS_LOCAL);	
	updateJointNode();
	if(&link==&client.model)
		jointNode->rotate(fmat::Quaternion::fromAxis(client.orientation).exportTo<Ogre::Quaternion>());
	kj.components.addCollectionListener(this);
	plistCollectionEntriesChanged(const_cast<plist::ArrayOf<LinkComponent>&>(kj.components));
	frameListener->graphicsInfoCreated(*this);
}

JointGraphicsInfo::~JointGraphicsInfo() {
	// we want to clear out bbNode and entities *first*, because we're deleting jointNode itself below...
	cleanup();
	
	parentKJ().components.removeCollectionListener(this);
	clear();
	
	jointNode->removeAndDestroyAllChildren(); // wouldn't need the explicit destructor call if we skipped this, just guarantees no leaks
	ogreSceneMgr->destroySceneNode(jointNode);
}

std::string JointGraphicsInfo::getName() const {
	return client.robotID + "#" + getKJ().outputOffset.get();
}

void JointGraphicsInfo::updateJointNode() {
	jointNode->resetOrientation();
	const KinematicJoint& kj = parentKJ();
	switch(kj.jointType) {
		case KinematicJoint::REVOLUTE: {
			jointNode->roll(Ogre::Radian(kj.theta));
			jointNode->pitch(Ogre::Radian(kj.alpha));
			jointNode->roll(Ogre::Radian(kj.qOffset+client.getPosition(kj.outputOffset.get())));
		} break;
		case KinematicJoint::PRISMATIC: {
			jointNode->setPosition(0,0,0);
			jointNode->roll(Ogre::Radian(kj.theta));
			jointNode->translate(kj.r,0,kj.d,Ogre::Node::TS_LOCAL);
			jointNode->pitch(Ogre::Radian(kj.alpha));
			jointNode->translate(0,0,kj.qOffset+client.getPosition(kj.outputOffset.get()),Ogre::Node::TS_LOCAL);
		} break;
	}
}

void JointGraphicsInfo::plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& primitive) {
	const plist::ArrayBase& comps = parentKJ().components;
	if(&col == &comps) {
		if(LinkComponent * comp = dynamic_cast<LinkComponent*>(&primitive)) {
			components[comp] = new GraphicsInfo(client,*comp, *this, jointNode);
		} else {
			throw XMLLoadSave::bad_format(NULL, "non-KinematicJoint::Component added to component list");
		}
		updateBB();
	} else {
		std::cerr << "KinematicComponentListListener::plistCollectionEntryAdded for unknown collection" << std::endl;
	}
}
void JointGraphicsInfo::plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& primitive) {
	const plist::ArrayBase& comps = parentKJ().components;
	if(&col == &comps) {
		if(LinkComponent * comp = dynamic_cast<LinkComponent*>(&primitive)) {
			delete components[comp];
			components.erase(comp);
		} else {
			throw XMLLoadSave::bad_format(NULL, "non-KinematicJoint::Component added to component list");
		}
		updateBB();
	} else {
		std::cerr << "KinematicComponentListListener::plistCollectionEntryRemoved for unknown collection" << std::endl;
	}
}
void JointGraphicsInfo::plistCollectionEntriesChanged(plist::Collection& col) {
	const plist::ArrayOf<LinkComponent>& comps = parentKJ().components;
	if(&col == &comps) {
		clear();
		for(plist::ArrayOf<LinkComponent>::const_iterator it=comps.begin(); it!=comps.end(); ++it)
			components[*it] = new GraphicsInfo(client,**it, *this, jointNode);
		updateBB();
	} else {
		std::cerr << "KinematicComponentListListener::plistCollectionEntriesChanged for unknown collection" << std::endl;
	}
}



/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
