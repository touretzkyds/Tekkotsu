//-*-c++-*-
#ifndef INCLUDED_GraphicsInfo_h_
#define INCLUDED_GraphicsInfo_h_

#include "Motion/KinematicJoint.h"
#include "Shared/plist.h"
#include <map>

class Client;
class JointGraphicsInfo;
namespace Ogre {
	class SceneNode;
	class Entity;
}

//! interfaces to Ogre 3D to maintain graphics for each LinkComponent
class GraphicsInfo {
public:
	GraphicsInfo(const Client& cl, const LinkComponent& linkSrc, JointGraphicsInfo& parentInfo, Ogre::SceneNode * kjNode)
		: client(cl), link(linkSrc), parent(parentInfo), jointNode(kjNode), bbNode(NULL), centerOfMassNode(NULL), inertiaNode(NULL), rfBeamNode(NULL), rfHitNode(NULL), 
		modelEntity(NULL), collisionModelEntity(NULL),
		rotationListener(NULL), scaleListener(NULL), offsetListener(NULL),
		collisionRotationListener(NULL), collisionScaleListener(NULL), collisionOffsetListener(NULL),
		centerOfMassListener(link.centerOfMass,*this,&GraphicsInfo::updateCenterOfMass),
		massListener(link.mass,*this,&GraphicsInfo::updateCenterOfMass),
		materialListener(link.material,*this,&GraphicsInfo::updateMaterial), 
		visibilityListener(link.visible,*this,&GraphicsInfo::updateVisibility),
		modelListener(link.model,*this,&GraphicsInfo::updateModel),
		collisionModelListener(link.collisionModel,*this,&GraphicsInfo::updateCollisionModel)
	{}
	
	virtual ~GraphicsInfo() { cleanup(); }
	
	enum VisibiltyFlags {
		ROBOT_CAMERA_MASK = 1<<0,
		BOUNDING_BOX_MASK = 1<<1,
		COLLISION_MODEL_MASK = 1<<2,
		GRAPHICS_MODEL_MASK = 1<<3
	};
	
	void select();
	static void unselect();
	static GraphicsInfo* getSelection() { return userSelection; }
	
	Ogre::SceneNode& getJointNode() const { return *jointNode; }
	
	JointGraphicsInfo& getParent() const { return parent; }
	
	virtual void rebuildCenterOfMass() { updateCenterOfMass(); }
	
	void createRangeFinderNodes(const SensorRangeFinder& rf);
	void updateRangeFinderNodes(bool hit, const SensorRangeFinder& rf);
	
protected:
	void cleanup();
	
	const KinematicJoint& parentKJ() const;
	Ogre::Entity* createEntity(const std::string& identifier, const std::string& mesh);

	void updateBB();
	void updateModel();
	void updateCollisionModel();
	void updateMaterial();
	void updateModelRotation();
	void updateModelScale();
	void updateModelOffset();
	void updateCollisionModelRotation();
	void updateCollisionModelScale();
	void updateCollisionModelOffset();
	void updateCenterOfMass();
	void updateVisibility();
	
	static GraphicsInfo* userSelection;
	
	const Client& client;
	const LinkComponent& link;
	JointGraphicsInfo& parent;
	Ogre::SceneNode* jointNode;
	Ogre::SceneNode* bbNode;
	Ogre::SceneNode* centerOfMassNode;
	Ogre::SceneNode* inertiaNode;
	Ogre::SceneNode* rfBeamNode;
	Ogre::SceneNode* rfHitNode;
	Ogre::Entity* modelEntity;
	Ogre::Entity* collisionModelEntity;
	
	plist::CollectionCallbackMember<GraphicsInfo> * rotationListener;
	plist::CollectionCallbackMember<GraphicsInfo> * scaleListener;
	plist::CollectionCallbackMember<GraphicsInfo> * offsetListener;
	plist::CollectionCallbackMember<GraphicsInfo> * collisionRotationListener;
	plist::CollectionCallbackMember<GraphicsInfo> * collisionScaleListener;
	plist::CollectionCallbackMember<GraphicsInfo> * collisionOffsetListener;
	
	plist::CollectionCallbackMember<GraphicsInfo> centerOfMassListener;
	plist::PrimitiveCallbackMember<GraphicsInfo> massListener;
	
	// these should go after the other members to ensure valid scoping during construction/destruction
	plist::PrimitiveCallbackMember<GraphicsInfo> materialListener;
	plist::PrimitiveCallbackMember<GraphicsInfo> visibilityListener;

	plist::PrimitiveCallbackMember<GraphicsInfo> modelListener;
	plist::PrimitiveCallbackMember<GraphicsInfo> collisionModelListener;
};

class JointGraphicsInfo : public GraphicsInfo, protected plist::CollectionListener {
public:
	JointGraphicsInfo(const Client& cl, const KinematicJoint& kjSrc, Ogre::SceneNode& parentNode);
	~JointGraphicsInfo();
	
	std::string getName() const;
	const KinematicJoint& getKJ() const { return static_cast<const KinematicJoint&>(link); }
	
	void updateJointNode();
	virtual void rebuildCenterOfMass() {
		GraphicsInfo::rebuildCenterOfMass();
		for(std::map<LinkComponent*,GraphicsInfo*>::const_iterator it=components.begin(); it!=components.end(); ++it)
			it->second->rebuildCenterOfMass();
	}
	
protected:
	void clear() {
		for(std::map<LinkComponent*,GraphicsInfo*>::const_iterator it=components.begin(); it!=components.end(); ++it)
			delete it->second;
		components.clear();
	}
	
	virtual void plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& primitive);
	virtual void plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& primitive);
	virtual void plistCollectionEntriesChanged(plist::Collection& col);	
	
	std::map<LinkComponent*,GraphicsInfo*> components;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
