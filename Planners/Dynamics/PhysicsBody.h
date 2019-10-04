//-*-c++-*-
#ifndef INCLUDED_PhysicsBody_h_
#define INCLUDED_PhysicsBody_h_

#include "Motion/KinematicJoint.h"
#include "PhysicsWorld.h"

#include <LinearMath/btTransform.h>

class btCollisionShape;
class btCompoundShape;
class btGeneric6DofConstraint;
class btRigidBody;
class CollisionData;
class PhysicsWorld;

class ConstraintInterface;
class MotorController;

//! an interface to the Bullet physics engine, although parameters controlled by PhysicsBody::ComponentListener and its associated LinkComponent
class PhysicsBody {
	friend class PhysicsWorld;
public:
	
	//! This object subscribes to a LinkComponent (usually the root KinematicJoint of a chain) and creates a corresponding PhysicsBodies for it and its children.
	/*! When parameters change in the KinematicJoint, the ComponentListener will update fields in the PhysicsBody.  This may require
	 *  rebuilding the chain of inter-body constraints. */
	class ComponentListener : virtual protected plist::CollectionListener, virtual protected KinematicJoint::BranchListener {
	public:
		//! constructor, pass the physics world, source component, a dictionary of actuator positions (from which this joint's target position will be monitored), and the initial location */
		ComponentListener(PhysicsWorld& world_, LinkComponent& comp_, plist::DictionaryOf<plist::Primitive<float> >& positions_, const plist::Point* location, const plist::Point* orientation)
		: plist::CollectionListener(), world(world_), comp(comp_), positions(positions_), parent(NULL), subComps(), bodyLocation(location), bodyOrientation(orientation), body(NULL), 
		centerOfMassListener(NULL), collisionModelScaleListener(NULL), collisionModelRotationListener(NULL), collisionModelOffsetListener(NULL),
		massListener(comp.mass,*this,&PhysicsBody::ComponentListener::updateMass, false),
		collisionModelListener(comp.collisionModel,*this,&PhysicsBody::ComponentListener::updateModel,false)
		{
			init();
		}
		//! destructor
		~ComponentListener();
		
		std::string getPath() const; //!< returns a string identifying the component within a chain
		std::string getModel() const { return (comp.model.size()>0) ? comp.model : comp.collisionModel; } //!< returns the name of the graphics model in use
		
		PhysicsBody* getBody() const { return body; } //!< returns the PhysicsBody if available, otherwise NULL
		PhysicsBody& findBody(); //!< returns the body this link is part of; either #body or the first parent who has a non-NULL #body (if it hits the root, a body is created!)
		
	protected:
		//! constructor for children attached to this link
		ComponentListener(PhysicsWorld& world_, LinkComponent& comp_, plist::DictionaryOf<plist::Primitive<float> >& positions_, ComponentListener* parent_)
		: plist::CollectionListener(), world(world_), comp(comp_), positions(positions_), parent(parent_), subComps(), bodyLocation(NULL), bodyOrientation(NULL), body(NULL), 
		centerOfMassListener(NULL), collisionModelScaleListener(NULL), collisionModelRotationListener(NULL), collisionModelOffsetListener(NULL),
		massListener(comp.mass,*this,&PhysicsBody::ComponentListener::updateMass, false),
		collisionModelListener(comp.collisionModel,*this,&PhysicsBody::ComponentListener::updateModel,false)
		{
			init();
		}
		
		void init(); //!< does the setup and initialization
		void allocBody(); //!< allocates #body (to be called if the link has mass and shape, or its children do.
		void freeBody(); //!< frees #body
		ComponentListener& findLinkRoot(); //!< returns the ComponentListener for the KinematicJoint; either *this, or the parent.
		void rebuildBody() { findBody().rebuild(); } //!< called when the body has significantly changed and must rebuild its parameters
		void updateListeners(); //!< reinitialize individual field listeners, and those of its subcomponents
		
		virtual void plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& entry);
		virtual void plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& entry);
		virtual void plistCollectionEntriesChanged(plist::Collection& col);
		
		virtual void kinematicJointBranchAdded(KinematicJoint& parent, KinematicJoint& branch);
		virtual void kinematicJointBranchRemoved(KinematicJoint& parent, KinematicJoint& branch);
		virtual void kinematicJointReconfigured(KinematicJoint& joint);
		
		void updateMass();
		void updateModel();
		static bool hasCollisionShape(const LinkComponent& c);
		
		PhysicsWorld& world;
		LinkComponent& comp;
		plist::DictionaryOf<plist::Primitive<float> >& positions;
		ComponentListener* parent;
		std::map<LinkComponent*, ComponentListener*> subComps;

		const plist::Point * bodyLocation;
		const plist::Point * bodyOrientation;
		PhysicsBody* body;

		plist::CollectionCallbackMember<PhysicsBody>* centerOfMassListener;
		plist::CollectionCallbackMember<PhysicsBody>* collisionModelScaleListener;
		plist::CollectionCallbackMember<PhysicsBody>* collisionModelRotationListener;
		plist::CollectionCallbackMember<PhysicsBody>* collisionModelOffsetListener;
		plist::PrimitiveCallbackMember<PhysicsBody::ComponentListener> massListener;
		plist::PrimitiveCallbackMember<PhysicsBody::ComponentListener> collisionModelListener;
	};
	
	KinematicJoint& getLinkComponent() { return link; }
	
	std::string getPath() const;
	std::string getModel() const { return (link.model.size()>0) ? link.model : link.collisionModel; }
	
	btRigidBody& getBody() const { return *body; }
	const btVector3& getCenterOffset() const { return centerOffset; }
	
	void getCurrent(fmat::Column<3>& pos, fmat::Quaternion& quat) const;
	bool hasJoint() const;
	float getJoint() const;
	fmat::Column<3> getInertia() const { return fmat::pack(inertia[0],inertia[1],inertia[2]); }
	
	void teleport(const fmat::Column<3>& pos); //!< removes from world, then re-builds at new location
	void teleport(const fmat::Quaternion& quat); //!< removes from world, then re-builds at new location
	void teleport(const fmat::Column<3>& pos, const fmat::Quaternion& quat); //!< removes from world, then re-builds at new location
	void teleport(const btTransform& tr); //!< removes from world, then re-builds at new location
	void setVelocity(const fmat::Column<3>& linear, const fmat::Column<3>& angular); //!< sets a linear and angular velocity (world coordinates), also applies to children
	void setVelocity(const btVector3& linear, const btVector3& angular); //!< sets a linear and angular velocity (world coordinates), also applies to children
	
protected:
	PhysicsBody(PhysicsWorld& world_, KinematicJoint& link_, const plist::Primitive<float>* pos_, const btTransform& tr, PhysicsBody* parent)
		: world(world_), link(link_), qpos(pos_), totalMass(0), centerOffset(), inertia(),
		compound(NULL), body(NULL), joint(NULL), jointInterface(NULL), jointController(NULL),
		lastTransform(tr), parentBody(parent), children(), collisionDataName(),
		frictionForceListener(link.frictionForce,*this,&PhysicsBody::updateFrictionForce, false),
		anistropicFrictionListener(link.anistropicFrictionRatio,*this,&PhysicsBody::updateAnistropicFriction, false)
	{
		init();
	}
	~PhysicsBody();
	
	void init();
	void build();
	void buildLink(const KinematicJoint& kj, const fmat::Transform& tr);
	void addComponent(const LinkComponent& comp, const fmat::Transform& tr);
	void resetTransform();
	void clear();
	
	void updateFrictionForce();
	void updateAnistropicFriction();
	
	static bool isChild(const LinkComponent& pb, const LinkComponent& child);

	void rebuild() { clear(); build(); }
	
	PhysicsWorld& world;
	KinematicJoint& link;
	const plist::Primitive<float>* qpos;
	
	float totalMass;
	btVector3 centerOffset;
	btVector3 inertia;
	
	btCompoundShape * compound;
	btRigidBody * body;
	btGeneric6DofConstraint * joint;
	ConstraintInterface * jointInterface;
	MotorController * jointController;
	btTransform lastTransform;
	PhysicsBody* parentBody;
	std::set<PhysicsBody*> children;
	
	std::string collisionDataName;
	static std::map<std::string,CollisionData*> collisionData;
	friend class CollisionData;
	
	plist::PrimitiveCallbackMember<PhysicsBody> frictionForceListener;
	plist::CollectionCallbackMember<PhysicsBody> anistropicFrictionListener;
	
private:
	PhysicsBody(const PhysicsBody&); //!< do not call
	PhysicsBody& operator=(const PhysicsBody&); //!< do not call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
