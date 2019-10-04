//-*-c++-*-
#ifndef INCLUDED_KinematicJoint_h_
#define INCLUDED_KinematicJoint_h_

#include "SensorInfo.h"

#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Shared/fmatSpatial.h"
#include "Shared/BoundingBox.h"

#include "Planners/PlannerObstacles.h"

#include <set>
#include <algorithm>

class KinematicJoint;
class IKSolver;

//! these let you build the link from a series of #model and #material settings instead of one static model/material for the whole link
/*! note recursive structure... in this case, each is relative to its recursive parent instead of preceeding item in the array (as is done for KinematicJoint's serialization) */
class LinkComponent : virtual public plist::Dictionary {
	friend class KinematicJoint;
	
public:
	//! constructor
	LinkComponent()
		: plist::Dictionary(), mass(0), centerOfMass(), visible(true), material(), 
		model("CollisionModel"), modelScale(1,1,1), modelRotation(), modelOffset(),
		collisionModel(), collisionModelScale(1,1,1), collisionModelRotation(), collisionModelOffset(),
		parent(NULL),
		bbDirty(false), boundingBox(),
		collisionModelListener(collisionModel,*this,&LinkComponent::dirtyBB,false),
		collisionModelScaleListener(collisionModelScale,*this,&LinkComponent::dirtyBB,false),
		collisionModelRotationListener(collisionModelRotation,*this,&LinkComponent::dirtyBB,false),
		collisionModelOffsetListener(collisionModelOffset,*this,&LinkComponent::dirtyBB,false)
	{
		init();
	}
	
	//! explicit copy constructor needed to set bbLowListener and bbHighListener (which aren't copied)
	LinkComponent(const LinkComponent& c)
		: plist::Dictionary(), mass(c.mass), centerOfMass(c.centerOfMass), visible(c.visible), material(c.material), 
		model(c.model), modelScale(c.modelScale), modelRotation(c.modelRotation), modelOffset(c.modelOffset),
		collisionModel(c.collisionModel), collisionModelScale(c.collisionModelScale), collisionModelRotation(c.collisionModelRotation), collisionModelOffset(c.collisionModelOffset),
		parent(NULL),
		bbDirty(c.bbDirty), boundingBox(c.boundingBox), 
		collisionModelListener(collisionModel,*this,&LinkComponent::dirtyBB,false),
		collisionModelScaleListener(collisionModelScale,*this,&LinkComponent::dirtyBB,false),
		collisionModelRotationListener(collisionModelRotation,*this,&LinkComponent::dirtyBB,false),
		collisionModelOffsetListener(collisionModelOffset,*this,&LinkComponent::dirtyBB,false)
	{
		init();
	}
	
	//! assignment, except for parent
	LinkComponent& operator=(const LinkComponent& link) {
		if(&link==this)
			return *this;
		mass = link.mass;
		centerOfMass = link.centerOfMass;
		visible = link.visible;
		material = link.material;
		model = link.model;
		modelScale = link.modelScale;
		modelRotation = link.modelRotation;
		modelOffset = link.modelOffset;
		collisionModel = link.collisionModel;
		collisionModelScale = link.collisionModelScale;
		collisionModelRotation = link.collisionModelRotation;
		collisionModelOffset = link.collisionModelOffset;
		// note no parent copy
		bbDirty = link.bbDirty;
		boundingBox = link.boundingBox;
		return *this;
	}
	
	virtual ~LinkComponent() {}
	
	plist::Primitive<float> mass; //!< the mass of the component in kilograms
	plist::Point centerOfMass; //!< the center of mass of this component
	
	plist::Primitive<bool> visible; //!< if false, will only be shown in the Mirage host's flying camera
	
	plist::Primitive<std::string> material; //!< mesh files specify "default" textures, but this can override with a different material
	
	plist::Primitive<std::string> model; //!< name of an OGRE mesh file; if set to "CollisionModel" will display collisionModel (and apply values from #collisionModelScale, #collisionModelRotation, and #collisionModelOffset)
	plist::Point modelScale; //!< scaling factor to apply to #model
	plist::Point modelRotation; //!< rotation to apply to #model (specifies axis component of quaternion, e.g. from axis/angle: axis * sin(angle/2) )
	plist::Point modelOffset; //!< a translation to apply to #model
	
	plist::Primitive<std::string> collisionModel; //!< name of an OGRE mesh file, or one of a set of primitives (Cube, Cylinder, Sphere)
	plist::Point collisionModelScale; //!< scaling factor to apply to #collisionModel
	plist::Point collisionModelRotation; //!< rotation to apply to #collisionModel (specifies axis component of quaternion, e.g. from axis/angle: axis * sin(angle/2) )
	plist::Point collisionModelOffset; //!< a translation to apply to #collisionModel
	
	//! Returns a 2D planner obstacle of this component's collision object, or NULL if no collision model is specified
	/*! This is a new allocation, caller is responsible for deallocation. */
	PlannerObstacle2D* getObstacle(const fmat::Transform& worldT) const;
	
	//! Inserts 2D projections of this link's collision components into @a obs, if @a recurse is set, recurses on branches
	void getObstacles(const fmat::Transform& worldT, HierarchicalObstacle& obs, bool recurse) const;
	
	//! Returns the axis-aligned bounding box (relative to link frame) of this component only (i.e. for KinematicJoint, ignores subcomponents)
	virtual BoundingBox3D getOwnAABB() const { return getAABB(); }
	
	//! Returns the axis-aligned bounding box (relative to link frame) of this component and any subcomponents
	const BoundingBox3D& getAABB() const { if(bbDirty) updateBB(); return boundingBox; }
	
	//! Computes the tightest-fitting 2D rectangle for this link (writing the result into @a ro).
	/*! @ro will be in the world coordinates, projecting the link collision model down the world's Z into its XY plane. */
	bool getOwnBB2D(const fmat::Transform& worldT, RectangularObstacle& ro) const;
    
    //! Computes a 3D box for this link (writing the result into @a bo).
	/*! @ro will be in the world coordinates */
	bool getOwnBB3D(const fmat::Transform& worldT, BoxObstacle& bo) const;
	
	//! Computes a 2D rectangle for this link and its subcomponents (writing the result into @a ro).
	/*! @ro will be in the world coordinates, projecting the link collision model down the world's Z into its XY plane.
	 The rectangle will be aligned to the link axis (i.e. this is getAABB() projected into the world.) */
	bool getBB2D(const fmat::Transform& worldT, RectangularObstacle& ro) const;
    
    //! Computes a 3D box for this link and its subcomponents (writing the result into @a bo).
	/*! @ro will be in the world coordinates.
	 The rectangle will be aligned to the link axis (i.e. this is getAABB() projected into the world.) */
	bool getBB3D(const fmat::Transform& worldT, BoxObstacle& bo) const;
	
	//! return the center of mass of this component, ONLY, as a homogenous scaled vector
	/*! The last element (homogeneous scale factor) is left as the total mass, so divide by this value to normalize.
	 *  Otherwise, just access #centerOfMass directly. */
	fmat::Column<4> getMassVector() const { return fmat::pack(mass*centerOfMass[0],mass*centerOfMass[1],mass*centerOfMass[2],mass); }
	
	//! returns the center of mass of this link only, not including any branches; if this is actually a KinematicJoint, will include subcomponents
	virtual void sumLinkCenterOfMass(fmat::Column<3>& cOfM, float& totalMass) const;
	
	//! returns the unnormalized center of mass of this link only, not including any branches; if this is actually a KinematicJoint, will include subcomponents
	/*! The last element (homogeneous scale factor) is left as the total mass, so divide by this value to normalize. */
	virtual fmat::Column<4> sumLinkCenterOfMass() const { return getMassVector(); }
	
	//! returns true if #mass is greater than zero, of if this is a KinematicJoint, if any of the components have mass greater than zero
	virtual bool hasMass() const { return mass>0; }
	
	//! converts the model rotation and offset members into the supplied fmat::Transform
	void getModelTransform(fmat::Transform& tr) const;
	
	//! returns the model rotation and offset members as a fmat::Transform
	fmat::Transform getModelTransform() const {
		fmat::Transform tr; getModelTransform(tr); return tr;
	}
	
	//! converts the collision model rotation and offset members into the supplied fmat::Transform
	void getCollisionModelTransform(fmat::Transform& tr) const;
	
	//! returns the model rotation and offset members as a fmat::Transform
	fmat::Transform getCollisionModelTransform() const {
		fmat::Transform tr; getCollisionModelTransform(tr); return tr;
	}
	
	//! returns #parent joint, or NULL if this is the first joint, see addBranch()/removeBranch()
	KinematicJoint* getParent() const { return parent; }
	
	//! returns the next mobile ancestor joint in the kinematic chain
	KinematicJoint* getNextMobileAncestor() const;
	
	PLIST_CLONE_DEF(LinkComponent,new LinkComponent(*this));
	
protected:
	KinematicJoint* parent; //!< if non-NULL, the parent joint to which this one is attached
	
	mutable bool bbDirty; //!< indicates bounding boxes need to be recomputed
	mutable BoundingBox3D boundingBox; //!< bounding box of this link (including subcomponents, but not child links)
	
	virtual void dirtyBB(); //!< sets #bbDirty to true to cause it to be recomputed on next getAABB() call
	virtual void updateBB() const; //!< recomputes #boundingBoxLow and #boundingBoxHigh based on collision model parameters
	void computeOwnAABB(BoundingBox3D& bb) const;
	static void computeBB2D(const fmat::Transform& fullT, RectangularObstacle& ro, const fmat::Column<3>& obD);
	
	void init() {
		addEntry("Mass",mass,"Mass of the component, in kilograms.  If 0, indicates static object, does not move but can still collide. (default 0)");
		addEntry("CenterOfMass",centerOfMass,"Position of average mass relative to parent frame.");
		addEntry("Visible",visible,"If true, indicates component should be rendered for simulated cameras, otherwise only appears in the Mirage user window. (default true)");
		addEntry("Material",material,"Name of an Ogre material, found in mirage/media/*.material files.");
		addEntry("Model",model,"An Ogre .mesh file, or \"CollisionModel\" to render the collision primitive. (default CollisionModel)");
		addEntry("ModelScale",modelScale,"Scales the graphics mesh loaded by Model.");
		addEntry("ModelRotation",modelRotation,"Rotates the graphics mesh loaded by Model, relative to parent frame. (supply axis component of quaternion, e.g. from axis/angle: axis * sin(angle/2) )");
		addEntry("ModelOffset",modelOffset,"Positions the graphics mesh loaded by Model, relative to parent frame.");
		addEntry("CollisionModel",collisionModel,"A Bullet primitive collision shape: { Cube | Cylinder | Sphere | Plane }");
		addEntry("CollisionModelScale",collisionModelScale,"Scales the CollisionModel, which by default is 1x1x1, so this sets the object dimensions.");
		addEntry("CollisionModelRotation",collisionModelRotation,"Rotates the CollisionModel relative to parent frame. (supply axis component of quaternion, e.g. from axis/angle: axis * sin(angle/2) )");
		addEntry("CollisionModelOffset",collisionModelOffset,"Positions the CollisionModel, relative to parent frame.");
		setLoadSavePolicy(FIXED,SYNC);
	}

	// listener callbacks should go last to ensure they are activated after members are initialized, and deactivated before they are destructed
	plist::PrimitiveCallbackMember<LinkComponent> collisionModelListener; //!< if going from empty string to non-empty string, indicates bounding box values need to be rebuilt
	plist::CollectionCallbackMember<LinkComponent> collisionModelScaleListener; //!< indicates bounding box values need to be rebuilt
	plist::CollectionCallbackMember<LinkComponent> collisionModelRotationListener; //!< indicates bounding box values need to be rebuilt
	plist::CollectionCallbackMember<LinkComponent> collisionModelOffsetListener; //!< indicates bounding box values need to be rebuilt
};


//! Manages parameters which define the position and type of motion produced by an actuator (i.e. forward kinematics)
class KinematicJoint : public virtual plist::PrimitiveListener, public LinkComponent {
	friend class KinematicJointLoader;
	friend class KinematicJointSaver;
public:
	class BranchListener : public plist::Listener {
	public:
		virtual void kinematicJointBranchAdded(KinematicJoint& parent, KinematicJoint& branch) {};
		virtual void kinematicJointBranchRemoved(KinematicJoint& parent, KinematicJoint& branch) {};
		virtual void kinematicJointReconfigured(KinematicJoint& joint) {};
	};
	
	//! constructor
	KinematicJoint() : 
	  plist::Dictionary(), LinkComponent(), 
	  jointType(REVOLUTE, jointTypeNames), theta(0), d(0), alpha(0), r(0), qOffset(0), qmin(0), qmax(0),
	  components(), frictionForce(0.5f), anistropicFrictionRatio(1,1,1), ikSolver(), sensorInfo(),
	  controllerInfo(), outputOffset(), branches(), branchListeners(NULL),
	  depth(0), q(0), To(), Tq(), ik(NULL), componentsListener(components,*this)
  {
		initEntries();
  }

	//! copy constructor (deep copy, but doesn't register with parent)
	KinematicJoint(const KinematicJoint& kj) : 
	  plist::Dictionary(), LinkComponent(kj), 
	  jointType(kj.jointType), theta(kj.theta), d(kj.d), alpha(kj.alpha), r(kj.r),
	  qOffset(kj.qOffset), qmin(kj.qmin), qmax(kj.qmax),
	  components(kj.components), frictionForce(kj.frictionForce),
	  anistropicFrictionRatio(kj.anistropicFrictionRatio), ikSolver(kj.ikSolver), 
	  sensorInfo(kj.sensorInfo), controllerInfo(kj.controllerInfo), outputOffset(kj.outputOffset),
	  branches(), branchListeners(NULL), depth(0), q(kj.q), To(kj.To), Tq(kj.Tq),
	  ik(NULL), componentsListener(components,*this)
	{
		initEntries();
		copyBranches(kj);
	}
	//! deep copy via copyBranches(), replaces current branches.  This does not affect listener list (other than calling them), the parent or depth values
	KinematicJoint& operator=(const KinematicJoint& kj) {
		shallowCopy(kj);
		copyBranches(kj);
		return *this;
	}
	//! copies all parameters from @a kj, @e except listeners, depth, parent or children; see copyBranches()
	void shallowCopy(const KinematicJoint& kj) {
		if(&kj==this)
			return;
		LinkComponent::operator=(kj);
		jointType = kj.jointType;
		theta = kj.theta;
		d = kj.d;
		alpha = kj.alpha;
		r = kj.r;
		qOffset = kj.qOffset;
		qmin = kj.qmin;
		qmax = kj.qmax;
		components = kj.components;
		frictionForce=kj.frictionForce;
		anistropicFrictionRatio=kj.anistropicFrictionRatio;
		ikSolver = kj.ikSolver;
		sensorInfo = kj.sensorInfo;
		controllerInfo = kj.controllerInfo;
		outputOffset = kj.outputOffset;
		q = kj.q;
		To = kj.To;
		Tq = kj.Tq;
		fireReconfigured();
		// ** note what is NOT copied: **
		//branchListeners = kj.branchListeners;
		//parent = kj.parent;
		//depth = kj.depth;
		// copyBranches();
	}
	//! releases current children and clones those from @kj; does not copy parameters, see shallowCopy()
	void copyBranches(const KinematicJoint& kj) {
		if(&kj==this)
			return;
		// watch out for assigning a sub-link to an ancestor, make copies of all sub-branches from source, then clear current branches, then add previously created clones as new branches
		std::set<KinematicJoint*> newBranches;
		for(std::set<KinematicJoint*>::const_iterator it=kj.branches.begin(); it!=kj.branches.end(); ++it)
			newBranches.insert(new KinematicJoint(**it));
		clearBranches();
		for(std::set<KinematicJoint*>::const_iterator it=newBranches.begin(); it!=newBranches.end(); ++it)
			addBranch(*it);
	}
	//! destructor, recursively destroys sub-tree
	virtual ~KinematicJoint();
	
	//! types of joints which are supported
	enum JointType_t {
		REVOLUTE, //!< a joint which rotates around the z-axis
		PRISMATIC //!< a joint which slides along the z-axis
	};
	//! provides human readable names for JointType_t, for loading/saving #jointType (null terminated)
	static const char* jointTypeNames[3];
	//! the type of motion produced by this joint
	plist::NamedEnumeration<JointType_t> jointType;
	
	// parameters in the order they are applied to get to this joint's frame
	plist::Angle theta; //!< rotation about the previous joint's z-axis to point its x-axis parallel to the common normal with this joint's z-axis (aka "Modified Denavit-Hartenberg")
	plist::Primitive<fmat::fmatReal> d; //!< translation along the previous joint's z-axis to align its x-axis with the common normal with this joint's z-axis (aka "Modified Denavit-Hartenberg")
	plist::Angle alpha; //!< rotation about this joint's x-axis to make the previous joint's z-axis parallel to this one (aka "Modified Denavit-Hartenberg")
	plist::Primitive<fmat::fmatReal> r; //!< translation from the previous joint's z-axis along the common normal to place this joint's origin (aka "Modified Denavit-Hartenberg")
	plist::Angle qOffset; //!< a constant offset to the #q setting to define the physical zero-point of the joint
	
	plist::Angle qmin; //!< indicates the minimum q value which inverse kinematics may provide (if equal to #qmax, the joint is considered immobile)
	plist::Angle qmax; //!< indicates the maximum q value which inverse kinematics may provide (if equal to #qmin, the joint is considered immobile)
	
	//! a list of link features, for simple display or collision models with more than one primitive
	plist::ArrayOf<LinkComponent> components;
	typedef plist::ArrayOf<LinkComponent>::const_iterator component_iterator; //!< for convenience when looping over #components
	
	plist::Primitive<float> frictionForce; //!< conversion from velocity to friction force (default 0.5)
	plist::Point anistropicFrictionRatio; //!< direction sensitivity of friction, '1' in all directions means it is not direction sensitive
		
	plist::Primitive<std::string> ikSolver; //!< specifies the name of the inverse kinematics solver to use with this appendage
	
	plist::ArrayOf<SensorInfo> sensorInfo; //!< Stores sensor parameters for simulated input, type indicated by SensorType entry corresponding to a SensorInfo subclass
	
	//! Parameters for joint motion model, used for simulation
	/*! You should probably sync entries made here with entries in Planners/Dynamics/MotorController */
	struct ControllerInfo : virtual public plist::Dictionary {
		explicit ControllerInfo() : plist::Dictionary(), velocity(false), forceControl(false) { init(); } //!< constructor
		ControllerInfo(const ControllerInfo& ci) : plist::Dictionary(), velocity(ci.velocity), forceControl(ci.forceControl) { init(); } //!< copy constructor for cloning
		plist::Primitive<bool> velocity; //!< Adjusts interpretation of feedback and application of friction, false implies position control
		plist::Primitive<bool> forceControl; //!< If true, simulation will use force control to move the joint rather than using position constraints.  Grippers should set this to true for more realistic object interaction.
	protected:
		//! performs common initialization
		void init() {
			addEntry("Velocity",velocity,"Adjusts interpretation of feedback and application of friction, false implies position control");
			addEntry("ForceControl",forceControl,"If true, simulation will use force control to move the joint rather than using position constraints.  Grippers should set this to true for more realistic object interaction.");
		}
	};
	ControllerInfo controllerInfo; //!< Stores controller parameters for simulated input, type indicated by ControllerType entry corresponding to a ControllerInfo subclass
	
	
	plist::OutputSelector outputOffset; //!< name or index of the joint this maps to so we can integrate with Tekkotsu's flat arrays
	
	void getObstacles(const fmat::Transform& worldT, class HierarchicalObstacle& obs, bool recurse) const;
	
	//! Returns the axis-aligned bounding box (relative to link frame) of this component only (i.e. KinematicJoint ignores subcomponents)
	/*! KinematicJoint uses cache members to store the expanded subcomponent BB, so must recompute the local BB each time */
	virtual BoundingBox3D getOwnAABB() const { BoundingBox3D bb; computeOwnAABB(bb); return bb; }
	
	//! returns the current #q value
	float getQ() const { return q; }
	
	//! sets the current joint position (#q) (ignoring #qmin, #qmax) and updates the transformation matrix (if #q has changed)
	void setQ(float x) { if(x!=q) { q=x; updateTq(); } }

	//! sets the current joint position (#q) to x, clipped to [#qmin,#qmax], returning true if in range
	bool tryQ(float x);
	
	//! sets the joint position, and #qmin and #qmax as well to prevent inverse kinematics from using it
	void freezeQ(float x) { qmin=qmax=x; setQ(x); }
	
	//! returns true if x is within [#qmin,#qmax] (inclusive)
	bool validQ(float x) const { return (x>=qmin && x<=qmax); }
	
	//! sets the joint position and its childrens' positions from a flat array, using #outputOffset
	/*! @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the joint values for only a region of outputs in use, e.g. values[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void pushChildrenQIntoArray(M& values, int deoffset, unsigned int max) const {
	  if(outputOffset!=plist::OutputSelector::UNUSED && static_cast<unsigned int>(outputOffset-deoffset)<max)
	    values[outputOffset-deoffset]=q;
	  for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
	    (*it)->pushChildrenQIntoArray(values, deoffset, max);
	}		
	
	//! sets the joint position and its ancestors' positions from a flat array, using #outputOffset
	/*! @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the joint values for only a region of outputs in use, e.g. values[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void pushAncestorsQIntoArray(M& values, int deoffset, unsigned int max) const {
		if(outputOffset!=plist::OutputSelector::UNUSED && static_cast<unsigned int>(outputOffset-deoffset)<max)
			values[outputOffset-deoffset]=q;
		if(parent!=NULL)
			parent->pushAncestorsQIntoArray(values, deoffset, max);
	}
	
	//! sets the joint position and its childrens' positions from a flat array, using #outputOffset
	/*! @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the joint values for only a region of outputs in use, e.g. values[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void pullChildrenQFromArray(M& values, int deoffset, unsigned int max) {
		if(outputOffset!=plist::OutputSelector::UNUSED && qmin!=qmax && static_cast<unsigned int>(outputOffset-deoffset)<max)
			setQ(values[outputOffset-deoffset]);
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			(*it)->pullChildrenQFromArray(values, deoffset, max);
	}
	
	//! sets the joint position and its ancestors' positions from a flat array, using #outputOffset
	/*! @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the joint values for only a region of outputs in use, e.g. values[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void pullAncestorsQFromArray(M& values, int deoffset, unsigned int max) {
		if(outputOffset!=plist::OutputSelector::UNUSED && qmin!=qmax && static_cast<unsigned int>(outputOffset-deoffset)<max)
			setQ(values[outputOffset-deoffset]);
		if(parent!=NULL)
			parent->pullAncestorsQFromArray(values, deoffset, max);
	}		
	
	//! sets the joint position and its childrens' to zero
	void zeroChildrenQ();
	
	//! sets the joint position and its ancestors' to zero
	void zeroAncestorQ();
	
	
	//! returns the tranformation matrix which converts from the link's reference frame (i.e. the frame which moves with the joint's #q) to the base frame
	fmat::Transform getFullT() const;
	
	//! returns the tranformation matrix which converts from the base frame to the link's reference frame (i.e. the frame which moves with the joint's #q)
	fmat::Transform getFullInvT() const { return getFullT().rigidInverse(); }
	
	//! returns the tranformation matrix which converts from the link's reference frame (i.e. the frame which moves with the joint's #q) to the specified joint's link frame
	fmat::Transform getT(const KinematicJoint& j) const;
	
	//! returns the tranformation matrix which converts from the joint's reference frame (i.e. the frame in which the link moves) to the parent's frame
	/*! This transformation is constant in terms of #q. */
	const fmat::Transform& getTo() const { return To; }
	
	//! returns the tranformation matrix which converts from the link's reference frame (i.e. the frame which moves with the joint's #q) to the parent's frame
	/*! This transformation is changes with #q. */
	const fmat::Transform& getTq() const { return Tq; }
	
	//! returns the center of mass of this link and all of its branches, given their current positions, resulting position relative to this link
	void sumCenterOfMass(fmat::Column<3>& cOfM, float& totalMass) const;
	
	//! returns the unnormalized center of mass of this link and all of its branches, given their current positions, relative to this link
	/*! The last element (homogeneous scale factor) is left as the total mass, so divide by this value to normalize. */
	fmat::Column<4> sumCenterOfMass() const;
	
	//! returns the unnormalized center of mass of this link only, not including any branches
	/*! The last element (homogeneous scale factor) is left as the total mass, so divide by this value to normalize. */
	virtual fmat::Column<4> sumLinkCenterOfMass() const;
	
	using LinkComponent::sumLinkCenterOfMass;
	
	virtual bool hasMass() const;
	
	//! returns a column vector indicating the joint's motion in base coordinates, at a point @a p (in base coordinates)
	/*! The first 3 rows indicate the ∂x/∂q, ∂y/∂q, and ∂z/∂q values.
	 *  The last 3 rows indicate the axis of rotation (or all 0's for a prismatic joint) */
	fmat::Column<6> getJointJacobian(const fmat::Column<3>& p) const;
	
	//! computes the motion of this joint and all of its ancestors at a point @a p (in base coordinates), @a j should be empty on call, may wish to reserve #depth+1 
	/*! The first 3 rows indicate the ∂x, ∂y, and ∂z values per ∂q corresponding to each column.
	 *  The last 3 rows indicate the axis of rotation for each joint (or all 0's for prismatic joints) */
	void getFullJacobian(const fmat::Column<3>& p, std::vector<fmat::Column<6> >& j) const {
		if(parent!=NULL)
			parent->getFullJacobian(p,j);
		j.push_back(getJointJacobian(p));
	}

	//! computes the motion of this joint and all of its ancestors at a point @a p (in base coordinates), @a j should be empty on call, may wish to reserve #depth+1 
	/*! The first 3 rows indicate the ∂x, ∂y, and ∂z values per ∂q corresponding to each column.
	 *  The last 3 rows indicate the axis of rotation for each joint (or all 0's for prismatic joints) */
	void getMobileJacobian(const fmat::Column<3>& p, std::vector<fmat::Column<6> >& j) const {
		if(parent!=NULL)
			parent->getMobileJacobian(p,j);
		if(isMobile())
			j.push_back(getJointJacobian(p));
	}
	
	//! returns the position of this joint in base coordinates (as a length 3 vector)
	fmat::Column<3> getWorldPosition() const { return getFullT().translation(); }
	//! returns the orientation of this joint in base coordinates (as a 3x3 matrix)
	fmat::Matrix<3,3> getWorldRotation() const { return getFullT().rotation(); }
	
	//! returns the position of this joint relative to its parent's link frame
	fmat::Column<3> getPosition() const { return Tq.translation(); }
	//! returns the position of the parent origin relative to this joint's link frame
	fmat::Column<3> getParentPosition() const;
	//! returns the orientation of this joint relative to its parent's link frame (as a 3x3 matrix)
	fmat::Matrix<3,3> getRotation() const { return Tq.rotation(); }
	
	//! returns the orientation of this joint in base coordinates (as a quaternion (w,x,y,z))
	fmat::Quaternion getWorldQuaternion() const;
	//! returns the orientation of this joint in base coordinates (as a quaternion (w,x,y,z))
	fmat::Quaternion getQuaternion(const KinematicJoint& j) const;
	//! returns the orientation of this joint relative to its parent's link frame (as a quaternion (w,x,y,z))
	fmat::Quaternion getQuaternion() const { return fmat::Quaternion::fromMatrix(Tq.rotation()); }
	
	//! if the node is an array, loads the sub-tree, otherwise expects a dict to load the joint's own parameters
	virtual void loadXML(xmlNode* node);
	//! if the node is NULL, has no parent, or is a plist node, writes human-readable help text as a comment and then recurses through the subtrees (otherwise saves only this joint's parameters)
	virtual void saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const;
	using plist::Dictionary::saveXML;
	
	
	//! Builds a map from output offsets to this and child joints (call it on the root node –– doesn't handle ancestors
	/*! The map must be initialized to NULL values before calling this method.
   * @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the mapping for only a region of outputs in use, e.g. childMap[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void buildChildMap(M& childMap, int deoffset, unsigned int max) {
		if(outputOffset!=plist::OutputSelector::UNUSED &&
			 static_cast<unsigned int>(outputOffset-deoffset)<max && !childMap[outputOffset-deoffset])
			childMap[outputOffset-deoffset] = this;
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			(*it)->buildChildMap(childMap,deoffset,max);
	}
	
	//! builds a map from output offsets to this and child joints (call it on the root node –– doesn't handle ancestors
	/*! The map must be initialized to NULL values before calling this method.
   * @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the mapping for only a region of outputs in use, e.g. childMap[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void buildChildMap(M& childMap, int deoffset, unsigned int max) const {
		if(outputOffset!=plist::OutputSelector::UNUSED && 
			 static_cast<unsigned int>(outputOffset-deoffset)<max && !childMap[outputOffset-deoffset])
			childMap[outputOffset-deoffset] = this;
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			(*it)->buildChildMap(childMap,deoffset,max);
	}
	
	//! builds a map from output offsets to this and child joints (call it on the root node –– doesn't handle ancestors
	/*! The map must be initialized to NULL values before calling this method.
   * @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the mapping for only a region of outputs in use, e.g. childMap[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void buildMobileChildMap(M& childMap, int deoffset, unsigned int max) {
		if(isMobile() && outputOffset!=plist::OutputSelector::UNUSED && static_cast<unsigned int>(outputOffset-deoffset)<max)
			childMap[outputOffset-deoffset] = this;
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			(*it)->buildMobileChildMap(childMap,deoffset,max);
	}
	
	//! builds a map from output offsets to this and child joints (call it on the root node –– doesn't handle ancestors
	/*! The map must be initialized to NULL values before calling this method.
   * @a deoffset will be subtracted from each index, and the result only added to the map if it (as unsigned int) is less than @a max.
	 *  This is intended to allow you to extract the mapping for only a region of outputs in use, e.g. childMap[NumArmJoints], offset=ArmOffset and max=NumArmJoints */
	template<class M> void buildMobileChildMap(M& childMap, int deoffset, unsigned int max) const {
		if(isMobile() && outputOffset!=plist::OutputSelector::UNUSED && static_cast<unsigned int>(outputOffset-deoffset)<max)
			childMap[outputOffset-deoffset] = this;
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			(*it)->buildMobileChildMap(childMap,deoffset,max);
	}
	
	//! type of iterators over sub-trees
	typedef std::set<KinematicJoint*>::const_iterator branch_iterator;
	
	//! returns a reference to #branches so you can iterator over sub-trees
	const std::set<KinematicJoint*>& getBranches() const { return branches; }
	
	//! adds the joint to the #branches, control over memory (de)allocation is assumed by this instance until/unless removeBranch() is called
	/*! For convenience, returns @a b */
	KinematicJoint* addBranch(KinematicJoint* b);
	
	//! removes the specified joint from #branches, returning control of (de)allocation of @a b to caller.
	/*! For convenience, returns @a b */
	KinematicJoint* removeBranch(KinematicJoint* b);
	
	//! deletes all entries of #branches en masse
	void clearBranches();
	
	//! returns true if the link following this joint has more than one additional joint attached (i.e. if #branches.size() > 1)
	bool isBranch() const { return branches.size()>1; }
	
	//! Clones joints from this back to and including the root, not cloning side branches; returns a pointer to the @e last (i.e. leaf) joint in the cloned branch.
	/*! To clone a joint and all its children (instead of its direct ancestors as is done here), use the copy constructor or operator=.
	  *  Just use getRoot() on the result if you want to store the root joint instead of the end joint. */
	KinematicJoint* cloneBranch() const;
	
	//! returns the 'first' element of #branches, or NULL if this is a leaf
	/*! The choice of 'first' is arbitrary (and not necessarily consistent run-to-run), you should not use this if you expect to encounter branching chains */
	KinematicJoint* nextJoint() const { return *branches.begin(); }
	
	//! returns true if qmin≠qmax (i.e. the joint is considered "mobile", and not just a abstract reference frame)
	bool isMobile() const { return qmin!=qmax; }
	
	//! returns root joint (will return 'this' if this is the first joint), see addBranch()/removeBranch()
	KinematicJoint* getRoot() { KinematicJoint* kj=this; while(kj->parent!=NULL) kj=kj->parent; return kj; }
	//! returns root joint (will return 'this' if this is the first joint), see addBranch()/removeBranch()
	const KinematicJoint* getRoot() const { const KinematicJoint* kj=this; while(kj->parent!=NULL) kj=kj->parent; return kj; }
	//! returns a string containing the names of all ancestors' outputOffset values separated by @a sep
	std::string getPath(const std::string& sep = "/") const { return (parent==NULL ? std::string() : parent->getPath(sep)) + sep + outputOffset.get(); }
	//! returns 0 if this is the root (no parent), otherwise one plus the parent's depth
	size_t getDepth() const { return depth; }
	
	void addBranchListener(BranchListener* l) const;
	void removeBranchListener(BranchListener* l) const;
	bool isBranchListener(BranchListener* l) const;
	
	//! returns an IKSolver instance, which will be cached internally for future accesses
	IKSolver& getIK() const;
	
	PLIST_CLONE_DEF(KinematicJoint,new KinematicJoint(*this));
	
protected:
	//! add the plist entries to the dictionary superclass
	void initEntries() {
		// Since we're going to have a lot of KinematicJoints listed together, we're not going to add
		// a description comment here, which would be repeated for each instance.
		// Instead, a single comment will be dumped at the beginning of the array of joints.
		addEntry("JointType",jointType);
		addEntry("θ",theta); // aka \u03B8, but we use the character literal because we explicitly want UTF-8 in the file output
		addEntry("d",d);
		addEntry("α",alpha); // aka \u03B1, but we use the character literal because we explicitly want UTF-8 in the file output
		addEntry("r",r);
		addEntry("qOffset",qOffset);
		addEntry("Min",qmin);
		addEntry("Max",qmax);
		addEntry("Components",components);
		addEntry("FrictionForce",frictionForce);
		addEntry("AnistropicFrictionRatio",anistropicFrictionRatio);
		addEntry("IKSolver",ikSolver);
		addEntry("SensorInfo",sensorInfo);
		addEntry("ControllerInfo",controllerInfo);
		addEntry("Name",outputOffset);
		outputOffset.setRange(0,-1U); // accept anything (i.e. reference frames are fine)
		addSelfListener();
		setLoadSavePolicy(FIXED,SYNC);
	}
	
	//! recursively computes the full transformation matrix, converting from this joint's frame to the specified joint's frame, or the base frame if @end is NULL
	/*! @a t does not need to be initialized to anything prior to call, but will be 4x4 on return \n
	  *  @a endj @e must be an ancestor of this joint or the function will segfault (NULL is the ancestor of the root, so that's valid) */
	void getFullT(fmat::Transform& t, const KinematicJoint* endj) const {
		if(parent!=endj) {
			parent->getFullT(t,endj);
			t*=Tq;
		} else {
			t=Tq;
		}
	}
	void getQuaternion(fmat::Quaternion& quat, const KinematicJoint* endj) const {
		if(parent!=endj) {
			parent->getQuaternion(quat,endj);
			quat *= getQuaternion();
		} else {
			quat = getQuaternion();
		}
	}
	
	//! shorthand for saving a specific node, forwarding call to saveXMLNode()
	void doSaveXMLNode(std::set<std::string>& seen, xmlNode* node, const std::string& key, const std::string& indentStr, size_t longestKeyLen) const {
		const_iterator it=findEntry(key);
		if(it==end())
			return;
		saveXMLNode(node, key, it->second, indentStr, longestKeyLen);
		//seen.insert(key);
	}
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	virtual void addSelfListener(); //!< subscribes the instance to be notified of changes to its public plist::Primitive members, and then calls updateTo()
	virtual void removeSelfListener(); //!< unsubscribes the instance from its public plist::Primitive members
	void updateTo(); //!< regenerates #To from the a, d, alpha, and theta parameters, includes call to updateTq() as well
	void updateTq(); //!< updates #Tq from the q and qOffset parameters (based on current #To)
	virtual void updateBB() const;
	void updateDepth() {
		if(parent==NULL)
			depth=0;
		else
			depth = parent->depth+1;
		std::for_each(branches.begin(), branches.end(), std::mem_fun(&KinematicJoint::updateDepth));
	}
	
	//! sub-chains of joints, supporting branches but not cycles (nor NULL entries)
	std::set<KinematicJoint*> branches;
	
	void fireBranchAdded(KinematicJoint& val);
	void fireBranchRemoved(KinematicJoint& val);
	void fireReconfigured();
	mutable std::set<BranchListener*>* branchListeners;
	
	size_t depth; //!< one plus #parent's depth, or 0 if parentless
	
	fmat::fmatReal q; //!< current joint position (radian rotation about z if revolute, displacement along z if prismatic)
	fmat::Transform To; //!< transformation to the joint's origin
	fmat::Transform Tq; //!< transformation to origin, including final q rotation
	mutable IKSolver * ik; //!< an instance of the IKSolver corresponding to #ikSolver

	void setParent(LinkComponent& link) { link.parent = this; dirtyBB(); } //!< for use by ComponentsListener, work around #parent being protected
	void unsetParent(LinkComponent& link) { link.parent = NULL; dirtyBB(); } //!< for use by ComponentsListener, work around #parent being protected
	
	class ComponentsListener : protected plist::CollectionListener {
	public:
		ComponentsListener(const plist::ArrayOf<LinkComponent>& source, KinematicJoint& kj) : plist::CollectionListener(), comps(source), parent(kj) {
			comps.addCollectionListener(this);
		}
		~ComponentsListener() { comps.removeCollectionListener(this); }
	protected:
		virtual void plistCollectionEntryAdded(plist::Collection& /*col*/, ObjectBase& primitive) { parent.setParent(dynamic_cast<LinkComponent&>(primitive)); }
		virtual void plistCollectionEntryRemoved(plist::Collection& /*col*/, ObjectBase& primitive) { parent.unsetParent(dynamic_cast<LinkComponent&>(primitive)); }
		virtual void plistCollectionEntriesChanged(plist::Collection& /*col*/) {
			for(component_iterator it = comps.begin(); it!=comps.end(); ++it)
				parent.setParent(**it);
		}
		const plist::ArrayOf<LinkComponent>& comps;
		KinematicJoint& parent;
	} componentsListener;
	
	virtual void dirtyBB() { bbDirty=true; }
};

//! handles the recursive loading of a tree of joints
class KinematicJointLoader : public plist::ArrayOf<KinematicJoint> {
public:
	//! constructor, start loading
	explicit KinematicJointLoader(KinematicJoint& root, xmlNode* node)
	: plist::ArrayOf<KinematicJoint>(), parent(&root)
	{
		addEntry(root);
		loadXML(node);
	}

protected:
	//! for each node, if it's an array, start loading the subtree via recursive instantiation, otherwise load the KinematicJoint
	virtual bool loadXMLNode(size_t index, xmlNode* val, const std::string& comment);
	
	//! the current joint for which sub-joints are being loaded
	KinematicJoint* parent;
	
private:
	KinematicJointLoader(const KinematicJointLoader&); //!< don't call
	KinematicJointLoader& operator=(const KinematicJointLoader&); //!< don't call
};

//! handles the recursive saving of a tree of joints
class KinematicJointSaver : public plist::Array {
public:
	//! given a single KinematicJoint, places it at the beginning of the array, followed by its children
	explicit KinematicJointSaver(const KinematicJoint& c, xmlNode* node=NULL) : plist::Array() {
		addEntry(const_cast<KinematicJoint&>(c));
		init(c.branches,node);
	}
	//! given a single KinematicJoint, places it at the beginning of the array, followed by its children
	explicit KinematicJointSaver(KinematicJoint& c, bool takeOwnership, xmlNode* node=NULL) : plist::Array() {
		if(takeOwnership)
			addEntry(&c);
		else
			addEntry(c);
		init(c.branches,node);
	}
	/* given a set of joints, adds the chain as elements of the array, so that they can be saved in order (by a second call from client)
	explicit KinematicJointSaver(const std::set<KinematicJoint*>& joints, xmlNode* node=NULL) : plist::Array() {
		init(joints,node);
	}*/
protected:
	//! saves the array of joints, prepending human-readable help text as a comment if this is a root XML node
	virtual void saveXML(xmlNode* node) const;
	//! adds the chain of joints as elements in the array, recursing on branching nodes to insert sub-arrays
	void init(const std::set<KinematicJoint*>& joints, xmlNode* node);
};

/*! @file
 * @brief Describes KinematicJoint, which manages parameters defining the position and type of motion produced by an actuator (i.e. forward kinematics)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
