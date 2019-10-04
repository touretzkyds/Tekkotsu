//-*-c++-*-
#ifndef INCLUDED_Client_h_
#define INCLUDED_Client_h_

#include "ImageBuffer.h"
#include "Physics.h"
#include "CameraInfo.h"

#include "IPC/Thread.h"
#include "IPC/DriverMessaging.h"
#include "Motion/KinematicJoint.h"
#include "Planners/Dynamics/PhysicsBody.h"
#include "Shared/MarkScope.h"
#include "Shared/TimeET.h"
#include "Shared/ReferenceCounter.h"

#include <map>

#ifdef __APPLE__
#  include <Ogre/OgreMesh.h>
#  include <Ogre/OgreVector3.h>
#  include <Ogre/OgreQuaternion.h>
#else
#  include <OGRE/OgreMesh.h>
#  include <OGRE/OgreVector3.h>
#  include <OGRE/OgreQuaternion.h>
#endif

class GraphicsInfo;
class JointGraphicsInfo;

//! Tracks state of each entity in the world
/*! Gets its updates by loading xml parse trees via setParseTree() and readParseTree() called from CommThread */
class Client : virtual public plist::Dictionary, virtual public ReferenceCounter, virtual protected plist::PrimitiveListener, virtual protected KinematicJoint::BranchListener, virtual protected plist::CollectionListener {
	friend class CameraInfo; // accesses kjInfos, could probably clean that up
public:
	class SensorListener {
	public:
		virtual ~SensorListener() {}
		virtual void sensorsUpdated(const DynamicRobotState& values)=0;
	};
	
	static Client& getClient(const std::string& clientID); //!< returns specified client or throws a std::runtime_error exception if it doesn't already exist
	static Client& getIncomingClient(const std::string& clientID); //!< client will be created if it doesn't exist, implies a followup call to activate()
	
	static void removeClient(const std::string& clientID);

	static void removeAllClients();
	
	static void renderCameras();
	
	static void updateGraphics();
	
	void addSensorListener(SensorListener* listener) { sensorListeners.insert(listener); }
	bool removeSensorListener(SensorListener* listener) { return sensorListeners.erase(listener)>0; }
	static void removeSensorListenerEverywhere(SensorListener* listener); //!< debugging function, searchs all clients for the listener, removes (with complaint) if found
		
	static size_t numClients() { return clients.size(); }
	
	static Ogre::MeshPtr loadMesh(const std::string& mesh);
	Ogre::Entity* createEntity(const std::string& identifier, const std::string& mesh) const;
	
	float getPosition(const std::string& name) const {
		plist::DictionaryOf<plist::Primitive<float> >::const_iterator it = positions.findEntry(name);
		return (it==positions.end()) ? 0.f : **it->second;
	}
	
	JointGraphicsInfo& getRootGraphics() const { return *kjInfos.find(const_cast<KinematicJoint*>(&model))->second; }
	
	//! Indicates initialization of external resources (e.g. model) is complete, create listeners to notify rest of system.
	/*! Deactivation is implied by destruction.  Safe to call multiple times. */
	void activate();
	
	//! returns true if the client has any sensor or camera listeners
	bool hasListeners();
	
	virtual bool removeReference();
	
protected:
	explicit Client(const std::string& clientID) : plist::Dictionary(), kjInfos(), childMap(), sensors(), sensorListeners(), physicsListener(NULL), lock(), 
		robotID(clientID), persist(false), location(), orientation(), positions(), physicsWheels(false), model(), cameras(), wheels(), wheelSpeeds(), wheelTime(),
		fixedPoints(), fixedContactPoints(), fixedVis(), fixedConstraints(), fixedInitialPosition(), fixedInitialOrientation(1,0,0,0)
	{ init(); }
	
	void init();
	
	virtual ~Client();
	
	virtual void loadXML(xmlNode* node) {
		MarkScope autolock(lock);
		plist::Dictionary::loadXML(node);
		//for_each(cameras.begin(), cameras.end(), std::mem_fun(&CameraInfo::setDirty));
		CameraInfo::setDirty();
	}
	
	//! overridden so we can tell when #fixedPoints is updated
	virtual bool loadXMLNode(const std::string& name, xmlNode* val, const std::string& comment);
	
	void locationUpdated(); //!< called from plistValue{Touched,Changed}, resets position to #location
	void orientationUpdated(); //!< called from plistValue{Touched,Changed}, resets orientation to #orientation
	
	// plist::PrimitiveListener, for location, and orientation
	virtual void plistValueTouched(const plist::PrimitiveBase& pl);
	// plist::PrimitiveListener, for RobotID, location, and orientation
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	// KinematicJoint::BranchListener, for model
	virtual void kinematicJointBranchAdded(KinematicJoint& parent, KinematicJoint& branch);
	virtual void kinematicJointBranchRemoved(KinematicJoint& parent, KinematicJoint& branch);
	virtual void kinematicJointReconfigured(KinematicJoint& joint);
	
	// plist::CollectionListener, for cameras
	virtual void plistCollectionEntryAdded(Collection& col, ObjectBase& entry);
	virtual void plistCollectionEntryRemoved(Collection& col, ObjectBase& entry);
	virtual void plistCollectionEntriesChanged(Collection& col);
	
	void updateGraphics(KinematicJoint& kj);
	void updateSensors();
	void recordLeafFrames(KinematicJoint& kj, bool pos);
	
	void updateKJTargets(KinematicJoint& kj);
	void updateKJPositions(KinematicJoint& kj);
	
	//! This performs "non-physics" based motion, either by leg or wheel (or both!)
	/*! This is to be used iff:
	 *   - there are fixed points (legged) and no mass
	 *   - OR there are wheels and not physicsWheels (or no mass, which implies non-physics)
	 *
	 *   Note if there are fixed points and there is mass, the physics constraints are used
	 *   so bullet does the body motion computation, not this function.
	 *
	 *   So fixed points are always applied, but wheels are not applied when there is mass and physicsWheels is set. */
	void updateContacts();
	
	void findBody(const KinematicJoint& src, PhysicsBody*& body, const KinematicJoint*& link); //!< find first parent of @a src which has a body, assign to @a body and @a link
	void calcFixedPoints(std::vector<fmat::Column<3> >& points);
	void filterPoints(std::vector<fmat::Column<3> >& points);
	static void fitTransform(const std::vector<fmat::Column<3> >& orig, const std::vector<fmat::Column<3> >& cur, Ogre::Vector3& posOff, Ogre::Quaternion& oriOff);
	
	static std::map<std::string,Client*> clients;
	
	std::map<KinematicJoint*, JointGraphicsInfo*> kjInfos;
	
	std::map<unsigned int, KinematicJoint*> childMap;
	
	typedef std::multimap<JointGraphicsInfo*, SensorInfo*> sensors_t;
	sensors_t sensors;
	std::set<SensorListener*> sensorListeners;
	
	PhysicsBody::ComponentListener* physicsListener;
	
	Thread::Lock lock;
	
	// these members (particularly 'model') must be listed *after* the protected members (particularly 'nodes') so that processing during destructor calls is valid
public:
	plist::Primitive<std::string> robotID;
	plist::Primitive<bool> persist;
	plist::Point location;
	plist::Point orientation;
	plist::DictionaryOf<plist::Primitive<float> > positions;
	plist::Primitive<bool> physicsWheels;
	DynamicRobotState sensorValues;
	
	KinematicJoint model;
	
	plist::ArrayOf<CameraInfo> cameras; // cameras must follow model
	std::set<KinematicJoint*> wheels;
	std::map<const KinematicJoint*,float> wheelSpeeds; //!< for non-phyiscs mode (#physicsWheels is false) updateContacts will use these values
	TimeET wheelTime;
	
	DriverMessaging::FixedPoints fixedPoints;
	std::vector<fmat::Column<3> > fixedContactPoints;
	std::vector<Ogre::SceneNode*> fixedVis;
	std::vector<Physics::PointConstraint*> fixedConstraints;
	Ogre::Vector3 fixedInitialPosition; //!< set to #model's world position at time of #fixedPoints update
	Ogre::Quaternion fixedInitialOrientation; //!< set to #model's world orientation at time of #fixedPoints update
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
