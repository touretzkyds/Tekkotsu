//-*-c++-*-
#ifndef INCLUDED_UIController_h_
#define INCLUDED_UIController_h_

#include "Shared/plist.h"
#include "Shared/TimeET.h"
#ifdef __APPLE__
#  include <Ogre/OgreVector3.h>
#  include <Ogre/OgreQuaternion.h>
#else
#  include <OGRE/OgreVector3.h>
#  include <OGRE/OgreQuaternion.h>
#endif

// Evil Globals configured by UIController::startup()
namespace Ogre {
	class Root;
	class RenderWindow;
	class SceneManager;
	class SceneNode;
	class Camera;
}
extern Ogre::Root *ogreRoot;
extern Ogre::RenderWindow *ogreWindow;
extern Ogre::SceneManager *ogreSceneMgr;
class JointGraphicsInfo;

class UIController {
public:
	UIController();
	virtual ~UIController() {}
	
	//! performs initialization, to be called once by platform-specific code
	void startup(const std::string& resourcePath, const std::string& configPath, const std::string& logPath);
	
	//! cleans up before exit, to be called by platform-specific code
	void shutdown();
	
	//! clears client list and rebuilds environment from #envFiles
	void resetEnvironment();
	
	//! produce error message (implementation is platform-specific) and then exit (never returns!)
	void exitWithMessage(const std::string& msg);
	
	//! platform init code should poll this at the rendering frame rate
	bool renderCallback();
	
	//! passed by command line argument, or on mac: double-click on file(s), or drag-and-drop on application...
	bool loadFile(const std::string& path);
	
	//! called when a Client is created
	void clientConnected(class Client&) {}

	//! called when a Client disconnects
	void clientDisconnecting(class Client& c);
	
	//! various choices for causing spectator camera to automatically track an object
	enum FollowMode {
		FOLLOW_NONE, //!< no tracking
		FOLLOW_POS, //!< move camera position to track object, but don't change its orientation
		FOLLOW_ORI, //!< rotate camera to look at object, but doesn't change camera position
		FOLLOW_LOCK //!< adds camera as a child of the target object, so it moves exactly with the target
	};
	//! track object at center of camera
	void cameraFollowMode(FollowMode mode) { JointGraphicsInfo* hit=NULL; Ogre::Vector3 pos; findCameraRayHit(0.5f,0.5f,hit,pos); cameraFollowMode(mode, hit, pos); }
	//! track object at specified position
	void cameraFollowMode(FollowMode mode, float x, float y) { JointGraphicsInfo* hit=NULL; Ogre::Vector3 pos; findCameraRayHit(x,y,hit,pos); cameraFollowMode(mode, hit, pos); }
	//! track specified object
	void cameraFollowMode(FollowMode mode, JointGraphicsInfo* hit, const Ogre::Vector3& pos);
	
	//! Interrupts camera following, but does not reset it (use cameraFollowMode(FOLLOW_NONE) for that)
	/*! This is to be called when the object being tracked is invalidated, but allows for tracking to resume if the object comes back */
	void cameraStopFollow();
	
	//! to be called prior to rendering each frame to adjust camera position for tracking objects
	void updateCamera();
	
	void dumpStats();
	void resetStats();
	float getGraphicsFPS() const { return static_cast<float>(gFrames / statTime.Age().Value()); }
	float getPhysicsFPS() const { return static_cast<float>(pFrames / statTime.Age().Value()); }
	float getPhysicsUsage() const { return static_cast<float>(physicsTime.Value() / statTime.Age().Value()); }
	float getRenderUsage() const { return static_cast<float>(renderTime.Value() / statTime.Age().Value()); }
	float getCameraUsage() const { return static_cast<float>(cameraTime.Value() / statTime.Age().Value()); }
	float getIdleUsage() const { return static_cast<float>(idleTime.Value() / statTime.Age().Value()); }

protected:
	void findCameraRayHit(float x, float y, JointGraphicsInfo*& hit, Ogre::Vector3& pos);
	
	Ogre::Camera* mCamera;
	Ogre::SceneNode* followNode; //!< the Ogre node which is being targeted for tracking when using FOLLOW_POS mode
	Ogre::Vector3 followOffset; //!< the offset from followNode origin which is being targeted in FOLLOW_POS mode
	Ogre::Vector3 followOffsetTrans; //!< when a client reconnects, this records the offset from the target offset to the current camera position so we can transition smoothly
	unsigned int followTransFrames; //!< set to a frame count remaining in the transition (-1u means transition is complete)
	Ogre::Real followDist; //!< the distance along camera ray to #followOffset point in FOLLOW_POS mode
	Ogre::Quaternion followRotation; //!< the orientation of the camera when entering FOLLOW_LOCK mode (so that it can be reset on client reconnect);
	Ogre::Quaternion followRotationTrans; //!< when a client reconnects, this records the offset from the target offset to the current camera position so we can transition smoothly
	JointGraphicsInfo* followGraphicsInfo; //!< the GraphicsInfo being tracked; if we get a graphicsInfoDestructed call for this structure, need to tear down tracking.
	FollowMode followMode;
	
	class EnvSceneryListener* envScenerySync; //!< listens to Environment::objects and generates EnvObjectListeners to sync against client list
	class EnvCameraListener* envCameraSync; //!< listens to Environment::camera and syncs against camera parameters
	class EnvBackgroundListener* envBackgroundSync; //!< listens to Environment::background and syncs against the viewport and scene manager
	class EnvLightingListener* envLightingSync; //!< listens to Environment::lights, Environment::ambientLight, and shadow parameters
	
	//! list of files to be opened to populate (or reset) the environment
	std::vector<std::string> envFiles;

	TimeET statTime; //!< time of starting to gather statistics
	TimeET physicsTime; //!< cumulative time spent in physics
	TimeET renderTime; //!< cumulative time spent doing on-screen graphics rendering
	TimeET cameraTime; //!< cumulative time spent doing off-screen rendering for robot cameras
	TimeET idleTime; //!< time spent between renderCallback calls
	size_t gFrames; //!< number of graphics frames processed since #startTime
	size_t pFrames; //!< number of physics frames processed since #startTime
};
extern UIController * ui;

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
