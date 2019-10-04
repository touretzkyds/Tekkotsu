//-*-c++-*-
#ifndef INCLUDED_MirageFrameListener_h
#define INCLUDED_MirageFrameListener_h

#ifdef __APPLE__
#  include <Carbon/Carbon.h>
#  include <Ogre/Ogre.h>
#else
#  include <OGRE/Ogre.h>
#endif

extern Ogre::SceneManager *ogreSceneMgr;

namespace OIS {
	class InputManager;
	class Mouse;
	class Keyboard;
	class JoyStick;
}

class GraphicsInfo;

class MirageFrameListener;
extern MirageFrameListener * frameListener;

class MirageFrameListener: public Ogre::FrameListener, public Ogre::WindowEventListener
{
public:
	// Constructor takes a RenderWindow because it uses that to determine input context
	MirageFrameListener(Ogre::RenderWindow* win, Ogre::Camera* cam, bool bufferedKeys = false, bool bufferedMouse = false, bool bufferedJoy = false ) :
		mCamera(cam), followNode(NULL), followOffset(0,0,0), followOffsetTrans(), followTransFrames(-1u),
		followDist(1000), followRotation(1,0,0,0), followRotationTrans(1,0,0,0), followGraphicsInfo(NULL), followMode(FOLLOW_ORI),
		mTranslateVector(Ogre::Vector3::ZERO), mWindow(win), mStatsOn(true), mNumScreenShots(0),
		mMoveScale(0.0f), mRotScale(0.0f), mTimeUntilNextToggle(0), mFiltering(Ogre::TFO_BILINEAR),
		mAniso(1), mSceneDetailIndex(0), baseMoveSpeed(400), moveSpeed(Ogre::Vector3::ZERO), moveAcc(50), 
		mRotateSpeed(45), mDebugOverlay(0),
		mInputManager(0), mMouse(0), mKeyboard(0), mJoy(0)
	{
		init(bufferedKeys,bufferedMouse,bufferedJoy);
	}
	
	void init(bool bufferedKeys, bool bufferedMouse, bool bufferedJoy);

	virtual ~MirageFrameListener();
	
	//Adjust mouse clipping area
	virtual void windowResized(Ogre::RenderWindow* rw);

	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(Ogre::RenderWindow* rw);
	
	virtual bool processUnbufferedKeyInput(const Ogre::FrameEvent& evt);

	bool processUnbufferedMouseInput(const Ogre::FrameEvent& evt);

	void moveCamera();

	void graphicsInfoCreated(GraphicsInfo& ginfo); //!< called after a new client is created and initialized
	void graphicsInfoDestructed(GraphicsInfo& ginfo); //!< called when a client is about to be destroyed, check #followClient
	
	void showDebugOverlay(bool show);

	// Override frameStarted event to process that (don't care about frameEnded)
	bool frameStarted(const Ogre::FrameEvent& evt);

	bool frameEnded(const Ogre::FrameEvent& evt);
	
protected:
	void lockCamera(GraphicsInfo* hit);
	void trackObject(GraphicsInfo* hit);

	Ogre::Camera* mCamera;
	Ogre::SceneNode* followNode; //!< the Ogre node which is being targeted for tracking when using FOLLOW_POS mode
	Ogre::Vector3 followOffset; //!< the offset from followNode origin which is being targeted in FOLLOW_POS mode
	Ogre::Vector3 followOffsetTrans; //!< when a client reconnects, this records the offset from the target offset to the current camera position so we can transition smoothly
	unsigned int followTransFrames; //!< set to a frame count remaining in the transition (0 means transition is complete)
	Ogre::Real followDist; //!< the distance along camera ray to #followOffset point in FOLLOW_POS mode
	Ogre::Quaternion followRotation; //!< the orientation of the camera when entering FOLLOW_LOCK mode (so that it can be reset on client reconnect);
	Ogre::Quaternion followRotationTrans; //!< when a client reconnects, this records the offset from the target offset to the current camera position so we can transition smoothly
	GraphicsInfo* followGraphicsInfo; //!< the GraphicsInfo being tracked; if we get a graphicsInfoDestructed call for this structure, need to tear down tracking.
	enum FollowMode {
		FOLLOW_NONE, //!< no tracking
		FOLLOW_POS, //!< move camera position to track object, but don't change its orientation
		FOLLOW_ORI, //!< rotate camera to look at object, but doesn't change camera position
		FOLLOW_LOCK //!< adds camera as a child of the target object, so it moves exactly with the target
	} followMode;

	Ogre::Vector3 mTranslateVector;
	Ogre::RenderWindow* mWindow;
	bool mStatsOn;

	std::string mDebugText;

	unsigned int mNumScreenShots;
	float mMoveScale;
	Ogre::Degree mRotScale;
	// just to stop toggles flipping too fast
	Ogre::Real mTimeUntilNextToggle ;
	Ogre::Radian mRotX, mRotY;
	Ogre::TextureFilterOptions mFiltering;
	int mAniso;

	int mSceneDetailIndex ;
	Ogre::Real baseMoveSpeed;
	Ogre::Vector3 moveSpeed;
	Ogre::Real moveAcc;
	Ogre::Degree mRotateSpeed;
	Ogre::Overlay* mDebugOverlay;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoy;
};

#endif
