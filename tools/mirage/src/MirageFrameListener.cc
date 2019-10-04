#include "MirageFrameListener.h"
#include "GraphicsInfo.h"
#include "EnvConfig.h"
#include "UIController.h"
#include "Client.h"

#include "Motion/KinematicJoint.h"
#include "Shared/debuget.h"

//Use this define to signify OIS will be used as a DLL
//(so that dll import/export macros are in effect)
#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

MirageFrameListener * frameListener = NULL;

void MirageFrameListener::init(bool bufferedKeys, bool bufferedMouse, bool bufferedJoy) {
	using namespace Ogre;
	using namespace OIS;
	
	mDebugOverlay = OverlayManager::getSingleton().getByName("Core/DebugOverlay");
	
	LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	ParamList pl;
	
#if defined OIS_WIN32_PLATFORM
	pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )));
	pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
	pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
	pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
#elif defined OIS_LINUX_PLATFORM
	pl.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
	pl.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));
	pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
	pl.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
#endif
	
	size_t windowHnd = 0;
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	std::ostringstream windowHndStr;
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
	
	mInputManager = InputManager::createInputSystem( pl );
	
	//Create all devices (We only catch joystick exceptions here, as, most people have Key/Mouse)
	mKeyboard = static_cast<Keyboard*>(mInputManager->createInputObject( OISKeyboard, bufferedKeys ));
	mMouse = NULL; //static_cast<Mouse*>(mInputManager->createInputObject( OISMouse, bufferedMouse ));
	try {
		mJoy = static_cast<JoyStick*>(mInputManager->createInputObject( OISJoyStick, bufferedJoy ));
	}
	catch(...) {
		mJoy = 0;
	}
	
	//Set initial mouse clipping size
	windowResized(mWindow);
	
	showDebugOverlay(true);
	
	//Register as a Window listener
	WindowEventUtilities::addWindowEventListener(mWindow, this);
	mWindow->getViewport(0)->setVisibilityMask(GraphicsInfo::GRAPHICS_MODEL_MASK);
}

MirageFrameListener::~MirageFrameListener() {
	//Remove ourself as a Window listener
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
}

//Adjust mouse clipping area
void MirageFrameListener::windowResized(Ogre::RenderWindow* rw) {
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);
	
	if(mMouse) {
		const OIS::MouseState &ms = mMouse->getMouseState();
		ms.width = width;
		ms.height = height;
	}
}

//Unattach OIS before window shutdown (very important under Linux)
void MirageFrameListener::windowClosed(Ogre::RenderWindow* rw) {
	//Only close for window that created OIS (the main window in these demos)
	if( rw == mWindow )
	{
		if( mInputManager )
		{
			mInputManager->destroyInputObject( mMouse );
			mInputManager->destroyInputObject( mKeyboard );
			mInputManager->destroyInputObject( mJoy );
			
			OIS::InputManager::destroyInputSystem(mInputManager);
			mInputManager = 0;
		}
	}
}

bool MirageFrameListener::processUnbufferedKeyInput(const Ogre::FrameEvent& evt) {
	using namespace Ogre;
	using namespace OIS;
	
	Ogre::Vector3 newMoveSpeed(mMoveScale);
	Ogre::Real moveScaleAcc = moveAcc * evt.timeSinceLastFrame;
	
	if(mKeyboard->isKeyDown(KC_A))
		mTranslateVector.x = -moveSpeed.x;	// Move camera left
	
	if(mKeyboard->isKeyDown(KC_D))
		mTranslateVector.x = moveSpeed.x;	// Move camera RIGHT
	
	if(mKeyboard->isKeyDown(KC_A) ^ mKeyboard->isKeyDown(KC_D))
		newMoveSpeed.x = moveSpeed.x + moveScaleAcc;
	
	if(mKeyboard->isKeyDown(KC_W) )
		mTranslateVector.z = -moveSpeed.z;	// Move camera forward
	
	if(mKeyboard->isKeyDown(KC_S) )
		mTranslateVector.z = moveSpeed.z;	// Move camera backward
	
	if(mKeyboard->isKeyDown(KC_W) ^ mKeyboard->isKeyDown(KC_S))
		newMoveSpeed.z = moveSpeed.z + moveScaleAcc;
	
	if(mKeyboard->isKeyDown(KC_PGUP))
		mTranslateVector.y = moveSpeed.y;	// Move camera up
	
	if(mKeyboard->isKeyDown(KC_PGDOWN))
		mTranslateVector.y = -moveSpeed.y;	// Move camera down
	
	if(mKeyboard->isKeyDown(KC_PGUP) ^ mKeyboard->isKeyDown(KC_PGDOWN))
		newMoveSpeed.y = moveSpeed.y + moveScaleAcc;
	
	moveSpeed = newMoveSpeed;
	
	if(mKeyboard->isKeyDown(KC_UP))
		mCamera->pitch(mRotScale);
	
	if(mKeyboard->isKeyDown(KC_DOWN))
		mCamera->pitch(-mRotScale);
	
	if(mKeyboard->isKeyDown(KC_RIGHT))
		mCamera->yaw(-mRotScale);
	
	if(mKeyboard->isKeyDown(KC_LEFT))
		mCamera->yaw(mRotScale);
	
	if( mKeyboard->isKeyDown(KC_ESCAPE) || mKeyboard->isKeyDown(KC_Q) )
		return false;
	
	if( mKeyboard->isKeyDown(KC_T) )
		ui->dumpStats();
	
	if( mKeyboard->isKeyDown(KC_1) ) {
		EnvConfig::singleton().camera.autoFollow = "RobotID-1#BaseFrame";
		followMode = FOLLOW_ORI;
		followGraphicsInfo=NULL;
		try {
			Client& c = Client::getClient("RobotID-1");
			JointGraphicsInfo& hit = c.getRootGraphics();
			EnvConfig::singleton().camera.autoFollow = hit.getName();
			graphicsInfoCreated(hit); // pretend this was just created to trigger pan-to-track
		} catch(const std::runtime_error& err) {} // exception means client not availble
	}
			
	/*if(mKeyboard->isKeyDown(KC_F)) {
	 std::cout << "F ";
	 }
	 if(mKeyboard->isKeyDown(KC_LMENU)) {
	 std::cout << "MENU ";
	 }
	 if(mKeyboard->isKeyDown(KC_LCONTROL)) {
	 std::cout << "CTRL ";
	 }
	 if(mKeyboard->isKeyDown(KC_LCONTROL) || mKeyboard->isKeyDown(KC_LMENU)) {
	 std::cout << std::endl;
	 }*/
	
	if( mKeyboard->isKeyDown(KC_F) && mTimeUntilNextToggle <= 0 )
	{
		//mStatsOn = !mStatsOn;
		//showDebugOverlay(mStatsOn);
		
		//System::Double mMouseXCoord = (MouseXPos / (System::Double)(this->RenderPanel->Width));
		//System::Double mMouseYCoord = (MouseYPos / (System::Double)(this->RenderPanel->Height));
		
		Ray ray = mCamera->getCameraToViewportRay( 0.5f, 0.5f );
		RaySceneQuery * raySceneQuery = ogreSceneMgr->createRayQuery(ray);
		raySceneQuery->setSortByDistance( true );
		RaySceneQueryResult rayResults = raySceneQuery->execute();
		Ogre::Real hitDist;
		
		// this is only testing AABB - might not be able to hit an object encased in the empty corners of a diagonal object
		// upgrade: http://www.ogre3d.org/wiki/index.php/Raycasting_to_the_polygon_level
		GraphicsInfo * directHit=NULL;
		for(Ogre::RaySceneQueryResult::iterator itr = rayResults.begin(); itr != rayResults.end(); ++itr ) {
			if(itr->movable && !itr->movable->getUserAny().isEmpty())  {
				directHit = any_cast<GraphicsInfo*>(itr->movable->getUserAny());
				hitDist = itr->distance;
				break;
			}
		}
		delete raySceneQuery;
		
		JointGraphicsInfo * hit = NULL;
		lockCamera(NULL);
		mCamera->setAutoTracking(false);
		if(directHit==NULL) {
			followNode = NULL;
			followGraphicsInfo = NULL;
			EnvConfig::singleton().camera.autoFollow.clear();
			GraphicsInfo::unselect();
		} else {
			hit = dynamic_cast<JointGraphicsInfo*>(directHit);
			if(hit==NULL)
				hit=&directHit->getParent();
			while(hit!=NULL && &hit->getParent()!=hit && hit->getKJ().outputOffset!=plist::OutputSelector::UNUSED)
				hit = &hit->getParent();
			followGraphicsInfo = hit;
			EnvConfig::singleton().camera.autoFollow = hit->getName();
			
			if(!mKeyboard->isKeyDown(KC_LCONTROL) && !mKeyboard->isKeyDown(KC_LMENU)) {
				// just select the object
				if(hit!=NULL)
					hit->select();
				else
					GraphicsInfo::unselect();
				
			} else if(mKeyboard->isKeyDown(KC_LCONTROL) && mKeyboard->isKeyDown(KC_LMENU)) {
				if(hit!=NULL)
					followDist = 0;
				lockCamera(hit);
				
			} else if(mKeyboard->isKeyDown(KC_LCONTROL)) {
				// follow the object, but don't change orientation
				if(hit==NULL) {
					followNode = NULL;
					followMode = FOLLOW_NONE;
				} else {
					Ogre::Matrix4 tr = hit->getJointNode()._getFullTransform();
					followOffset = tr.inverseAffine() * (ray * hitDist);
					followDist = hitDist;
					followRotation = hit->getJointNode()._getDerivedOrientation().UnitInverse() * mCamera->getDerivedOrientation();
					followMode = FOLLOW_POS;
					followNode = &hit->getJointNode();
				}
				
			} else if(mKeyboard->isKeyDown(KC_LMENU)) {
				// follow the object, but don't change position
				followNode = NULL;
				if(hit==NULL) {
					mCamera->setAutoTracking(false);
					followMode = FOLLOW_NONE;
				} else {
					Ogre::Matrix4 tr = hit->getJointNode()._getFullTransform();
					followOffset = tr.inverseAffine() * (ray * hitDist);
					followDist = hitDist;
					followRotation = hit->getJointNode()._getDerivedOrientation().UnitInverse() * mCamera->getDerivedOrientation();
					followMode = FOLLOW_ORI;
					mCamera->setAutoTracking(true, &hit->getJointNode(), (ray * hitDist) - tr.getTrans());
				}
			}
		}
		
		mTimeUntilNextToggle = 1;
	}
	
	bool rotate_key = (mKeyboard->isKeyDown(KC_UP) || mKeyboard->isKeyDown(KC_DOWN) || mKeyboard->isKeyDown(KC_LEFT) || mKeyboard->isKeyDown(KC_RIGHT));
	if(mKeyboard->isKeyDown(KC_U) || (followMode==FOLLOW_ORI && rotate_key && mCamera->getAutoTrackTarget()!=NULL)) {
		lockCamera(NULL);
		mCamera->setAutoTracking(false);
		followNode = NULL;
		followGraphicsInfo = NULL;
		EnvConfig::singleton().camera.autoFollow.clear();
		GraphicsInfo::unselect();
	}
	
	if( mKeyboard->isKeyDown(KC_T) && mTimeUntilNextToggle <= 0 )
	{
		switch(mFiltering)
		{
			case TFO_BILINEAR:
				mFiltering = TFO_TRILINEAR;
				mAniso = 1;
				break;
			case TFO_TRILINEAR:
				mFiltering = TFO_ANISOTROPIC;
				mAniso = 8;
				break;
			case TFO_ANISOTROPIC:
				mFiltering = TFO_BILINEAR;
				mAniso = 1;
				break;
			default: break;
		}
		MaterialManager::getSingleton().setDefaultTextureFiltering(mFiltering);
		MaterialManager::getSingleton().setDefaultAnisotropy(mAniso);
		
		showDebugOverlay(mStatsOn);
		mTimeUntilNextToggle = 1;
	}
	
	if(mKeyboard->isKeyDown(KC_SYSRQ) && mTimeUntilNextToggle <= 0)
	{
		std::ostringstream ss;
		ss << "screenshot_" << ++mNumScreenShots << ".png";
		mWindow->writeContentsToFile(ss.str());
		mTimeUntilNextToggle = 0.5f;
		mDebugText = "Saved: " + ss.str();
	}
	
	if(mKeyboard->isKeyDown(KC_R) && mTimeUntilNextToggle <=0)
	{
		/*mSceneDetailIndex = (mSceneDetailIndex+1)%3 ;
		switch(mSceneDetailIndex) {
			case 0 : mCamera->setPolygonMode(PM_SOLID); break;
			case 1 : mCamera->setPolygonMode(PM_WIREFRAME); break;
			case 2 : mCamera->setPolygonMode(PM_POINTS); break;
		}*/
		ui->resetEnvironment();
		mTimeUntilNextToggle = 0.5f;
	}
	
	if(mKeyboard->isKeyDown(KC_C)) {
		SceneNode * camnode=NULL;
		try {
			camnode = ogreSceneMgr->getEntity("RobotID-1-CameraFrame")->getParentSceneNode();
		} catch(const Ogre::Exception&) {}
		if(camnode==NULL) {
			std::cerr << "No camera frame found" << std::endl;
		} else {
			mCamera->setPosition(camnode->_getDerivedPosition());
			mCamera->setOrientation(camnode->_getDerivedOrientation());
			mCamera->pitch(Ogre::Radian((Ogre::Real)M_PI));
		}
	}
	
	if(mKeyboard->isKeyDown(KC_B) && mTimeUntilNextToggle <=0) {
		int x = mWindow->getViewport(0)->getVisibilityMask();
		mWindow->getViewport(0)->setVisibilityMask( x^GraphicsInfo::BOUNDING_BOX_MASK );
		mTimeUntilNextToggle = 0.5f;
	}
	
	if(mKeyboard->isKeyDown(KC_H) && mTimeUntilNextToggle <=0) {
		int x = mWindow->getViewport(0)->getVisibilityMask();
		mWindow->getViewport(0)->setVisibilityMask( x^(GraphicsInfo::GRAPHICS_MODEL_MASK | GraphicsInfo::COLLISION_MODEL_MASK) );
		mTimeUntilNextToggle = 0.5f;
	}
	
	static bool displayCameraDetails = false;
	if(mKeyboard->isKeyDown(KC_P) && mTimeUntilNextToggle <= 0)
	{
		displayCameraDetails = !displayCameraDetails;
		mTimeUntilNextToggle = 0.5f;
		if (!displayCameraDetails)
			mDebugText = "";
	}
	
	// Print camera details
	if(displayCameraDetails)
		mDebugText = "P: " + StringConverter::toString(mCamera->getDerivedPosition()) +
		" " + "O: " + StringConverter::toString(mCamera->getDerivedOrientation());
	
	// Return true to continue rendering
	return true;
}

void MirageFrameListener::lockCamera(GraphicsInfo* hit) {
	mCamera->setAutoTracking(false);
	followNode = NULL;
	followGraphicsInfo = hit;
	
	// lock to the object (both position and orientation)
	std::cout << "old parent " << mCamera->getParentSceneNode() << std::endl;
	Ogre::Matrix4 oldPos(mCamera->getOrientation());
	oldPos.setTrans(mCamera->getPosition());
	Ogre::Matrix4 oldTr = Ogre::Matrix4::IDENTITY;
	if(mCamera->getParentSceneNode()!=NULL) {
		oldTr = mCamera->getParentSceneNode()->_getFullTransform();
		mCamera->getParentSceneNode()->detachObject(mCamera);
		//Ogre::Vector3 pos = camnode->_getDerivedPosition();
		//Ogre::Quaternion ori = camnode->_getDerivedOrientation();
	}
	if(hit==NULL) {
		ogreSceneMgr->getRootSceneNode()->attachObject(mCamera);
		followMode = FOLLOW_NONE;
	} else {
		hit->getJointNode().attachObject(mCamera);
		followMode = FOLLOW_LOCK;
	}
	Ogre::Matrix4 newPos = mCamera->getParentSceneNode()->_getFullTransform().inverseAffine() * oldTr * oldPos;
	mCamera->setPosition(newPos.getTrans());
	mCamera->setOrientation(newPos.extractQuaternion());
	
	// reset roll so up is up
	Ogre::Vector3 r = mCamera->getDerivedRight();
	Ogre::Vector3 u = mCamera->getDerivedUp();
	// solve for length along "up" from "right" to z=0 plane.
	float t = r.z / u.z;
	if(!std::isfinite(t))
		t=0;
	// t is length of far side of triangle, close is 1, atan2 to find angle
	std::cout << r << ' ' << u << ' ' << t << ' ' << std::atan2(t,1) << std::endl;
	mCamera->roll(Ogre::Radian(-std::atan2(t,1)));
	if(mCamera->getDerivedUp().z<0)
		mCamera->roll(Ogre::Radian((Ogre::Real)M_PI));
}

bool MirageFrameListener::processUnbufferedMouseInput(const Ogre::FrameEvent& evt) {
	using namespace OIS;
	
	// Rotation factors, may not be used if the second mouse button is pressed
	// 2nd mouse button - slide, otherwise rotate
	const MouseState &ms = mMouse->getMouseState();
	if( ms.buttonDown( MB_Right ) )
	{
		mTranslateVector.x += ms.X.rel * 0.13f;
		mTranslateVector.y -= ms.Y.rel * 0.13f;
	}
	else
	{
		mRotX = Ogre::Degree(-ms.X.rel * 0.13f);
		mRotY = Ogre::Degree(-ms.Y.rel * 0.13f);
	}
	
	return true;
}

void MirageFrameListener::moveCamera() {
	// Make all the changes to the camera
	// Note that YAW direction is around a fixed axis (freelook style) rather than a natural YAW
	//(e.g. airplane)
	if(mRotX.valueRadians()!=0 || mRotY.valueRadians()!=0) {
		if(mCamera->getAutoTrackTarget()!=NULL) {
			// move target offset?
		} else {
			mCamera->yaw(mRotX);
			mCamera->pitch(mRotY);
		}
	}
	
	if(followMode == FOLLOW_POS && followNode!=NULL) {
		Ogre::Matrix4 followT = followNode->_getFullTransform();
		if(mTranslateVector.x!=0 || mTranslateVector.y!=0 || mTranslateVector.z!=0) {
			Ogre::Matrix3 m3x3;
			followT.extract3x3Matrix(m3x3);
			followOffset += m3x3.Transpose() * (mCamera->getOrientation() * Ogre::Vector3(mTranslateVector.x,mTranslateVector.y,0));
			followDist += mTranslateVector.z;
		}
		Ogre::Ray ray = mCamera->getCameraToViewportRay( 0.5f, 0.5f );
		Ogre::Vector3 camTgt = ray * followDist;
		Ogre::Vector3 objTgt = followT * followOffset;
		if(followTransFrames!=-1u) {
			Ogre::Real t = followTransFrames / EnvConfig::singleton().fps;
			t = (1 - std::cos(t*(Ogre::Real)M_PI))/2; // smooth in & out
			objTgt += followT * (followOffsetTrans * t);
		}
		mCamera->move(objTgt-camTgt);
		
	} else {
		
		mCamera->moveRelative(mTranslateVector);
		
		if(followTransFrames!=-1u && followGraphicsInfo!=NULL) {
			if(followMode == FOLLOW_ORI) {
				Ogre::Real t = followTransFrames / EnvConfig::singleton().fps;
				t = (1 - std::cos(t*(Ogre::Real)M_PI))/2; // smooth in & out
				Ogre::Matrix4 followT = followGraphicsInfo->getJointNode()._getFullTransform();
				mCamera->setAutoTracking(true, &followGraphicsInfo->getJointNode(), followT * (followOffset + followOffsetTrans * t) - followT.getTrans());
			} else if(followMode == FOLLOW_LOCK) {
				/*Ogre::Real t = (followTransFrames+1) / EnvConfig::singleton().fps;
				if(t>1)
					t=1;
				t = (1 - std::cos(t*(Ogre::Real)M_PI))/2; // smooth in & out
				Ogre::Vector3 oldOff = followOffsetTrans * t;
				t = followTransFrames / EnvConfig::singleton().fps;
				t = (1 - std::cos(t*(Ogre::Real)M_PI))/2; // smooth in & out
				Ogre::Vector3 newOff = followOffsetTrans * t;
				mCamera->move(newOff-oldOff);*/
				
				Ogre::Real t = followTransFrames / EnvConfig::singleton().fps;
				t = (1 - std::cos(t*(Ogre::Real)M_PI))/2; // smooth in & out
				
				mCamera->setPosition(followOffset + followOffsetTrans * t);
				
				Ogre::Radian angle;
				Ogre::Vector3 axis;
				followRotationTrans.ToAngleAxis(angle,axis);
				mCamera->setOrientation(followRotation * Ogre::Quaternion(angle * t,axis));
				
			}
		}
	}
	
	if(followTransFrames!=-1u)
		--followTransFrames;
}

void MirageFrameListener::graphicsInfoCreated(GraphicsInfo& ginfo) {
	if(followGraphicsInfo==NULL && followMode!=FOLLOW_NONE) {
		if(JointGraphicsInfo * jginfo = dynamic_cast<JointGraphicsInfo*>(&ginfo)) {
			if(EnvConfig::singleton().camera.autoFollow == jginfo->getName()) {
				followGraphicsInfo = jginfo;
				followTransFrames = EnvConfig::singleton().fps;
				
				Ogre::Matrix4 followT = jginfo->getJointNode()._getFullTransform();
				Ogre::Ray ray = mCamera->getCameraToViewportRay( 0.5f, 0.5f );
				Ogre::Vector3 camTgt = followT.inverseAffine() * (ray * followDist);
				followOffsetTrans = camTgt - followOffset;
				
				Ogre::Quaternion curRot = jginfo->getJointNode()._getDerivedOrientation().UnitInverse() * mCamera->getDerivedOrientation();
				std::cout << "Refollow rotation " << curRot << " vs " << followRotation << std::endl;
				followRotationTrans = followRotation.UnitInverse() * curRot;
				std::cout << "Trans rotation " << followRotationTrans << std::endl;
				
				switch(followMode) {
					case FOLLOW_NONE:
						break; // should not happen
					case FOLLOW_POS:
						followNode = &followGraphicsInfo->getJointNode();
						break;
					case FOLLOW_ORI:
						mCamera->setAutoTracking(true, &followGraphicsInfo->getJointNode(), followT * (followOffset + followOffsetTrans) - followT.getTrans());
						break;
					case FOLLOW_LOCK:
						lockCamera(followGraphicsInfo);
						mTranslateVector=0;
						moveCamera();
						break;
				}
			}
		}
	}
}

void MirageFrameListener::graphicsInfoDestructed(GraphicsInfo& ginfo) {
	if(&ginfo == followGraphicsInfo) {
		if(followMode == FOLLOW_LOCK) {
			followOffset = mCamera->getPosition();
			followRotation = mCamera->getOrientation();
			std::cout << "Ending rotation " << followRotation << std::endl;
		}

		FollowMode fm = followMode; // followMode will be overridden
		lockCamera(NULL); // by call to lockCamera, but only want to reset to world
		followMode = fm; // reset the mode parameter so we can reconnect if client comes back
	}
}

void MirageFrameListener::showDebugOverlay(bool show) {
	if (mDebugOverlay)
	{
		if (show)
			mDebugOverlay->show();
		else
			mDebugOverlay->hide();
	}
}

// Override frameStarted event to process that (don't care about frameEnded)
bool MirageFrameListener::frameStarted(const Ogre::FrameEvent& evt) {
	using namespace OIS;
	
	if(mWindow->isClosed())	return false;
	
	//Need to capture/update each device
	mKeyboard->capture();
	if( mMouse ) mMouse->capture();
	if( mJoy ) mJoy->capture();
	
	bool buffJ = (mJoy) ? mJoy->buffered() : true;
	
	//Check if one of the devices is not buffered
	if( (mMouse && !mMouse->buffered()) || !mKeyboard->buffered() || !buffJ )
	{
		// one of the input modes is immediate, so setup what is needed for immediate movement
		if (mTimeUntilNextToggle >= 0)
			mTimeUntilNextToggle -= evt.timeSinceLastFrame;
		
		// Move about 100 units per second
		mMoveScale = baseMoveSpeed * evt.timeSinceLastFrame;
		// Take about 10 seconds for full rotation
		mRotScale = mRotateSpeed * evt.timeSinceLastFrame;
		
		mRotX = 0;
		mRotY = 0;
		mTranslateVector = Ogre::Vector3::ZERO;
	}
	
	//Check to see which device is not buffered, and handle it
	if( !mKeyboard->buffered() )
		if( processUnbufferedKeyInput(evt) == false )
			return false;
	if( mMouse && !mMouse->buffered() )
		if( processUnbufferedMouseInput(evt) == false )
			return false;
	
	if( (mMouse && !mMouse->buffered()) || !mKeyboard->buffered() || !buffJ )
		moveCamera();
	
	return true;
}

bool MirageFrameListener::frameEnded(const Ogre::FrameEvent& evt) {
	return true;
}
