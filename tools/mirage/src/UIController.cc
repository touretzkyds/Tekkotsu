#include "UIController.h"

#include "EnvConfig.h"
#include "CommThread.h"
#include "Client.h"
#include "MirageFrameListener.h"
#include "Physics.h"
#include "GraphicsInfo.h"
#include "IPC/Thread.h"

#define TK_ENABLE_THREADING
#include "local/minisim.h"

#ifdef __APPLE__
#  include <Ogre/Ogre.h>
#else
#  include <OGRE/Ogre.h>
#endif

using namespace std; 
using namespace Ogre;

// Evil Globals
Root *ogreRoot=NULL;
RenderWindow *ogreWindow=NULL;
SceneManager *ogreSceneMgr=NULL;

UIController * ui=NULL;

// defined in main.mm
void get_mac_dirs(std::vector<std::string>& paths);


/***********************************************************
               Environmental Configuration Listeners
************************************************************/

//! Syncs a single value from one plist::Primitive to another
class ValueSync : virtual protected plist::PrimitiveListener {
public:
	//! constructor, do initial copy and subscribe for future changes
	ValueSync(const plist::PrimitiveBase& from, plist::PrimitiveBase& to) : plist::PrimitiveListener(), src(from), dst(to) {
		src.addPrimitiveListener(this);
		dst.set(src);
	}
	//! destructor, stop subscription
	~ValueSync() { src.removePrimitiveListener(this); }
	const plist::PrimitiveBase& src; //!< what we're listening to
	plist::PrimitiveBase& dst; //!< where we're sending updates
protected:
	virtual void plistValueChanged(const plist::PrimitiveBase& p) { dst.set(p); }
};


//! Syncs fields of one plist::Collection against corresponding fields a Client
class CollectionSync {
public:
	//! constructor: syncs against both the client @a to directly and Client::model
	CollectionSync(const plist::Collection& from, Client& to) : src(from), client(to), colsyncs(), valsyncs() {
		//std::cout << "Sync Client" << std::endl;
		addDestination(to);
		//std::cout << "Sync Client::model" << std::endl;
		addDestination(to.model);
	}
	//! destructor, remove listeners
	~CollectionSync() {
		for(std::vector<CollectionSync*>::const_iterator it=colsyncs.begin(); it!=colsyncs.end(); ++it)
			delete *it;
		colsyncs.clear();
		for(std::vector<ValueSync*>::const_iterator it=valsyncs.begin(); it!=valsyncs.end(); ++it)
			delete *it;
		valsyncs.clear();
	}
	const plist::Collection& src; //!< what we're listening to
	Client& client; //!< where we're sending updates
protected:
	//! constructor: for each entry in @a from, if there is a corresponding value in @a to, set up a listener to keep it in sync
	/*! this version only used for sub-collections */
	CollectionSync(const plist::Collection& from, plist::Collection& to, Client& cl) : src(from), client(cl), colsyncs(), valsyncs() {
		addDestination(to);
	}
	//! adds ValueSync or sub-CollectionSync instances for each entry in #src which matches an entry in @a dst
	void addDestination(plist::Collection& dst) {
		const plist::DictionaryBase * dsrc = dynamic_cast<const plist::DictionaryBase*>(&src);
		plist::DictionaryBase * ddst = dynamic_cast<plist::DictionaryBase*>(&dst);
		if(dsrc!=NULL && ddst!=NULL) {
			//std::cout << "It's a dictionary " << dsrc << ' ' << ddst << std::endl;
			for(plist::DictionaryBase::const_iterator sit=dsrc->begin(); sit!=dsrc->end(); ++sit) {
				//std::cout << "testing " << sit->first << std::endl;
				plist::DictionaryBase::const_iterator dit=ddst->findEntry(sit->first);
				if(dit!=ddst->end()) {
					const plist::PrimitiveBase* sprim = dynamic_cast<const plist::PrimitiveBase*>(sit->second);
					plist::PrimitiveBase* dprim = dynamic_cast<plist::PrimitiveBase*>(dit->second);
					//std::cout << "found " << sit->first << ' ' << sprim << ' ' << dprim << std::endl;
					if(sprim!=NULL && dprim!=NULL) {
						//std::cout << "is prim " << sit->first << std::endl;
						valsyncs.push_back(new ValueSync(*sprim,*dprim));
					} else if(sprim==NULL && dprim==NULL) {
						//std::cout << "is collection " << sit->first << std::endl;
						colsyncs.push_back(new CollectionSync(dynamic_cast<const plist::Collection&>(*sit->second),dynamic_cast<plist::Collection&>(*dit->second),client));
					}
				}
			}
		} else if(dsrc==NULL && ddst==NULL) {
			const plist::ArrayBase * asrc = dynamic_cast<const plist::ArrayBase*>(&src);
			plist::ArrayBase * adst = dynamic_cast<plist::ArrayBase*>(&dst);
			if(asrc==NULL && adst==NULL) {
				std::cerr << "WARNING: unknown pair of collection types " << typeid(src).name() << " and " << typeid(dst).name() << std::endl;
			} else {
				//std::cout << "It's an array " << asrc << ' ' << adst << std::endl;
				for(size_t i=0; i<asrc->size() && i<adst->size(); ++i) {
					const plist::PrimitiveBase* sprim = dynamic_cast<const plist::PrimitiveBase*>(&(*asrc)[i]);
					plist::PrimitiveBase* dprim = dynamic_cast<plist::PrimitiveBase*>(&(*adst)[i]);
					//std::cout << "entry " << i << ' ' << sprim << ' ' << dprim << std::endl;
					if(sprim!=NULL && dprim!=NULL) {
						//std::cout << "is prim " << i << std::endl;
						valsyncs.push_back(new ValueSync(*sprim,*dprim));
					} else if(sprim==NULL && dprim==NULL) {
						//std::cout << "is col " << i << std::endl;
						colsyncs.push_back(new CollectionSync(dynamic_cast<const plist::Collection&>((*asrc)[i]),dynamic_cast<plist::Collection&>((*adst)[i]),client));
					}
				}
				if(adst->getLoadPolicy()&plist::Collection::ADDITIONS) {
					for(size_t i=adst->size(); i<asrc->size(); ++i) {
						plist::ObjectBase* d = (*asrc)[i].clone();
						try {
							adst->forceEntry(d);
						} catch(const XMLLoadSave::bad_format& ex) {
							std::cerr << "ERROR: " << ex.what() << std::endl;
							delete d;
							continue;
						}
						const plist::PrimitiveBase* sprim = dynamic_cast<const plist::PrimitiveBase*>(&(*asrc)[i]);
						plist::PrimitiveBase* dprim = dynamic_cast<plist::PrimitiveBase*>(d);
						//std::cout << "entry " << i << ' ' << sprim << " new:" << dprim << std::endl;
						if(sprim!=NULL) {
							//std::cout << "is prim " << i << std::endl;
							valsyncs.push_back(new ValueSync(*sprim,*dprim));
						} else if(sprim==NULL) {
							//std::cout << "is col " << i << std::endl;
							colsyncs.push_back(new CollectionSync(dynamic_cast<const plist::Collection&>((*asrc)[i]),dynamic_cast<plist::Collection&>(*d),client));
						}
					}
				}
			}
		}
	}
	std::vector<CollectionSync*> colsyncs; //!< sychronizers for subcollections
	std::vector<ValueSync*> valsyncs; //!< synchronizers for value entries
};


//! Subscribes to Environment::objects to maintain client list
/*! Also notified by UIController if a client disconnects, in case one of the environment objects became a 'live' non-persistent client and then disconnects */
class EnvSceneryListener : protected virtual plist::CollectionListener {
public:
	//! constructor, do subscription
	EnvSceneryListener() : plist::CollectionListener(), objects(EnvConfig::singleton().objects), envToSync(), clientToSync() {
		EnvConfig::singleton().objects.addCollectionListener(this);
		plistCollectionEntriesChanged(objects);
	}
	//! destructor, unsubscribe
	virtual ~EnvSceneryListener() { EnvConfig::singleton().objects.removeCollectionListener(this); }
	//! a client has been removed, either by its own volition if claimed by a network connection, or our own remove call in response to plistCollectionEntryRemoved() below
	void clientDisconnecting(Client& c) {
		std::map<Client*,CollectionSync*>::iterator c2e = clientToSync.find(&c);
		if(c2e!=clientToSync.end()) {
			objects.removeCollectionListener(this);
			envToSync.erase(&c2e->second->src);
			clientToSync.erase(c2e);
			objects.removeEntry(c.robotID);
			objects.addCollectionListener(this);
		}
	}
	//! tests if a file is present
	bool testFile(const std::string& f) {
		struct stat buf;
		if(stat(f.c_str(),&buf)<0)
			return false;
		return buf.st_mode!=S_IFDIR;
	}
	//! an environmental object has been added, create a client
	virtual void plistCollectionEntryAdded(plist::Collection& /*col*/, plist::ObjectBase& e) {
		//stupid, have to search for name...
		plist::DictionaryOf<EnvConfig::PhysicalObject>::const_iterator it=objects.begin();
		for(; it!=objects.end(); ++it)
			if(it->second == &e)
				break;
		
		Client& client = Client::getIncomingClient(it->first);
		EnvConfig::PhysicalObject& oinfo = *it->second;
		clientToSync[&client] = envToSync[&e] = new CollectionSync(oinfo,client);
		
		client.persist = true;
		
		/*KinematicJoint * kj = new KinematicJoint;
		kj->setUnusedWarning(false); // ok to ignore extra fields like Location, Rotation, Kinematics, and PointAt
		//static_cast<LinkComponent&>(*kj) = oinfo;
		kj->set(oinfo);
		kj->setUnusedWarning(true);
		client.model.addBranch(kj);*/
		
		if(oinfo.kinematics.size()>0) {
			const std::string f = string_util::makePath(oinfo.sourceFile, oinfo.kinematics);
			if(testFile(f)) {
				client.model.loadFile(f.c_str());
			} else {
				client.model.loadFile(oinfo.kinematics.c_str());
			}
		}
		
		client.activate();
	}
	//! an environmental object has been removed, remove the client
	virtual void plistCollectionEntryRemoved(plist::Collection& /*col*/, plist::ObjectBase& e) {
		Client::removeClient(envToSync[&e]->client.robotID); // triggers clientDisconnected() which does cleanup
	}
	//! in case of mass change on first subscribing or shutdown
	virtual void plistCollectionEntriesChanged(plist::Collection& /*col*/) {
		std::map<const plist::ObjectBase*,CollectionSync*> removals = envToSync;
		std::vector<plist::ObjectBase*> additions;
		for(plist::DictionaryOf<EnvConfig::PhysicalObject>::const_iterator it=objects.begin(); it!=objects.end(); ++it)
			if(!removals.erase(it->second))
				additions.push_back(it->second);
		for(std::map<const plist::ObjectBase*,CollectionSync*>::const_iterator it=removals.begin(); it!=removals.end(); ++it)
			Client::removeClient(it->second->client.robotID); // triggers clientDisconnected() which does cleanup
		for(std::vector<plist::ObjectBase*>::const_iterator it=additions.begin(); it!=additions.end(); ++it)
			plistCollectionEntryAdded(objects,**it);
	}
protected:
	plist::DictionaryOf<EnvConfig::PhysicalObject>& objects; //!< our subscription target
	std::map<const plist::ObjectBase*,CollectionSync*> envToSync; //!< mapping for each environmental object to its CollectionSync in case the object is removed first
	std::map<Client*,CollectionSync*> clientToSync; //!< mapping for each client to its CollectionSync in case the client is removed first
};


//! Subscribes to Environment::camera to reset camera when environment is loaded
class EnvCameraListener {
public:
	EnvCameraListener(Ogre::Camera& ogreCamera)
		: envCam(EnvConfig::singleton().camera), ogCam(ogreCamera),
		locationListener(envCam.location,*this,&EnvCameraListener::syncLocation,true),
		rotationListener(envCam.orientation,*this,&EnvCameraListener::syncOrientation,false),
		pointAtListener(envCam.pointAt,*this,&EnvCameraListener::syncOrientation,true)
	{}
	void snapshot() {
		locationListener.deactivate();
		rotationListener.deactivate();
		pointAtListener.deactivate();
		for(unsigned int i=0; i<3; ++i)
			envCam.location[i] = ogCam.getPosition()[i];
		Ogre::Quaternion ori = ogCam.getOrientation();
		envCam.orientation[0] = ori.x;
		envCam.orientation[1] = ori.y;
		envCam.orientation[2] = ori.z;
		envCam.pointAt.clear();
		locationListener.activate(true);
		rotationListener.activate(true);
		pointAtListener.activate(true);
	}
	void syncLocation() {
		ogCam.setPosition(envCam.location[0], envCam.location[1], envCam.location[2]);
	}
	void syncOrientation() {
		Ogre::Vector3 qaxis(envCam.orientation[0],envCam.orientation[1],envCam.orientation[2]);
		float sx_2 = qaxis.length();
		if(sx_2>std::numeric_limits<float>::epsilon()) {
			float x = (sx_2>1) ? (float)M_PI : (2 * std::asin(sx_2));
			ogCam.setOrientation(Ogre::Quaternion(Ogre::Radian(x),qaxis/sx_2));
		}
		if(envCam.pointAt.size()>=3)
			ogCam.lookAt(envCam.pointAt[0], envCam.pointAt[1], envCam.pointAt[2]);
	}
	
protected:
	EnvConfig::Object& envCam;
	Ogre::Camera& ogCam;
	plist::CollectionCallbackMember<EnvCameraListener> locationListener, rotationListener, pointAtListener;
};


//! Subscribes to fields in Environment::background to reset the viewport background and/or scene manager
class EnvBackgroundListener {
public:
	EnvBackgroundListener(Viewport& view)
		: bg(EnvConfig::singleton().background), vp(view),
		modelListener(bg.model,*this,&EnvBackgroundListener::syncModel,false),
		materialListener(bg.material,*this,&EnvBackgroundListener::syncModel,false),
		curvatureListener(bg.curvature,*this,&EnvBackgroundListener::syncModel,false),
		tilingListener(bg.tiling,*this,&EnvBackgroundListener::syncModel,false),
		distanceListener(bg.distance,*this,&EnvBackgroundListener::syncModel,true),
		colorListener(bg.color,*this,&EnvBackgroundListener::syncBackgroundColor,true)
	{}
	void syncBackgroundColor() {
		vp.setBackgroundColour(ColourValue(bg.color.red,bg.color.green,bg.color.blue));
	}
	void syncModel() {
		Ogre::Plane loc(0, 0, -1, bg.distance);
		ogreSceneMgr->setSkyPlane(bg.model==EnvConfig::Background::PLANE, loc, bg.material, 750, bg.tiling, true, bg.curvature, 8, 8);
		ogreSceneMgr->setSkyBox(bg.model==EnvConfig::Background::BOX, bg.material, bg.distance,true,Quaternion(Degree(90),Vector3::UNIT_X));
		ogreSceneMgr->setSkyDome(bg.model==EnvConfig::Background::DOME, bg.material, bg.curvature, bg.tiling, bg.distance,true,Quaternion(Degree(90),Vector3::UNIT_X));
	}

protected:
	EnvConfig::Background& bg;
	Ogre::Viewport& vp;
	plist::PrimitiveCallbackMember<EnvBackgroundListener> modelListener, materialListener, curvatureListener, tilingListener, distanceListener;
	plist::CollectionCallbackMember<EnvBackgroundListener> colorListener;
};


//! Subscribes to fields in EnvConfig::Light to control an individual light's parameters
class EnvLightListener {
public:
	EnvLightListener(const std::string& name, EnvConfig::Light& l) : envLight(l), ogreLight(ogreSceneMgr->createLight(name)),
		locationListener(envLight.location,*this,&EnvLightListener::syncLocation,true),
		colorListener(envLight.color,*this,&EnvLightListener::syncColor,true),
		rangeListener(envLight.attenuateRange,*this,&EnvLightListener::syncAttenuation,false),
		constListener(envLight.attenuateConst,*this,&EnvLightListener::syncAttenuation,false),
		linearListener(envLight.attenuateLinear,*this,&EnvLightListener::syncAttenuation,false),
		quadListener(envLight.attenuateQuad,*this,&EnvLightListener::syncAttenuation,true)
	{}
	~EnvLightListener() {
		ogreSceneMgr->destroyLight(ogreLight);
	}
	void syncLocation() {
		ogreLight->setPosition(envLight.location[0],envLight.location[1],envLight.location[2]);
	}
	void syncColor() {
		ogreLight->setDiffuseColour(envLight.color.red,envLight.color.green,envLight.color.blue);
		ogreLight->setSpecularColour(envLight.color.red,envLight.color.green,envLight.color.blue);
	}
	void syncAttenuation() {
		ogreLight->setAttenuation(envLight.attenuateRange,envLight.attenuateConst,envLight.attenuateLinear,envLight.attenuateQuad);
	}

protected:
	EnvConfig::Light& envLight;
	Ogre::Light* ogreLight;
	plist::CollectionCallbackMember<EnvLightListener> locationListener;
	plist::CollectionCallbackMember<EnvLightListener> colorListener;
	plist::PrimitiveCallbackMember<EnvLightListener> rangeListener, constListener, linearListener, quadListener;
};


//! Subscribes to Environment::ambientLight and manages a list of EnvLightListeners synced to Environment::lights
class EnvLightingListener : virtual protected plist::CollectionListener {
public:
	EnvLightingListener() : env(EnvConfig::singleton()),
	ambientColorListener(env.ambientLight,*this,&EnvLightingListener::syncAmbientLight,true),
	shadowEnableListener(env.shadows.enabled,*this,&EnvLightingListener::syncShadows,false),
	shadowStencilListener(env.shadows.stencil,*this,&EnvLightingListener::syncShadows,false),
	shadowModulativeListener(env.shadows.modulative,*this,&EnvLightingListener::syncShadows,true),
	shadowColorListener(env.shadows.color,*this,&EnvLightingListener::syncShadowColor,true)
	{
		env.lights.addCollectionListener(this);
		plistCollectionEntriesChanged(env.lights);
	}
	~EnvLightingListener() {
		env.lights.removeCollectionListener(this);
		for(std::map<EnvConfig::Light*,EnvLightListener*>::const_iterator it=envListeners.begin(); it!=envListeners.end(); ++it)
			delete it->second;
		envListeners.clear();
	}
	void syncAmbientLight() {
		ogreSceneMgr->setAmbientLight(ColourValue(env.ambientLight.red, env.ambientLight.green, env.ambientLight.blue, env.ambientLight.alpha));
	}
	void syncShadows() {
		int shadowmode = SHADOWTYPE_NONE;
		if(env.shadows.enabled) {
			shadowmode |= (env.shadows.stencil ? SHADOWDETAILTYPE_STENCIL : SHADOWDETAILTYPE_TEXTURE);
			shadowmode |= (env.shadows.modulative ? SHADOWDETAILTYPE_MODULATIVE : SHADOWDETAILTYPE_ADDITIVE);
		}
		ogreSceneMgr->setShadowTechnique( static_cast<ShadowTechnique>(shadowmode) );
	}
	void syncShadowColor() {
		ogreSceneMgr->setShadowColour(ColourValue(env.shadows.color.red, env.shadows.color.green, env.shadows.color.blue));
	}
protected:
	EnvConfig::Environment& env;
	virtual void plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& entry) {
		//stupid, have to search for name...
		plist::DictionaryOf<EnvConfig::Light>::const_iterator it=env.lights.begin();
		for(; it!=env.lights.end(); ++it)
			if(it->second == &entry)
				break;
		envListeners[it->second] = new EnvLightListener(it->first,*it->second);
	}
	virtual void plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& entry) {
		EnvConfig::Light& l = dynamic_cast<EnvConfig::Light&>(entry);
		delete envListeners[&l];
		envListeners.erase(&l);
	}
	virtual void plistCollectionEntriesChanged(plist::Collection& col) {
		for(std::map<EnvConfig::Light*,EnvLightListener*>::const_iterator it=envListeners.begin(); it!=envListeners.end(); ++it)
			delete it->second;
		envListeners.clear();
		for(plist::DictionaryOf<EnvConfig::Light>::const_iterator it=env.lights.begin(); it!=env.lights.end(); ++it)
			envListeners[it->second] = new EnvLightListener(it->first,*it->second);
	}
	std::map<EnvConfig::Light*,EnvLightListener*> envListeners;
	plist::CollectionCallbackMember<EnvLightingListener> ambientColorListener;
	plist::PrimitiveCallbackMember<EnvLightingListener> shadowEnableListener, shadowStencilListener, shadowModulativeListener;
	plist::CollectionCallbackMember<EnvLightingListener> shadowColorListener;
};





/***********************************************************
				UIController
 ************************************************************/

UIController::UIController()
	: mCamera(NULL), followNode(NULL), followOffset(0,0,0), followOffsetTrans(), followTransFrames(-1u),
	followDist(1000), followRotation(1,0,0,0), followRotationTrans(1,0,0,0), followGraphicsInfo(NULL), followMode(FOLLOW_ORI),
	envScenerySync(NULL), envCameraSync(NULL), envBackgroundSync(NULL), envLightingSync(NULL),
	envFiles(), statTime(), physicsTime(0L), renderTime(0L), cameraTime(0L), idleTime(0L), gFrames(0), pFrames(0)
{}

void UIController::startup(const std::string& resourcePath, const std::string& configPath, const std::string& logPath) {
	minisim::initialize();
	
	// Create a new root object with the correct paths
	
#ifdef __APPLE__
	ogreRoot = new Root(resourcePath+"/plugins-osx.cfg", configPath+"/org.tekkotsu.mirage.ogre.cfg", logPath+"/Ogre.log");
#else
	ogreRoot = new Root(resourcePath+"/plugins.cfg", configPath+"/ogre.cfg", logPath+"/Ogre.log");
#endif
	
#if OGRE_VERSION_MAJOR==1 && OGRE_VERSION_MINOR<7
	if(ogreRoot->getAvailableRenderers()->size()==0) {
		exitWithMessage("Unable to set render system... missing plugins.cfg?");
		return;
	} else {
		ogreRoot->setRenderSystem(ogreRoot->getAvailableRenderers()->front());
	}
#else
	if(ogreRoot->getAvailableRenderers().size()==0) {
		exitWithMessage("Unable to set render system... missing plugins.cfg?");
		return;
	} else {
		ogreRoot->setRenderSystem(ogreRoot->getAvailableRenderers().front());
	}
#endif
	
#ifdef __APPLE__
	// Ogre doesn't seem to load the settings in the GUI it displays... barf
	// Just load without GUI unless user holds down option key at launch
	if( (GetCurrentKeyModifiers() & optionKey)==0 && ogreRoot->restoreConfig() ) {
		std::cout << "## Read OGRE config from " << configPath+"/org.tekkotsu.mirage.ogre.cfg, to reconfigure,\n"
		"## either edit this text file directly, or hold option key at launch to use GUI configuration editor.\n";
	} else // note fall-through to control showConfigDialog call below!
#endif
	if(!ogreRoot->showConfigDialog()) {
		exitWithMessage("");
		return;
	}
	
	// Initialise, we do not want an auto created window, as that will create a carbon window
	ogreWindow = ogreRoot->initialise(true);
	if(ogreWindow==NULL) {
		exitWithMessage("Unable to initalize OGRE window!");
		return;
	}
	
	// Add resource locations -- looking at folders recursively
	ResourceGroupManager::getSingleton().addResourceLocation(resourcePath, "FileSystem", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, false);
	ResourceGroupManager::getSingleton().addResourceLocation(resourcePath+"/media", "FileSystem", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	ResourceGroupManager::getSingleton().addResourceLocation("media", "FileSystem", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
#ifdef __APPLE__
	std::vector<std::string> sysdirs;
	get_mac_dirs(sysdirs);
	for(std::vector<std::string>::const_iterator it=sysdirs.begin(); it!=sysdirs.end(); ++it)
		ResourceGroupManager::getSingleton().addResourceLocation(*it, "FileSystem", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
#else
	const char* homedir = getenv("HOME");
	if(homedir!=NULL)
		ResourceGroupManager::getSingleton().addResourceLocation(std::string(homedir)+"/.mirage", "FileSystem", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
#endif
	
	ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	MeshManager::getSingleton().createPlane("Plane.mesh", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Plane(Vector3::UNIT_Z,0),1,1,5,5);
	MeshManager::getSingleton().createCurvedPlane("ParabolaUp.mesh", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Plane(Vector3::UNIT_Z,0),1,1,1,21,21);
	MeshManager::getSingleton().createCurvedPlane("ParabolaDown.mesh", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Plane(Vector3::UNIT_Z,0),1,1,-1,21,21);
	
	ogreSceneMgr = ogreRoot->createSceneManager(ST_GENERIC, "MySceneManager");
	//ogreSceneMgr->getRootSceneNode()->pitch(Ogre::Degree(-90)); // we want z "up", not "out"
	
	// Create the camera, node & attach camera
	mCamera = ogreSceneMgr->createCamera("Spectator");
	mCamera->setFixedYawAxis(true,Vector3::UNIT_Z);
	mCamera->setNearClipDistance(5);
	ogreSceneMgr->getRootSceneNode()->attachObject(mCamera);
	Viewport* vp = ogreWindow->addViewport(mCamera);
	
	// Alter the camera aspect ratio to match the viewport
	mCamera->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
	
	// Set default mipmap level (NB some APIs ignore this)
	TextureManager::getSingleton().setDefaultNumMipmaps(5);
	
	frameListener = new MirageFrameListener(ogreWindow, mCamera);
	frameListener->showDebugOverlay(true);
	ogreRoot->addFrameListener(frameListener);
	
	// Customizable configurations
	EnvConfig::Environment& env = EnvConfig::singleton();
	env.resetDefaults();
	env.resourceRoot = resourcePath;
	
	for(std::vector<std::string>::const_iterator it=envFiles.begin(); it!=envFiles.end(); ++it)
		loadFile(*it);
	envLightingSync = new EnvLightingListener; // Ogre docs say to set shadows before loading meshes (http://www.ogre3d.org/docs/manual/manual_70.html#SEC304)
	envCameraSync = new EnvCameraListener(*mCamera);
	envBackgroundSync = new EnvBackgroundListener(*vp);
	envScenerySync = new EnvSceneryListener;
	
	// Launch server to accept new clients
	serverThread = new ServerThread();
	serverThread->start();
}

void UIController::shutdown() {
	if(serverThread!=NULL && serverThread->isStarted())
		serverThread->stop().join();
	CommThread::shutdown();
	CommThread::processMessages();
	Client::removeAllClients();
	delete serverThread;
	serverThread = NULL;
	
	delete envScenerySync;
	envScenerySync=NULL;
	delete envCameraSync;
	envCameraSync=NULL;
	delete envBackgroundSync;
	envBackgroundSync=NULL;
	delete envLightingSync;
	envLightingSync=NULL;
	
	Physics::close();
	Ogre::Root::getSingleton().shutdown();
	minisim::destruct();
}

void UIController::resetEnvironment() {
	if(serverThread!=NULL && serverThread->isStarted())
		serverThread->stop().join();
	CommThread::shutdown();
	CommThread::processMessages();
	Client::removeAllClients();
	EnvConfig::singleton().resetScenery();
	envCameraSync->snapshot();
	for(std::vector<std::string>::const_iterator it=envFiles.begin(); it!=envFiles.end(); ++it)
		loadFile(*it);
	serverThread->start();
}

bool UIController::renderCallback() {
	static TimeET idle;
	idleTime += idle.Age();
	if(gFrames==0)
		resetStats();
	
	TimeET phys;
	pFrames += Physics::update();
	physicsTime += phys.Age();
	
	static unsigned int skip=-1U;
	if(CommThread::numConnections()>0 || skip++>2 || true) {
		Ogre::WindowEventUtilities::messagePump();
		
		TimeET render;
		if(!Ogre::Root::getSingleton().renderOneFrame()) {
			idle.Set();
			return false;
		}
		renderTime += render.Age();
		
		render.Set();
		Client::renderCameras();
		cameraTime += render.Age();
		
		skip=0;
	}
	CommThread::processMessages();
	
	++gFrames;
	idle.Set();
	return true;
}

class PlistTypeDetector : virtual protected XMLLoadSave, public std::string {
public:
	explicit PlistTypeDetector(const std::string& file) : XMLLoadSave(), std::string() { loadFile(file.c_str()); }
protected:
	virtual void loadXML(xmlNode* node) { assign((char*)node->name); }
	virtual void saveXML(xmlNode * node) const {}
	xmlNode* FindRootXMLElement(xmlDoc* doc) const {
		if(doc==NULL)
			return NULL;
		xmlNode* root=XMLLoadSave::FindRootXMLElement(doc);
		if(root == NULL)
			throw bad_format(root,"Error: plist read empty document");
		if(xmlStrcmp(root->name, (const xmlChar *)"plist"))
			throw bad_format(root,"Error: plist read document of the wrong type, root node != plist");
		// find first element node within the plist
		xmlNode* cur=root->children;
		while(cur!=NULL && cur->type!=XML_ELEMENT_NODE)
			cur=cur->next;
		if(cur==NULL) //empty plist
			cur = xmlNewChild(root,NULL,(const xmlChar*)"",NULL);
		return cur;
	}
};

bool UIController::loadFile(const std::string& path) {
	if(ogreRoot==NULL) {
		// still starting up, add to environment file list to be loaded later
		// (these define the base environment for resets)
		envFiles.push_back(path);
		return true;
	} else {
		// already running, load now
		
		// first check if path is actually a variable assignment
		if(path.find("=")!=std::string::npos)
			return EnvConfig::singleton().resolveAssignment(path);
		
		// ok, assume it's a file
		PlistTypeDetector pl(path);
		if(pl == "array") {
			// array implies a kinematics file
			EnvConfig::PhysicalObject * obj = new EnvConfig::PhysicalObject;
			obj->kinematics = path;
			EnvConfig::singleton().objects.setEntry(path, obj);
		} else if(pl == "dict") {
			// dictionary implies an environment file
			EnvConfig::singleton().loadFile(path.c_str());
		} else {
			std::cerr << "Unknown or unreadable file: " << path << std::endl;
			return false;
		}
		return true;
	}
}

void UIController::clientDisconnecting(Client& c) {
	envScenerySync->clientDisconnecting(c);
}

void UIController::cameraFollowMode(FollowMode mode, JointGraphicsInfo* hit, const Ogre::Vector3& pos) {
	followTransFrames=-1u; // cancel any transition in progress

	// first disconnect from previous tracking
	cameraStopFollow();
	Ogre::Matrix4 oldPos(mCamera->getOrientation());
	oldPos.setTrans(mCamera->getPosition());
	
	// now set up new tracking
	followGraphicsInfo = hit;
	if(hit!=NULL || mode==FOLLOW_NONE) {
		// disable follow
		EnvConfig::singleton().camera.autoFollow.clear(); 
		followMode=FOLLOW_NONE;
	} else {
		EnvConfig::singleton().camera.autoFollow = hit->getName();
		if(mode==FOLLOW_LOCK) {
			// attach to object's reference frame
			mCamera->getParentSceneNode()->detachObject(mCamera);
			hit->getJointNode().attachObject(mCamera);
			Ogre::Matrix4 newPos = mCamera->getParentSceneNode()->_getFullTransform().inverseAffine() * oldPos;
			mCamera->setPosition(newPos.getTrans());
			mCamera->setOrientation(newPos.extractQuaternion());
		} else {
			if(followMode==FOLLOW_LOCK) {
				// leaving lock mode, might have rotated non-upright
				// reset roll so up is up
				Ogre::Vector3 r = mCamera->getDerivedRight();
				Ogre::Vector3 u = mCamera->getDerivedUp();
				// solve for length along "up" from "right" to z=0 plane.
				float t = r.z / u.z;
				if(!std::isfinite(t))
				t=0;
				// t is length of far side of triangle, close is 1, atan2 to find angle
				//std::cout << r << ' ' << u << ' ' << t << ' ' << std::atan2(t,1) << std::endl;
				mCamera->roll(Ogre::Radian(-std::atan2(t,1)));
				if(mCamera->getDerivedUp().z<0)
				mCamera->roll(Ogre::Radian((Ogre::Real)M_PI));
			}
			if(mode==FOLLOW_ORI) {
				// follow the object, but don't change position
				Ogre::Matrix4 tr = hit->getJointNode()._getFullTransform();
				followOffset = tr.inverseAffine() * pos;
				followRotation = hit->getJointNode()._getDerivedOrientation().UnitInverse() * mCamera->getDerivedOrientation();
				followDist = (pos - oldPos.getTrans()).length();
				mCamera->setAutoTracking(true, &hit->getJointNode(), pos - tr.getTrans());
			} else if(mode==FOLLOW_POS) {
				// follow the object, but don't change orientation
				followNode = &hit->getJointNode();
				Ogre::Matrix4 itr = followNode->_getFullTransform().inverseAffine();
				followOffset = itr * pos;
				followRotation = itr.extractQuaternion() * oldPos.extractQuaternion();
				followDist = (pos - oldPos.getTrans()).length();
			} else {
				assert(false && "unknown mode setting");
			}
		}
		followMode=mode;
	}
}

void UIController::cameraStopFollow() {
	mCamera->setAutoTracking(false);
	followNode = NULL;
	// std::cout << "old parent " << mCamera->getParentSceneNode() << std::endl;
	Ogre::Matrix4 oldPos(mCamera->getOrientation());
	oldPos.setTrans(mCamera->getPosition());
	if(mCamera->getParentSceneNode() != ogreSceneMgr->getRootSceneNode()) {
		oldPos = mCamera->getParentSceneNode()->_getFullTransform() * oldPos;
		mCamera->getParentSceneNode()->detachObject(mCamera);
		ogreSceneMgr->getRootSceneNode()->attachObject(mCamera);
		mCamera->setPosition(oldPos.getTrans());
		mCamera->setOrientation(oldPos.extractQuaternion());
	}
}

void UIController::updateCamera() {
	if(followMode == FOLLOW_POS && followNode!=NULL) {
		Ogre::Matrix4 followT = followNode->_getFullTransform();
		/*if(mTranslateVector.x!=0 || mTranslateVector.y!=0 || mTranslateVector.z!=0) {
			Ogre::Matrix3 m3x3;
			followT.extract3x3Matrix(m3x3);
			followOffset += m3x3.Transpose() * (mCamera->getOrientation() * Ogre::Vector3(mTranslateVector.x,mTranslateVector.y,0));
			followDist += mTranslateVector.z;
		}*/
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
		
		//mCamera->moveRelative(mTranslateVector);
		
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

void UIController::dumpStats() {
	if(statTime.Age().Value()<1)
		return;
	int p = std::cout.precision();
	std::cout.precision(1);
	std::cout << fixed;
	std::cout << "Performance stats for last " << statTime.Age().Value() << " seconds:" << std::endl;
	std::cout << "  " << gFrames << " graphics frames processed: " << getGraphicsFPS() << " fps ("<< EnvConfig::singleton().fps << " requested)" << std::endl;
	std::cout << "  " << pFrames << " physics frames processed: " << getPhysicsFPS() << " fps (";
	if(Physics::singleton().physicsFPS > 0) {
		std::cout << Physics::singleton().physicsFPS << " requested)" << std::endl;
	} else {
		std::cout << EnvConfig::singleton().fps*Physics::singleton().stepsPerFrame << " requested)" << std::endl;
	}

	float physics = getPhysicsUsage() * 100;
	float render = getRenderUsage() * 100;
	float camera = getCameraUsage() * 100;
	float idle = getIdleUsage() * 100;
	float other = 100-(physics+render+camera+idle);
	std::cout << "  Time usage: " << physics << "% physics, " << render << "% on-screen rendering, "
		<< camera << "% camera rendering, " << other << "% other (" << idle << "% idle)" << std::endl;
	std::cout.unsetf(std::ios_base::floatfield);
	std::cout.precision(p);
	resetStats();
}

void UIController::resetStats() {
	physicsTime = renderTime = cameraTime = idleTime = 0L;
	gFrames = pFrames = 0;
	statTime.Set();
}

void UIController::findCameraRayHit(float x, float y, JointGraphicsInfo*& hit, Ogre::Vector3& pos) {
	Ray ray = mCamera->getCameraToViewportRay( x, y );
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
	if(directHit==NULL) {
		hit = NULL;
	} else {
		hit = dynamic_cast<JointGraphicsInfo*>(directHit);
		if(hit==NULL)
			hit=&directHit->getParent();
		while(hit!=NULL && &hit->getParent()!=hit && hit->getKJ().outputOffset!=plist::OutputSelector::UNUSED)
			hit = &hit->getParent();
		pos = ray * hitDist;
	}
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
