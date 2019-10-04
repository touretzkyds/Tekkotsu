#include "CameraInfo.h"
#include "Client.h"
#include "GraphicsInfo.h"

#include "Motion/KinematicJoint.h"

#ifdef __APPLE__
#  include <Carbon/Carbon.h>
#  include <Ogre/Ogre.h>
#else
#  include <OGRE/Ogre.h>
#endif

extern Ogre::SceneManager *ogreSceneMgr;

using namespace std;
using namespace Ogre;

class MaterialSwitcher : public MaterialManager::Listener {
private: 
	MaterialPtr depthMat;
	Technique* technique;

public: 
	MaterialSwitcher() {
		depthMat = MaterialManager::getSingleton().getByName("DepthMaterial");
		if (!depthMat.isNull()) {
			depthMat->load();
			technique = depthMat->getBestTechnique();
		}
		// Otherwise try to reload the material later
	}

	virtual Technique* handleSchemeNotFound(unsigned short schemeIndex, const String& schemeName,
		Material* originalMaterial, unsigned short lodIndex, const Renderable* rend) {
			// Try to reload the material if it wasn't loaded already			
			if (depthMat.isNull()) {
				depthMat = MaterialManager::getSingleton().getByName("DepthMaterial");
				depthMat->load();
				technique = depthMat->getBestTechnique();
			}
			return technique;
	}
};

class DepthRenderListener : public RenderTargetListener {
private:
	MaterialSwitcher* switcher;
public:
	DepthRenderListener(MaterialSwitcher* matSwitcher) {
		switcher = matSwitcher;
	}
protected:
	virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
		MaterialManager::getSingleton().addListener(switcher);
	}
	virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
		MaterialManager::getSingleton().removeListener(switcher);
	}
};

TimeET CameraInfo::dirtyTime;

void CameraInfo::setParent(Client* p) {
	if(p==parent)
		return;
	if(parent!=NULL) {
		frameName.removePrimitiveListener(this);
		parent->robotID.removePrimitiveListener(this);
		deleteCamera();
	}
	parent=p;
	if(parent!=NULL) {
		frameName.addPrimitiveListener(this);
		parent->robotID.addPrimitiveListener(this);
		setupCamera();
	}
}

void CameraInfo::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&frameName || (parent!=NULL && &pl==&parent->robotID) ) {
		if(ogreCamera!=NULL)
			deleteCamera();
		setupCamera();
		
	} else if(&pl==&width || &pl==&height) {
		deleteTexture();
		setupTexture();
		
	} else if(&pl==&fovY) {
		if(fovY>0)
			ogreCamera->setFOVy(Ogre::Radian(fovY));
		
	} else {
		std::cerr << "unknown CameraInfo field changed" << std::endl;
	}
}

void CameraInfo::addImageListener(ImageListener * l) {
	listeners.insert(l);
}

void CameraInfo::removeImageListener(ImageListener * l) {
	listeners.erase(l);
}

void CameraInfo::setupCamera() {
	using namespace Ogre;
	
	if(frameName==plist::OutputSelector::UNUSED || parent==NULL || parent->robotID.size()==0)
		return;
	
	SceneNode * parnode = NULL;
	for(std::map<KinematicJoint*, JointGraphicsInfo*>::const_iterator it=parent->kjInfos.begin(); it!=parent->kjInfos.end(); ++it) {
		// A hack. The depth camera needs to be attached to a scene node which controls its
		// movement just like for the regular camera. We just attach it to the regular camera's
		// scene node
		if (frameName.get() == "DepthFrame") {
				if(it->first->outputOffset.get() == "CameraFrame") {
				parnode = &it->second->getJointNode();
				break;
			}
		}
		else if(it->first->outputOffset == frameName) {
			parnode = &it->second->getJointNode();
			break;
		}
	}
	
	if(parnode==NULL) {
		// not actually abnormal, might be getting camera setup before all frames have been declared.
		// we'll try again on first frame render
		//std::cerr << "Mirage could not find frame for camera " << frameName.get() << " on " << parent->robotID << std::endl;
		return;
	}
	ogreCamera = ogreSceneMgr->createCamera(parent->robotID+"-"+frameName.get()+":Camera");
	if(fovY>0)
		ogreCamera->setFOVy(Radian(fovY));
	ogreCamera->setNearClipDistance(40);
	parnode->attachObject(ogreCamera);
	ogreCamera->pitch(Radian((Ogre::Real)M_PI));
	fovY.addPrimitiveListener(this);
	
	setupTexture();
}

void CameraInfo::setupTexture() {
	using namespace Ogre;

	// enable anti-aliasing if the image isn't huge
	unsigned int w=width, h=height;
	if(w<800 && h<600) {
		w*=2;
		h*=2;
	}
	/* // up to 4x anti-aliasing? :)
	 if(w<800 && h<600) {
	 w*=2;
	 h*=2;
	 }*/
	
	TexturePtr texture = TextureManager::getSingleton().createManual( parent->robotID+"-"+frameName.get()+":RttTex", 
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D, 
		w, h, 0, PF_BYTE_RGB, TU_RENDERTARGET );
	RenderTexture *rttTex = texture->getBuffer()->getRenderTarget();

	Viewport *v = rttTex->addViewport( ogreCamera );
	//v->setClearEveryFrame( true );
	v->setBackgroundColour( ColourValue::Black );
	v->setVisibilityMask(GraphicsInfo::ROBOT_CAMERA_MASK);
	
	// If the camera is a depth cam, add a material scheme and a listener
	// to the viewport and RTT
	if (isDepthCam) {
		v->setMaterialScheme("DepthPass");
		rttTex->addListener(new DepthRenderListener(new MaterialSwitcher()));
	}

	width.addPrimitiveListener(this);
	height.addPrimitiveListener(this);
}

void CameraInfo::renderImage() {
	using namespace Ogre;
	if(/*dirtyTime<renderTime ||*/ listeners.size()==0)
		return;
	
	TexturePtr texture = TextureManager::getSingleton().getByName(parent->robotID+"-"+frameName.get()+":RttTex");
	if(texture.isNull()) {
		setupCamera(); // catches case where camera loaded before frames were set up, try again now
		
		texture = TextureManager::getSingleton().getByName(parent->robotID+"-"+frameName.get()+":RttTex");
		if(texture.isNull()) { // still not set up, complain and fail out
			std::cerr << "no render-texture for camera " << frameName.get() << std::endl;
			return;
		}
	}
	
	ImageBuffer * img = getFreeBuffer();
	PixelBox box(width,height,1,PF_BYTE_RGB,img->getRGBBuffer());
	texture->getBuffer()->blitToMemory(box);
	
	renderTime.Set();
	fireImageRendered(img);
}

bool CameraInfo::isValid() {
	return !Ogre::TextureManager::getSingleton().getByName(parent->robotID+"-"+frameName.get()+":RttTex").isNull();
}


ImageBuffer* CameraInfo::getFreeBuffer() {
	std::vector<ImageBuffer*> inuse;
	ImageBuffer * img = NULL;
	while(!bufferQueue.empty()) {
		img = bufferQueue.front();
		bufferQueue.pop();
		if(img->getWidth()!=width || img->getHeight()!=height) {
			img->removeReference(); // camera size changed, drop old buffers
			img=NULL;
		} else if(img->getReferences()>1) {
			//cout << "Busy " << img << endl;
			inuse.push_back(img); // still in transit, skip it
			img=NULL;
		} else {
			img->clear(); // this will work, claim it
			break;
		}
	}
	for(std::vector<ImageBuffer*>::const_iterator it=inuse.begin(); it!=inuse.end(); ++it)
		bufferQueue.push(*it); // put buffers we skipped over back in the queue
	if(img==NULL) {
		img = new ImageBuffer(width,height);
		img->addReference();
		//cout << "Created " << img << std::endl;
	}
	bufferQueue.push(img);
	return img;
}

void CameraInfo::deleteTexture() {
	width.removePrimitiveListener(this);
	height.removePrimitiveListener(this);
	if(parent!=NULL)
		Ogre::TextureManager::getSingleton().remove(parent->robotID+"-"+frameName.get()+":RttTex");
}

void CameraInfo::deleteCamera() {
	deleteTexture();
	fovY.removePrimitiveListener(this);
	if(ogreCamera!=NULL) {
		if(ogreCamera->getParentNode()!=NULL)
			ogreCamera->getParentSceneNode()->detachObject(ogreCamera);
		ogreSceneMgr->destroyCamera(ogreCamera);
		ogreCamera=NULL;
	}
}

void CameraInfo::fireImageRendered(ImageBuffer* img) const {
	// copy listeners so we aren't screwed if a listener is removed during processing
	std::set<ImageListener*> pls=listeners;
	for(std::set<ImageListener*>::const_iterator it=pls.begin(); it!=pls.end(); ++it) {
		// make sure current listener hasn't been removed
		if(listeners.count(*it)>0)
			(*it)->renderedImage(*this,img);
	}
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
