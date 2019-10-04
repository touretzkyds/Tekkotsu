//-*-c++-*-
#ifndef INCLUDED_CameraInfo_h_
#define INCLUDED_CameraInfo_h_

#include "ImageBuffer.h"

#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Shared/TimeET.h"

#include <set>
#include <queue>

class Client;
namespace Ogre {
	class Camera;
}

class CameraInfo : virtual public plist::Dictionary, public plist::PrimitiveListener {
public:
	class ImageListener {
	public:
		virtual ~ImageListener() {}
		virtual void renderedImage(const CameraInfo& cam, ImageBuffer * img)=0;
	};
	
	CameraInfo() : plist::Dictionary(), plist::PrimitiveListener(), 
	frameName(), width(640), height(480), fovY(45*(float)M_PI/180), isDepthCam(false),
	parent(NULL), renderTime(0L), ogreCamera(NULL), listeners(), bufferQueue()
	{
		addEntry("FrameName",frameName);
		addEntry("Width",width);
		addEntry("Height",height);
		addEntry("FOV_Y",fovY);
		addEntry("IsDepthCam",isDepthCam);

		frameName.setRange(0,-1U); // accept anything (i.e. reference frames are fine)
		setLoadSavePolicy(FIXED,SYNC);
	}
	~CameraInfo() {
		deleteCamera();
		while(!bufferQueue.empty()) {
			bufferQueue.front()->removeReference();
			bufferQueue.pop();
		}
	}
	
	void setParent(Client* p);
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	void addImageListener(ImageListener * l);
	void removeImageListener(ImageListener * l);
	size_t getNumImageListeners() const { return listeners.size(); }
	void renderImage();
	static void setDirty() { dirtyTime.Set(); }
	bool isValid();
	
	
	plist::OutputSelector frameName;
	plist::Primitive<unsigned int> width;
	plist::Primitive<unsigned int> height;
	plist::Angle fovY;
	plist::Primitive<bool> isDepthCam;
	
protected:
	ImageBuffer* getFreeBuffer();
	void setupCamera();
	void setupTexture();
	void fireImageRendered(ImageBuffer* img) const;
	void deleteTexture();
	void deleteCamera();
	
	Client * parent;
	static TimeET dirtyTime;
	TimeET renderTime;
	Ogre::Camera* ogreCamera;
	std::set<ImageListener*> listeners;
	std::queue<ImageBuffer*> bufferQueue;
};


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
