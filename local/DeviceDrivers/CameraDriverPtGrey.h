//-*-c++-*-
#ifndef INCLUDED_CameraDriverPtGrey_h_
#define INCLUDED_CameraDriverPtGrey_h_

#include "local/DeviceDriver.h"
#include "local/DataSource.h"
#include "Shared/get_time.h"
#include "IPC/CallbackThread.h"

#include <FlyCapture2.h>

//! description of CameraDriverPtGrey
class CameraDriverPtGrey : public virtual DeviceDriver, public virtual plist::PrimitiveListener, public DataSource
{
public:
	explicit CameraDriverPtGrey(const std::string& name)
		: DeviceDriver(autoRegisterCameraDriverPtGrey,name), DataSource(),
			index(0),
			thread(&CameraDriverPtGrey::threadrun,*this), lock(),
			cam(), frameCount(0), timestamp(0)
	{
		addEntry("Index",index,"Index of camera in PGR BusManager (should be same as flycap)");
	}
	
	~CameraDriverPtGrey() {
		index.removePrimitiveListener(this);
	}
	
	virtual std::string getClassName() const { return autoRegisterCameraDriverPtGrey; }
	
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear(); sources.insert(std::make_pair("Camera",this));
	}
	
	virtual unsigned int nextTimestamp() { return get_time(); }
	virtual const std::string& nextName() { return instanceName; }
	
	virtual bool advance();
	virtual void registerSource();
	virtual void deregisterSource();
	
	//! watches #index, triggers a close() and re-open() if it changes
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<unsigned int> index;
	
protected:
	void threadrun();
	CallbackThread thread;
	Thread::Lock lock; // buffer/img_size lock so we can't change resolution while reading

	void doFreeze();
	void doUnfreeze();

	void openCam();
	void closeCam();

	bool convertImage(FlyCapture2::Image & rawImage);

	FlyCapture2::Camera cam;

	unsigned int frameCount;
	unsigned int timestamp;

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterCameraDriverPtGrey;
};

/*! @file
* @brief 
* @author Ethan Tira-Thompson (ejt)
* @author Alex Grubb (agrubb1)
*/

#endif
