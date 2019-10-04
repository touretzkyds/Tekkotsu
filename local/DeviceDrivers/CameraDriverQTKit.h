//-*-c++-*-
#ifndef INCLUDED_CameraDriverQTKit_h_
#define INCLUDED_CameraDriverQTKit_h_

#include "local/DeviceDriver.h"
#include "local/DataSources/CameraSourceQTKit.h"

//! This driver provides camera capture through the QTKit API, which is the only capture interface which supports 64-bit on OS X
class CameraDriverQTKit : public DeviceDriver, public virtual plist::CollectionListener {
public:
	//! constructor, may throw a const char* on error
	explicit CameraDriverQTKit(const std::string& name) : DeviceDriver(autoRegisterCameraDriver,name) {
		updateCameraList();
	}
	
	~CameraDriverQTKit() {}
	
	virtual std::string getClassName() const { return autoRegisterCameraDriver; }
	
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		for(const_iterator it=begin(); it!=end(); ++it) {
			if(it->first==".type")
				continue;
			if(CameraSourceQTKit * ds = dynamic_cast<CameraSourceQTKit*>(it->second))
				sources[it->first]=ds;
			else
				std::cerr << "WARNING CameraDriver entry " << it->first << " is not actually a CameraSource!!!" << std::endl;
		}
	}
	
protected:
	void updateCameraList();
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterCameraDriver;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
