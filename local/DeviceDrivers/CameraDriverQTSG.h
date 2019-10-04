//-*-c++-*-
#ifndef INCLUDED_CameraDriverQTSG_h_
#define INCLUDED_CameraDriverQTSG_h_

#include "local/DeviceDriver.h"
#include "local/DataSources/CameraSourceQTSG.h"

//! This driver provides camera capture through QuickTime and the Sequence Grabber, which is deprecated.  See the alternative CameraDriverQTKit implementation.
/*! This camera driver is used on pre-10.6 systems, supporting 32 bit only */
class CameraDriverQTSG : public virtual DeviceDriver, public virtual plist::CollectionListener {
public:
	//! constructor, may throw a const char* on error
	explicit CameraDriverQTSG(const std::string& name)
		: DeviceDriver(autoRegisterCameraDriver,name)
	{
		if(!checkQTThreadInit()) {
			std::cerr << "CameraDriver: Couldn't initialize QuickTime" << std::endl;
			return;
		}
		
		updateCameraList();
	}
	
	~CameraDriverQTSG() {}
	
	virtual std::string getClassName() const { return autoRegisterCameraDriver; }
	
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		for(const_iterator it=begin(); it!=end(); ++it) {
			if(it->first==".type")
				continue;
			if(CameraSourceQTSG * ds = dynamic_cast<CameraSourceQTSG*>(it->second))
				sources[it->first]=ds;
			else
				std::cerr << "WARNING CameraDriver entry " << it->first << " is not actually a CameraSource!!!" << std::endl;
		}
	}
	
protected:
	//! converts from pascal-format string to c-format string
	static std::string p2c(unsigned char pascalStr[]) {
		unsigned char len = *pascalStr++;
		return std::string(reinterpret_cast<char*>(pascalStr),len);
	}
	
	static void dumpLiteral(OSType t);
	
	void updateCameraList();
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterCameraDriver;
};

/*! @file
 * @brief Describes CameraDriverQTSG, which provides camera capture through QuickTime and the Sequence Grabber, now deprecated.  See the alternative CameraDriverQTKit implementation.
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
