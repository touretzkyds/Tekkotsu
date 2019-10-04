//-*-c++-*-
#ifndef INCLUDED_LoggedDataDriver_h_
#define INCLUDED_LoggedDataDriver_h_

#include "local/DeviceDriver.h"
#include "local/DataSources/FileSystemSensorSource.h"
#include "local/DataSources/FileSystemImageSource.h"

//! description of LoggedDataDriver
class LoggedDataDriver : public virtual DeviceDriver, public virtual plist::PrimitiveListener {
public:
	LoggedDataDriver(const std::string& name) : DeviceDriver(autoRegisterLoggedDataDriver,name),
		path("logged"), fsSensorSrc(*this, ".*\\.pos$"), fsImageSrc(*this, ".*\\.(jpg|jpeg|png)$")
	{
		addEntry("Path",path,"The directory from which data samples should be loaded, or a single specific file.\nA single file can be either a single data file (e.g. sensor or camera image), or an index file as output by VisionGUI, or in the format 'filename <tab> time', where 'filename' is an absolute path or relative to the directory containing the index file, and 'time' is in milliseconds, relative to the time at which the index file is loaded.\nIn the future, this could also be network addresses for teleoperation and remote processing.");
		addEntry("Sensors",fsSensorSrc,"Settings for loading logged sensor readings (stored in PostureEngine format)");
		addEntry("Camera",fsImageSrc,"Settings for loading logged camera images (stored as either JPEG or PNG)");
		path.addPrimitiveListener(this);
		fsSensorSrc.path.addPrimitiveListener(this);
		fsImageSrc.path.addPrimitiveListener(this);
	}
	
	virtual std::string getClassName() const { return autoRegisterLoggedDataDriver; }
	
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"]=&fsSensorSrc;
	}
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Camera"]=&fsImageSrc;
	}
	
	virtual void plistValueTouched(const plist::PrimitiveBase& pl) {
		plistValueChanged(pl);
	}
	virtual void plistValueChanged(const plist::PrimitiveBase& pl) {
		if(&pl==&path) {
			// reload file lists for data sources which aren't overriding the common path
			bool good=true;
			if(fsSensorSrc.path.size()==0 && fsSensorSrc.isRegistered())
				good=fsSensorSrc.loadFileList();
			if(fsImageSrc.path.size()==0 && fsImageSrc.isRegistered()) {
				if(good)
					fsImageSrc.loadFileList();
				else // already got (and reported) an error on the path, don't try again
					fsImageSrc.clearFiles();
			}
		}
		if(&pl==&fsSensorSrc.path || &pl==&fsImageSrc.path || &pl==&path) {
			// if both streams are using the same source...
			if(fsSensorSrc.getUsedPath()==fsImageSrc.getUsedPath()) {
				//keep them in sync across loops, smooth offset between end times
				//std::cout << "SYNCING LOOP TIMES" << std::endl;
				double looptime=std::max(fsSensorSrc.getLoopTime(false),fsImageSrc.getLoopTime(false));
				if(looptime>0) { // if negative, indicates neither in use, framerates are unknown
					fsSensorSrc.setLoopTime(looptime);
					fsImageSrc.setLoopTime(looptime);
				}
			}
		} else {
			std::cerr << "LoggedDataDriver didn't handle call to plistValueChanged for " << pl.get() << std::endl;
		}
	}
	
	plist::Primitive<std::string> path;
	
protected:
	FileSystemSensorSource fsSensorSrc;
	FileSystemImageSource fsImageSrc;
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterLoggedDataDriver;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
