//-*-c++-*-
#ifndef INCLUDED_FileSystemSensorSource_h_
#define INCLUDED_FileSystemSensorSource_h_

#include "FileSystemDataSource.h"
#include "Motion/PostureEngine.h"

//! Extends FileSystemDataSource to parse sensor data
class FileSystemSensorSource : public FileSystemDataSource {
public:
	//! constructor
	FileSystemSensorSource(LoggedDataDriver& p, const std::string& filter) : FileSystemDataSource(p,filter) {}
	
	virtual void registerSource() {
		FileSystemDataSource::registerSource();
		for(unsigned int i=0; i<NumOutputs; i++)
			providingOutput(i);
	}
	
	virtual void deregisterSource() {
		for(unsigned int i=0; i<NumOutputs; i++)
			ignoringOutput(i);
		FileSystemDataSource::deregisterSource();
	}
	
protected:
	virtual bool sendData() {
		SensorInfo& sensors = dynamic_cast<SensorInfo&>(**curfile);
		if(!sensors.isValid())
			return false;
		
		MarkScope l(getSensorWriteLock());
		for(unsigned int i=0; i<NumOutputs; ++i)
			if(sensors.getOutput(i).weight>0)
				setOutputValue(i, sensors.getOutput(i).value);
		for(std::map<unsigned int,float>::const_iterator it=sensors.getButtons().begin(); it!=sensors.getButtons().end(); ++it)
			setButtonValue(it->first, it->second);
		for(std::map<unsigned int,float>::const_iterator it=sensors.getSensors().begin(); it!=sensors.getSensors().end(); ++it)
			setSensorValue(it->first, it->second);
		for(std::map<unsigned int,float>::const_iterator it=sensors.getPIDDuties().begin(); it!=sensors.getPIDDuties().end(); ++it)
			setPIDDutyValue(it->first, it->second);
		return true;
	}
	
	//! extends FileInfo to provide sensor parsing
	class SensorInfo : public FileInfo {
	public:
		//! constructor
		SensorInfo(const std::string& name, double time) : FileInfo(name,time), pose(), sensors(), valid(false) {}
		
		//! uses FileInfo's prepare to load file into memory, and then replaces with a decompressed version
		virtual void prepare() {
			if(pose.getLoadedSensors()==NULL) {
				pose.setLoadedSensors(&sensors);
				FileInfo::prepare();
				if(data==NULL)
					return;
				valid = pose.loadBuffer(data,size,filename.c_str());
			}
		}
		virtual void release() {
			pose.setLoadedSensors(NULL);
			valid=false;
		}
		
		bool isValid() const { return valid; }
		const OutputCmd& getOutput(unsigned int i) const { return pose(i); }
		const std::map<unsigned int,float>& getButtons() const { return sensors.buttons; }
		const std::map<unsigned int,float>& getSensors() const { return sensors.sensors; }
		const std::map<unsigned int,float>& getPIDDuties() const { return sensors.pidduties; }
		
	protected:
		PostureEngine pose;
		PostureEngine::SensorInfo sensors;
		bool valid;
	};
	
	virtual void enqueueFile(const std::string& name, double lifetime) { files.push_back(new SensorInfo(name,lifetime)); } 
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
