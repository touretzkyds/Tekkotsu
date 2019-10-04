//-*-c++-*-
#ifndef INCLUDED_PListSensorDriver_h_
#define INCLUDED_PListSensorDriver_h_

#include "DataStreamDriver.h"

//! Accepts a stream of plist packets, each containing dictionaries for output, button, and sensor values (see DynamicRobotState)
class PListSensorDriver : public DataStreamDriver {
public:
	explicit PListSensorDriver(const std::string& name)
		: DeviceDriver(autoRegisterDriver,name), DataStreamDriver(autoRegisterDriver,name)
	{
		for(unsigned int i=0; i<NumOutputs; ++i)
			providing[i] = false;
	}
	
	virtual std::string getClassName() const { return autoRegisterDriver; }
	
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"]=this;
	}
	
	virtual void deregisterSource();
	
protected:
	virtual bool readData(std::istream& is);
	
	bool providing[NumOutputs];  //!< outputs which are being provided by the input stream, so we can claim them and block local feedback
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterDriver;
	PListSensorDriver(const PListSensorDriver&); //!< don't call
	PListSensorDriver& operator=(const PListSensorDriver&); //!< don't call
};

/*! @file
 * @brief Describes PListSensorDriver, which accepts a stream of plist packets, each containing dictionaries for output, button, and sensor values
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
