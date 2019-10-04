//-*-c++-*-
#ifndef INCLUDED_DataStreamDriver_h_
#define INCLUDED_DataStreamDriver_h_

#include "local/DeviceDriver.h"
#include "local/DataSource.h"
#include "local/CommPort.h"

//! A generic base class for drivers which are receiving a stream of data through a comm port, also providing an interface to be used as a data source in another driver
class DataStreamDriver : public virtual DeviceDriver, public DataSource, public virtual plist::PrimitiveListener, protected Thread  {
public:
	explicit DataStreamDriver(const std::string& className, const std::string& localName)
	: DeviceDriver(className,localName), DataSource(),
	commName(), srcFrameRate(0), paceInput(false),
	timestamp(0), frameNumber(0), registered(false), realtime(false)
	{
		addEntry("CommPort",commName,"The name of the comm port from which data will be read.");
		addEntry("SourceFPS",srcFrameRate,"The expected source framerate, used to regulate buffering");
		addEntry("PaceInput",paceInput,"If set to false, the driver will attempt to detect and run through any backlog on the comm port; if true, the driver will process each image in turn, enforcing SourceFPS.\nThis mainly comes up if you do something like 'mkfifo pipe; cat *.jpg > pipe', and then open 'pipe' as the input for ImageStreamDriver... if PaceInput is turned off, you'll only get the last image.\nHowever, using PaceInput on 'live' sources may cause a backlog to form.");
	}
	
	virtual unsigned int nextTimestamp();
	virtual const std::string& nextName() { return instanceName; }
	virtual bool advance();
	virtual void registerSource();
	virtual void deregisterSource();
	
	//! The name of the comm port from which data will be read
	plist::Primitive<std::string> commName;
	
	//! The expected source framerate, used to regulate buffering
	plist::Primitive<float> srcFrameRate;
	
	//! If set to false, the driver will attempt to detect and run through any backlog on the comm port; if true, the driver will process each image in turn, enforcing SourceFPS.
	/*! This mainly comes up if you do something like 'mkfifo pipe; cat *.jpg > pipe', and then open 'pipe' as the input
	 *  for ImageStreamDriver... if paceInput is turned off, you'll only get the last image.
	 *  However, using PaceInput on 'live' sources may cause a backlog to form.*/
	plist::Primitive<bool> paceInput;
	
protected:
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	virtual void* run();
	
	void doFreeze();
	void doUnfreeze();
	
	//! Called when data should be read from the specified data stream and then provided to the framework
	/*! If #paceInput is set, your implementation should use readsome() to skim through
	 *  to the last available data packet, and provide only that data to the framework */
	virtual bool readData(std::istream& is)=0;
	
	//! allows subclasses to override the commport lookup
	virtual CommPort * getComm(const std::string& name) { return CommPort::getRegistry().getInstance(name); }
	//! called when connection initialization is required (opens comm port, etc.)
	virtual void connect(CommPort* comm);
	//! called when connection destruction is required (closes comm port, etc.)
	virtual void disconnect(CommPort* comm);
	//! called if no image data is readily available, should return true if thread should block for frame
	virtual bool requestFrame(CommPort& comm) { return false; }
	
	unsigned int timestamp; //!< timestamp of last data received
	unsigned int frameNumber; //!< image frame number
	
	bool registered; //!< true if registerSource has been called and deregisterSource has not
	bool realtime; //!< true if enteringRealtime has been called more recently than leavingRealtime
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterDriver;
	DataStreamDriver(const DataStreamDriver&); //!< don't call
	DataStreamDriver& operator=(const DataStreamDriver&); //!< don't call
};

/*! @file
 * @brief Describes DataStreamDriver, a generic base class for drivers which are receiving a stream of data through a comm port, also providing an interface to be used as a data source in another driver
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
