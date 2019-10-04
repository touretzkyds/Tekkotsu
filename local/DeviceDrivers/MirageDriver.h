//-*-c++-*-
#ifndef INCLUDED_MirageDriver_h_
#define INCLUDED_MirageDriver_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DeviceDrivers/ImageStreamDriver.h"
#include "local/DeviceDrivers/PListSensorDriver.h"
#include "local/CommPorts/NetworkCommPort.h"
#include "Wireless/netstream.h"
#include "IPC/CallbackThread.h"
#include "Shared/MarkScope.h"
#include "IPC/DriverMessaging.h"

//! description of MirageDriver
class MirageDriver : public virtual DeviceDriver, public MotionHook, public virtual plist::PrimitiveListener, public virtual DriverMessaging::Listener {
public:
	static const unsigned int DEFAULT_PORT = 19785;
	static const unsigned int DEFAULT_FPS = 25;
	
	enum Encoding { ENCODE_YUV, ENCODE_PNG, ENCODE_JPG }; //!< must be kept in sync with tools/mirage/CommThread.h
	static const char* encodingNames[]; //!< must be kept in sync with tools/mirage/CommThread.h

	explicit MirageDriver(const std::string& name)
		: DeviceDriver(autoRegisterDriver,name), MotionHook(), persist(false), highres(false), initLocation(), initOrientation(), physicsWalk(false), physicsWheels(true),
		commLock(), comm(), bufferedMsg(), opener(&MirageDriver::openConnection,*this), opening(false), positions(), sensrc(name), imgsrc(name), depthsrc(name)
	{
		addEntry("Host",imgsrc.host,"Specifies the hostname running the Mirage client");
		addEntry("Port",imgsrc.port,"Specifies the port number of the Mirage client");
		addEntry("Persist",persist,"If true, the robot model will not be removed when the Tekkotsu executable disconnects.");
		addEntry("Encoding",imgsrc.encoding,"Indicates whether to request JPG or PNG images from the simulated camera input");
		addEntry("PNGLevel",imgsrc.pngCompressionLevel,"The compression level to pass to libpng, and thus zlib.  0 for no compression (fastest), 9 for maximum (slowest).  Image quality is constant as this is lossless.");
		addEntry("JPGQuality",imgsrc.jpgQualityLevel,"The compression level to pass to libjpeg, 0 for very poor quality and small file size, 100 for best quality but larger files.");
		addEntry("HighRes",highres,"If true, will render simulated camera input at 'double' resolution instead of 'full'");
		addEntry("InitialLocation",initLocation,"The x, y, and z coordinates the robot will start out at if Persist is false.");
		addEntry("InitialOrientation",initOrientation,"The axis component of a quaternion representing robot orientation to use if Persist is false.");
		addEntry("PhysicsWalk",physicsWalk,"If true (and the kinematics configuration specifies mass for the robot), Mirage will use friction and physics to model the effects of walking; if false, Mirage will use a \"perfect\" model based on hints from the WalkMC itself.");
		addEntry("PhysicsWheels",physicsWheels,"If true (and the kinematics configuration specifies mass for the robot), then Mirage wheel use friction and physics to model robot motion; if false, Mirage will try to directly compute and move the robot.");
		highres.addPrimitiveListener(this);
		imgsrc.encoding.addPrimitiveListener(this);
		setLoadSavePolicy(FIXED,SYNC);
	}

	virtual std::string getClassName() const { return autoRegisterDriver; }
	
	virtual MotionHook* getMotionSink() { return dynamic_cast<MotionHook*>(this); }
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"]=&sensrc;
	}
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Camera"]=&imgsrc;
		sources["Depth"] = &depthsrc;
	}
	
	virtual void motionStarting();
	virtual void updateAllOutputs();
	virtual bool isConnected();
	virtual void motionStopping();
	virtual void motionUpdated(const std::vector<size_t>& changedIndices, const float outputs[][NumOutputs]);
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<bool> persist;
	
	plist::Primitive<bool> highres; //!< If true, will render simulated camera input at "double" resolution instead of "full"
	
	plist::Point initLocation;
	plist::Point initOrientation;
	plist::Primitive<bool> physicsWalk; //!< If true (and the kinematics configuration specifies mass for the robot), Mirage will use friction and physics to model the effects of walking; if false, Mirage will use a "perfect" model based on hints from the WalkMC itself.
	plist::Primitive<bool> physicsWheels; //!< If true (and the kinematics configuration specifies mass for the robot), then Mirage wheel use friction and physics to model robot motion; if false, Mirage will try to directly compute and move the robot.
	
protected:
	Thread::Lock commLock;
	ionetstream comm;
	std::string bufferedMsg;
	CallbackThread opener;
	bool opening;
	plist::Primitive<float> positions[NumOutputs];
	
	template<class T>
	class Subscription : public T, public NetworkCommPort {
	public:
		Subscription(const std::string& name) : DeviceDriver("MirageDriver::Subscription",name+"::Subscription"), T(name), NetworkCommPort(name) {
			host="localhost";
			port=DEFAULT_PORT;
			T::srcFrameRate = DEFAULT_FPS;
		}
		virtual const std::string& nextName() { return T::instanceName; }
		virtual void doFreeze();
		virtual void doUnfreeze();
	protected:
		virtual CommPort * getComm(const std::string& name) { return this; }
		virtual bool requestFrame(CommPort& comm);
		virtual void opened()=0;
		virtual void closing();
		virtual void plistValueChanged(const plist::PrimitiveBase& pl) {
			if(&pl==&host || &pl==&port)
				NetworkCommPort::plistValueChanged(pl);
			else
				T::plistValueChanged(pl);
		}
	};
	
	class SensorSubscription : public Subscription<PListSensorDriver> {
	public:
		SensorSubscription(const std::string& name)
		: DeviceDriver("MirageDriver::ImageSubscription",name+"::SensorSubscription"), Subscription<PListSensorDriver>(name+"::SensorSubscription") {}
	protected:
		virtual void opened();
	} sensrc;
	
	class ImageSubscription : public Subscription<ImageStreamDriver> {
	public:
		ImageSubscription(const std::string& name)
			: DeviceDriver("MirageDriver::ImageSubscription",name+"::ImageSubscription"), Subscription<ImageStreamDriver>(name+"::ImageSubscription"),
			encoding(ENCODE_PNG,encodingNames), pngCompressionLevel(1), jpgQualityLevel(85)
		{
			encoding.addNameForVal("jpeg",ENCODE_JPG);
			encoding.addPrimitiveListener(this);
			pngCompressionLevel.addPrimitiveListener(this);
			jpgQualityLevel.addPrimitiveListener(this);
			format.set(encoding.get()); // pass as string to select appropriate enum
		}
		
		virtual void registerSource();
		virtual void deregisterSource();
		
		plist::NamedEnumeration<Encoding> encoding; //!< indicates whether to stream JPG or PNG images as simulated camera input
		plist::Primitive<unsigned int> pngCompressionLevel; //!< the compression level to pass to libpng, and thus zlib.  0 for no compression (fastest), 9 for maximum compression (slowest), image quality is constant as this is lossless
		plist::Primitive<unsigned int> jpgQualityLevel; //!< the compression level to pass to libjpeg, 0 for very poor quality and small file size, 100 for best quality but larger files
	protected:
		virtual void opened();
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	} imgsrc;

	class DepthSubscription : public Subscription<ImageStreamDriver> {
	public:
		DepthSubscription(const std::string& name)
		: DeviceDriver("MirageDriver::DepthSubscription", name + "::DepthSubscription"),
			Subscription<ImageStreamDriver> (name + "::DepthSubscription"),
			encoding(ENCODE_PNG, encodingNames), pngCompressionLevel(1), jpgQualityLevel(85)
		{
			encoding.addNameForVal("jpeg", ENCODE_JPG);
			encoding.addPrimitiveListener(this);
			pngCompressionLevel.addPrimitiveListener(this);
			jpgQualityLevel.addPrimitiveListener(this);
			format.set(encoding.get());
			setSID(ProjectInterface::visRawDepthSID);
		}

		virtual void registerSource();
		virtual void deregisterSource();

		plist::NamedEnumeration<Encoding> encoding;
		plist::Primitive<unsigned int> pngCompressionLevel;
		plist::Primitive<unsigned int> jpgQualityLevel;
	protected:
		virtual void opened();
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	} depthsrc;

	void openConnection();
	void sendUpdate(plist::Dictionary& msg);
	
	void processDriverMessage(const DriverMessaging::Message& dmsg);

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterDriver;
};

template<class T>
void MirageDriver::Subscription<T>::doUnfreeze() {
	T::doUnfreeze();
	if(!isWriteable())
		return;
	plist::Dictionary d;
	plist::Primitive<bool> singleFrame(false);
	d.addEntry("SingleFrame",singleFrame);
	//MarkScope autolock(*this); // not a query-response, don't need lock
	std::ostream os(&getWriteStreambuf());
	d.saveStream(os,true);
	os.flush();
}

template<class T>
void MirageDriver::Subscription<T>::doFreeze() {
	T::doFreeze();
	requestFrame(*this);
}

template<class T>
bool MirageDriver::Subscription<T>::requestFrame(CommPort& comm) {
	if(!isWriteable() || T::realtime)
		return false;
	plist::Dictionary d;
	plist::Primitive<bool> singleFrame(true);
	d.addEntry("SingleFrame",singleFrame);
	MarkScope autolock(*this);
	std::ostream os(&getWriteStreambuf());
	d.saveStream(os,true);
	os.flush();

	return true;
}

template<class T>
void MirageDriver::Subscription<T>::closing() {
	MarkScope autolock(*this);
	std::ostream os(&getWriteStreambuf());
	os << "</subscription>" << std::flush;
	NetworkCommPort::closing();
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
