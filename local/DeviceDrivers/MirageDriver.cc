#include "MirageDriver.h"
#include "local/CommPort.h"
#include "Motion/KinematicJoint.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"
#include "Shared/get_time.h"
#include "Shared/WorldState.h"
#include <libxml/tree.h>

//better to put this here instead of the header
using namespace std; 

const std::string MirageDriver::autoRegisterDriver = DeviceDriver::getRegistry().registerType<MirageDriver>("Mirage");

const char* MirageDriver::encodingNames[] = { "YUV", "PNG", "JPG", NULL };
INSTANTIATE_NAMEDENUMERATION_STATICS(MirageDriver::Encoding);

void MirageDriver::DepthSubscription::registerSource() {
	DataStreamDriver::registerSource();
#if defined(TGT_HAS_CAMERA)
	CameraName = "Mirage";
#endif
}

void MirageDriver::DepthSubscription::deregisterSource() {
#if defined(TGT_HAS_CAMERA)
	CameraName = CameraModelName;
#endif
	DataStreamDriver::deregisterSource();
}

void MirageDriver::ImageSubscription::registerSource() {
	DataStreamDriver::registerSource();
#if defined(TGT_HAS_CAMERA)
	CameraName = "Mirage";
#endif
}

void MirageDriver::ImageSubscription::deregisterSource() {
#if defined(TGT_HAS_CAMERA)
	CameraName = CameraModelName;
#endif
	DataStreamDriver::deregisterSource();
}

void MirageDriver::motionStarting() {
	MotionHook::motionStarting();
	opener.start();
	imgsrc.host.addPrimitiveListener(this);
	imgsrc.port.addPrimitiveListener(this);
	sensrc.host=imgsrc.host;
	sensrc.port=imgsrc.port;
	for(unsigned int i=0; i<NumOutputs; ++i)
		positions[i] = DataSource::getOutputValue(i);
}

void MirageDriver::updateAllOutputs() {
	std::vector<size_t> changedIndices;
	changedIndices.reserve(NumOutputs);
	float outputs[NumFrames][NumOutputs];
	for(unsigned int i=0; i<NumOutputs; ++i) {
		changedIndices.push_back(i);
		outputs[NumFrames-1][i]  = DataSource::getOutputValue(i);
	}
	motionUpdated(changedIndices, outputs);
	// std::cout << "outputs[24] = " << outputs[NumFrames-1][24] << std::endl;
}

bool MirageDriver::isConnected() {
	return comm.is_open() || comm.is_connecting();
}

void MirageDriver::motionStopping() {
	persist.removePrimitiveListener(this);
	imgsrc.host.removePrimitiveListener(this);
	imgsrc.port.removePrimitiveListener(this);
	if(opener.isStarted())
		opener.stop().join();
	MotionHook::motionStopping();
}

void MirageDriver::motionUpdated(const std::vector<size_t>& changedIndices, const float outputs[][NumOutputs]) {
	MotionHook::motionUpdated(changedIndices,outputs);
	if(changedIndices.size()==0 || !comm.is_open() || !comm) {
		// just copy values, don't send update
		for(std::vector<size_t>::const_iterator it=changedIndices.begin(); it!=changedIndices.end(); ++it)
			positions[*it]=outputs[NumFrames-1][*it];
	} else {
		plist::Dictionary d;
		plist::Dictionary pos;
		for(std::vector<size_t>::const_iterator it=changedIndices.begin(); it!=changedIndices.end(); ++it) {
			positions[*it]=outputs[NumFrames-1][*it];
			// if ( *it == 24 ) std::cout << "outputs[NumFrames-1][24] = " << outputs[NumFrames-1][24] << std::endl;
			pos.addEntry(capabilities.getOutputName(*it),positions[*it]);
		}
		d.addEntry("Positions",pos);
		sendUpdate(d);
	}
}

void MirageDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&imgsrc.host || &pl==&imgsrc.port) {
		// if here, then motionStarted or setDataSourceThread has been called, thus when host changes,
		// need to close old one and reopen new one
		if(opener.isStarted())
			opener.stop().join();
		sensrc.host=imgsrc.host;
		sensrc.port=imgsrc.port;
		opener.start();
	} else if(&pl==&persist) {
		plist::Dictionary d;
		d.addEntry("Persist",persist);
		sendUpdate(d);
	} else if(&pl==&highres) {
		if(highres && imgsrc.encoding==ENCODE_YUV) {
			std::cerr << "MirageDriver: YUV encoding does not support highres mode (try PNG)" << std::endl;
			highres=false;
			return;
		}
		if(!comm.is_open())
			return;
#ifdef TGT_HAS_CAMERA
		plist::Dictionary cam1;
		cam1.addEntry("Width",new plist::Primitive<unsigned int>(highres ? CameraResolutionX*2 : CameraResolutionX));
		cam1.addEntry("Height",new plist::Primitive<unsigned int>(highres ? CameraResolutionY*2 : CameraResolutionY));
		cam1.addEntry("IsDepthCam", new plist::Primitive<bool>(false));

		plist::Dictionary cam2;
		cam2.addEntry("Width",new plist::Primitive<unsigned int>(highres ? CameraResolutionX*2 : CameraResolutionX));
		cam2.addEntry("Height",new plist::Primitive<unsigned int>(highres ? CameraResolutionY*2 : CameraResolutionY));
		cam2.addEntry("IsDepthCam", new plist::Primitive<bool>(true));

		plist::Array cameras;
		cameras.addEntry(cam1);
		cameras.addEntry(cam2);
		plist::Dictionary d;
		d.addEntry("Cameras",cameras);
		sendUpdate(d);
#endif
	} else if(&pl==&imgsrc.encoding) {
		if(highres && imgsrc.encoding==ENCODE_YUV) {
			std::cerr << "MirageDriver: YUV encoding does not support highres mode" << std::endl;
			highres=false;
			return;
		}
	} else if(&pl==&physicsWalk) {
		if(!physicsWalk) {
			DriverMessaging::addListener(this,DriverMessaging::FixedPoints::NAME);
		} else {
			std::cout << "removing fixed points" << std::endl;
			DriverMessaging::removeListener(this,DriverMessaging::FixedPoints::NAME);
			// send empty FixedPoints list to clear them out
			plist::Array arr;
			plist::Dictionary msg;
			msg.addEntry("FixedPoints",arr);
			sendUpdate(msg);
		}
	} else if(&pl==&physicsWheels) {
		plist::Dictionary msg;
		msg.addEntry("PhysicsWheels",physicsWheels);
		sendUpdate(msg);
	} else {
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

void MirageDriver::openConnection() {
	IPaddr addr(imgsrc.host,imgsrc.port);
	if(!addr.is_valid()) {
		std::cerr << instanceName << " could not open connection to " << imgsrc.host << ":" << imgsrc.port << std::endl;
		return;
	}

	try {
		if(!physicsWalk)
			DriverMessaging::addListener(this,DriverMessaging::FixedPoints::NAME);
		physicsWalk.addPrimitiveListener(this);
		physicsWheels.addPrimitiveListener(this);
		while(true) {
			{ // scope limiting
				MarkScope autolock(commLock);
				
				comm.clear(); // clear out iostream state bits... new connection, try again
				comm.seekp(0); // clear out any old unflushed messages (e.g. what we were trying to write when received notification of closure)
				while(!comm.open(addr)) {
					Thread::testCurrentCancel();
					usleep(1000*1000);
					Thread::testCurrentCancel();
					//std::cout << "attempt open to " << addr.get_name() << " on " << addr.get_port() << std::endl;
				}
				comm << "<messages>\n";
				
				KinematicJoint kin;
				if(kin.loadFile(::config->makePath(::config->motion.kinematics).c_str())==0) {
					std::cerr << "MirageDriver unable to find/parse kinematics file '" << ::config->motion.kinematics << "'" << std::endl;
					return;
				}
				
				plist::Dictionary d;
				d.addEntry("Persist",persist);
				if(!persist) {
					d.addEntry("Location",initLocation);
					d.addEntry("Orientation",initOrientation);
				}
				KinematicJointSaver kinSaver(kin);
				d.addEntry("Model",kinSaver);
				plist::Dictionary pos;
				for(unsigned int i=0; i<NumOutputs; ++i) {
					positions[i] = state->outputs[i];
					pos.addEntry(outputNames[i],positions[i]);
					//if ( i == 24 ) {
					//	std::cout << "positions[24] = " << positions[i]
					//						<< "  sensorValues[24] = " << DataSource::getOutputValue(i) << std::endl;
					//}
				}
				d.addEntry("Positions",pos);
				d.addEntry("PhysicsWheels",physicsWheels);

#ifdef TGT_HAS_CAMERA
				plist::Dictionary cam1;
				cam1.addValue("FrameName",outputNames[CameraFrameOffset]);
				cam1.addValue("Width",highres ? CameraResolutionX*2 : CameraResolutionX);
				cam1.addValue("Height",highres ? CameraResolutionY*2 : CameraResolutionY);
				cam1.addEntry("IsDepthCam", new plist::Primitive<bool>(false));

				/* Mirage (or really Ogre) does not support rendering non-square pixels.
				 * Ogre specifies camera by Y FOV.  We pass CameraVertFOV below, which will
				 * maintain the vertical FOV for accurate distance estimates, but note
				 * in the case of non-square pixels this sacrifies heading measurements.
				 *
				 * If you want to reverse this trade-off, recompute 'square' Y field of view:
				 *   yfov = 2 * std::atan( std::tan(CameraHorizFOV/2) * CameraResolutionY / CameraResolutionX );
				 */
				cam1.addValue("FOV_Y",CameraVertFOV);

				plist::Dictionary cam2;
				cam2.addValue("FrameName","DepthFrame");
				cam2.addValue("Width",highres ? CameraResolutionX*2 : CameraResolutionX);
				cam2.addValue("Height",highres ? CameraResolutionY*2 : CameraResolutionY);
				cam2.addEntry("IsDepthCam", new plist::Primitive<bool>(true));
				cam2.addValue("FOV_Y",CameraVertFOV);

				plist::Array cameras;
				cameras.addEntry(cam1);
				cameras.addEntry(cam2);
				d.addEntry("Cameras",cameras);
				highres.addPrimitiveListener(this);
#endif
				
				sendUpdate(d);
				// Don't know why we need to call updateAllOutputs() here,
				// since we already initialized the joint positions above, but
				// something in MotionExec::poll seems to be messing up the
				// initial joint angles established by the Motion.StartPose
				// parameter, and calling updateAllOutputs() here overrides
				// that.  -- DST 2/2014
				updateAllOutputs();
				
				// these aren't recursive, so just do every time:
				persist.addPrimitiveListener(this);
			} // release lock while we block on connection to close
			
			comm.get(); // either remote closes first and we get an error here ...
			if(comm.is_open()) {
				// try to close the connection now
				MarkScope autolock(commLock);
				comm.clear(); // breaking out of the comm.get causes an error bit, but connection may be fine
				comm << "</messages>" << std::endl;
				comm.close();
			}
			Thread::testCurrentCancel(); // ... or we close first and get a thread cancellation from the destructor
			ASSERT(!comm,"comm closed, but still active?");
		}
	} catch(...) { // thread cancellation?
		physicsWalk.removePrimitiveListener(this);
		physicsWheels.removePrimitiveListener(this);
		DriverMessaging::removeListener(this);
		if(comm.is_open()) {
			// try to close the connection now
			MarkScope autolock(commLock);
			comm.clear(); // breaking out of the comm.get causes an error bit, but connection may be fine
			comm << "</messages>" << std::endl;
			comm.close();
		}
		throw;
	}
}

void MirageDriver::sendUpdate(plist::Dictionary& msg) {
	std::stringstream ss;
	ss << "RobotID-" << ::config->wireless.id;
	plist::Primitive<std::string> wid(ss.str());
	msg.addEntry("ID",wid);
	MarkScope autolock(commLock);
	msg.saveStream(comm,true);
	if(bufferedMsg.size()>0) {
		comm << bufferedMsg;
		bufferedMsg.clear();
	}
	comm.flush();
}

void MirageDriver::SensorSubscription::opened() {
	NetworkCommPort::opened();
	
	plist::Dictionary msg;
	
	std::stringstream ss;
	ss << "RobotID-" << ::config->wireless.id;
	plist::Primitive<std::string> wid(ss.str());
	msg.addEntry("ID",wid);
	
	plist::Primitive<bool> sendSensors(true);
	msg.addEntry("SendSensors",sendSensors);
	
	MarkScope autolock(*this);
	std::ostream os(&getWriteStreambuf());
	os << "<subscription>\n";
	msg.saveStream(os,true);
	os.flush();
}

void MirageDriver::DepthSubscription::opened() {
	NetworkCommPort::opened();

	plist::Dictionary msg;
	
	std::stringstream ss;
	ss << "RobotID-" << ::config->wireless.id;
	plist::Primitive<std::string> wid(ss.str());
	msg.addEntry("ID",wid);
	
	plist::Primitive<unsigned int> depthIdx(1);
	msg.addEntry("CameraIndex",depthIdx);
	
	msg.addEntry("Encoding",encoding);
	msg.addEntry("PNGLevel",pngCompressionLevel);
	msg.addEntry("JPGQuality",jpgQualityLevel);
	
	MarkScope autolock(*this);
	std::ostream os(&getWriteStreambuf());
	os << "<subscription>\n";
	msg.saveStream(os,true);
	os.flush();
}

void MirageDriver::ImageSubscription::opened() {
	NetworkCommPort::opened();

	plist::Dictionary msg;
	
	std::stringstream ss;
	ss << "RobotID-" << ::config->wireless.id;
	plist::Primitive<std::string> wid(ss.str());
	msg.addEntry("ID",wid);
	
	plist::Primitive<unsigned int> camIdx(0);
	msg.addEntry("CameraIndex",camIdx);
	
	msg.addEntry("Encoding",encoding);
	msg.addEntry("PNGLevel",pngCompressionLevel);
	msg.addEntry("JPGQuality",jpgQualityLevel);
	
	MarkScope autolock(*this);
	std::ostream os(&getWriteStreambuf());
	os << "<subscription>\n";
	msg.saveStream(os,true);
	os.flush();
}

void MirageDriver::DepthSubscription::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&encoding) {
		format.set(encoding.get());
	} else if(&pl==&pngCompressionLevel || &pl==&jpgQualityLevel) {
		if(!isWriteable())
			return;
		plist::Dictionary d;
		// just send them both every time because I'm lazy
		d.addEntry("PNGLevel",pngCompressionLevel);
		d.addEntry("JPGQuality",jpgQualityLevel);
		MarkScope autolock(*this);
		std::ostream os(&getWriteStreambuf());
		d.saveStream(os,true);
		os.flush();
	} else
		Subscription<ImageStreamDriver>::plistValueChanged(pl);
}

void MirageDriver::ImageSubscription::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&encoding) {
		format.set(encoding.get());
	} else if(&pl==&pngCompressionLevel || &pl==&jpgQualityLevel) {
		if(!isWriteable())
			return;
		plist::Dictionary d;
		// just send them both every time because I'm lazy
		d.addEntry("PNGLevel",pngCompressionLevel);
		d.addEntry("JPGQuality",jpgQualityLevel);
		MarkScope autolock(*this);
		std::ostream os(&getWriteStreambuf());
		d.saveStream(os,true);
		os.flush();
	} else
		Subscription<ImageStreamDriver>::plistValueChanged(pl);
}

void MirageDriver::processDriverMessage(const DriverMessaging::Message& dmsg) {
	if(!comm.is_open() || !comm)
		return;
	if(dmsg.CLASS_NAME==DriverMessaging::FixedPoints::NAME) {
		plist::Dictionary msg;
		const DriverMessaging::FixedPoints& fpmsg = dynamic_cast<const DriverMessaging::FixedPoints&>(dmsg);
		msg.addEntry("FixedPoints",const_cast<DriverMessaging::FixedPoints&>(fpmsg));
		if(!fpmsg.flushOnMotionUpdate)
			sendUpdate(msg);
		else {
			std::stringstream ss;
			ss << "RobotID-" << ::config->wireless.id;
			plist::Primitive<std::string> wid(ss.str());
			msg.addEntry("ID",wid);
			ss.str(std::string());
			msg.saveStream(ss,true);
			bufferedMsg = ss.str();
		}
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
