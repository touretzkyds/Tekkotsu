#include "Dynamixel.h"
#include "Shared/MarkScope.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "Shared/TimeET.h"
#include "IPC/CallbackThread.h"
#include <algorithm>

using namespace std; 
using namespace DynamixelProtocol;

INSTANTIATE_NAMEDENUMERATION_STATICS(SensorOffset_t);
enum SeenState { LED_UNSEEN, LED_SAME, LED_DIFF };

unsigned int DynamixelDriver::VOLTAGE_SENSOR_OFFSET = capabilities.findSensorOffset("PowerVoltage");
unsigned int DynamixelDriver::TEMP_SENSOR_OFFSET = capabilities.findSensorOffset("PowerThermo");

const std::string DynamixelDriver::autoRegisterDynamixelDriver = DeviceDriver::getRegistry().registerType<DynamixelDriver>("Dynamixel");

void DynamixelDriver::motionStarting() {
	ASSERTRET(!motionActive,"DynamixelDriver::motionStarting, but motionActive is true");
	MotionHook::motionStarting();
	
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL) {
		if(commName.size()>0)
			std::cerr << "DynamixelDriver \"" << instanceName << "\": initialization failed, CommPort \"" << commName << "\" not found" << std::endl;
	} else if(!comm->open() || !comm->isWriteable()) {
		std::cerr << "DynamixelDriver \"" << instanceName << "\": initialization failed, CommPort disconnected" << std::endl;
	} else {
		bool restartComm=false;
		if(commThread.isStarted()) { // commThread hogs the comm lock, stop it so we don't starve
			commThread.stop().join();
			restartComm=true;
		}
		MarkScope autolock(*comm);
		std::ostream os(&comm->getWriteStreambuf());
		// we'll reset these individually on the first motion update
		for(servo_iterator it=servos.begin(); it!=servos.end(); ++it)
			it->second->punch = it->second->margin = it->second->slope = 0;
		write(os, BroadcastFullComplianceCmd()).flush();
		write(os, BroadcastNoPunchCmd()).flush();
		write(os, BroadcastZeroSpeedCmd()).flush();
		// we use compliance slope to disable torque, so make sure it's enabled here
		write(os, BroadcastTorqueCmd(true)).flush();
		
		if(!sensorsActive) // first to become active, ping servos
			pingServos();
		
		if(restartComm) // we might have stopped commThread
			commThread.start();
	}
	
	motionActive=true;
	commName.addPrimitiveListener(this);
}

bool DynamixelDriver::isConnected() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	return (comm!=NULL && comm->isReadable() && comm->isWriteable());
}

void DynamixelDriver::motionStopping() {
	ASSERTRET(motionActive,"DynamixelDriver::motionStopping, but motionActive is false");
	motionActive=false;
	if(!sensorsActive) { // last one to stay active...
		if(commThread.isStarted())
			commThread.stop().join();
		// listener count is not recursive, so only remove if we're the last one
		commName.removePrimitiveListener(this);
	}
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL) {
		sendZeroTorqueCmd(*comm);
		comm->close(); // this *is* recursive, so we always close it to match our open() in motionStarting()
	}
	MotionHook::motionStopping();
}

void DynamixelDriver::motionCheck(const float outputs[][NumOutputs]) {
	float * buf = commThread.getWriteBuffer();
	for(unsigned int i=NumOutputs; i!=0; ) {
		--i;
		buf[i] = outputs[NumFrames-1][i];
	}
	commThread.setWriteBufferTimestamp(buf);
}

void DynamixelDriver::updatePIDs(const std::vector<MotionHook::PIDUpdate>& pids) {
	MarkScope autolock(commThread.pidLock);
	for(std::vector<MotionHook::PIDUpdate>::const_iterator it=pids.begin(); it!=pids.end(); ++it)
		commThread.pidValues[it->idx]=*it;
	commThread.dirtyPIDs+=pids.size();
}

void DynamixelDriver::registerSource() {
	ASSERTRET(!sensorsActive,"DynamixelDriver::registerSource, but sensorsActive is true");
	sensorsActive=true;
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->open();
	for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
		if(it->second->detected) {
			provideOutput(it->second->output);
			provideOutput(it->second->freeSpinOutput);
		}
		it->second->output.addPrimitiveListener(this);
		it->second->freeSpinOutput.addPrimitiveListener(this);
		it->second->detected.addPrimitiveListener(this);
	}
	if(!motionActive && comm!=NULL && comm->isWriteable()) // first to become active, ping servos
		pingServos();
	commName.addPrimitiveListener(this);
	commLatency.addPrimitiveListener(this);
	numPoll.addPrimitiveListener(this);
	responseTime.addPrimitiveListener(this);
	plistValueChanged(numPoll); // just to trigger sanity check on these values
}

void DynamixelDriver::deregisterSource() {
	ASSERTRET(sensorsActive,"DynamixelDriver::deregisterSource, but sensorsActive is false");
	sensorsActive=false;
	if(!motionActive) { // last to stay active...
		if(commThread.isStarted())
			commThread.stop().join();
		// listener count is not recursive, so only remove if we're the last one
		commName.removePrimitiveListener(this);
	}
	commLatency.removePrimitiveListener(this);
	numPoll.removePrimitiveListener(this);
	responseTime.removePrimitiveListener(this);
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->close();
	for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
		it->second->detected.removePrimitiveListener(this);
		it->second->freeSpinOutput.removePrimitiveListener(this);
		it->second->output.removePrimitiveListener(this);
		if(it->second->detected)
			ignoreOutput(it->second->output);
	}
}

void DynamixelDriver::doUnfreeze() {
	MarkScope sl(commThread.getStartLock());
	if(!commThread.isStarted()) {
		commThread.start();
	}
}

void DynamixelDriver::doFreeze() {
	MarkScope sl(commThread.getStartLock());
	if(commThread.isStarted()) {
		commThread.stop().join();
		ASSERT(!commThread.isStarted(),"DynamixelDriver::CommThread ended, but still running?");
	}
}

unsigned int DynamixelDriver::nextTimestamp() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
		return -1U;
	return commThread.nextTimestamp();
}

bool DynamixelDriver::advance() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
		return false;
	
	if(!commThread.isStarted()) {
		// indicates we're in non-realtime mode
		pingServos(true);
	}
	static unsigned int lockeduptest=0;
	if(!commThread.takeUpdate()) {
		if(++lockeduptest==8)
			std::cerr << "WARNING: DynamixelDriver appears to be locked up, not getting new sensor readings" << std::endl;
		return false;
	}
	if(lockeduptest>=8)
		std::cerr << "DynamixelDriver has gotten unwedged, sending sensor updates again." << std::endl;
	lockeduptest=0;
	
	return true;
}

void DynamixelDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&commName) {
		// if here, then motionStarted or registerSource has been called, thus when commName changes,
		// need to close old one and reopen new one
		if(commThread.isStarted())
			commThread.stop().join();
		
		CommPort * comm = CommPort::getRegistry().getInstance(commName.getPreviousValue());
		if(comm!=NULL) {
			// close each of our old references
			if(sensorsActive)
				comm->close();
			if(motionActive) {
				sendZeroTorqueCmd(*comm);
				comm->close();
			}
		}
		bool motionWasActive=motionActive, sensorsWereActive=sensorsActive;
		sensorsActive=motionActive=false;
		
		// open each of our new references
		if(motionWasActive)
			motionStarting();
		if(sensorsWereActive)
			registerSource();
		
		if(getTimeScale()>0)
			commThread.start();
	} else if(&pl==&commLatency || &pl==&numPoll || &pl==&responseTime) {
		if(numPoll==0) {
			std::cerr << "NumPoll must be at least 1, remove " << instanceName << " from the Sensors.Sources list if you want to disable sensor polling." << std::endl;
			numPoll=1;
		} else if(numPoll>1 && (numPoll * responseTime)/1000 > commLatency) {
			std::cerr << "WARNING: NumPoll * ResponseTime (" << numPoll << "·" << responseTime << "µs=" << (numPoll*responseTime/1000) << "ms) exceeds BufferLatency (" << commLatency << "ms)\n"
			"You may be missing synchronization with buffer flushes" << std::endl;
		}
	} else {
		// check if it's one of the individual servos... if it is, means we're providing servo feedback,
		// need to call providingOutput/ignoringOutput as appropriate
		for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
			if(&pl==&it->second->detected) {
				if(it->second->detected) {
					provideOutput(it->second->output);
					provideOutput(it->second->freeSpinOutput);
				} else {
					ignoreOutput(it->second->freeSpinOutput);
					ignoreOutput(it->second->output);
				}
				return; // found it, DON'T fall through to error message below...
			} else if(it->second->detected) {
				if(&pl==&it->second->output) {
					ignoreOutput(it->second->output.getPreviousValue());
					provideOutput(it->second->output);
					return; // found it, DON'T fall through to error message below...
				} else if(&pl==&it->second->freeSpinOutput) {
					ignoreOutput(it->second->freeSpinOutput.getPreviousValue());
					provideOutput(it->second->freeSpinOutput);
					return; // found it, DON'T fall through to error message below...
				}
			} else if(&pl==&it->second->output || &pl==&it->second->freeSpinOutput) {
				return; // found it, DON'T fall through to error message below...
			}
		}
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

void DynamixelDriver::processDriverMessage(const DriverMessaging::Message& d) {
	if(d.CLASS_NAME==DriverMessaging::LoadPrediction::NAME) {
		const DriverMessaging::LoadPrediction& loads = dynamic_cast<const DriverMessaging::LoadPrediction&>(d);
		for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
			plist::DictionaryOf<plist::Primitive<float> >::const_iterator dit = loads.loads.findEntry(it->second->output.get());
			if(dit!=loads.loads.end())
				it->second->predictedLoad=*dit->second;
		}
	} else if(d.CLASS_NAME==DriverMessaging::SensorPriority::NAME) {
		const DriverMessaging::SensorPriority& pri = dynamic_cast<const DriverMessaging::SensorPriority&>(d);
		for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
			plist::DictionaryOf<plist::Primitive<float> >::const_iterator dit = pri.outputs.findEntry(it->second->output.get());
			if(dit!=pri.outputs.end())
				it->second->sensorPriority=*dit->second;
		}
	}
}

void DynamixelDriver::pingServos(bool detectedOnly) {
	if(!detectedOnly)
		for(servo_iterator it=servos.begin(); it!=servos.end(); ++it)
			it->second->detected=false;

	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL)
		return;
	
	bool restartComm=false;
	if(commThread.isStarted()) { // commThread hogs the comm lock, stop it so we don't starve
		commThread.stop().join();
		restartComm=true;
	}
	
	MarkScope autolock(comm->getLock());
	if(!comm->isWriteable() || !comm->isReadable()) {
		std::cerr << "DynamixelDriver \"" << instanceName << "\": cannot ping servos, CommPort disconnected." << std::endl;
		ASSERT(!restartComm,"How was the comm thread still running?  Not restarting it...");
		return;
	}
	std::istream is(&comm->getReadStreambuf());
	std::ostream os(&comm->getWriteStreambuf());
	if(!is || !os) {
		std::cerr << "DynamixelDriver \"" << instanceName << "\": cannot ping servos, CommPort gave bad iostreams." << std::endl;
		ASSERT(!restartComm,"How was the comm thread still running?  Not restarting it...");
		return;
	}
	
	// request exception on badbit so we can thread_cancel read under linux
	// (otherwise we get FATAL: exception not rethrown / Abort error)
	is.exceptions(ios_base::badbit);
	
	MarkScope writeLock(getSensorWriteLock());
	ServoSensorsResponse servoSensors;
	AXS1SensorsResponse axs1Sensors;
	for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
		if( (it->second->getModel()==MODEL_UNKNOWN && !it->second->hasSensorOffset() && it->second->output==UNUSED)
		   || ( it->second->getModel()==MODEL_AXS1 && !it->second->hasSensorOffset() )
		   || ( it->second->getModel()!=MODEL_UNKNOWN && it->second->getModel()!=MODEL_AXS1 && it->second->output==UNUSED ))
			continue; // don't poll units we're not using
		if(!detectedOnly) {
			it->second->setModel(MODEL_UNKNOWN);
			it->second->detected=false;
		} else if(!it->second->detected) {
			continue;
		}
		unsigned int servoid = it->second->servoID;
		PingThread ping(is, os, servoid, it->second->output, &servoSensors, &axs1Sensors);
		const char* model = static_cast<const char*>(ping.join());
		is.clear();
		os.clear();
		if(model==Thread::CANCELLED || model==NULL) {
			unsigned int idx = it->second->output;
			if(static_cast<unsigned int>(idx)<NumOutputs) {
				cerr << "Warning: Dynamixel servo " << it->first
				 << " (mapped to output offset " << outputNames[idx] << ")"
				 << " was disconnected or not found" << endl;
			} else if(idx!=UNUSED) {
				cerr << "Warning: Dynamixel servo " << it->first
				 << " (mapped to invalid output " << idx << "))"
				 << " was disconnected or not found" << endl;
			} else if(servoid>=NumOutputs+START_SERVO_ID && (it->second->leftIRDistOffset!=-1 || it->second->centerIRDistOffset!=-1 || it->second->rightIRDistOffset!=-1
					|| it->second->leftLuminosityOffset!=-1 || it->second->centerLuminosityOffset!=-1 || it->second->rightLuminosityOffset!=-1
					|| it->second->micVolumeOffset!=-1 || it->second->micSpikeCountOffset!=-1))
			{
				cerr << "Warning: Dynamixel sensor module " << it->first
				<< " was disconnected or not found" << endl;
			}
			it->second->sensorActivation=0;
			continue;
		}
		it->second->setModelName(model);
		for(std::map<DynamixelProtocol::ModelID_t, const std::string>::const_iterator nit=DynamixelProtocol::dynamixelModels.begin(); nit!=DynamixelProtocol::dynamixelModels.end(); ++nit) {
			if(nit->second==model) {
				it->second->setModel(nit->first);
				break;
			}
		}
		//std::cerr << "Ping " << it->second->servoID << " detected model " << it->second->getModel() << std::endl;
		it->second->detected=true;
		if(it->second->sensorActivation==0)
			it->second->sensorActivation=1;
		if(it->second->getModel()==MODEL_AXS1) {
			provideValues(*it->second,axs1Sensors);
		} else if(it->second->getModel()!=MODEL_UNKNOWN) {
			it->second->lastCmd=servoSensors.getPosition();
			provideValues(*it->second,servoSensors);
		}
	}
	
	if(restartComm)
		commThread.start();
}

void DynamixelDriver::sendZeroTorqueCmd(CommPort& comm) {
	if(!comm.isWriteable()) {
		std::cerr << "DynamixelDriver \"" << instanceName << "\": unable to send shutdown, CommPort disconnected" << std::endl;
	} else {
		MarkScope autolock(comm);
		std::ostream os(&comm.getWriteStreambuf());
		//printf("\nHEADER: ");
		//debuget::hexout(BroadcastTorqueCmd(true),sizeof(BroadcastTorqueCmd));
		write(os, BroadcastTorqueCmd(false)).flush();
	}
}

/// @cond INTERNAL
static bool sensorActivationCompare(const DynamixelDriver::ServoInfo* a, const DynamixelDriver::ServoInfo* b) { return a->sensorActivation > b->sensorActivation; }
/// @endcond

Thread& DynamixelDriver::CommThread::stop() {
	// we might be stopped by the user while failsafe is also engaged
	if(getCurrent()!=&failsafe) {
		// not stopped by failsafe, so stop the failsafe first
		failsafe.restartFlag=false;
		failsafe.stop().join();
	}
	// failsafe may have already stopped this thread
	if(isStarted())
		Thread::stop();
	return *this;
}

void DynamixelDriver::CommThread::waitForUpdate() {
	if(!isStarted()) {
		if(!updated)
			std::cerr << "DynamixelDriver::CommThread::waitForUpdate: thread not running!" << std::endl;
		return;
	}
	if(updated)
		return;
	join();
	ASSERT(!failsafe.isStarted(),"DynamixelDriver::CommThread ended, but failsafe still running?");
}

float * DynamixelDriver::CommThread::getWriteBuffer() {
	float * bufs[3];
	if(timestampBufA<timestampBufB) {
		if(timestampBufA<timestampBufC) {
			bufs[0]=outputBufA;
			if(timestampBufB<timestampBufC) {
				bufs[1]=outputBufB;
				bufs[2]=outputBufC;
			} else {
				bufs[1]=outputBufC;
				bufs[2]=outputBufB;
			}
		}  else {
			bufs[0]=outputBufC;
			bufs[1]=outputBufA;
			bufs[2]=outputBufB;
		}
	} else if(timestampBufB<timestampBufC) {
		bufs[0]=outputBufB;
		if(timestampBufA<timestampBufC) {
			bufs[1]=outputBufA;
			bufs[2]=outputBufC;
		}  else {
			bufs[1]=outputBufC;
			bufs[2]=outputBufA;
		}
	} else {
		bufs[0]=outputBufC;
		bufs[1]=outputBufB;
		bufs[2]=outputBufA;
	}
	return (bufs[0]==curBuf) ? bufs[1] : bufs[0];
}

void DynamixelDriver::CommThread::setWriteBufferTimestamp(float * buf) {
	if(buf==outputBufA)
		timestampBufA=get_time();
	else if(buf==outputBufB)
		timestampBufB=get_time();
	else if(buf==outputBufC)
		timestampBufC=get_time();
	else
		std::cerr << "DynamixelDriver::CommThread::setWriteBufferTimestamp was passed a unknown buffer" << std::endl;
}

bool DynamixelDriver::CommThread::launched() {
	isFirstCheck=true;
	if(!failsafe.isRunning())
		failsafe.start();
	return true;
}

void DynamixelDriver::CommThread::cancelled() {
	if(!failsafe.isEngaged()) {
		ASSERT(!failsafe.isStarted(),"Failsafe is still running!  How was CommThread cancelled?");
		if(responsePending) {
			// clear read buffer so it doesn't interfere with whatever we do next
			CallbackThread cb(std::mem_fun(&CommThread::clearBuffer), this, true);
			usleep(50*1000);
			if(cb.isStarted())
				cb.stop().join();
		}
	} else {
		if(servoPollQueue.empty())
			return;
		ServoInfo* cur=NULL;
		// find first with some activation energy, that was the one in progress
		for(std::vector<ServoInfo*>::const_iterator it=servoPollQueue.begin(); it!=servoPollQueue.end(); ++it) {
			cur=*it;
			if(cur->sensorActivation>0)
				break;
		}
		cur->sensorActivation=0;
		//std::cerr << "DynamixelDriver: comm timeout reading from servo #" << cur->servoID << ", restarting thread" << std::endl;
		if(++(cur->failures) >= 20) {
			// consistently failing, mark servo as missing so we'll ignore it from now on
			std::cerr << "DynamixelDriver: too many failures on #" << cur->servoID << ", disabling 'Detected' flag" << std::endl;
			cur->detected=false;
			cur->sensorActivation=0;
		}
	}
}

void DynamixelDriver::CommThread::clearBuffer() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
		return;
	
	MarkScope autolock(comm->getLock());
	std::istream is(&comm->getReadStreambuf());
	
	// request exception on badbit so we can thread_cancel read under linux
	// (otherwise we get FATAL: exception not rethrown / Abort error)
	is.exceptions(ios_base::badbit);
	
	while(is) {
		is.get();
		Thread::testCurrentCancel();
	}
}

unsigned int DynamixelDriver::CommThread::runloop() {
	testCancel();
	
	failsafe.progressFlag=true;
	
	// if there aren't any servos detected, just return now
	if(servoPollQueue.empty())
		return FrameTime*NumFrames*1000;
	
	// get the comm port
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
		return FrameTime*NumFrames*1000;
		
	MarkScope autolock(comm->getLock());
	std::ostream os(&comm->getWriteStreambuf());
	std::istream is(&comm->getReadStreambuf());
	
	// request exception on badbit so we can thread_cancel read under linux
	// (otherwise we get FATAL: exception not rethrown / Abort error)
	os.exceptions(ios_base::badbit);
	is.exceptions(ios_base::badbit);
	
	if(!driver.sensorsActive && continuousUpdates)
		updateCommands(is,os);
	
	unsigned int NUM_AVAIL_POLL = std::min<unsigned int>(driver.numPoll,servoPollQueue.size());
	std::partial_sort(servoPollQueue.begin(),servoPollQueue.begin()+NUM_AVAIL_POLL,servoPollQueue.end(),sensorActivationCompare);
	
	for(unsigned int i=0; driver.sensorsActive && i<NUM_AVAIL_POLL; ++i) {
		failsafe.progressFlag=true;
		if(servoPollQueue[i]->sensorActivation<=0) {
			NUM_AVAIL_POLL=i;
			break;
		}
		if(i>0) {
			// it's important we wait for at least the specified response time so the next query doesn't collide with a response in progress
			struct timespec st, rt;
			st.tv_sec = 0;
			st.tv_nsec = driver.responseTime*1000;
			while(nanosleep(&st,&rt)!=0) {
				testCancel();
				st=rt;
			}
		}
		failsafe.progressFlag=true;
		if(continuousUpdates)
			updateCommands(is,os);
		
		//cout << get_time() << " requesting from " << servoPollQueue[i]->servoID << " model " << servoPollQueue[i]->getModel() << endl;
		
		// send request for sensor values...
		//TimeET resptime;
		if(servoPollQueue[i]->getModel()==MODEL_AXS1) {
			//std::cout << "Query AX-S1 " << servoPollQueue[i]->servoID << std::endl;
			write(os, ReadAXS1SensorsCmd(servoPollQueue[i]->servoID)).flush();
			responsePending=true;
		} else if(servoPollQueue[i]->getModel()!=MODEL_UNKNOWN) {
			//std::cout << "Query Servo " << servoPollQueue[i]->servoID << std::endl;
			write(os, ReadServoSensorsCmd(servoPollQueue[i]->servoID)).flush();
			responsePending=true;
		} else {
			std::cerr << "Dynamixel Driver, attempting to poll unknown model '" << servoPollQueue[i]->getModelName() << "' (" << servoPollQueue[i]->getModel() << ") at servo ID " << servoPollQueue[i]->servoID << " (ignoring)" << std::endl;
			// drop it so we don't keep repeating the message
			std::vector<ServoInfo*>::iterator it=servoPollQueue.begin();
			std::advance(it,i);
			servoPollQueue.erase(it);
			--i;
			os.flush(); // flush any pending data (e.g. updateCommands) before the sleep on next iteration...
		}
		if(!os) {
			std::cerr << "DynamixelDriver " << driver.instanceName << " unable to send sensor poll request, lost comm?" << std::endl;
			return false;
		}
	}
	
	//TimeET latency;
	
	//std::cout << "READ " << get_time() << '\t' << (get_time()-lastSensorTime) << std::endl;
	lastSensorTime=get_time(); // actually should be set to time of first successful read... will be reset if that goes through
	
	ServoSensorsResponse servoresponse;
	AXS1SensorsResponse axs1response;
	for(unsigned int i=0; driver.sensorsActive && i<NUM_AVAIL_POLL; ++i) {
		failsafe.progressFlag=true;
		
		//cout << "reading from " << servoPollQueue[i]->servoID << endl;
		
		if(servoPollQueue[i]->getModel()==MODEL_AXS1) {
			readResponse(axs1response, is, *servoPollQueue[i]);
			
			// if we clear the count every poll, it disrupts count in progress, so count would always stay 0.  So only clear after a count has been registered.
			// buffer the clear commands, we don't want to interfere with incoming reads.  Will wait to flush after reads are complete.
			unsigned char checksum = 0;
			if(axs1response.sndCount>0) {
				write(os, SyncWriteHeader<SyncWriteSoundHoldAndCountEntry>(1), checksum);
				write(os, SyncWriteSoundHoldAndCountEntry(servoPollQueue[i]->servoID), checksum);
			} else {
				write(os, SyncWriteHeader<SyncWriteSoundHoldEntry>(1), checksum);
				write(os, SyncWriteSoundHoldEntry(servoPollQueue[i]->servoID), checksum);
			}
			os.put(~checksum);
			// note there is no flush... ok to wait and let it go out with the next write.
		
		} else if(servoPollQueue[i]->getModel()!=MODEL_UNKNOWN) {
			readResponse(servoresponse, is, *servoPollQueue[i]);
		}
		
		if(i==0)
			lastSensorTime=get_time(); // the rest should be coming in short order, but this syncs us to the comm buffer clear period
		
		/*
		 // adaptive load compensation not working... using loadCompensation directly as value instead of flag to enable servoDeflection
		unsigned short pos = response.getPosition();
		//cout << "Polling " << servoPollQueue[i]->servoID << " pos " << pos << " predicted " << servoPollQueue[i]->predictedLoad << endl;
		if(std::abs(servoPollQueue[i]->predictedLoad)>0.25 && pos>0 && pos<1024) {
			float defl = pos - servoPollQueue[i]->lastCmd;
			float f = defl / servoPollQueue[i]->predictedLoad;
			servoDeflection = servoDeflection*.999 + f*.001;
			//cout << servoPollQueue[i]->servoID << " at " << pos << " deflection " << defl << " prediction " << servoPollQueue[i]->predictedLoad << " = " << servoDeflection << endl;
		}
		*/
	}
	responsePending=false;
	
	// update activations to indicate which servos to poll next in CommThread...
	// Increment activation so the end of the list should have an activation of 20
	// (just a heuristic to be on par with a motion of 20 servo tics per update)
	const float actInc = (driver.numPoll>servoPollQueue.size()) ? 20 : 20.f*driver.numPoll/servoPollQueue.size();
	for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
		ServoInfo * s = it->second;
		if(s->detected) {
			if(s->sensorPriority>=0) {
				s->sensorActivation += s->sensorPriority;
			} else { // auto mode
				// we add more activation for movement too...
				s->sensorActivation += actInc + s->recentCmdMotion;
			}
		}
		/*if(s->sensorActivation>0)
		 cout << "Servo " << s->servoID << " has activation " << s->sensorActivation << endl;*/
	}
	
	//std::cout << "Latency: " << latency.Age() << std::endl;
	
	updated=true;
	//std::cout << "updated " /*<< (driver.thread==NULL) << ' ' << continuousUpdates << ' ' << NUM_AVAIL_POLL<< ' ' << servoPollQueue.size() << ' '*/ << get_time() << std::endl;
	
	if(!continuousUpdates) {
		failsafe.stop();
		return -1U;
	}
	
	return (driver.sensorsActive && NUM_AVAIL_POLL>0) ? 0 : FrameTime*NumFrames*1000;
}

void DynamixelDriver::provideValues(const ServoInfo& info, const DynamixelProtocol::ServoSensorsResponse& response) {
	MarkScope writeLock(getSensorWriteLock());
	
	if(info.freeSpinOutput<NumOutputs) {
		float x=response.getSpeed();
		if(info.invertRotation)
			x = info.maxTic - x;
		unsigned int idx=info.freeSpinOutput;
		float outputValue = x * info.repSpeedSlope; // apply calibration
		setOutputValue(idx,outputValue);
		if(idx-PIDJointOffset<NumPIDJoints) {
			float dutyValue = (info.curRotationMode!=ServoInfo::CONTINUOUS) ? 0 : response.getLoad() / 1023.f;
			setPIDDutyValue(idx-PIDJointOffset, (info.invertRotation) ? -dutyValue : dutyValue);
		}
	}
	if(info.output<NumOutputs && info.output!=info.freeSpinOutput) {
		float x = response.getPosition();
		if(info.invertRotation)
			x=info.maxTic-x;
		x = x/info.maxTic*info.maxAngle;
		x -= info.zeroAngle + info.maxAngle/2;
		unsigned int idx=info.output;
		setOutputValue(idx,x);
		if(idx-PIDJointOffset<NumPIDJoints) {
			float dutyValue = (info.curRotationMode!=ServoInfo::POSITION) ? 0 : response.getLoad() / 1023.f;
			setPIDDutyValue(idx-PIDJointOffset, (info.invertRotation) ? -dutyValue : dutyValue);
		}
	}
	ASSERT(info.output!=info.freeSpinOutput || info.curRotationMode==ServoInfo::CONTINUOUS,"Permanent free-spin, but curRotationMode not CONTINUOUS");
	
	if(VOLTAGE_SENSOR_OFFSET<NumSensors)
		setSensorValue(VOLTAGE_SENSOR_OFFSET, response.voltage/10.f);
	if(TEMP_SENSOR_OFFSET<NumSensors)
		setSensorValue(TEMP_SENSOR_OFFSET, response.temp);
}

/*!
 TODO: let sndCount be mapped to a button instead of sensor to provide events?
 */
void DynamixelDriver::provideValues(ServoInfo& info, const DynamixelProtocol::AXS1SensorsResponse& response) {
	if(info.leftIRDistOffset!=-1)
		setSensorValue(info.leftIRDistOffset, response.leftIR/255.f);
	if(info.centerIRDistOffset!=-1)
		setSensorValue(info.centerIRDistOffset, response.centerIR/255.f);
	if(info.rightIRDistOffset!=-1)
		setSensorValue(info.rightIRDistOffset, response.rightIR/255.f);
	
	if(info.leftLuminosityOffset!=-1)
		setSensorValue(info.leftLuminosityOffset, response.leftLum/255.f);
	if(info.centerLuminosityOffset!=-1)
		setSensorValue(info.centerLuminosityOffset, response.centerLum/255.f);
	if(info.rightLuminosityOffset!=-1)
		setSensorValue(info.rightLuminosityOffset, response.rightLum/255.f);
	
	if(info.micVolumeOffset!=-1 && response.sndMaxHold>0) // if zero, didn't get a reading; shouldn't see any values<127
		setSensorValue(info.micVolumeOffset, std::abs(response.sndMaxHold-127)/128.f);
	if(info.micSpikeCountOffset!=-1) {
		if(info.micSpikeFrameNumber != getSensorFrameNumber()) {
			// count has been read since last update, reset count to current value
			setSensorValue(info.micSpikeCountOffset, response.sndCount);
			info.micSpikeFrameNumber = getSensorFrameNumber();
		} else if(response.sndCount>0) {
			// previous spikes haven't been picked up yet - if new spikes, add on to the count
			setSensorValue(info.micSpikeCountOffset, getSensorValue(info.micSpikeCountOffset) + response.sndCount);
		}
	}

	/*
	if(VOLTAGE_SENSOR_OFFSET<NumSensors)
		setSensorValue(VOLTAGE_SENSOR_OFFSET, response.voltage/10.f);
	if(TEMP_SENSOR_OFFSET<NumSensors)
		setSensorValue(TEMP_SENSOR_OFFSET, response.temp);
	 */
}

void DynamixelDriver::CommThread::updateCommands(std::istream& is, std::ostream& os) {
	if(timestampBufA>timestampBufB) {
		if(timestampBufA>timestampBufC) {
			if(curBuf == outputBufA)
				return;
			curBuf = outputBufA;
		}  else {
			if(curBuf == outputBufC)
				return;
			curBuf = outputBufC;
		}
	} else if(timestampBufB>timestampBufC) {
		if(curBuf == outputBufB)
			return;
		curBuf = outputBufB;
	} else {
		if(timestampBufC==0)
			return; // no data yet, don't send commands to servos
		if(curBuf == outputBufC)
			return;
		curBuf = outputBufC;
	}
	
	float period = NumFrames*FrameTime;
	if(getTimeScale()>0)
		period /= ::getTimeScale();
	
	std::vector<SyncWritePosSpeedEntry> packets;
	packets.reserve(servos.size());
	
	std::vector<SyncWriteContinuousRotationEntry> modes;
	modes.reserve(servos.size());
	
	typedef std::vector<std::pair<std::string,ServoInfo::RotationMode> > modeUpdates_t;
	modeUpdates_t modeUpdates;
	modeUpdates.reserve(servos.size());
	
	std::vector<SyncWriteLEDEntry> leds;
	std::vector<SeenState> seenLEDs(NumLEDs,LED_UNSEEN);
	leds.reserve(servos.size());
	
	std::vector<SyncWriteComplianceEntry> compliances;
	compliances.reserve(servos.size());
	
	std::vector<SyncWritePunchEntry> punches;
	punches.reserve(servos.size());

	std::vector<SyncWritePIDEntry> pidentries;
	pidentries.reserve(servos.size());
	
	std::vector<SyncWriteTorqueEntry> torqueToggles;
	torqueToggles.reserve(servos.size());
	
	const bool havePIDUpdates = (dirtyPIDs>0);
	if(havePIDUpdates)
		pidLock.lock();
	
	for(servo_iterator it=servos.begin(); it!=servos.end(); ++it) {
		if(!it->second->detected || it->second->getModel()==MODEL_AXS1)
			continue;
		unsigned int servoid = it->second->servoID;
		
		unsigned int ledidx = it->second->led;
#ifndef TGT_HAS_LEDS
		if(ledidx!=UNUSED)
			std::cerr << "Warning: Dynamixel driver mapping servo " << it->first << " to invalid led output index " << ledidx << std::endl;
#else
		unsigned int ledsubidx = ledidx - LEDOffset;
		if(ledidx>=NumOutputs) {
			if(ledidx!=UNUSED)
				std::cerr << "Warning: Dynamixel driver mapping servo " << it->first << " to invalid led output index " << ledidx << std::endl;
		} else if(static_cast<unsigned int>(ledsubidx)>=NumLEDs) {
			std::cerr << "Warning: Dynamixel driver mapping servo " << it->first << " to invalid led index " << ledsubidx << std::endl;
		} else if(lastOutputs[ledidx]!=curBuf[ledidx] || lastLEDState[ledsubidx]==LED_UNKNOWN || (curBuf[ledidx]>0 && curBuf[ledidx]<1)) {
			if(seenLEDs[ledsubidx]==LED_UNSEEN) {
				// ensures we only calculate the led activation value once per LED signal
				// (even if it might be mapped to several servo LEDs...)
				LedState cur = calcLEDValue(ledsubidx,curBuf[ledidx]) ? LED_ON : LED_OFF;
				seenLEDs[ledsubidx] = (cur==lastLEDState[ledsubidx]) ? LED_SAME : LED_DIFF;
				lastLEDState[ledsubidx]=cur;
			}
			if(seenLEDs[ledsubidx]==LED_DIFF) {
				//cout << "\t\t" << it->first << " led " << lastLEDState[ledsubidx] << " @ " << get_time() << endl;
				leds.push_back(SyncWriteLEDEntry(servoid,lastLEDState[ledsubidx]==LED_ON));
			}
		}
#endif
		
		unsigned int idx = it->second->freeSpinOutput;
		ServoInfo::RotationMode rm = ServoInfo::CONTINUOUS;
		if(it->second->output!=it->second->freeSpinOutput) {
			if(it->second->freeSpinOutput>=NumOutputs) {
				if(it->second->freeSpinOutput!=UNUSED)
					std::cerr << "Warning: Dynamixel driver mapping servo " << it->first << " to invalid free spin output index " << it->second->freeSpinOutput << std::endl;
				idx = it->second->output;
				rm = ServoInfo::POSITION;
			} else if(curBuf[it->second->freeSpinOutput]==0) {
				idx = it->second->output;
				rm = ServoInfo::POSITION;
			}
		}
		if(idx>=NumOutputs) {
			if(idx!=UNUSED)
				std::cerr << "Warning: Dynamixel driver mapping servo " << it->first << " to invalid output index " << idx << std::endl;
			continue; // invalid/unused servo
		}
		/*if(it->second->predictedLoad==0) {
			// send position if first starting, if changed value, or if rotation mode changed
			if(isFirstCheck || lastOutputs[idx]!=curBuf[idx] || rm!=it->second->curRotationMode) {
				float speed;
				if(rm==ServoInfo::CONTINUOUS) {
					// continuous rotation mode, speed specified directly
					speed = curBuf[idx];
				} else if(lastOutputs[idx]!=curBuf[idx]) {
					// here because value changed, speed based on distance to travel
					speed = (curBuf[idx] - lastOutputs[idx]) / (period/1000);
				} else {
					// initial start or change of mode, choose a reasonable speed
					speed = .35;
				}
				packets.push_back(setServo(it, rm, curBuf[idx], speed));
			}
		} else*/ 
		{ // subject to load prediction, may need to adjust command
			// find current position in servo units
			float speed;
			if(rm==ServoInfo::CONTINUOUS) {
				// continuous rotation mode, speed specified directly
				speed = curBuf[idx];
			} else if(lastOutputs[idx]!=curBuf[idx]) {
				// here because value changed, speed based on distance to travel
				speed = (curBuf[idx] - lastOutputs[idx]) / (period/1000);
			} else {
				// initial start or change of mode, choose a reasonable speed
				speed = .35f;
			}
			unsigned short oldLastCmd = it->second->lastCmd;
			DynamixelProtocol::SyncWritePosSpeedEntry packet = setServo(it, rm, curBuf[idx], speed);
			// send position if first starting, if changed value, or if rotation mode changed
			if(isFirstCheck || oldLastCmd!=it->second->lastCmd || rm!=it->second->curRotationMode) {
				packets.push_back(packet);
			}
		}
		
		if(rm!=it->second->curRotationMode) {
		modes.push_back(SyncWriteContinuousRotationEntry(servoid,rm==ServoInfo::CONTINUOUS,it->second->getModel()));
			modeUpdates.push_back(std::make_pair(it->first,rm));
		}
		
		if(havePIDUpdates) {
			if(it->second->slope!=pidValues[idx].pids[DYNAMIXEL_SLOPE] || it->second->margin!=pidValues[idx].pids[DYNAMIXEL_MARGIN]) {
				if(it->second->slope!=pidValues[idx].pids[DYNAMIXEL_SLOPE]) {
					it->second->slope = pidValues[idx].pids[DYNAMIXEL_SLOPE];
					unsigned short torque = (it->second->slope>0) ? 0x3FF : 0;
					// Have to set max torque to zero when we set slope to 0, otherwise it still fights a little.
					// Setting torque enable instead doesn't cut it because it apparently gets re-enabled on sensor query
					//std::cout << "Max torque " << servoid << " set to " << torque << std::endl;
					torqueToggles.push_back(SyncWriteTorqueEntry(servoid,torque));
				}
				it->second->margin = pidValues[idx].pids[DYNAMIXEL_MARGIN];
				// Since margin=0 causes tremors, at least in AX-12s, don't
				// allow positive margin < 1 to be rounded down to 0.
				if ( it->second->margin > 0 && it->second->margin < 1 )
					it->second->margin = 1;
				/*
				if ( servoid == 1 )
					std::cout << "Servo " << servoid << "   margin = "
										<< pidValues[idx].pids[DYNAMIXEL_MARGIN] << " / " << it->second->margin
										<< "   slope = " << pidValues[idx].pids[DYNAMIXEL_SLOPE] << std::endl;
				*/
				compliances.push_back(SyncWriteComplianceEntry(servoid,(unsigned char)it->second->margin,(unsigned char)it->second->slope));
			}
			if(it->second->punch!=pidValues[idx].pids[DYNAMIXEL_PUNCH]) {
				it->second->punch = pidValues[idx].pids[DYNAMIXEL_PUNCH];
				// std::cout << "Punch " << servoid << " set to " << pidValues[idx].pids[DYNAMIXEL_PUNCH] << std::endl;
				punches.push_back(SyncWritePunchEntry(servoid,(unsigned short)it->second->punch));
			}
		}
	}
	
	if(havePIDUpdates) {
		dirtyPIDs=0;
		pidLock.unlock();
	}
	
	// if no changes, skip update altogether -- but if anything has changed, have to take the lock and send the updates
	if(packets.size()>0 || modes.size()>0 || leds.size()>0 || compliances.size()>0 || punches.size()>0) { 
		// send rotation mode changes, need to go before corresponding position command
		writeSyncEntries(os,modes);
		// send positions and speeds
		writeSyncEntries(os,packets);
		// send LED values
		writeSyncEntries(os,leds);
		// send servo controller parameters
		writeSyncEntries(os,torqueToggles);
		writeSyncEntries(os,compliances);
		writeSyncEntries(os,punches);
		// now dump it on the wire
		os.flush();
		if(os) {
			for(modeUpdates_t::const_iterator it=modeUpdates.begin(); it!=modeUpdates.end(); ++it) {
				//printf("\t\t%s Mode switch (%d -> %d)\n",it->first.c_str(),servos[it->first].curRotationMode,it->second);
				servos[it->first].curRotationMode=it->second;
			}
		} else {
			std::cerr << "WARNING: DynamixelDriver couldn't write update, bad output stream" << std::endl;
		}
	}
	memcpy(lastOutputs,curBuf,sizeof(lastOutputs));
	isFirstCheck=false;
}

DynamixelProtocol::SyncWritePosSpeedEntry DynamixelDriver::CommThread::setServo(const servo_iterator& servo, ServoInfo::RotationMode rm, float v, float speed) {
	const int MAX_CMD = servo->second->maxTic;
	const int MIN_CMD = 0;
	const int MAX_SPDCMD = 1023;
	unsigned int servoIdx = servo->second->servoID;
	if(servoIdx>255)
		throw std::runtime_error("DynamixelDriver::setServo, bad servo index!");
	//unsigned int outputIdx = servo->second->output;
	// get output's range in radians
	float outRange = servo->second->maxAngle;
	// get servo's range
	unsigned int servoRange = MAX_CMD - MIN_CMD;
	// get commanded position as percent of range of motion
	float cmd = (v+servo->second->zeroAngle+outRange/2)/outRange;
	// do conversion from radians (output domain) to pulse width (servo domain)
	float pw = cmd*servoRange;
	// account for servo load deflection
	pw -= servo->second->predictedLoad*driver.loadCompensation;
	
	// round to int
	int bpw = static_cast<int>(pw+0.5);
	// check bounds
	if(bpw<MIN_CMD)
		bpw = MIN_CMD;
	else if(bpw>MAX_CMD)
		bpw = MAX_CMD;
	if(servo->second->invertRotation)
		bpw = MAX_CMD - (bpw-MIN_CMD);
	
	// same for speed
	int bpwSpeed;
	if(rm==ServoInfo::CONTINUOUS) {
		if(speed>0)
			speed = speed*servo->second->cmdSpeedSlopeP + servo->second->cmdSpeedOffsetP; // apply calibration
		else if(speed<0)
			speed = speed*servo->second->cmdSpeedSlopeN + servo->second->cmdSpeedOffsetN; // apply calibration
		bpwSpeed = static_cast<int>(speed/outRange*servoRange+.5f);
		if(bpwSpeed<-MAX_SPDCMD)
			bpwSpeed=-MAX_SPDCMD;
		if(bpwSpeed>MAX_SPDCMD)
			bpwSpeed=MAX_SPDCMD;
		if(servo->second->invertRotation)
			bpwSpeed=-bpwSpeed;
		// 10th bit is turn direction
		if(bpwSpeed<0)
			bpwSpeed = -bpwSpeed + 1024;
	} else {
		bpwSpeed = 0; // speed control seems to make it jumpier than just running at maximum... :(
		/*if(speed>0)
		 speed = speed*servo->second->cmdSpeedSlopeP + servo->second->cmdSpeedOffsetP; // apply calibration
		 else if(speed<0)
		 speed = speed*servo->second->cmdSpeedSlopeN + servo->second->cmdSpeedOffsetN; // apply calibration
		 bpwSpeed = abs(static_cast<int>(speed/outRange*servoRange+.5f));
		 if(bpwSpeed<1)
		 bpwSpeed=1;
		 if(bpwSpeed>1023)
		 bpwSpeed=0; // max speed */
	}
	// LOW LEVEL LOGGING:
	//std::cout << get_time() << "\t" << servoIdx << " POS: " << bpw << " Speed: " << bpwSpeed << std::endl;
	
	const float GAMMA = 0.9f;
	float motion = std::abs(servo->second->lastCmd-bpw);
	servo->second->recentCmdMotion*=GAMMA;
	if(motion>servo->second->recentCmdMotion)
		servo->second->recentCmdMotion=motion;
	
	servo->second->lastCmd = bpw;
	return SyncWritePosSpeedEntry(servoIdx,bpw,bpwSpeed);
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
