#include "SSC32Driver.h"
#include "Shared/MarkScope.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"

using namespace std; 

const unsigned int SSC32Driver::NUM_SERVO;
const int SSC32Driver::UNUSED;
const std::string SSC32Driver::autoRegisterSSC32Driver = DeviceDriver::getRegistry().registerType<SSC32Driver>("SSC32");

void SSC32Driver::motionStarting() {
	ASSERTRET(!motionActive,"SSC32Driver::motionStarting, but motionActive is true");
	MotionHook::motionStarting();
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL)
		std::cerr << "SSC32Driver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
	else
		comm->open();
	motionActive=true;
	commName.addPrimitiveListener(this);
	
	for(unsigned i = 0; i < NUM_SERVO; i++) {
		if(killSignalDelayed[i]) {
			sparse=true;
			queryServos=true;
		}
	}
}

bool SSC32Driver::isConnected() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	return (comm!=NULL && comm->isWriteable());
}

void SSC32Driver::motionStopping() {
	ASSERTRET(motionActive,"SSC32Driver::motionStopping, but motionActive is false");
	motionActive=false;
	if(!sensorsActive) // listener count is not recursive, so only remove if we're the last one
		commName.removePrimitiveListener(this);
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->close(); // this *is* recursive, so we always close it to match our open() in motionStarting()
	MotionHook::motionStopping();
}

void SSC32Driver::motionCheck(const float outputs[][NumOutputs]) {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isWriteable())
		return;
	
	stringstream ss;
	for(unsigned int i=0; i<NUM_SERVO; i++) {
		int idx=servos[i];
		if(idx<0 || static_cast<unsigned int>(idx)>=NumOutputs) {
			if(idx!=UNUSED)
				std::cerr << "Warning: SSC32 driver mapping servo " << i << " to invalid output index " << idx << std::endl;
			continue; // invalid/unused servo
		}
		
		//std::cout << "Test : " << i << " , " << outputs[NumFrames-1][idx] << std::endl;
		
		//int mode = modeCheck(outputs); || ((mode == 1) && (idx == 10))
		if(isFirstCheck || !sparse || lastOutputs[idx]!=outputs[NumFrames-1][idx] ) {
			/*std::cout << "Mode : " <<  mode << ", Servo: " << i << ", " << outputs[NumFrames-1][idx] << ", " << angleCalibration(i, mode, outputs[NumFrames-1][idx]) << ", " << lastOutputs[idx] << std::endl;*/
			setServo(ss, i, outputs[NumFrames-1][idx]);
		}
	}
	string s=ss.str();
	if(s.size()>0) { // if sparse and no changes, skip update altogether
		Thread::Lock& l = comm->getLock();
		unsigned int t=get_time();
		// keep trying to get the lock, sleeping 1 ms each time, until 3/4 the frame time is gone (then give up)
		unsigned int dt = static_cast<unsigned int>(NumFrames*FrameTime/((getTimeScale()>0)?getTimeScale():1.f));
		unsigned int giveup = t+dt*3/4;
		t+=dt;
		while(!l.trylock()) {
			if(get_time()>=giveup) {
				if(MotionHook::verbose>0)
					cerr << "Dropping SSC32 motion update: couldn't get lock on comm port" << endl;
				return;
			}
			usleep(1000);
		}
		MarkScope autolock(l); l.unlock(); //transfer lock to MarkScope
		std::ostream os(&comm->getWriteStreambuf());

		// request exception on badbit so we can thread_cancel read under linux
		// (otherwise we get FATAL: exception not rethrown / Abort error)
		os.exceptions(ios_base::badbit);
	
		unsigned int curt = get_time();
		if(curt>=t) // too late!
			os << s << '\r' << flush;
		else {
			dt=t-curt;
			os << s << 'T' << dt << '\r' << flush; // indicate time until next update
		}
	}
	
	MotionHook::motionCheck(outputs); // updates lastOutputs and isFirstCheck, we ignore its motionUpdated() call
}


unsigned int SSC32Driver::nextTimestamp() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable())
		return -1U;
	return static_cast<unsigned int>(lastSensorTime + 1000.f/(*sensorFramerate) + .5f);
}

void SSC32Driver::registerSource() {
	ASSERTRET(!sensorsActive,"SSC32Driver::registerSource, but sensorsActive is true");
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->open();
	queryServos.addPrimitiveListener(this);
	if(queryServos) {
		for(unsigned int i=0; i<NUM_SERVO; i++) {
			provideOutput(servos[i]);
			servos[i].addPrimitiveListener(this);
		}
	}
	sensorsActive=true;
	commName.addPrimitiveListener(this);
}
void SSC32Driver::deregisterSource() {
	ASSERTRET(sensorsActive,"SSC32Driver::deregisterSource, but sensorsActive is false");
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->close();
	if(queryServos) {
		for(unsigned int i=0; i<NUM_SERVO; ++i) {
			servos[i].removePrimitiveListener(this);
			ignoreOutput(servos[i]);
		}
	}
	queryServos.removePrimitiveListener(this);
	sensorsActive=false;
	if(!motionActive) // listener count is not recursive, so only remove if we're the last one
		commName.removePrimitiveListener(this);
}
void SSC32Driver::doUnfreeze() {
	MarkScope sl(poller.getStartLock());
	if(!poller.isStarted()) {
		poller.resetPeriod(1.0/(*sensorFramerate));
		poller.start();
	}
	sensorFramerate->addPrimitiveListener(this);
}
void SSC32Driver::doFreeze() {
	MarkScope sl(poller.getStartLock());
	if(poller.isStarted())
		poller.stop().join();
	sensorFramerate->removePrimitiveListener(this);
}


bool SSC32Driver::advance() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
		return false;
	
	MarkScope commlock(comm->getLock());
	std::ostream os(&comm->getWriteStreambuf());
	std::istream is(&comm->getReadStreambuf());
	
	// request exception on badbit so we can thread_cancel read under linux
	// (otherwise we get FATAL: exception not rethrown / Abort error)
	os.exceptions(ios_base::badbit);
	is.exceptions(ios_base::badbit);
	
	unsigned T = get_time();
	
	MarkScope writelock(getSensorWriteLock());
	// generally, bad idea to request servo position before position has been sent
	// But if the arm is still powered, it'll know it's position.  If the user has set WaitForSensors,
	// then DataSource::requiresFirstSensor will be set, otherwise we skip queries until we've sent
	// a servo position (and thus set the isFirstCheck flag
	if((!isFirstCheck || DataSource::requiresFirstSensor) && queryServos) {
		// check joint positions
		stringstream q;
		for(unsigned int i=0; i<NUM_SERVO; i++) {
			int idx=servos[i];
			if(idx<0 || static_cast<unsigned int>(idx)>=NumOutputs)
				continue; // invalid/unused servo
			q << "QP " << i << ' ';
		}
		try {
			os << q.rdbuf() << '\r' << flush;
		} catch(const std::exception&) {}
		if(!os) {
			std::cerr << "SSC32 position query failed (bad write)" << std::endl;
			return false;
		}
		for(unsigned int i=0; i<NUM_SERVO; i++) {
			int idx=servos[i];
			if(idx<0 || static_cast<unsigned int>(idx)>=NumOutputs)
				continue; // invalid/unused servo
			int check=is.get();
			if(check==-1) {
				cerr << "SSC32Driver: bad read!" << endl;
				return false;
			}
			unsigned int v=(unsigned char)check;
			if(killSignalDelayed[i]) {
				float val = getServo(i,v*10);
				float prev = getOutputValue(idx);
				
				if(val != prev) {
					timeLastChanged[i] = T;
				} else if(timeLastChanged[i] != 0 && T - timeLastChanged[i] > 1000) {
					os<<"#"<<i<<"L\r";
					os.flush();
					timeLastChanged[i] = 0;
				}
			}
			setOutputValue(idx, getServo(i,v*10));
		}
	}
	stringstream aq,dq;
	bool acnt=false,dcnt=false;
	for(unsigned int i=0; i<NUM_INPUT; i++) {
		int idx=inputs[i];
		if(!buttonMode[i]) {
			if(idx<0 || static_cast<unsigned int>(idx)>=NumSensors) {
				if(idx!=UNUSED)
					std::cerr << "Warning: SSC32 driver mapping input " << i << " to invalid sensor index " << idx << std::endl;
				continue; // invalid/unused servo
			}
			acnt=true;
			aq << 'V' << char('A'+i) << ' ';
		} else {
			if(idx<0 || static_cast<unsigned int>(idx)>=NumButtons) {
				if(idx!=UNUSED)
					std::cerr << "Warning: SSC32 driver mapping input " << i << " to invalid button index " << idx << std::endl;
				continue; // invalid/unused servo
			}
			dcnt=true;
			dq << char('A'+i) << ' ' << char('A'+i) << "L ";
		}
	}
	
	// send both queries now, we can process first response while SSC is processing second query
	try {
		if(dcnt)
			os << dq.str() << '\r';
		if(acnt)
			os << aq.str() << '\r';
		if(dcnt || acnt)
			os << flush;
	} catch(const std::exception&) {}
	if(!os) {
		std::cerr << "SSC32 sensor query failed (bad write)" << std::endl;
		return false;
	}
	
	// store responses
	if(dcnt) {
		for(unsigned int i=0; i<NUM_INPUT; i++) {
			int idx=inputs[i];
			if(idx>=0 && static_cast<unsigned int>(idx)<NumButtons && buttonMode[i]) {
				int check=is.get();
				if(check==-1) {
					cerr << "SSC32Driver: bad read!" << endl;
					return false;
				}
				unsigned char cur=check;
				check=is.get();
				if(check==-1) {
					cerr << "SSC32Driver: bad read!" << endl;
					return false;
				}
				unsigned char latch=check;
				setButtonValue(idx, getDigital(i,cur,latch));
			}
		}
	}
	if(acnt) {
		for(unsigned int i=0; i<NUM_INPUT; i++) {
			int idx=inputs[i];
			if(idx>=0 && static_cast<unsigned int>(idx)<NumSensors && !buttonMode[i]) {
				int check=is.get();
				if(check==-1) {
					cerr << "SSC32Driver: bad read!" << endl;
					return false;
				}
				setSensorValue(idx, getAnalog(i,check));
			}
		}
	}
	lastSensorTime=get_time();
	++frameNumber;
	return true;
}

void SSC32Driver::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&commName) {
		// if here, then motionStarted or setDataSourceThread has been called, thus when commName changes,
		// need to close old one and reopen new one
		if(poller.isStarted())
			poller.stop().join();
		
		CommPort * comm = CommPort::getRegistry().getInstance(commName.getPreviousValue());
		if(comm!=NULL) {
			// close each of our old references
			if(sensorsActive)
				comm->close();
			if(motionActive)
				comm->close();
		}
		comm = CommPort::getRegistry().getInstance(commName);
		if(comm==NULL) {
			std::cerr << "SSC32Driver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
		} else {
			// open each of our new references
			if(sensorsActive)
				comm->open();
			if(motionActive)
				comm->open();
		}
		
		if(getTimeScale()>0) {
			poller.resetPeriod(1.0/(*sensorFramerate));
			poller.start();
		}			
	} else if(&pl==&queryServos) {
		// if here, LoadDataThread has been assigned, need to update providing/ignoring outputs
		// (and maintain listeners for individual servos while providing)
		if(queryServos) {
			for(unsigned int i=0; i<NUM_SERVO; i++) {
				provideOutput(servos[i]);
				servos[i].addPrimitiveListener(this);
			}
		} else {
			for(unsigned int i=0; i<NUM_SERVO; ++i) {
				servos[i].removePrimitiveListener(this);
				ignoreOutput(servos[i]);
			}
		}
	} else if(&pl==sensorFramerate) {
		poller.resetPeriod(1.0/(*sensorFramerate));
	} else {
		// check if it's one of the individual servos... if it is, means we're providing servo feedback,
		// need to call providingOutput/ignoringOutput as appropriate
		for(unsigned int i=0; i<NUM_SERVO; ++i) {
			if(&pl==&servos[i]) {
				ignoreOutput(servos[i].getPreviousValue());
				provideOutput(servos[i]);
				return; // found it, DON'T fall through to error message below...
			}
		}
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

void SSC32Driver::setServo(std::ostream& ss, unsigned int servoIdx, float v) {
  
  	unsigned int outputIdx = servos[servoIdx];
	// get output's range in radians
	float outRange = outputRanges[outputIdx][MaxRange]-outputRanges[outputIdx][MinRange];
	// get servo's range in pulse width (ms)
	unsigned int servoRange = maxPW[servoIdx]-minPW[servoIdx];
	// get commanded position as percent of range of motion
	float cmd = (v-outputRanges[outputIdx][MinRange])/outRange;
	// flip commanded position -- map positive (high) rotation to low pulse width
	// this is so if you mount a servo "up", coordinate system will work correctly
#ifdef TGT_HAS_LEDS
	if(outputIdx<LEDOffset || outputIdx>=LEDOffset+NumLEDs) // only flip non-LEDs though
		cmd=1-cmd;
#endif
	// do conversion from radians (output domain) to pulse width (servo domain)
	float pw = cmd*servoRange+minPW[servoIdx];
	if(pw<0)
		pw=0;
	// round to int
	unsigned int bpw = static_cast<unsigned int>(pw+0.5);
	// check bounds
	if(bpw<minPW[servoIdx])
		bpw=minPW[servoIdx];
	if(bpw>maxPW[servoIdx])
		bpw=maxPW[servoIdx];
	// send to output buffer
	ss << '#' << servoIdx << " P" << bpw << ' ';
}

float SSC32Driver::getServo(unsigned int servoIdx, unsigned int pw) {
	unsigned int outputIdx = servos[servoIdx];
	// get output's range in radians
	float outRange = outputRanges[outputIdx][MaxRange]-outputRanges[outputIdx][MinRange];
	// get servo's range in pulse width (ms)
	unsigned int servoRange = maxPW[servoIdx]-minPW[servoIdx];
	// do conversion from pulse width (servo domain) to radians (output domain)
	return (pw-minPW[servoIdx])*outRange/servoRange + outputRanges[outputIdx][MinRange];
}

float SSC32Driver::getAnalog(unsigned int /*inputIdx*/, unsigned char s) {
	return s*5.f/256;
}

float SSC32Driver::getDigital(unsigned int /*inputIdx*/, unsigned char cur, unsigned char latch) {
	// The SSC-32 only  latches on 1->0 transitions, so switches should be wired with NC to +5V and NO to GND.
	if (cur=='1')  // switch hasn't been pressed since last query
		return 0;
	// switch has been pressed, so...
	return (latch=='1') ? 0.5f : 1;
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
