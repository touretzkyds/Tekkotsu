#ifdef HAVE_SKEWLZONE

#include "SkewlZoneDriver.h"
#include "Shared/MarkScope.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"

using namespace std; 

const unsigned int SkewlZoneDriver::NUM_SERVO;
const int SkewlZoneDriver::UNUSED;
const std::string SkewlZoneDriver::autoRegisterSkewlZoneDriver = DeviceDriver::getRegistry().registerType<SkewlZoneDriver>("SkewlZone");

void SkewlZoneDriver::motionStarting() {
	ASSERTRET(!motionActive,"SkewlZoneDriver::motionStarting, but motionActive is true");
	MotionHook::motionStarting();
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL)
		std::cerr << "SkewlZoneDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
	else
		comm->open();
	motionActive=true;
	commName.addPrimitiveListener(this);
}

bool SkewlZoneDriver::isConnected() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	return (comm!=NULL && comm->isWriteable());
}

void SkewlZoneDriver::motionStopping() {
	ASSERTRET(motionActive,"SkewlZoneDriver::motionStopping, but motionActive is false");
	motionActive=false;
	if(!sensorsActive) // listener count is not recursive, so only remove if we're the last one
		commName.removePrimitiveListener(this);
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm!=NULL)
		comm->close(); // this *is* recursive, so we always close it to match our open() in motionStarting()
	MotionHook::motionStopping();
}

void SkewlZoneDriver::motionCheck(const float outputs[][NumOutputs]) {

	bool	something_changed = false;	// variable to flag changes
	int	RCB3_index = 0;			// RCB3 index for writing to the move vector

	for(unsigned int i=0; i<NUM_SERVO; i++) {
		int idx=servos[i];
		if(idx<0 || static_cast<unsigned int>(idx)>=NumOutputs) {
			if(idx!=UNUSED)
				std::cerr << "Warning: SkewlZone driver mapping servo " << i << " to invalid output index " << idx << std::endl;
			continue; // invalid/unused servo
		}
		
		if(isFirstCheck || lastOutputs[idx]!=outputs[NumFrames-1][idx] ) {
			// for debugging purposes, the servo position is printed out to the command line
			std::cout << "Servo:" << i << "=" << outputs[NumFrames-1][idx] << std::endl;

			something_changed = true;

			// convert the Tekkotsu index to the RCB3 index for writing to the move vector
			RCB3_index = convertIndex(i);

			// if you are not going to hit an undesired ("special") position,
			// and it's not the first check, go ahead with the operation
			// otherwise, do nothing
			if(!bsMotorPosIsSpecial(vecMotorPos.m_fBuf[RCB3_index]) && !isFirstCheck)
			{
				// the speed is hard-coded to 100% to allow Tekkotsu to do all interpolation
				vecMotorMove.m_fBufSpeed[RCB3_index] = 100.0;

				// if the angle for this index should be inverted, invert it
				// otherwise, pass it through
				if(indexInverted(RCB3_index))
				{
					vecMotorMove.m_fBufPos[RCB3_index] = (-1)*(outputs[NumFrames-1][idx]);
				}
				else
				{
					vecMotorMove.m_fBufPos[RCB3_index] = outputs[NumFrames-1][idx];
				}
			}
			else
			{
				// set speed to zero and make the index a NOOP
				vecMotorMove.m_fBufSpeed[RCB3_index] = 0.0;
				vecMotorMove.m_fBufPos[RCB3_index] = BSBOT_MOTOR_POS_NOOP;
			}
		}
	}

	if(!isFirstCheck) {	//All the motors go to zero on startup anyway...so don't move on the first check
		if(something_changed)	// if a position has changed, send the update
		{
			rc = bsBotMoveToPosAllMotors(pClient, &vecMotorMove);
		}
	}

	MotionHook::motionCheck(outputs); // updates lastOutputs and isFirstCheck, we ignore its motionUpdated() call
}


unsigned int SkewlZoneDriver::nextTimestamp() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if(comm==NULL || !comm->isReadable())
		return -1U;
	return static_cast<unsigned int>(lastSensorTime + 1000.f/(*sensorFramerate) + .5f);
}

void SkewlZoneDriver::registerSource() {
	ASSERTRET(!sensorsActive,"SkewlZoneDriver::registerSource, but sensorsActive is true");
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
void SkewlZoneDriver::deregisterSource() {
	ASSERTRET(sensorsActive,"SkewlZoneDriver::deregisterSource, but sensorsActive is false");
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
void SkewlZoneDriver::doUnfreeze() {
	MarkScope sl(poller.getStartLock());
	if(!poller.isStarted()) {
		poller.resetPeriod(1.0/(*sensorFramerate));
		poller.start();
	}
	sensorFramerate->addPrimitiveListener(this);
}
void SkewlZoneDriver::doFreeze() {
	MarkScope sl(poller.getStartLock());
	if(poller.isStarted())
		poller.stop().join();
	sensorFramerate->removePrimitiveListener(this);
}


bool SkewlZoneDriver::advance() {
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
			std::cerr << "SkewlZone position query failed (bad write)" << std::endl;
			return false;
		}
		for(unsigned int i=0; i<NUM_SERVO; i++) {
			int idx=servos[i];
			if(idx<0 || static_cast<unsigned int>(idx)>=NumOutputs)
				continue; // invalid/unused servo
			int check=is.get();
			if(check==-1) {
				cerr << "SkewlZoneDriver: bad read!" << endl;
				return false;
			}
			unsigned int v=(unsigned char)check;
			setOutputValue(idx, getServo(i,v*10));
		}
	}
	stringstream aq,dq;
	bool acnt=0,dcnt=0;
	for(unsigned int i=0; i<NUM_INPUT; i++) {
		int idx=inputs[i];
		if(!buttonMode[i]) {
			if(idx<0 || static_cast<unsigned int>(idx)>=NumSensors) {
				if(idx!=UNUSED)
					std::cerr << "Warning: SkewlZone driver mapping input " << i << " to invalid sensor index " << idx << std::endl;
				continue; // invalid/unused servo
			}
			++acnt;
			aq << 'V' << char('A'+i) << ' ';
		} else {
			if(idx<0 || static_cast<unsigned int>(idx)>=NumButtons) {
				if(idx!=UNUSED)
					std::cerr << "Warning: SkewlZone driver mapping input " << i << " to invalid button index " << idx << std::endl;
				continue; // invalid/unused servo
			}
			++dcnt;
			dq << char('A'+i) << ' ' << char('A'+i) << "L ";
		}
	}
	
	// send both queries now, we can process first response while SSC is processing second query
	try {
		if(dcnt>0)
			os << dq.str() << '\r';
		if(acnt>0)
			os << aq.str() << '\r';
		if(dcnt>0 || acnt>0)
			os << flush;
	} catch(const std::exception&) {}
	if(!os) {
		std::cerr << "SkewlZone sensor query failed (bad write)" << std::endl;
		return false;
	}
	
	// store responses
	if(dcnt>0) {
		for(unsigned int i=0; i<NUM_INPUT; i++) {
			int idx=inputs[i];
			if(idx>=0 && static_cast<unsigned int>(idx)<NumButtons && buttonMode[i]) {
				int check=is.get();
				if(check==-1) {
					cerr << "SkewlZoneDriver: bad read!" << endl;
					return false;
				}
				unsigned char cur=check;
				check=is.get();
				if(check==-1) {
					cerr << "SkewlZoneDriver: bad read!" << endl;
					return false;
				}
				unsigned char latch=check;
				setButtonValue(idx, getDigital(i,cur,latch));
			}
		}
	}
	if(acnt>0) {
		for(unsigned int i=0; i<NUM_INPUT; i++) {
			int idx=inputs[i];
			if(idx>=0 && static_cast<unsigned int>(idx)<NumSensors && !buttonMode[i]) {
				int check=is.get();
				if(check==-1) {
					cerr << "SkewlZoneDriver: bad read!" << endl;
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

void SkewlZoneDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
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
			std::cerr << "SkewlZoneDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
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

void SkewlZoneDriver::setServo(std::ostream& ss, unsigned int servoIdx, float v) {
  
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

float SkewlZoneDriver::getServo(unsigned int servoIdx, unsigned int pw) {
	unsigned int outputIdx = servos[servoIdx];
	// get output's range in radians
	float outRange = outputRanges[outputIdx][MaxRange]-outputRanges[outputIdx][MinRange];
	// get servo's range in pulse width (ms)
	unsigned int servoRange = maxPW[servoIdx]-minPW[servoIdx];
	// do conversion from pulse width (servo domain) to radians (output domain)
	return (pw-minPW[servoIdx])*outRange/servoRange + outputRanges[outputIdx][MinRange];
}

float SkewlZoneDriver::getAnalog(unsigned int /*inputIdx*/, unsigned char s) {
	return s*5.f/256;
}

float SkewlZoneDriver::getDigital(unsigned int /*inputIdx*/, unsigned char cur, unsigned char latch) {
	// The SSC-32 only  latches on 1->0 transitions, so switches should be wired with NC to +5V and NO to GND.
	if (cur=='1')  // switch hasn't been pressed since last query
		return 0;
	// switch has been pressed, so...
	return (latch=='1') ? 0.5f : 1;
}

void SkewlZoneDriver::simplePrintVec(const char *sPrefix, double fBuf[], size_t uCount)
{
  	size_t  i;
  	//BsBotVec_T  *pVec = (BsBotVec_T *)p;

  	printf("%s:", sPrefix);
  	for(i=0; i<uCount; ++i)
  	{
    		if( (i % 8) == 0 )
    		{
      			printf("\n");
    		}
		printf("  %7.3f", fBuf[i]);
  	}
	printf("\n");
}

int SkewlZoneDriver::createSZclient()
{
	if( (pClient = bsClientNew(BsBotTypeKHR2, "GeorgeOfThe_eJungle")) == NULL )
  	{
    		LOGERROR("Failed to create client\n");
    		return SK_EC_EXEC;
  	}
	else
	{
		return SK_EC_OK;
	}
}

int SkewlZoneDriver::connectSZserver()
{
	if((rc = bsServerConnect(pClient, SKProxyIPAddr, SKProxyIPPort)) < 0)
  	{
    		LOGERROR("bsProxy @%s:%d: %s\n",
        	SKProxyIPAddr, SKProxyIPPort, bsStrError(rc));
    		return SK_EC_EXEC;
  	}
        else
        {
                return SK_EC_OK;
        }

}

int SkewlZoneDriver::openRCB3()
{
	if((h=bsReqProxDevOpenRCB3(pClient, serialPath.c_str(), 115200, RCB3ModelJ)) < 0)
  	{
    		LOGERROR("%s: %s\n", serialPath.c_str(), bsStrError(h));
    		return SK_EC_EXEC;
  	}
	else
        {
                return SK_EC_OK;
        }

}

int SkewlZoneDriver::RCB3EStop()
{
	rc = bsBotEStop(pClient);

	return rc;
}

int SkewlZoneDriver::convertIndex(int i)
{
	switch(i)
	{
		case 0:
			return 1;
		case 1:
			return 2;
		case 2:
			return 3;
		case 3:
			return 5;
		case 4:
			return 6;
		case 5:
			return 7;
		case 6:
			return 10;
		case 7:
			return 11;
		case 8:
			return 12;
		case 9:
			return 13;
		case 10:
			return 14;
		case 11:
			return 16;
		case 12:
			return 17;
		case 13:
			return 18;
		case 14:
			return 19;
		case 15:
			return 20;
		case 16:
			return 0;
	}
}

int SkewlZoneDriver::indexInverted(int i)
{
	switch(i)
	{
		case 0:
			return 0;
		case 1:
			return 0;
		case 2:
			return 0;
		case 3:
			return 0;
		case 4:
			return 0;
		case 5:
			return 1;
		case 6:
			return 1;
		case 7:
			return 1;
		case 8:
			return 0;
		case 9:
			return 0;
		case 10:
			return 1;
		case 11:
			return 0;
		case 12:
			return 0;
		case 13:
			return 1;
		case 14:
			return 1;
		case 15:
			return 0;
		case 16:
			return 0;
		case 17:
			return 1;
		case 18:
			return 1;
		case 19:
			return 0;
		case 20:
			return 0;
	}
}

void SkewlZoneDriver::attachRCB3()
{
	bsAttrSetInterface(pClient, BsAttrKeyIFRobotController, h);
}

void SkewlZoneDriver::setSZunits()
{
	bsAttrSetUnits(pClient, BsAttrKeyUnitsMotorPos, units_radians);
  	bsAttrSetUnits(pClient, BsAttrKeyUnitsMotorSpeed, units_percent);
  	//bsAttrSetUnits(pClient, BsAttrKeyUnitsFootPressure, units_kgf);
  	//bsAttrSetUnits(pClient, BsAttrKeyUnitsIMUAccel, units_m_per_s2);
}

void SkewlZoneDriver::printServos()
{
	rc = bsBotReadAllMotorPos(pClient, &vecMotorPos);
  	if( rc == BS_OK )
  	{
    		simplePrintVec("Current servo positions (radians)",
                vecMotorPos.m_fBuf,
        	vecMotorPos.m_uCount);
	}
}

void SkewlZoneDriver::initMotorVec()
{
	vecMotorMove.m_uCount = vecMotorPos.m_uCount;

	//std::cout << "Motor Move Count = " << vecMotorMove.m_uCount << std::endl;

	for(int i=0; i<vecMotorMove.m_uCount; i++) {
		vecMotorMove.m_fBufSpeed[i] = 0.0;
		vecMotorMove.m_fBufPos[i]   = BSBOT_MOTOR_POS_NOOP;
	}
}

/*! @file
 * @brief Implements SkewlZoneDriver, based on SSC32Driver, ported to support SkewlZone interface by Jason Tennyson and Aaron Parker
 */

#endif
