//-*-c++-*-
#ifndef INCLUDED_SkewlZoneDriver_h_
#define INCLUDED_SkewlZoneDriver_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DataSource.h"
#include "local/CommPort.h"
#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "IPC/CallbackThread.h"
#include "Shared/RobotInfo.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
// Files and declarations needed by Botsense

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <libgen.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "rnr/i2c.h"
#include "rnr/serdev.h"

#include "rcb3/rcb3prot.h"
#include "rcb3/rcb3lib.h"

#include "botsense/szFoot.h"
#include "botsense/szIMU.h"
#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

//#include "version.h"

//
// Simple SkewlZone BotSense Application Exit Codes
//
#define SK_EC_OK        0                 ///< success exit code
#define SK_EC_USAGE     2                 ///< usage error exit code
#define SK_EC_EXEC      4                 ///< execution error exit code

//
// The command and command-line options.
//
//static char *OptsIFSz         = "/dev/i2c-0";       ///< SkewlZone \h_i2c device

//
// State
//
// TODO should this be a configuration parameter?
static char    *SKProxyIPAddr     = "localhost";
static int      SKProxyIPPort     = BSPROXY_LISTEN_PORT_DFT;

// End of Botsense declarations
////////////////////////////////////////////////////////////////////////////////

//! description of SkewlZoneDriver
class SkewlZoneDriver : public virtual DeviceDriver, public MotionHook, public DataSource, public virtual plist::PrimitiveListener {
public:
	static const unsigned int NUM_SERVO=NumOutputs;
	static const unsigned int NUM_INPUT=NumSensors;
	static const int UNUSED=plist::OutputSelector::UNUSED;

	////////////////////////////////////////////////////////////////////////////////
	// more botsense stuff

	BsLibClient_P           pClient;      // client instance
	int                     h;            // proxied device handle
	BsBotMotorPosVec_T      vecMotorPos;  // vector of motor positions
	BsBotMotorSpeedPosVec_T vecMotorMove; // vector of motor speed-position tuples
	int                     index;        // working index
	int                     rc;           // return code

	// End of more botsense stuff
	////////////////////////////////////////////////////////////////////////////////

	
	explicit SkewlZoneDriver(const std::string& name)
		: DeviceDriver(autoRegisterSkewlZoneDriver,name), MotionHook(), DataSource(),
		serialPath("/dev/ttyUSB0"), servos(NUM_SERVO,UNUSED), inputs(NUM_INPUT,UNUSED),
		minPW(NUM_SERVO,500), maxPW(NUM_SERVO,2500), buttonMode(NUM_INPUT,false),
	sparse(false), commName(), queryServos(false), poller(&SkewlZoneDriver::advance,*this,TimeET(0L),TimeET(1.0/(*sensorFramerate)),true,CallbackPollThread::IGNORE_RETURN),
		motionActive(false), sensorsActive(false), lastSensorTime(), frameNumber(0)
	{
		for(unsigned int i=0; i<NumOutputs && i<NUM_SERVO; ++i)
			servos[i]=i;
		for(unsigned int i=0; i<NumSensors && i<NUM_INPUT; ++i)
			inputs[i]=i;
		addEntry("SerialPath",serialPath,"Path to connect to botsense using a serial device");
		addEntry("OutputMap",servos,"For each of the SkewlZone's servo pins, lists the output index it should take its values from; -1 to mark unused");
		addEntry("InputMap",inputs,"For each of the SkewlZone's input pins, lists the sensor index it should send its value to; -1 to mark unused");
		addEntry("MinPulseWidth",minPW,"The low end of the servo's legal pulse width range (may correspond to unreachable position, use RobotInfo's outputRange[] to limit motion, not this)");	
		addEntry("MaxPulseWidth",maxPW,"The high end of the servo's legal pulse width range (may correspond to unreachable position, use RobotInfo's outputRange[] to limit motion, not this)");	
		addEntry("ButtonMode",buttonMode,"Controls interpretation of the input pin.\nFalse means directly measure voltage, true means test for high (1),\nhigh now but low was detected in interval (0.5), or low (0).\nButton mode implies interpreting inputMap value as a button index instead of sensor index.");
		addEntry("SparseUpdates",sparse,"If true, only send servo positions to SkewlZone when they change, instead of all servos on every update (don't use a lossy transport like UDP if you turn this on!)");
		addEntry("CommPort",commName,"The name of the comm port where output will be sent");
		addEntry("QueryServos",queryServos,"If set to true, will attempt to query the servo positions with each sensor update.\nThis may decrease the sampling frequency");



		////////////////////////////////////////////////////////////////////
		// call the SZ initialization functions
		
		// TODO this should probably be moved to an init function which is called
		//   by the first of motionStarting or registerSource, and then set a flag
		//   so the second doesn't re-initialize.  Then to be matched by de-initialization
		//   in the last called of motionStopping and deregisterSource.  See
		//   look how the commport stuff is handled and replace that since
		//   this driver internalizes its communication.

		// create the SZ client
        	if(createSZclient() != SK_EC_EXEC) {
			// if that is successful, connect it
			if(connectSZserver() != SK_EC_EXEC) {
				// if that is successful, open an RCB3 connection
				if(openRCB3() != SK_EC_EXEC) {
					// if that is successful, attach the RCB3, set units, apply E-stop, and print initial positions
					attachRCB3();
					setSZunits();
					if(RCB3EStop() != SK_EC_EXEC) {
						printf("E Stop Applied to SkewlZone!\n");
					}
					printServos();
					initMotorVec();
				}
			}
		}

		// initialize the motor move vector with what was read into the position vector
		for (unsigned int i=0; i<vecMotorPos.m_uCount; i++) {
			vecMotorMove.m_fBufSpeed[i] = 0.0;
			vecMotorMove.m_fBufPos[i] = vecMotorPos.m_fBuf[i];
		}
			
		// end of SZ initialization
		////////////////////////////////////////////////////////////////////

	}
	virtual ~SkewlZoneDriver() {}
	
	virtual std::string getClassName() const { return autoRegisterSkewlZoneDriver; }
	
	virtual MotionHook* getMotionSink() { return dynamic_cast<MotionHook*>(this); }
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"]=dynamic_cast<DataSource*>(this);
	}
	
	virtual void motionStarting();
	virtual bool isConnected();
	virtual void motionStopping();
	virtual void motionCheck(const float outputs[][NumOutputs]);
	
	virtual unsigned int nextTimestamp();
	virtual const std::string& nextName() { return instanceName; }
	virtual void registerSource();
	virtual void deregisterSource();
	virtual void enteringRealtime(const plist::Primitive<double>& simTimeScale) { DataSource::enteringRealtime(simTimeScale); }
	virtual void leavingRealtime(bool isFullSpeed) { DataSource::leavingRealtime(isFullSpeed); }
	virtual bool advance();
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> serialPath; //! TODO this should replace commName (see where commName is used to allow runtime reconfiguration)
	plist::ArrayOf<plist::OutputSelector> servos;
	plist::ArrayOf<plist::Primitive<int> > inputs;
	plist::ArrayOf<plist::Primitive<unsigned int> > minPW;
	plist::ArrayOf<plist::Primitive<unsigned int> > maxPW;
	plist::ArrayOf<plist::Primitive<bool> > buttonMode;
	plist::Primitive<bool> sparse;
	plist::Primitive<std::string> commName; //!< TODO this should be replaced by serialPath
	plist::Primitive<bool> queryServos; //!< TODO this might not make sense for this driver?

	///////////////////////////////////////////////////////////////////////////////////
	// Botsense function declarations

	// prints a character prefix followed by a vector, typically used for current motor position readings
	virtual void simplePrintVec(const char *sPrefix, double fBuf[], size_t uCount);
	// creates a new SkewlZone client
	virtual int createSZclient();
	// attaches a SkewlZone client to the botsense server
	virtual int connectSZserver();
	// opens a connection to the RCB3 controller board
	virtual int openRCB3();
	// attach the RCB3 connection
	virtual void attachRCB3();
	// emergency stop for the RCB3 controller board
	virtual int RCB3EStop();
	// converts the Tekkotsu index of the servo to the corresponding RCB3 port
	virtual int convertIndex(int index);
	// returns 0 or 1 to check whether or not the passed index value should have its angle
	// inverted to maintain mirrored movement about the center of the robot
	virtual int indexInverted(int index);
	// defines units of operation (radians, degrees, percent, m/s2)
	virtual void setSZunits();
	// uses the vector print function to print the last servo position sent
	virtual void printServos();
	// initializes the motor vector to a NOOP
	virtual void initMotorVec();

	// End of Botsense function declarations
	///////////////////////////////////////////////////////////////////////////////////

protected:
	void doFreeze();
	void doUnfreeze();
	
	//! forwards call to DataSource::providingOutput() if the index is valid
	void provideOutput(unsigned int idx) { if(idx<NumOutputs) providingOutput(idx); }
	//! forwards call to DataSource::ignoringOutput() if the index is valid
	void ignoreOutput(unsigned int idx) { if(idx<NumOutputs) ignoringOutput(idx); }
	
	//! converts the value @a v from radians into the specified servo's pulse width range
	virtual void setServo(std::ostream& ss, unsigned int servoIdx, float v);
	//! converts the value @a pw from specified servo's pulse width range into radians
	virtual float getServo(unsigned int servoIdx, unsigned int pw);
	//! converts the value @a s from specified input's signal to voltage
	virtual float getAnalog(unsigned int inputIdx, unsigned char s);
	//! converts the value @a cur and @a latch to the output format (0 if low, 0.5 if high but has been low, 1 if consistent high)
	virtual float getDigital(unsigned int inputIdx, unsigned char cur, unsigned char latch);
	//int modeCheck(const float outputs[][NumOutputs]);
	float angleCalibration(unsigned int servoIdx, int mode, float v);
		
	CallbackPollThread poller;
        
	
	bool motionActive;
	bool sensorsActive;
	unsigned int lastSensorTime;
	unsigned int frameNumber;
	
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterSkewlZoneDriver;
};

/*! @file
 * @brief Implements SkewlZoneDriver, based on SSC32Driver, ported to support SkewlZone interface by Jason Tennyson and Aaron Parker
 */

#endif
