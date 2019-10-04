//-*-c++-*-
#ifndef INCLUDED_CreateDriver_h_
#define INCLUDED_CreateDriver_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DataSource.h"
#include "local/CommPort.h"
#include "Shared/plist.h"
#include "IPC/CallbackThread.h"
#include <iostream>

/*
Specs
Big Endian <---!!!!!


Motors: 5
0 - LD1          - 0.5A
1 - LD0          - 0.5A
2 - LD2          - 1.6A
3 - Right Wheel  - 1.0A
4 - Left Wheel   - 1.0A

Digital Inputs: 4

Digital Outputs: 3

LEDs: 2 (Play + Advanced)

Buttons: 2 (Play + Advanced)

Sensors: 5
0 - bump right
1 - bump left
2 - wheeldrop right
3 - wheeldrop left
4 - wheeldrop caster

Wall Sensor

Cliff Left

Cliff Right

Cliff Front Left

Cliff Front Right

Distance (per wheel) -- capped if not polled frequent enough

Angle (per wheel) -- capped if not polled frequent enough

Voltage

Current

Battery Temperature

Battery Charge

Battery Capacity (not accurate for alkaline)
*/
#ifdef TGT_IS_CREATE2
typedef struct CreateStatus_t{

    unsigned char bumpsWheelDrops;
    unsigned char wall;
    unsigned char cliffLeft;
    unsigned char cliffFrontLeft;
    unsigned char cliffFrontRight;
    unsigned char cliffRight;
    unsigned char virtualWall;
    unsigned char overcurrents;
    unsigned char ir;
    unsigned char buttons;
    float distance;
    float angle;
    unsigned char chargingState;
    unsigned short voltage;
    short current;
    char batteryTemperature;
    unsigned short batteryCharge;
    unsigned short batteryCapacity;
    unsigned short wallSignal;
    unsigned short cliffLeftSignal;
    unsigned short cliffFrontLeftSignal;
    unsigned short cliffFrontRightSignal;
    unsigned short cliffRightSignal;
    unsigned char chargingSourcesAvailable;
    unsigned char oiMode;
    unsigned char songNumber;
    unsigned char songPlay;
    unsigned char streamSize;
    short velocity;
    short radius;
    short rightVelocity;
    short leftVelocity;


    char dirt;
    unsigned char irLeft;
    unsigned char irRight;
    short encoderLeft;
    short encoderRight;
    unsigned char lightBumper;
    unsigned short lightBumperLeft;
    unsigned short lightBumperRight;
    unsigned short lightBumperFrontLeft;
    unsigned short lightBumperFrontRight;
    unsigned short lightBumperCenterLeft;
    unsigned short lightBumperCenterRight;
    short currentLeftMotor;
    short currentRightMotor;
    short currentMainBrush;
    short currentSideBrush;
    unsigned char stasis;
} CreateStatus;
#else
typedef struct CreateStatus_t{
    unsigned short userAnalogInput;
    unsigned char userDigitalInputs;
    unsigned char bumpsWheelDrops;
    unsigned char wall;
    unsigned char cliffLeft;
    unsigned char cliffFrontLeft;
    unsigned char cliffFrontRight;
    unsigned char cliffRight;
    unsigned char virtualWall;
    unsigned char overcurrents;
    unsigned char ir;
    unsigned char buttons;
    short distance;
    short angle;
    unsigned char chargingState;
    unsigned short voltage;
    short current;
    char batteryTemperature;
    unsigned short batteryCharge;
    unsigned short batteryCapacity;
    unsigned short wallSignal;
    unsigned short cliffLeftSignal;
    unsigned short cliffFrontLeftSignal;
    unsigned short cliffFrontRightSignal;
    unsigned short cliffRightSignal;
    unsigned char chargingSourcesAvailable;
    unsigned char oiMode;
    unsigned char songNumber;
    unsigned char songPlay;
    unsigned char streamSize;
    short velocity;
    short radius;
    short rightVelocity;
    short leftVelocity;
} CreateStatus;
#endif
//! description of CreateDriver
class CreateDriver : public virtual DeviceDriver, public MotionHook, public DataSource, public virtual plist::PrimitiveListener {
public:
	explicit CreateDriver(const std::string& name)
		: DeviceDriver(autoRegisterCreateDriver,name), MotionHook(), DataSource(),
		  commName(), poller(&CreateDriver::advance,*this,TimeET(0L),TimeET(1L),true,CallbackPollThread::IGNORE_RETURN),
		  motionActive(false), sensorsActive(false), firstMessage(true),
		  frameNumber(0), globalStatus(), lastDesiredMode(0), packetFailures(0), lastPollTime(0),
                  prevEncoderLeft(0), prevEncoderRight(0), distOffset(0), angleOffset(0)
	{
	  resetStatus(globalStatus);
		addEntry("CommPort",commName,"The name of the comm port where output will be sent");
	}
	virtual ~CreateDriver() {}
	
	virtual std::string getClassName() const { return autoRegisterCreateDriver; }
	
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
	virtual bool advance();
	virtual void registerSource();
	virtual void deregisterSource();
	virtual void enteringRealtime(const plist::Primitive<double>& simTimeScale) { DataSource::enteringRealtime(simTimeScale); }
	virtual void leavingRealtime(bool isFullSpeed) { DataSource::leavingRealtime(isFullSpeed); }
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> commName;
	
protected:
	void doFreeze();
	void doUnfreeze();
	
	bool sendCommand(std::vector<unsigned char> bytes, unsigned int timeout);
	virtual void connect();

	virtual unsigned int readUnsignedChar(std::istream &is, unsigned char &chk);
	virtual unsigned int readUnsignedShort(std::istream &is, unsigned char &chk);
	virtual int readPacket(std::istream &is, const char &type, CreateStatus &createStatus, unsigned char &chk);
	bool attemptPacketRead(std::istream &is, CreateStatus &createStatus);

	bool attemptStreamRead(std::istream &is, CreateStatus &createStatus); //!< Not used at present

	void resetStatus(CreateStatus &status);
	void mergeStatus(CreateStatus &oldStatus, CreateStatus &newStatus);

	//! forwards call to DataSource::providingOutput() if the index is valid
	void provideOutput(unsigned int idx) { if(idx<NumOutputs) providingOutput(idx); }
	//! forwards call to DataSource::ignoringOutput() if the index is valid
	void ignoreOutput(unsigned int idx) { if(idx<NumOutputs) ignoringOutput(idx); }
	
	//! converts the value @a s from specified input's signal to voltage
	virtual float getAnalog(unsigned int inputIdx, unsigned char s);
	//! converts the value @a cur and @a latch to the output format (0 if low, 0.5 if high but has been low, 1 if consistent high)
	virtual float getDigital(unsigned int inputIdx, unsigned char cur, unsigned char latch);
	
	//! allows LEDs to flicker at various frequencies to emulate having linear brightness control instead of boolean control
	inline bool calcLEDValue(unsigned int i,float x) {
		if(x<=0.0) {
			ledActivation[i]*=.9f; //decay activation... resets to keeps LEDs in sync, looks a little better
			return false;
		} else if(x>=1.0) {
			return true;
		} else {
			float x3 = x*x*x;
			x = .25f*x3*x3 + .75f*x; // fancy but unscientific equation to "gamma correct" - we can see a single pulse better than a single flicker - after image and all that
			ledActivation[i]+=x;
			if(ledActivation[i]>=1) {
				ledActivation[i]-=1;
				return true;
			} else {
				return false;
			}
		}
	}
	float ledActivation[NumLEDs]; //!< used to track partial LED activation (see calcLEDValue())
	
	CallbackPollThread poller;

	bool motionActive;
	bool sensorsActive;
        bool firstMessage;
	unsigned int frameNumber;
	CreateStatus globalStatus;
  
	unsigned char lastDesiredMode;
	unsigned int packetFailures;
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterCreateDriver;

        static const int PACKET_LENGTH = 52+4;

  unsigned int lastPollTime;
  short prevEncoderLeft, prevEncoderRight;
  float distOffset, angleOffset;
  static const unsigned int pollInterval = 100; // msec; was 250
};

/*! @file
 * @brief 
 * @author Benson Tsai (btsai)
 * @author Ethan Tira-Thompson (ejt)
 * @author Alex Grubb (agrub)
 */

#endif
