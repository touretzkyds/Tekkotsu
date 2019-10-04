#ifndef INCLUDED_KobukiDriver_h_
#define INCLUDED_KobukiDriver_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DataSource.h"
#include "local/CommPort.h"
#include "Shared/plist.h"
#include "IPC/CallbackThread.h"
#include <iostream>

namespace Kobuki {

struct CoreSensor {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char timestamp[2];		 		// 2
		unsigned char bumper;					 		// 1
		unsigned char wheelDrop;			 		// 1
		unsigned char cliff;					 		// 1
		unsigned char leftEncoder[2];	 		// 2
		unsigned char rightEncoder[2]; 		// 2
		unsigned char leftPwm;				 		// 1
		unsigned char rightPwm;				 		// 1
		unsigned char buttons;				 		// 1
		unsigned char charger;				 		// 1
		unsigned char battery;				 		// 1
		unsigned char overCurrent;		 		// 1
	};

	struct DockInfraRed {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char docking[3];			 		// 3
	};

	struct Inertia {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char angle[2];				 		// 2
		unsigned char angleRate[2];		 		// 2
		unsigned char acc[3];					 		// 3
	};

	struct Cliff {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char bottom0[2];					// 2
		unsigned char bottom1[2];					// 2
		unsigned char bottom2[2];					// 2
	};

	struct Current {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char current[2];			 		// 2
	};
	
	struct ThreeAxisGyro {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char frameId;				 		// 1
		unsigned char followedDataLength;	// 1
		unsigned char parameters[0];			// 12 or 18
	};

	struct GpInput {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char digitalInput[2];		// 2
		unsigned char analogInput[14];
	};
	
	struct KobukiSensors {
		//CoreSensors
		unsigned short int timeStamp;
		unsigned char bumper;
		unsigned char wheelDrop;
		unsigned char cliff;
	 	short int leftEncoder;
		short int rightEncoder;
		unsigned char leftPwm;
		unsigned char rightPwm;
		unsigned char buttons;
		unsigned char charger;
		unsigned char battery;
		unsigned char overCurrent;
		
		//Dock Infra Red
		unsigned char docking[3];
		
		//Inertia
		short int angle;
		short int angleRate;
		unsigned char acc[3];
		
		//Cliff
		short int bottom[3];
		
		//Current
		unsigned char current[2];

		
		//Three Axis Gyro
		unsigned char frameId;
		unsigned char followedDataLength;
		short parameters[0];
				
		//GpInput
		short int digitalInput;
		short int analogInput[7];
	};
	
	
	enum State { 
	  lookingForHeader0, 
	  lookingForHeader1, 
	  waitingForPacket, 
	  gotPacket
	};
	
	enum Sensor { 
		coreSensor = 1,
		dockInfraRed = 3,
		inertia = 4,
		cliff = 5,
		current = 6,
		threeAxisGyro = 13,
		gpInput = 16,
	};

	struct BaseControl {
			unsigned char speed[2];					//2
			unsigned char radius[2];				//2
		};


	struct SetDigitalOut {
			unsigned char gpOut;
		};

	struct KobukiCommand {
			unsigned int Length;
			unsigned int commandData;
			unsigned int commandDataSize;			


			//BaseControl
			unsigned char speedHigh;
			unsigned char speedLow;
			unsigned char radiusHigh;
			unsigned char radiusLow;
	};

}

class KobukiDriver : public virtual DeviceDriver, public MotionHook, public DataSource, public virtual plist::PrimitiveListener {
public:
	explicit KobukiDriver(const std::string& name)
		: DeviceDriver(autoRegisterKobukiDriver,name), MotionHook(), DataSource(),
		  commName(), poller(&KobukiDriver::advance,*this,TimeET(0L),TimeET(1L),true,CallbackPollThread::IGNORE_RETURN),
		  motionActive(false), sensorsActive(false),
		  frameNumber(0), lastDesiredMode(0), lastPollTime(0)
	{
		addEntry("CommPort",commName,"The name of the comm port where output will be sent");
	}
	virtual ~KobukiDriver() {}
	
	virtual std::string getClassName() const { return autoRegisterKobukiDriver; }
	
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
	
	virtual bool readPacket(std::istream &is);
	virtual void packetParser(unsigned char packet[],const unsigned int packetLenght);
	
	virtual void enteringRealtime(const plist::Primitive<double>& simTimeScale) { DataSource::enteringRealtime(simTimeScale); }
	virtual void leavingRealtime(bool isFullSpeed) { DataSource::leavingRealtime(isFullSpeed); }
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	plist::Primitive<std::string> commName;
	
protected:
	void doFreeze();
	void doUnfreeze();
	
	virtual bool sendCommand(std::vector<unsigned char> bytes, unsigned int timeout);
	virtual void connect();

	
	
	float ledActivation[NumLEDs]; //!< used to track partial LED activation (see calcLEDValue())
	
	CallbackPollThread poller;

	bool motionActive;
	bool sensorsActive;
	unsigned int frameNumber;
	
	unsigned char lastDesiredMode;
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterKobukiDriver;

  static const int PACKET_LENGTH = 52+4;

  unsigned int lastPollTime;
  static const unsigned int pollInterval = 250; // msec
};

#endif
