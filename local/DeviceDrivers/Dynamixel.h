//-*-c++-*-
#ifndef INCLUDED_Dynamixel_h_
#define INCLUDED_Dynamixel_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DataSource.h"
#include "local/CommPort.h"
#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Shared/get_time.h"
#include "IPC/Thread.h"
#include "IPC/FailsafeThread.h"
#include "IPC/DriverMessaging.h"
#include "DynamixelProtocol.h"
#include <iostream>
#include <sstream>
#include <iomanip>

//! description of Dynamixel
class DynamixelDriver : public virtual DeviceDriver, public virtual DriverMessaging::Listener, public MotionHook, public DataSource, public virtual plist::PrimitiveListener {
public:
	static const int START_SERVO_ID = 1; //!< bioloid kits start the id count at 1
	static const unsigned int UNUSED = plist::OutputSelector::UNUSED;
	
	//! Offsets into OutputPIDs for MX series servos
	enum ServoParam_t {
		DYNAMIXEL_P = 0, //!< P parameter for PID control
		DYNAMIXEL_I, //!< I parameter for PID control
		DYNAMIXEL_D //!< D parameter for PID control
	};

	//! Offsets into OuputPIDs, since Dynamixel AX and RX servos don't actually use PID control, but a different set of parameters
	enum OldServoParam_t {
		DYNAMIXEL_SLOPE=0, //!< compliance slope, the proportional control (P in PID) 
		DYNAMIXEL_PUNCH, //!< punch, a constant added to the slope once error exceeds compliance margin
		DYNAMIXEL_MARGIN //!< compliance margin, the amount of error to tolerate before triggering a torque response
	};

	explicit DynamixelDriver(const std::string& name)
		: DeviceDriver(autoRegisterDynamixelDriver,name), DriverMessaging::Listener(), MotionHook(), DataSource(),
		servos(), commName(), loadCompensation(28.f), commLatency(16), numPoll(3), responseTime(4000), servoIDSync(), 
		commThread(servos,commName,*this), motionActive(false), sensorsActive(false), lastSensor()
	{
		// redo these lookups, they aren't set reliably otherwise due to static initialization order issues
		VOLTAGE_SENSOR_OFFSET = capabilities.findSensorOffset("PowerVoltage");
		TEMP_SENSOR_OFFSET = capabilities.findSensorOffset("PowerThermo");
		
		addEntry("Servos",servos,"Maps servo IDs to Tekkotsu output offsets, use command line new/delete commands to add/remove mappings.");
		addEntry("CommPort",commName,"Name of the CommPort to use, generally a SerialCommPort with direct binary TTL communication with the servos");
		addEntry("LoadCompensation",loadCompensation,"Ratio of load to deflection, if non-zero will use LoadPrediction messages and attempt to counter anticipated loads");
		addEntry("CommLatency",commLatency,"The delay (in MILLIseconds, ms) to get a response from the servos due to buffering in the USB-to-TTL interface.");
		addEntry("NumPoll",numPoll,"Number of sensor queries to send before reading responses.  This lets us front-load the queries so we can get multiple responses per read buffer flush period");
		addEntry("ResponseTime",responseTime,"The amount of time (in MICROseconds, µs) to wait between sending a sensor query and sending another when front-loading the buffer.\nIf this is too short, the next query will collide with the previous response.");
		
		for(unsigned int i=0; i<NumPIDJoints; ++i) {
			std::stringstream bio_id;
			bio_id << std::setw(3) << std::setfill('0') << (i+START_SERVO_ID);
			servos[bio_id.str()]=ServoInfo(i+START_SERVO_ID);
		}
		servos.addCollectionListener(&servoIDSync);
		DriverMessaging::addListener(this,DriverMessaging::LoadPrediction::NAME);
		DriverMessaging::addListener(this,DriverMessaging::SensorPriority::NAME);
	}
	virtual ~DynamixelDriver() {
		servos.removeCollectionListener(&servoIDSync);
		DriverMessaging::removeListener(this,DriverMessaging::SensorPriority::NAME);
		DriverMessaging::removeListener(this,DriverMessaging::LoadPrediction::NAME);
	}
	
	virtual std::string getClassName() const { return autoRegisterDynamixelDriver; }
	
	virtual MotionHook* getMotionSink() { return dynamic_cast<MotionHook*>(this); }
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"]=dynamic_cast<DataSource*>(this);
	}
	
	virtual void motionStarting();
	virtual bool isConnected();
	virtual void motionStopping();
	virtual void motionCheck(const float outputs[][NumOutputs]);
	virtual void updatePIDs(const std::vector<MotionHook::PIDUpdate>& pids);
	virtual void registerSource();
	virtual void deregisterSource();
	virtual void enteringRealtime(const plist::Primitive<double>& simTimeScale) { DataSource::enteringRealtime(simTimeScale); }
	virtual void leavingRealtime(bool isFullSpeed) { DataSource::leavingRealtime(isFullSpeed); }
	
	virtual unsigned int nextTimestamp();
	virtual const std::string& nextName() { return instanceName; }
	virtual bool advance();
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);

	virtual void processDriverMessage(const DriverMessaging::Message& d);
	
	struct ServoInfo : public virtual plist::Dictionary {
		ServoInfo(unsigned int i=DynamixelProtocol::INVALID_ID)
			: plist::Dictionary(false), detected(false), output(), led(), freeSpinOutput(), /*maxSpeed(0),*/ invertRotation(false), zeroAngle(0), maxTic(0), maxAngle(0),
			cmdSpeedSlopeP(0.6575f), cmdSpeedOffsetP(0.1557f), cmdSpeedSlopeN(0.6554f), cmdSpeedOffsetN(-0.1256f),
			repSpeedSlope(0.111f / 60 * 2 * (float)M_PI), // from Dynamixel documentation, rpm = reported speed * 0.111 (114rpm ≈ 1023) ... rpm / value / 60 seconds per minute * 2π radians = 0.011623892818282 rad/s per value
			leftIRDistOffset(), centerIRDistOffset(), rightIRDistOffset(),
			leftLuminosityOffset(), centerLuminosityOffset(), rightLuminosityOffset(), 
			micVolumeOffset(), micSpikeCountOffset(), 
			servoID(i), slope(0), punch(0), margin(0), curRotationMode(UNKNOWN), lastCmd(), recentCmdMotion(0), failures(0), 
			predictedLoad(0), sensorPriority(-1), sensorActivation(1), micSpikeFrameNumber(-1U), model(DynamixelProtocol::MODEL_UNKNOWN), modelName()
		{
			i-=START_SERVO_ID;
#ifdef TGT_HAS_WHEELS
			if(WheelOffset<PIDJointOffset || WheelOffset>=PIDJointOffset+NumPIDJoints) {
				// wheels are not a subset of PIDJoints, so we'll default map the first NumWheel servos to wheels, and the rest to PIDJoints
				if(i<(int)NumWheels) {
					output = ( i+WheelOffset );
					freeSpinOutput = ( i+WheelOffset );
				} else {
					output = ( (i-NumWheels<(int)NumPIDJoints) ? i-NumWheels+PIDJointOffset : UNUSED );
				}
			} else {
				// wheels are a subset of PIDJoints, don't subtract NumWheels...
				output = ( (i<NumPIDJoints) ? i+PIDJointOffset : UNUSED);
				if(i>=(int)WheelOffset && i<(int)(WheelOffset+NumWheels)) {
					freeSpinOutput = ( i+PIDJointOffset );
				}
			}
#else
			output = ( (i<NumPIDJoints) ? i+PIDJointOffset : UNUSED);
#endif
#ifdef TGT_HAS_LEDS
			led = (i<NumLEDs ? i+LEDOffset : UNUSED);
#endif
			
			plist::NamedEnumeration<SensorOffset_t>::setNames(sensorNames);
			plist::NamedEnumeration<SensorOffset_t>::addNameForVal("UNUSED",static_cast<SensorOffset_t>(-1));
			leftIRDistOffset.set(capabilities.findSensorOffset("LeftIRDist"));
			centerIRDistOffset.set(capabilities.findSensorOffset("CenterIRDist"));
			rightIRDistOffset.set(capabilities.findSensorOffset("RightIRDist"));
			leftLuminosityOffset.set(capabilities.findSensorOffset("LeftLuminosity"));
			centerLuminosityOffset.set(capabilities.findSensorOffset("CenterLuminosity"));
			rightLuminosityOffset.set(capabilities.findSensorOffset("RightLuminosity"));
			micVolumeOffset.set(capabilities.findSensorOffset("MicVolume"));
			micSpikeCountOffset.set(capabilities.findSensorOffset("MicSpikeCount"));
			
			setLoadSavePolicy(FIXED,SYNC);
			initEntries();
		}
		
		void setModel(DynamixelProtocol::ModelID_t m) { if(model!=m) { model=m; initEntries(); } }
		DynamixelProtocol::ModelID_t getModel() const { return model; }
		void setModelName(const std::string m) { modelName=m; }
		const std::string& getModelName() const { return modelName; }
		
		//! returns true if any of the AX-S1 sensor offsets are in use
		bool hasSensorOffset() const {
			const SensorOffset_t US = static_cast<SensorOffset_t>(-1);
			return leftIRDistOffset!=US || centerIRDistOffset!=US || rightIRDistOffset!=US
				|| leftLuminosityOffset!=US || centerLuminosityOffset!=US || rightLuminosityOffset!=US
				|| micVolumeOffset!=US || micSpikeCountOffset!=US;
		}
		
		plist::Primitive<bool> detected;
		
		// Servo stuff
		plist::OutputSelector output;
		plist::OutputSelector led;
		plist::OutputSelector freeSpinOutput;
		//plist::Primitive<float> maxSpeed;
		plist::Primitive<bool> invertRotation;
		plist::Angle zeroAngle;
		plist::Primitive<unsigned int> maxTic;
		plist::Angle maxAngle;
		plist::Primitive<float> cmdSpeedSlopeP;
		plist::Primitive<float> cmdSpeedOffsetP;
		plist::Primitive<float> cmdSpeedSlopeN;
		plist::Primitive<float> cmdSpeedOffsetN;
		plist::Primitive<float> repSpeedSlope;
		
		// Sensor (AX-S1) stuff
		plist::NamedEnumeration<SensorOffset_t> leftIRDistOffset;
		plist::NamedEnumeration<SensorOffset_t> centerIRDistOffset;
		plist::NamedEnumeration<SensorOffset_t> rightIRDistOffset;
		plist::NamedEnumeration<SensorOffset_t> leftLuminosityOffset;
		plist::NamedEnumeration<SensorOffset_t> centerLuminosityOffset;
		plist::NamedEnumeration<SensorOffset_t> rightLuminosityOffset;
		plist::NamedEnumeration<SensorOffset_t> micVolumeOffset;
		plist::NamedEnumeration<SensorOffset_t> micSpikeCountOffset;
		
		unsigned int servoID;
		float slope;
		float punch;
		float margin;
		enum RotationMode { POSITION, CONTINUOUS, UNKNOWN } curRotationMode;
		unsigned short lastCmd;
		float recentCmdMotion;
		unsigned short failures;
		float predictedLoad;
		float sensorPriority; //!< amount to add to sensor activation each sensor check, negative values indicate "auto" mode based on movement speed with decay
		float sensorActivation; //!< the accumulated "energy" for polling this servo, the servos with highest activation are polled as avaiable
		unsigned int micSpikeFrameNumber; //!< stores the frame number when the micSpike occurred, so we can tell when the system has received it
		
		DynamixelProtocol::ModelID_t model;
		plist::Primitive<std::string> modelName; // assigned during pingServos to string name so we can include numeric value of unknown servo model IDs
	protected:
		void initEntries() {
			using namespace DynamixelProtocol;
			clear();
			
			addEntry("Model",modelName,"Model name for this device; set automatically via pingServos");
			addEntry("Detected",detected,"If true, servo is detected and functioning.  If a servo is reconnected, set back to true to attempt to re-enable communication.");
			
			if(model!=MODEL_AXS1 || model==MODEL_UNKNOWN) {
				addEntry("Output",output,"Tekkotsu offset to pull servo positions from; -1 or empty string to mark unused\n"
								 "(May still rotate via FreeSpinOutput if position control by Output is unused...)");
				addEntry("LED",led,"Tekkotsu offset to pull LED values from; -1 or empty string to mark unused");
				addEntry("FreeSpinOutput",freeSpinOutput,"Tekkotsu output to control the 'continuous rotation mode'. If specified output's\n"
								 "value is non-zero, the value from Output is ignored, and the servo is rotated at the\n"
								 "specified speed.  If Output and FreeSpinOutput are equal, then the servo is permanently\n"
								 "in continuous rotation mode.");
				//addEntry("MaxSpeed",maxSpeed,"Indicates maximum joint speed in rad/s, 0 for no maximum");
				addEntry("InvertRotation",invertRotation,"If true, the servo moves in the reverse direction.  Calibration parameters are always applied relative to the 'native' orientation.");
				addEntry("ZeroAngle",zeroAngle,"Specifies the angle to which the zero point of the output will be mapped.\n"
								 "For an AX or RX servo, this must be in the range ±2.618 radians (i.e. ±150°), such that\n"
								 "a value of zero is centered in the range of motion.");
				maxTic = IS_MXEX(model) ? 4095 : 1023 ;
				maxAngle = IS_MXEX(model) ? (360*(float)M_PI/180) : (300*(float)M_PI/180);
				addEntry("MaxTic",maxTic,"The maximum servo tic value (e.g. 1023 for AX and RX, 4095 for EX-106 and MX series)");
				addEntry("MaxAngle",maxAngle,"The maximum servo angle (e.g. 300° for AX and RX, 280.6° for EX-106, 360° for MX series )");
				addEntry("CmdSpeedSlopePos",cmdSpeedSlopeP,"Calibration parameter for converting desired physical positive speeds to speed command for the servo.");
				addEntry("CmdSpeedOffsetPos",cmdSpeedOffsetP,"Calibration parameter for converting desired physical positive speeds to speed command for the servo.");
				addEntry("CmdSpeedSlopeNeg",cmdSpeedSlopeN,"Calibration parameter for converting desired physical negative speeds to speed command for the servo.");
				addEntry("CmdSpeedOffsetNeg",cmdSpeedOffsetN,"Calibration parameter for converting desired physical negative speeds to speed command for the servo.");
				addEntry("ReportedSpeedSlope",repSpeedSlope,"Calibration parameter for converting speed feedback from servo to actual physical speed.");
			}
			
			if(model==MODEL_AXS1 || model==MODEL_UNKNOWN) {
				addEntry("LeftIRDistOffset",leftIRDistOffset,"Tekkotsu sensor offset where the left IR distance reading should be stored.");
				addEntry("CenterIRDistOffset",centerIRDistOffset,"Tekkotsu sensor offset where the center IR distance reading should be stored.");
				addEntry("RightIRDistOffset",rightIRDistOffset,"Tekkotsu sensor offset where the right IR distance reading should be stored.");
				addEntry("LeftLuminosityOffset",leftLuminosityOffset,"Tekkotsu sensor offset where the left luminosity reading should be stored.");
				addEntry("CenterLuminosityOffset",centerLuminosityOffset,"Tekkotsu sensor offset where the center luminosity reading should be stored.");
				addEntry("RightLuminosityOffset",rightLuminosityOffset,"Tekkotsu sensor offset where the right luminosity reading should be stored.");
				addEntry("MicVolumeOffset",micVolumeOffset,"Tekkotsu sensor offset where the volume magnitude reading should be stored.");
				addEntry("MicSpikeCountOffset",micSpikeCountOffset,"Tekkotsu sensor offset where the volume spike count should be stored.");
			}
		}
		
	};
	
	plist::DictionaryOf< ServoInfo > servos; //!< Maps servo IDs to Tekkotsu output offsets, use command line new/delete commands to add/remove mappings.
	typedef plist::DictionaryOf< ServoInfo >::const_iterator servo_iterator;
	
	plist::Primitive<std::string> commName; //!< Name of the CommPort to use, generally a SerialCommPort with direct binary TTL communication with the servos
	plist::Primitive<float> loadCompensation; //!< Ratio of load to deflection, if non-zero will use LoadPrediction messages and attempt to counter anticipated loads
	plist::Primitive<unsigned int> commLatency; //!< The delay (in MILLIseconds, ms) to get a response from the servos due to buffering in the USB-to-TTL interface.
	plist::Primitive<unsigned int> numPoll; //!< Number of sensor queries to send before reading responses.  This lets us front-load the queries so we can get multiple responses per read buffer flush period
	plist::Primitive<unsigned int> responseTime; //!< The amount of time (in MICROseconds, µs) to wait between sending a sensor query and sending another when front-loading the buffer.  If this is too short, the next query will collide with the previous response.
	
protected:	
	static unsigned int VOLTAGE_SENSOR_OFFSET; //!< index of the voltage sensor
	static unsigned int TEMP_SENSOR_OFFSET; //!< index of the temperature sensor
	
	struct ServoInfoIDSync : plist::CollectionListener {
		virtual void plistCollectionEntryAdded(plist::Collection& col, ObjectBase&) { plistCollectionEntriesChanged(col); }
		virtual void plistCollectionEntriesChanged(plist::Collection& col) {
			plist::DictionaryOf< ServoInfo >& s = dynamic_cast<plist::DictionaryOf<ServoInfo>& > (col);
			for(servo_iterator it=s.begin(); it!=s.end(); ++it)
				it->second->servoID = atoi(it->first.c_str());
		}
	} servoIDSync;
	
	class CommThread : public Thread, public plist::CollectionListener {
	public:
		CommThread(plist::DictionaryOf< ServoInfo >& servoList, const plist::Primitive<std::string>& comm, DynamixelDriver& parent)
			: Thread(), pidLock(), dirtyPIDs(0), commName(comm), servos(servoList), driver(parent), servoPollQueue(),
			failsafe(*this,FrameTime*NumFrames*3/2000.0,false), continuousUpdates(), updated(false), responsePending(false), lastSensorTime(0), 
			servoDeflection(0), isFirstCheck(true), timestampBufA(0), timestampBufB(0), timestampBufC(0), curBuf(NULL)
		{
			failsafe.restartFlag=true;
			for(unsigned int i=0; i<NumLEDs; ++i)
				lastLEDState[i]=LED_UNKNOWN;
			servos.addCollectionListener(this);
			plistCollectionEntriesChanged(servos);
		}
		~CommThread() { servos.removeCollectionListener(this); }
		
		virtual void plistCollectionEntryAdded(Collection& /*col*/, ObjectBase& primitive) {
			if(ServoInfo* si = dynamic_cast<ServoInfo*>(&primitive)) {
				servoPollQueue.push_back(si);
				si->detected=true;
				if(isStarted())
					driver.pingServos(true);
			}
		}
		virtual void plistCollectionEntryRemoved(Collection& /*col*/, ObjectBase& primitive) {
			ServoInfo* si = dynamic_cast<ServoInfo*>(&primitive);
			std::vector<ServoInfo*>::iterator it = std::find(servoPollQueue.begin(),servoPollQueue.end(),si);
			if(it!=servoPollQueue.end())
				servoPollQueue.erase(it);
		}
		virtual void plistCollectionEntriesChanged(Collection& /*col*/) {
			servoPollQueue.resize(servos.size());
			unsigned int i=0;
			for(plist::DictionaryOf< ServoInfo >::const_iterator it=servos.begin(); it!=servos.end(); ++it) {
				servoPollQueue[i++] = it->second;
				if(isStarted())
					it->second->detected=true;
			}
			if(isStarted())
				driver.pingServos(true);
		}
		
		virtual void start() { updated=false; continuousUpdates=true; Thread::start(); }
		virtual void startOneUpdate() { updated=false; continuousUpdates=false; Thread::start(); }
		virtual Thread& stop();
		virtual void waitForUpdate();
		virtual bool isStarted() const { return Thread::isStarted() || failsafe.isStarted(); }
		bool takeUpdate() { if(!updated) { return false; } else { updated=false; return true; } }
		
		//! find the oldest of the output buffers that isn't currently in use
		float* getWriteBuffer();
		//! sets the timestamp on the indicated buffer (indicates you're done writing)
		void setWriteBufferTimestamp(float * buf);
		
		unsigned int nextTimestamp() const { return lastSensorTime + driver.commLatency; }
		
		Thread::Lock pidLock;
		PIDUpdate pidValues[NumOutputs];
		unsigned int dirtyPIDs;
		
	protected:
		virtual bool launched();
		virtual void cancelled();
		void clearBuffer();
		virtual unsigned int runloop();
		template<class T> void readResponse(T& response, std::istream& is, ServoInfo& module);
		virtual void updateCommands(std::istream& is, std::ostream& os);
		
		//! converts the value @a v from radians into the specified servo's range, with expected speed (rad/s)
		DynamixelProtocol::SyncWritePosSpeedEntry setServo(const servo_iterator& servo, ServoInfo::RotationMode rm, float v, float speed);
		
		const plist::Primitive<std::string>& commName;
		plist::DictionaryOf< ServoInfo >& servos;
		DynamixelDriver& driver;
		std::vector<ServoInfo*> servoPollQueue; //!< priority queue of servos waiting to be polled for sensor information
		FailsafeThread failsafe;
		bool continuousUpdates;
		bool updated;
		bool responsePending;
		unsigned int lastSensorTime;
		float servoDeflection; //!< 'tics' of servo deflection from target per newton·meter of torque applied
		
		bool isFirstCheck;
		float outputBufA[NumOutputs];
		unsigned int timestampBufA;
		float outputBufB[NumOutputs];
		unsigned int timestampBufB;
		float outputBufC[NumOutputs];
		unsigned int timestampBufC;
		float lastOutputs[NumOutputs];
		float * curBuf;
		
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
		enum LedState { LED_OFF=0, LED_ON, LED_UNKNOWN } lastLEDState[NumLEDs]; //!< keeps track of the last full activation value sent to servo
		
	private:
		CommThread(const CommThread&); //!< don't copy
		CommThread& operator=(const CommThread&); //!< don't assign
	} commThread;
	
	void doFreeze();
	void doUnfreeze();
	
	//! sends servo feedback values into the framework
	static void provideValues(const ServoInfo& info, const DynamixelProtocol::ServoSensorsResponse& response);
	//! sends AX-S1 sensor values into the framework
	static void provideValues(ServoInfo& info, const DynamixelProtocol::AXS1SensorsResponse& response);
	
	//! forwards call to DataSource::providingOutput() if the index is valid
	void provideOutput(unsigned int idx) { if(idx<NumOutputs) providingOutput(idx); }
	//! forwards call to DataSource::ignoringOutput() if the index is valid
	void ignoreOutput(unsigned int idx) { if(idx<NumOutputs) ignoringOutput(idx); }
	
	//! tests each servo to see if it is connected
	void pingServos(bool detectedOnly=false);
	
	//! broadcasts a "relax" command to all servos
	void sendZeroTorqueCmd(CommPort& comm);
	
	//! sends a series of sync write entries into a stream (adding appropriate header and checksum)
	template<class T> static void writeSyncEntries(std::ostream& os, const std::vector<T>& entries) {
		if(entries.size()==0)
			return;
		unsigned char checksum=0;
		DynamixelProtocol::write(os, DynamixelProtocol::SyncWriteHeader<T>(entries.size()), checksum);
		os.write(entries[0],entries.size()*sizeof(T));
		checksum+=DynamixelProtocol::nchecksum(entries[0],entries.size()*sizeof(T));
		os.put(~checksum);
	}
	
	bool motionActive;
	bool sensorsActive;
	std::string lastSensor;

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterDynamixelDriver;
};

template<class T> void DynamixelDriver::CommThread::readResponse(T& response, std::istream& is, ServoInfo& module) {
	do {
		testCancel();
		while(!DynamixelProtocol::readResponse(is,response,module.output)) {
			/*std::cerr << "DynamixelDriver got an invalid response from " << (int)response.servoid << " to sensor query of " << module.servoID << ", re-reading." << std::endl;
			 if(response.servoid!=module.servoID)
				 std::cerr << "DynamixelDriver got an unexpected response from " << (int)response.servoid << " to sensor query of " << module.servoID << ", re-reading." << std::endl;*/
			failsafe.progressFlag=true;
			testCancel();
		}
	} while(response.servoid!=module.servoID);
		
	module.failures=0;
	module.sensorActivation=0;
	// LOW LEVEL LOGGING:
	//cout << get_time() << '\t' << response.getPosition() << '\t' << response.getLoad() << '\t' << servoPollQueue[i]->lastCmd << endl;
	//cout << servoPollQueue[i]->servoID << ": " << get_time() << /*" = " <<  TimeET() <<*/ ' ' << validate(response) << ' ' << response.getPosition() << ' ' << response.getSpeed() << ' ' << response.getLoad() << ' ' << (int)response.voltage << ' ' << (int)response.temp << endl;
	
	provideValues(module, response);
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
