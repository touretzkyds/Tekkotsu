#include "WorldState.h"
#include "Shared/get_time.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "ERS210Info.h"
#include "ERS220Info.h"
#include "ERS7Info.h"
#include "Shared/Config.h"

#ifdef PLATFORM_APERIOS
#  include <OPENR/core_macro.h>
#  include <OPENR/ObjcommTypes.h>
#  include <OPENR/OPENR.h>
#  include <OPENR/OPENRAPI.h>
#  include <OPENR/OPENRMessages.h>
#  include <OPENR/OPower.h>
#else
#  include "local/DataSource.h"
#endif

using namespace std;

#define GETD(cpc) (((float)sensor.GetData(cpc)->frame[lastFrame].value) / 1.0E6f) //!< returns value from OPEN-R, converted from micro in int to base in float
#define GETB(cpc) ((bool)sensor.GetData(cpc)->frame[lastFrame].value) //!< returns value from OPEN-R, as bool
#define GETSENSOR(cpc) ((float)sensor.GetData(cpc)->frame[lastFrame].value) //!< return value from OPEN-R, as int
#define GETSIG(cpc) ((word)sensor.GetData(cpc)->frame[lastFrame].signal) //!< returns signal from OPEN-R as word
#define GETDUTY(cpc) (((OJointValue*)(void*)&sensor.GetData(cpc)->frame[lastFrame])->pwmDuty/512.0f) //!< returns duty cycle from OPEN-R as float; -1 (full reverse) to 0 (idle) to 1 (full forward)

const double WorldState::g=9.80665;
const double WorldState::IROORDist = 900.0;

#ifdef PLATFORM_APERIOS
WorldState * state=NULL;
#else
WorldStateLookup state;
#endif

WorldState::WorldState()
	: alwaysGenerateStatus(false), vel_x(0), vel_y(0), vel_a(0), vel_time(0),
		robotStatus(0), batteryStatus(0),
		lastSensorUpdateTime(0), frameNumber(0), framesProcessed(0),
		curtime(0)
{
	for(unsigned int i=0; i< NumOutputs; i++)
		outputs[i]=0;
	for(unsigned int i=0; i< NumButtons; i++)
		buttons[i]=0;
	for(unsigned int i=0; i< NumSensors; i++)
		sensors[i]=0;
	for(unsigned int i=0; i< NumPIDJoints; i++)
		for(unsigned int j=0; j<3; j++)
			pids[i][j]=DefaultPIDs[i][j];
	for(unsigned int i=0; i< NumPIDJoints; i++)
		pidduties[i]=0;
	memset(powerFlags,0,sizeof(unsigned int)*PowerSrcID::NumPowerSIDs);
	memset(button_times,0,sizeof(unsigned int)*NumButtons);
}

#ifdef PLATFORM_APERIOS

/*! This will cause events to be posted */
void WorldState::read(OSensorFrameVectorData& sensor, EventRouter* er) {
	//always using GetInfo(0) to get "global" information for the vector, other infos contain metadata for individual data fields
	unsigned int newFrameNumber=sensor.GetInfo(0)->frameNumber;
	if(frameNumber>=newFrameNumber)
		return; //sensors have already been filled in
	
	curtime=get_time();

	std::vector<EventBase> evtBuf;
	evtBuf.reserve(NumButtons);
	unsigned int lastFrame=sensor.GetInfo(0)->numFrames-1;

	if(RobotName == ERS210Info::TargetName) {
		outputs[LFrLegOffset + RotatorOffset   ] = GETD(ERS210Info::CPCJointLFRotator);
		outputs[LFrLegOffset + ElevatorOffset  ] = GETD(ERS210Info::CPCJointLFElevator);
		outputs[LFrLegOffset + KneeOffset      ] = GETD(ERS210Info::CPCJointLFKnee);
		pidduties[LFrLegOffset + RotatorOffset ] = GETDUTY(ERS210Info::CPCJointLFRotator);
		pidduties[LFrLegOffset + ElevatorOffset] = GETDUTY(ERS210Info::CPCJointLFElevator);
		pidduties[LFrLegOffset + KneeOffset    ] = GETDUTY(ERS210Info::CPCJointLFKnee);
	
		outputs[RFrLegOffset + RotatorOffset   ] = GETD(ERS210Info::CPCJointRFRotator);
		outputs[RFrLegOffset + ElevatorOffset  ] = GETD(ERS210Info::CPCJointRFElevator);
		outputs[RFrLegOffset + KneeOffset      ] = GETD(ERS210Info::CPCJointRFKnee);
		pidduties[RFrLegOffset + RotatorOffset ] = GETDUTY(ERS210Info::CPCJointRFRotator);
		pidduties[RFrLegOffset + ElevatorOffset] = GETDUTY(ERS210Info::CPCJointRFElevator);
		pidduties[RFrLegOffset + KneeOffset    ] = GETDUTY(ERS210Info::CPCJointRFKnee);

		outputs[LBkLegOffset + RotatorOffset   ] = GETD(ERS210Info::CPCJointLHRotator);
		outputs[LBkLegOffset + ElevatorOffset  ] = GETD(ERS210Info::CPCJointLHElevator);
		outputs[LBkLegOffset + KneeOffset      ] = GETD(ERS210Info::CPCJointLHKnee);
		pidduties[LBkLegOffset + RotatorOffset ] = GETDUTY(ERS210Info::CPCJointLHRotator);
		pidduties[LBkLegOffset + ElevatorOffset] = GETDUTY(ERS210Info::CPCJointLHElevator);
		pidduties[LBkLegOffset + KneeOffset    ] = GETDUTY(ERS210Info::CPCJointLHKnee);

		outputs[RBkLegOffset + RotatorOffset   ] = GETD(ERS210Info::CPCJointRHRotator);
		outputs[RBkLegOffset + ElevatorOffset  ] = GETD(ERS210Info::CPCJointRHElevator);
		outputs[RBkLegOffset + KneeOffset      ] = GETD(ERS210Info::CPCJointRHKnee);
		pidduties[RBkLegOffset + RotatorOffset ] = GETDUTY(ERS210Info::CPCJointRHRotator);
		pidduties[RBkLegOffset + ElevatorOffset] = GETDUTY(ERS210Info::CPCJointRHElevator);
		pidduties[RBkLegOffset + KneeOffset    ] = GETDUTY(ERS210Info::CPCJointRHKnee);

		// Get head tilt,pan,roll joint angles
		outputs[HeadOffset+TiltOffset] = GETD(ERS210Info::CPCJointNeckTilt);
		outputs[HeadOffset+PanOffset ] = GETD(ERS210Info::CPCJointNeckPan);
		outputs[HeadOffset+RollOffset] = GETD(ERS210Info::CPCJointNeckRoll);
		pidduties[HeadOffset+TiltOffset] = GETDUTY(ERS210Info::CPCJointNeckTilt);
		pidduties[HeadOffset+PanOffset ] = GETDUTY(ERS210Info::CPCJointNeckPan);
		pidduties[HeadOffset+RollOffset] = GETDUTY(ERS210Info::CPCJointNeckRoll);

#ifdef TGT_ERS210
		unsigned tail = ERS210Info::TailOffset;
		unsigned mouth = ERS210Info::MouthOffset;
#elif defined(TGT_ERS2xx)
		unsigned tail = ERS2xxInfo::TailOffset;
		unsigned mouth = ERS2xxInfo::MouthOffset;
#else
		unsigned tail = capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::TailOffset]);
		unsigned mouth = capabilities.getOutputOffset(ERS210Info::outputNames[ERS210Info::MouthOffset]);
#endif
		outputs[tail+TiltOffset] = GETD(ERS210Info::CPCJointTailTilt);
		outputs[tail+PanOffset]  = GETD(ERS210Info::CPCJointTailPan);
		pidduties[tail+TiltOffset] = GETDUTY(ERS210Info::CPCJointTailTilt);
		pidduties[tail+PanOffset]  = GETDUTY(ERS210Info::CPCJointTailPan);
		
		outputs[mouth] = GETD(ERS210Info::CPCJointMouth);
		pidduties[mouth] = GETDUTY(ERS210Info::CPCJointMouth);

		// Get foot switches
		chkEvent(evtBuf,ERS210Info::LFrPawOffset,GETB(ERS210Info::CPCSensorLFPaw),buttonNames[ERS210Info::LFrPawOffset]);
		chkEvent(evtBuf,ERS210Info::RFrPawOffset,GETB(ERS210Info::CPCSensorRFPaw),buttonNames[ERS210Info::RFrPawOffset]);
		chkEvent(evtBuf,ERS210Info::LBkPawOffset,GETB(ERS210Info::CPCSensorLHPaw),buttonNames[ERS210Info::LBkPawOffset]);
		chkEvent(evtBuf,ERS210Info::RBkPawOffset,GETB(ERS210Info::CPCSensorRHPaw),buttonNames[ERS210Info::RBkPawOffset]);

		// Get buttons
		chkEvent(evtBuf,ERS210Info::ChinButOffset,  GETB(ERS210Info::CPCSensorChinSwitch),buttonNames[ERS210Info::ChinButOffset]);
		chkEvent(evtBuf,ERS210Info::BackButOffset,  GETB(ERS210Info::CPCSensorBackSwitch),buttonNames[ERS210Info::BackButOffset]);
		chkEvent(evtBuf,ERS210Info::HeadFrButOffset,GETD(ERS210Info::CPCSensorHeadFrontPressure),buttonNames[ERS210Info::HeadFrButOffset]);
		chkEvent(evtBuf,ERS210Info::HeadBkButOffset,GETD(ERS210Info::CPCSensorHeadBackPressure),buttonNames[ERS210Info::HeadBkButOffset]);

		// Get IR distance sensor
		sensors[ERS210Info::IRDistOffset]=GETSENSOR(ERS210Info::CPCSensorPSD) / 1000.0f;

		// Get acceleration sensors
		sensors[BAccelOffset] = GETD(ERS210Info::CPCSensorAccelFB);
		sensors[LAccelOffset] = GETD(ERS210Info::CPCSensorAccelLR);
		sensors[DAccelOffset] = GETD(ERS210Info::CPCSensorAccelUD);

		sensors[ERS210Info::ThermoOffset] = GETD(ERS210Info::CPCSensorThermoSensor);
	}

	// (ERS-220 only)
	if(RobotName == ERS220Info::TargetName) {
		outputs[LFrLegOffset + RotatorOffset   ] = GETD(ERS220Info::CPCJointLFRotator);
		outputs[LFrLegOffset + ElevatorOffset  ] = GETD(ERS220Info::CPCJointLFElevator);
		outputs[LFrLegOffset + KneeOffset      ] = GETD(ERS220Info::CPCJointLFKnee);
		pidduties[LFrLegOffset + RotatorOffset ] = GETDUTY(ERS220Info::CPCJointLFRotator);
		pidduties[LFrLegOffset + ElevatorOffset] = GETDUTY(ERS220Info::CPCJointLFElevator);
		pidduties[LFrLegOffset + KneeOffset    ] = GETDUTY(ERS220Info::CPCJointLFKnee);
	
		outputs[RFrLegOffset + RotatorOffset   ] = GETD(ERS220Info::CPCJointRFRotator);
		outputs[RFrLegOffset + ElevatorOffset  ] = GETD(ERS220Info::CPCJointRFElevator);
		outputs[RFrLegOffset + KneeOffset      ] = GETD(ERS220Info::CPCJointRFKnee);
		pidduties[RFrLegOffset + RotatorOffset ] = GETDUTY(ERS220Info::CPCJointRFRotator);
		pidduties[RFrLegOffset + ElevatorOffset] = GETDUTY(ERS220Info::CPCJointRFElevator);
		pidduties[RFrLegOffset + KneeOffset    ] = GETDUTY(ERS220Info::CPCJointRFKnee);
	
		outputs[LBkLegOffset + RotatorOffset   ] = GETD(ERS220Info::CPCJointLHRotator);
		outputs[LBkLegOffset + ElevatorOffset  ] = GETD(ERS220Info::CPCJointLHElevator);
		outputs[LBkLegOffset + KneeOffset      ] = GETD(ERS220Info::CPCJointLHKnee);
		pidduties[LBkLegOffset + RotatorOffset ] = GETDUTY(ERS220Info::CPCJointLHRotator);
		pidduties[LBkLegOffset + ElevatorOffset] = GETDUTY(ERS220Info::CPCJointLHElevator);
		pidduties[LBkLegOffset + KneeOffset    ] = GETDUTY(ERS220Info::CPCJointLHKnee);

		outputs[RBkLegOffset + RotatorOffset   ] = GETD(ERS220Info::CPCJointRHRotator);
		outputs[RBkLegOffset + ElevatorOffset  ] = GETD(ERS220Info::CPCJointRHElevator);
		outputs[RBkLegOffset + KneeOffset      ] = GETD(ERS220Info::CPCJointRHKnee);
		pidduties[RBkLegOffset + RotatorOffset ] = GETDUTY(ERS220Info::CPCJointRHRotator);
		pidduties[RBkLegOffset + ElevatorOffset] = GETDUTY(ERS220Info::CPCJointRHElevator);
		pidduties[RBkLegOffset + KneeOffset    ] = GETDUTY(ERS220Info::CPCJointRHKnee);

		// Get head tilt,pan,roll joint angles
		outputs[HeadOffset+TiltOffset] = GETD(ERS220Info::CPCJointNeckTilt);
		outputs[HeadOffset+PanOffset ] = GETD(ERS220Info::CPCJointNeckPan);
		outputs[HeadOffset+RollOffset] = GETD(ERS220Info::CPCJointNeckRoll);
		pidduties[HeadOffset+TiltOffset] = GETDUTY(ERS220Info::CPCJointNeckTilt);
		pidduties[HeadOffset+PanOffset ] = GETDUTY(ERS220Info::CPCJointNeckPan);
		pidduties[HeadOffset+RollOffset] = GETDUTY(ERS220Info::CPCJointNeckRoll);

		// Get foot switches
		chkEvent(evtBuf,ERS220Info::LFrPawOffset,GETB(ERS220Info::CPCSensorLFPaw),buttonNames[ERS220Info::LFrPawOffset]);
		chkEvent(evtBuf,ERS220Info::RFrPawOffset,GETB(ERS220Info::CPCSensorRFPaw),buttonNames[ERS220Info::RFrPawOffset]);
		chkEvent(evtBuf,ERS220Info::LBkPawOffset,GETB(ERS220Info::CPCSensorLHPaw),buttonNames[ERS220Info::LBkPawOffset]);
		chkEvent(evtBuf,ERS220Info::RBkPawOffset,GETB(ERS220Info::CPCSensorRHPaw),buttonNames[ERS220Info::RBkPawOffset]);

		// Get buttons
		chkEvent(evtBuf,ERS220Info::ChinButOffset,  GETB(ERS220Info::CPCSensorChinSwitch),buttonNames[ERS220Info::ChinButOffset]);
		chkEvent(evtBuf,ERS220Info::BackButOffset,  GETB(ERS220Info::CPCSensorBackSwitch),buttonNames[ERS220Info::BackButOffset]);
		chkEvent(evtBuf,ERS220Info::HeadFrButOffset,GETD(ERS220Info::CPCSensorHeadFrontPressure),buttonNames[ERS220Info::HeadFrButOffset]);
		chkEvent(evtBuf,ERS220Info::HeadBkButOffset,GETD(ERS220Info::CPCSensorHeadBackPressure),buttonNames[ERS220Info::HeadBkButOffset]);
		chkEvent(evtBuf,ERS220Info::TailLeftButOffset, GETB(ERS220Info::CPCSensorTailLeftSwitch),  buttonNames[ERS220Info::TailLeftButOffset]);
		chkEvent(evtBuf,ERS220Info::TailCenterButOffset, GETB(ERS220Info::CPCSensorTailCenterSwitch),buttonNames[ERS220Info::TailCenterButOffset]);
		chkEvent(evtBuf,ERS220Info::TailRightButOffset, GETB(ERS220Info::CPCSensorTailRightSwitch), buttonNames[ERS220Info::TailRightButOffset]);

		// Get IR distance sensor
		sensors[ERS220Info::IRDistOffset]=GETSENSOR(ERS220Info::CPCSensorPSD) / 1000.0f;

		// Get acceleration sensors
		sensors[BAccelOffset] = GETD(ERS220Info::CPCSensorAccelFB);
		sensors[LAccelOffset] = GETD(ERS220Info::CPCSensorAccelLR);
		sensors[DAccelOffset] = GETD(ERS220Info::CPCSensorAccelUD);

		sensors[ERS220Info::ThermoOffset] = GETD(ERS220Info::CPCSensorThermoSensor);
	}

	// (ERS-7 only)
	if(RobotName == ERS7Info::TargetName) {
		outputs[LFrLegOffset + RotatorOffset   ] = GETD(ERS7Info::CPCJointLFRotator);
		outputs[LFrLegOffset + ElevatorOffset  ] = GETD(ERS7Info::CPCJointLFElevator);
		outputs[LFrLegOffset + KneeOffset      ] = GETD(ERS7Info::CPCJointLFKnee);
		pidduties[LFrLegOffset + RotatorOffset ] = GETDUTY(ERS7Info::CPCJointLFRotator);
		pidduties[LFrLegOffset + ElevatorOffset] = GETDUTY(ERS7Info::CPCJointLFElevator);
		pidduties[LFrLegOffset + KneeOffset    ] = GETDUTY(ERS7Info::CPCJointLFKnee);
	
		outputs[RFrLegOffset + RotatorOffset   ] = GETD(ERS7Info::CPCJointRFRotator);
		outputs[RFrLegOffset + ElevatorOffset  ] = GETD(ERS7Info::CPCJointRFElevator);
		outputs[RFrLegOffset + KneeOffset      ] = GETD(ERS7Info::CPCJointRFKnee);
		pidduties[RFrLegOffset + RotatorOffset ] = GETDUTY(ERS7Info::CPCJointRFRotator);
		pidduties[RFrLegOffset + ElevatorOffset] = GETDUTY(ERS7Info::CPCJointRFElevator);
		pidduties[RFrLegOffset + KneeOffset    ] = GETDUTY(ERS7Info::CPCJointRFKnee);
	
		outputs[LBkLegOffset + RotatorOffset   ] = GETD(ERS7Info::CPCJointLHRotator);
		outputs[LBkLegOffset + ElevatorOffset  ] = GETD(ERS7Info::CPCJointLHElevator);
		outputs[LBkLegOffset + KneeOffset      ] = GETD(ERS7Info::CPCJointLHKnee);
		pidduties[LBkLegOffset + RotatorOffset ] = GETDUTY(ERS7Info::CPCJointLHRotator);
		pidduties[LBkLegOffset + ElevatorOffset] = GETDUTY(ERS7Info::CPCJointLHElevator);
		pidduties[LBkLegOffset + KneeOffset    ] = GETDUTY(ERS7Info::CPCJointLHKnee);

		outputs[RBkLegOffset + RotatorOffset   ] = GETD(ERS7Info::CPCJointRHRotator);
		outputs[RBkLegOffset + ElevatorOffset  ] = GETD(ERS7Info::CPCJointRHElevator);
		outputs[RBkLegOffset + KneeOffset      ] = GETD(ERS7Info::CPCJointRHKnee);
		pidduties[RBkLegOffset + RotatorOffset ] = GETDUTY(ERS7Info::CPCJointRHRotator);
		pidduties[RBkLegOffset + ElevatorOffset] = GETDUTY(ERS7Info::CPCJointRHElevator);
		pidduties[RBkLegOffset + KneeOffset    ] = GETDUTY(ERS7Info::CPCJointRHKnee);

		// Get head tilt,pan,nod joint angles
		outputs[HeadOffset+TiltOffset] = GETD(ERS7Info::CPCJointNeckTilt);
		outputs[HeadOffset+PanOffset ] = GETD(ERS7Info::CPCJointNeckPan);
		outputs[HeadOffset+RollOffset] = GETD(ERS7Info::CPCJointNeckNod);
		pidduties[HeadOffset+TiltOffset] = GETDUTY(ERS7Info::CPCJointNeckTilt);
		pidduties[HeadOffset+PanOffset ] = GETDUTY(ERS7Info::CPCJointNeckPan);
		pidduties[HeadOffset+RollOffset] = GETDUTY(ERS7Info::CPCJointNeckNod);

		outputs[ERS7Info::TailOffset+TiltOffset] = GETD(ERS7Info::CPCJointTailTilt);
		outputs[ERS7Info::TailOffset+PanOffset]  = GETD(ERS7Info::CPCJointTailPan);
		pidduties[ERS7Info::TailOffset+TiltOffset] = GETDUTY(ERS7Info::CPCJointTailTilt);
		pidduties[ERS7Info::TailOffset+PanOffset]  = GETDUTY(ERS7Info::CPCJointTailPan);
		
		outputs[ERS7Info::MouthOffset] = GETD(ERS7Info::CPCJointMouth);
		pidduties[ERS7Info::MouthOffset] = GETDUTY(ERS7Info::CPCJointMouth);

		// Get foot switches
		chkEvent(evtBuf,ERS7Info::LFrPawOffset,GETB(ERS7Info::CPCSwitchLFPaw),buttonNames[ERS7Info::LFrPawOffset]);
		chkEvent(evtBuf,ERS7Info::RFrPawOffset,GETB(ERS7Info::CPCSwitchRFPaw),buttonNames[ERS7Info::RFrPawOffset]);
		chkEvent(evtBuf,ERS7Info::LBkPawOffset,GETB(ERS7Info::CPCSwitchLHPaw),buttonNames[ERS7Info::LBkPawOffset]);
		chkEvent(evtBuf,ERS7Info::RBkPawOffset,GETB(ERS7Info::CPCSwitchRHPaw),buttonNames[ERS7Info::RBkPawOffset]);

		// Get buttons/switches
		// the sensors are scaled to be relatively similar to the pressure values given by the head on the 210
		chkEvent(evtBuf, ERS7Info::ChinButOffset,       GETSENSOR(ERS7Info::CPCSwitchChin),      buttonNames[ERS7Info::ChinButOffset]);
		chkEvent(evtBuf, ERS7Info::HeadButOffset,       GETSENSOR(ERS7Info::CPCSensorHead)/120,      buttonNames[ERS7Info::HeadButOffset]);
		chkEvent(evtBuf, ERS7Info::FrontBackButOffset,  GETSENSOR(ERS7Info::CPCSensorBackFront)/150, buttonNames[ERS7Info::FrontBackButOffset]);
		chkEvent(evtBuf, ERS7Info::MiddleBackButOffset, GETSENSOR(ERS7Info::CPCSensorBackMiddle)/150,buttonNames[ERS7Info::MiddleBackButOffset]);
		chkEvent(evtBuf, ERS7Info::RearBackButOffset,   GETSENSOR(ERS7Info::CPCSensorBackRear)/150,  buttonNames[ERS7Info::RearBackButOffset]);
		chkEvent(evtBuf, ERS7Info::WirelessSwOffset,GETSENSOR(ERS7Info::CPCSwitchWireless),  buttonNames[ERS7Info::WirelessSwOffset]);

		// Get IR distance sensor
		sensors[ERS7Info::NearIRDistOffset] = GETSENSOR(ERS7Info::CPCSensorNearPSD) / 1000.0f;
		sensors[ERS7Info::FarIRDistOffset] = GETSENSOR(ERS7Info::CPCSensorFarPSD) / 1000.0f;
		sensors[ERS7Info::ChestIRDistOffset] = GETSENSOR(ERS7Info::CPCSensorChestPSD) / 1000.0f;

		// Get acceleration sensors
		sensors[BAccelOffset] = GETD(ERS7Info::CPCSensorAccelFB);
		sensors[LAccelOffset] = GETD(ERS7Info::CPCSensorAccelLR);
		sensors[DAccelOffset] = GETD(ERS7Info::CPCSensorAccelUD);
	}

	//unsigned int dif=curtime-(lastState==NULL ? lastSensorUpdateTime : lastState->lastSensorUpdateTime);
	lastSensorUpdateTime=curtime;
	frameNumber=newFrameNumber;
	++framesProcessed;
	
	//Apply sensor calibrations (currently only PID joint positions - perhaps sensors as well?)
	applyCalibration();

	for(unsigned int i=0; i<evtBuf.size(); i++)
		er->postEvent(evtBuf[i]);

	//this version of read doesn't post the sensor update event -- the caller should do that
	//this event gets posted by MMCombo only if there's no back log on the interprocess event queue (don't want to stack these up for sensor frames missed by main)
	//er->postEvent(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID,dif,"SensorSouceID::UpdatedSID",1);
}

/*! This will cause events to be posted */
void WorldState::read(const OPowerStatus& power, EventRouter* er) {
	std::string actnames[PowerSrcID::NumPowerSIDs];
	std::string denames[PowerSrcID::NumPowerSIDs];
	unsigned int actmasks[PowerSrcID::NumPowerSIDs];
	memset(actmasks,0,sizeof(unsigned int)*PowerSrcID::NumPowerSIDs);

	//RobotStatus
	chkPowerEvent(PowerSrcID::PauseSID,          power.robotStatus,orsbPAUSE,                        "Pause",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::MotorPowerSID,     power.robotStatus,orsbMOTOR_POWER,                  "MotorPower",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::VibrationSID,      power.robotStatus,orsbVIBRATION_DETECT,             "Vibration",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ExternalPortSID,   power.robotStatus,orsbEX_PORT_CONNECTED,            "ExternalPort",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::StationConnectSID, power.robotStatus,orsbSTATION_CONNECTED,            "StationConnect",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ExternalPowerSID,  power.robotStatus,orsbEX_POWER_CONNECTED,           "ExternalPower",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BatteryConnectSID, power.robotStatus,orsbBATTERY_CONNECTED,            "BatteryConnect",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ChargingSID,       power.robotStatus,orsbBATTERY_CHARGING,             "BatteryCharging",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BatteryFullSID,    power.robotStatus,orsbBATTERY_CAPACITY_FULL,        "BatteryFull",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::LowPowerWarnSID,   power.robotStatus,orsbBATTERY_CAPACITY_LOW,         "BatteryLow",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::OverChargedSID,    power.robotStatus,orsbBATTERY_OVER_CURRENT,         "BatteryOverCurrent",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::OverheatingSID,    power.robotStatus,orsbBATTERY_OVER_TEMP_DISCHARGING,"BatteryOverTempDischarge",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::OverheatingSID,    power.robotStatus,orsbBATTERY_OVER_TEMP_CHARGING,   "BatteryOverTempCharge",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ErrorSID,          power.robotStatus,orsbBATTERY_ERROR_OF_CHARGING,    "BatteryChargeError",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ErrorSID,          power.robotStatus,orsbERROR_OF_PLUNGER,             "PlungerError",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::PowerGoodSID,      power.robotStatus,orsbOPEN_R_POWER_GOOD,            "PowerGood",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ErrorSID,          power.robotStatus,orsbERROR_OF_FAN,                 "FanError",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::DataFromStationSID,power.robotStatus,orsbDATA_STREAM_FROM_STATION,     "DataFromStation",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::RegisterUpdateSID, power.robotStatus,orsbREGISTER_UPDATED_BY_STATION,  "RegisterUpdate",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ErrorSID,          power.robotStatus,orsbRTC_ERROR,                    "RTCError",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::RTCSID,            power.robotStatus,orsbRTC_OVERFLOW,                 "RTCOverflow",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::RTCSID,            power.robotStatus,orsbRTC_RESET,                    "RTCReset",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::RTCSID,            power.robotStatus,orsbRTC_SET,                      "RTCSet",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::SpecialModeSID,    power.robotStatus,orsbSPECIAL_MODE,                 "SpecialMode",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BMNDebugModeSID,   power.robotStatus,orsbBMN_DEBUG_MODE,               "BMNDebugMode",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::ChargerStatusSID,  power.robotStatus,orsbCHARGER_STATUS,               "ChargerStatus",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::PlungerSID,        power.robotStatus,orsbPLUNGER,                      "Plunger",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::SuspendedSID,      power.robotStatus,orsbSUSPENDED,                    "Suspended",actnames,denames,actmasks);

	//BatteryStatus
	chkPowerEvent(PowerSrcID::ErrorSID,        power.batteryStatus,obsbERROR_CODE_MASK,             "BatteryError",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BatteryEmptySID, power.batteryStatus,obsbFULLY_DISCHARGED,            "FullyDischarged",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BatteryFullSID,  power.batteryStatus,obsbFULLY_CHARGED,               "FullyCharged",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::DischargingSID,  power.batteryStatus,obsbDISCHARGING,                 "Discharging",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::BatteryInitSID,  power.batteryStatus,obsbINITIALIZED,                 "BatteryInit",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::LowPowerWarnSID, power.batteryStatus,obsbREMAINING_TIME_ALARM,        "RemainingTimeAlarm",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::LowPowerWarnSID, power.batteryStatus,obsbREMAINING_CAPACITY_ALARM,    "RemainingCapacityAlarm",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::TermDischargeSID,power.batteryStatus,obsbTERMINATED_DISCHARGING_ALARM,"TermDischargeAlarm",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::OverheatingSID,  power.batteryStatus,obsbOVER_TEMP_ALARM,             "OverTempAlarm",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::TermChargeSID,   power.batteryStatus,obsbTERMINATED_CHARGING_ALARM,   "TermChargeAlarm",actnames,denames,actmasks);
	chkPowerEvent(PowerSrcID::OverChargedSID,  power.batteryStatus,obsbOVER_CHARGED_ALARM,          "OverChargeAlarm",actnames,denames,actmasks);
	
	sensors[PowerRemainOffset] = power.remainingCapacity/100.0;
	sensors[PowerThermoOffset] = power.temperature/100.0;
	sensors[PowerCapacityOffset] = power.fullyChargedCapacity;
	sensors[PowerVoltageOffset] = power.voltage/1000.0;
	sensors[PowerCurrentOffset] = power.current;

	//only generate status events when a change happens
	for(unsigned int i=0; i<PowerSrcID::NumPowerSIDs; i++) {
		if(actmasks[i]) { //now on
			if(!powerFlags[i]) //was off: activation
				er->postEvent(EventBase::powerEGID,i,EventBase::activateETID,0,actnames[i],1);
			else if(actmasks[i]!=powerFlags[i]) //already on - change? : status
				er->postEvent(EventBase::powerEGID,i,EventBase::statusETID,0,actnames[i],1);
		} else { // now off
			if(powerFlags[i]) //was on: deactivation
				er->postEvent(EventBase::powerEGID,i,EventBase::deactivateETID,0,denames[i],0);
		}
		powerFlags[i]=actmasks[i];
	}

	er->postEvent(EventBase::powerEGID,PowerSrcID::UpdatedSID,EventBase::statusETID,0,"PowerSrcID::UpdatedSID",1);
}

#else // PLATFORM_LOCAL

void WorldState::read(const SensorState& sensor, bool sendEvents) {
	++framesProcessed;
	
	unsigned int dif = sensor.timestamp - lastSensorUpdateTime;
	
	// motion thread polls for updates, which might not actually be any, so only assert if !sendEvents
	// if sending events, there definitely should be an increment in frame number
	ASSERT(!sendEvents || frameNumber!=sensor.frameNumber,"duplicate sensor frame processing, old " << frameNumber << " at " << get_time() << " (stamp=" << sensor.timestamp <<")");
	
	if(frameNumber!=sensor.frameNumber || lastSensorUpdateTime!=sensor.timestamp) {
		ASSERT(frameNumber<=sensor.frameNumber,"frameNumbers running in reverse, old " << frameNumber << " new " << sensor.frameNumber << " at " << get_time() << " (stamp=" << sensor.timestamp <<")");
		ASSERT(lastSensorUpdateTime<=sensor.timestamp,"timestamps running in reverse, old " << lastSensorUpdateTime << " new " << sensor.timestamp << " at " << get_time());

		frameNumber=sensor.frameNumber;
		curtime = lastSensorUpdateTime = sensor.timestamp;
		
		for(unsigned int i=0; i<PIDJointOffset; i++)
			outputs[i] = sensor.outputs[i];
		for(unsigned int i=PIDJointOffset; i<PIDJointOffset+NumPIDJoints; i++) {
			float calScale = config->motion.calibration_scale[i-PIDJointOffset];
			float calOffset = config->motion.calibration_offset[i-PIDJointOffset];
			outputs[i] = sensor.outputs[i] / calScale - calOffset;
		}
		for(unsigned int i=PIDJointOffset+NumPIDJoints; i<NumOutputs; i++)
			outputs[i] = sensor.outputs[i];
		
		if(!sendEvents) {
			// first set button_times properly:
			for(unsigned int i=0; i<NumButtons; i++) {
				if(sensor.buttons[i]>=0.1) {
					if(buttons[i]<0.1)
						button_times[i]=curtime;
				} else {
					if(buttons[i]>=0.1)
						button_times[i]=0;
				}
			}
			// this copies buttons, sensors, pids[][], and pidduties in one go:
			// (may eventually want to split sensors off to apply calibration factors...)
			memcpy(buttons,sensor.buttons,sizeof(float)*(NumButtons + NumSensors + NumPIDJoints*3 + NumPIDJoints));
		} else {
			std::vector<EventBase> evtBuf;
			evtBuf.reserve(NumButtons);
			for(unsigned int i=0; i<NumButtons; i++)
				chkEvent(evtBuf,i,sensor.buttons[i],buttonNames[i]);
			
			// this copies sensors, pids[][], and pidduties in one go:
			// (may eventually want to split sensors off to apply calibration factors...)
			memcpy(sensors,sensor.sensors,sizeof(float)*(NumSensors + NumPIDJoints*3 + NumPIDJoints));
			for(unsigned int i=0; i<evtBuf.size(); i++)
				erouter->postEvent(evtBuf[i]);
		}
	}
	
	if(sendEvents)
		erouter->postEvent(EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID,dif,"SensorSouceID::UpdatedSID",1);
}

#endif //platform-specific sensor updating

void WorldState::chkEvent(std::vector<EventBase>& evtBuf, unsigned int sid, float newval, const char* name) {
	if(newval>=0.1) { //now on
		if(buttons[sid]<0.1) { //was off: activation
			//cout << ProcessID::getIDStr() << " post activate button " << name << endl;
			evtBuf.push_back(EventBase(EventBase::buttonEGID,sid,EventBase::activateETID,0,name,newval));
			button_times[sid]=curtime;
		} else if(alwaysGenerateStatus || buttons[sid]!=newval) { //already on - always or change? : status
			unsigned int dur=curtime-button_times[sid];
			//cout << ProcessID::getIDStr() << " post status" << endl;
			evtBuf.push_back(EventBase(EventBase::buttonEGID,sid,EventBase::statusETID,dur,name,newval));
		}
	} else { //now off
		if(buttons[sid]>=0.1) { //was on: deactivation
			unsigned int dur=curtime-button_times[sid];
			button_times[sid]=0;
			//cout << ProcessID::getIDStr() << " post deactivate" << endl;
			evtBuf.push_back(EventBase(EventBase::buttonEGID,sid,EventBase::deactivateETID,dur,name,0));
		}
	}
	//update value
	buttons[sid]=newval;
}

void WorldState::applyCalibration() {
	for (unsigned int i=PIDJointOffset; i<PIDJointOffset+NumPIDJoints; i++) {
		float calScale = config->motion.calibration_scale[i-PIDJointOffset];
		float calOffset = config->motion.calibration_offset[i-PIDJointOffset];
		outputs[i] = outputs[i] / calScale - calOffset;
	}
}

/*! @file
 * @brief Implements WorldState, maintains information about the robot's environment, namely sensors and power status
 * @author ejt (Creator)
 */
