//-*-c++-*-
#ifndef INCLUDED_TorqueCalibrate_h
#define INCLUDED_TorqueCalibrate_h

#include "Shared/RobotInfo.h"
#if !defined(TGT_HAS_BUTTONS) && !defined(TORQUE_CALIBRATE_NO_WARN_NOOP)
#	warning TorqueCalibrate control needs some kind of trigger, target model does not have any buttons?
#endif

#include "Behaviors/Controls/ControlBase.h"
#include "Behaviors/Controls/FileInputControl.h"
#include "Motion/MotionManager.h"
#include "Events/EventListener.h"
#include "Shared/Config.h"
#include "Shared/WorldState.h"

//! Provides an interface for making measurements to correlate PID duty cycle and actual force output for each of the motors
class TorqueCalibrate : public ControlBase {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
	// Not all of these necessarily make sense to implement... feel free
	// to remove those which don't -- none are required.

public:
	//! default constructor
	TorqueCalibrate()
		: ControlBase("TorqueCalibrate","Provides an interface for making measurements to correlate PID duty cycle and actual force output for each of the motors"),
		filename(config->portPath("data/torque.dat")), filenameInput(NULL)
	{init();}
	//! constructor which allows a custom name
	TorqueCalibrate(const std::string& n)
		: ControlBase(n,"Provides an interface for making measurements to correlate PID duty cycle and actual force output for each of the motors"),
		filename(config->portPath("data/torque.dat")), filenameInput(NULL)
	{init();}
	//! constructor which allows a custom name and tooltip
	TorqueCalibrate(const std::string& n, const std::string& d)
		: ControlBase(n,d),
		filename(config->portPath("data/torque.dat")), filenameInput(NULL)
	{init();}

	//! destructor
	virtual ~TorqueCalibrate() {}

protected:
	class TakeMeasurementControl : public ControlBase, public EventListener  {
	public:
		TakeMeasurementControl(const TorqueCalibrate& tcParent, unsigned int jointToMeasure)
			: ControlBase(outputNames[jointToMeasure]), parent(tcParent), joint(jointToMeasure),
			basePosition(state->outputs[jointToMeasure]), maxDuty(0), sensorDist(0), cstate(ZERO_JOINT),
			pidID(invalid_MC_ID), pulseID(invalid_MC_ID)
		{}
		virtual ControlBase * activate(MC_ID disp_id, Socket * gui);
		virtual void processEvent(const EventBase& event);
		virtual void refresh();
		virtual ControlBase * takeInput(const std::string& msg);
		virtual void deactivate();
	protected:
		//! the states the TakeMeasurementControl goes through when recording measurements
		enum State_t {
			ZERO_JOINT,  //!< turn off PID of joints, allow user to reposition them to the force sensor
			RECORD_POSITION, //!< record the length of the lever arm (distance from point of rotation to force sensor)
			INPUT_PULSE, //!< wait for user to specify size of pulse to perform
			DO_PULSE, //!< make the joint do the pulse
			RECORD_FORCE //!< wait for user to report the recorded force applied
		};
		
		//! requests a transition to another state
		void transition(State_t newstate);
		
		const TorqueCalibrate& parent;
		unsigned int joint;
		float basePosition;
		float maxDuty;
		float sensorDist;
		State_t cstate;
		MC_ID pidID;
		MC_ID pulseID;
	};
	
	//! initialization
	virtual void init() {
		pushSlot(filenameInput=new FileInputControl("Storage: ","Location where data will be appended to any previous contents",""));
		pushSlot(NULL);
		for(unsigned int i=PIDJointOffset; i<PIDJointOffset+NumPIDJoints; i++)
			pushSlot(new TakeMeasurementControl(*this,i));
		
		filenameInput->setAcceptNonExistant(true);
		filenameInput->takeInput(filename);
		filename=""; //force refresh
	}


	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	void record(unsigned int joint, float sensorDist, float maxDuty, float maxForce) const;
	virtual void refresh();
	

	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	std::string filename;
	FileInputControl * filenameInput;


	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	TorqueCalibrate(const TorqueCalibrate&); //!< you can override, but don't call this...
	TorqueCalibrate& operator=(const TorqueCalibrate&); //!< you can override, but don't call this...
};

/*! @file
 * @brief Defines TorqueCalibrate, which provides an interface for making measurements to correlate PID duty cycle and actual force output for each of the motors
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
#endif
