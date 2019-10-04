//-*-c++-*-
#ifndef INCLUDED_ArmMC_h
#define INCLUDED_ArmMC_h

#include "MotionCommand.h"
#include "OutputCmd.h"
#include "PostureEngine.h"
#include "PostureMC.h"
#include "Shared/RobotInfo.h"
#include "Shared/mathutils.h"

class ArmMC : public MotionCommand {
public:
	//! Constructor, defaults to all joints to current value in ::state (i.e. calls takeSnapshot() automatically)
	ArmMC();
	virtual ~ArmMC() {}

	//! Constructor
	
	//!Sets #hold - if this is set to false, it will allow a persistent motion to behave the same as a pruned motion, without being pruned
	virtual void setHold(bool h=true) { hold=h; }
	virtual bool getHold() { return hold; } //!< return #hold

	virtual void setTolerance(float t) { tolerance=t; } //!< sets #tolerance
	virtual float getTolerance() { return tolerance; } //!< returns #tolerance
	virtual void setTimeout(unsigned int delay) { timeout=delay; } //!< sets #timeout
	virtual unsigned int getTimeout() { return timeout; } //!< returns #timeout

	//! sets the target to last sent commands, and dirty to false; essentially freezes motion in place
	/*! This is very similar to takeSnapshot(), but will do the "right thing" (retain current position) when motion blending is involved.
	 *  A status event will be generated if/when the joints reach the currently commanded position.
	 *  Probably should use freezeMotion() if you want to stop a motion underway, but takeSnapshot() if you want to reset/intialize to the current joint positions. */
	virtual void freezeMotion();

	//! sets the target joint positions to current sensor values
	/*! Similar to freezeMotion() when a motion is underway, but only if no other MotionCommands are using neck joints.
	  *  A status event will @e not be generated unless a motion was already underway.
	  *  Probably should use freezeMotion() if you want to stop a motion underway, but takeSnapshot() if you want to reset/intialize to the current joint positions. */
	virtual void takeSnapshot();

	//!@name Speed Control
	
	//! Sets #maxSpeed to 0 (no maximum)
	void noMaxSpeed() { for(unsigned int i=0; i<NumArmJoints; i++) maxSpeed[i]=0; }

	//! Restores #maxSpeed to default settings from Config::Motion_Config
	/*! @param x ratio of the max speed to use; so 0.5 would limit motion to half the recommended upper limit */
	void defaultMaxSpeed(float x=1);

	//! Sets #maxSpeed for all joints in rad/sec
	void setMaxSpeed(float x) { for(unsigned int i=0; i<NumArmJoints; i++) setMaxSpeed(i,x); }

	//! Sets #maxSpeed for join @a i in rad/sec
	/*! @param i joint offset relative to ArmOffset (i.e. one of TPROffset_t)
	 *  @param x maximum radians per second to move */
	void setMaxSpeed(unsigned int i, float x) { maxSpeed[i]=x*FrameTime/1000; }

	//! Set speed of gripper joints
	void setGripperSpeed(float x);

	//! Returns #maxSpeed in rad/sec
	/*! @param i joint offset relative to ArmOffset (i.e. one of TPROffset_t)
	 *  @return the maximum speed of joint @a i in radians per second */
	float getMaxSpeed(unsigned int i) { return maxSpeed[i]*1000/FrameTime; }

	//@}
	
	//!@name Joint Accessors
	
	//! Sets the weight values for all the arm joints
	void setWeight(float w);	
	void setWeight(int x, float w);
	float armJointValue(unsigned int i);


	void setJointValue(unsigned int i, float value) {
#ifdef TGT_HAS_ARMS
		if(!ensureValidJoint(i)) return;
		setWeight(i, 1);
		armTargets[i]=clipAngularRange(ArmOffset+i,value);
		setDirty();
#endif
	}

	//! Returns the target value of joint @a i.  Use this if you want to know the current @b commanded joint value; To get the current joint @b position, look in WorldState::outputs
	/*! @param i joint offset relative to ArmOffset (i.e. one of TPROffset_t) */
	float getJointValue(unsigned int i) const {
		if(ensureValidJoint(i))
			return armTargets[i];
		else
			return 0;
	}

	void setJoints(float shoulder, float elbow, float wrist);  //!< Set joint values for a three-link arm
	void setJoints(float shoulder, float elbow, float yaw, float pitch, float roll, float gripper); //!< Set joint values for a Chiara-style arm
	void setWrist(float pitch, float roll, float gripper);

	//! Pulse the gripper to prevent load errors
	void setGripperPulse(unsigned int onPeriod, unsigned int offPeriod);
	void clearGripperPulse();

	//-----------------------------NEW CODE-----------------------------
	//! Sets the desired load value for subsequent grip operations
	void requestGripperLoad(int newLoad);
	//! Returns the current active load on the gripper's servos
	int getGripperLoad();
	//! Sets the member #angleIncrement to the specified value (default = 0.05)
	/*! This is only applicable for CALLIOPE2 and will do nothing for any other robot.	*/
	void setGraspSpeed(float speed);
	//! Sets the member #idleCycles to the specified value (default = 8)
	/*! This is only applicable for CALLIOPE2 and will do nothing for any other robot.
			The purpose of this parameter is to control how fast often #angleIncrement is used.
			If it is used too often, the gripper will move too fast, generate an invalid load,
			and then not close correctly.*/
	void setGraspWait(unsigned int cycles);
	//! Returns the value of desiredLoad last set by the user
	int getDesiredLoad() { return desiredLoad; }
	//------------------------------------------------------------------

	//! Move arm to specified point in base frame coordinates; return false if unreachable
	bool moveToPoint(float x, float y, float z) { return moveOffsetToPoint(fmat::Column<3>(), fmat::pack(x,y,z)); }
	bool moveToPoint(const fmat::Column<3>& tgt) { return moveOffsetToPoint(fmat::Column<3>(), tgt); }
	bool moveOffsetToPoint(const fmat::Column<3>& offset, const fmat::Column<3>& tgt);
	bool moveOffsetToPointWithOrientation(const fmat::Column<3>& offset, const fmat::Column<3>& tgt, const fmat::Quaternion& ori);
	
	//! Move the fingers or gripper to achieve the specified gap
	bool setFingerGap(float dist);

	//! Open the gripper to a certain percentage of its range of travel.
	bool openGripper(float percentage=0.5);

	void setOutputCmd(unsigned int i, const OutputCmd& c) { 
#ifdef TGT_HAS_ARMS
		dirty=true; 
		completionReported=false; 
		armTargets[i-ArmOffset]=c.value;
		if(armCmds[i-ArmOffset].weight==0)
			armCmds[i-ArmOffset].value = state->outputs[i];
		armCmds[i-ArmOffset].weight = c.weight;
#endif
	}
	
public:
	//!@name Inherited:
	virtual int updateOutputs(); //!< Updates where the arm is looking
	virtual int isDirty() { return (dirty || !completionReported) ? NumArmJoints : 0; } //!< true if a change has been made since the last updateJointCmds() and we're active
	virtual int isAlive(); //!< Alive while target is not reached
	virtual void doStart() { MotionCommand::doStart(); setDirty(); } //!< marks this as dirty each time it is added
	//@}


	//! if completionReported, copies armCmds from MotionManager::getOutputCmd(), then sets dirty to true and completionReported to false
	/*! should be called each time a joint value gets modified in case
	 *  the arm isn't where it's supposed to be, it won't jerk around
	 * 
	 *  MotionManager::getOutputCmd() is called instead of
	 *  WorldState::outputs[] because if this is being called rapidly
	 *  (i.e. after every sensor reading) using the sensor values will
	 *  cause problems with very slow acceleration due to sensor lag
	 *  continually resetting the current position.  Using the last
	 *  value sent by the MotionManager fixes this.*/
	void setDirty();

protected:
         /*//! checks if target point or direction is actually reachable
        bool isReachable(const NEWMAT::ColumnVector& Pobj) {
	  NEWMAT::ColumnVector poE=armkin.convertLink(0,armkin.get_dof())*Pobj;
	  const float theta = mathutils::rad2deg(acos(poE(3)/sqrt(poE(1)*poE(1)+poE(2)*poE(2)+poE(3)*poE(3))));
	  //	  cout << "theta: " << theta << " degrees\n";
	  return theta < 5.0;
	}*/

	//! puts x in the range (-pi,pi)
	static float normalizeAngle(float x) { return x - static_cast<float>( rint(x/(2*M_PI)) * (2*M_PI) ); }
	
	//! if @a x is outside of the range of joint @a i, it is set to either the min or the max, whichever is closer
	static float clipAngularRange(unsigned int i, float x) {
		float min=outputRanges[i][MinRange];
		float max=outputRanges[i][MaxRange];
		if(x<min || x>max) {
			float mn_dist=std::abs(normalizeAngle(min-x));
			float mx_dist=std::abs(normalizeAngle(max-x));
			if(mn_dist<mx_dist)
				return min;
			else
				return max;
		} else
			return x;
	}

	//! Makes sure @a i is in the range (0,NumArmJoints).  If it is instead in the range (ArmOffset,ArmOffset+NumArmJoints), output a warning and reset @a i to the obviously intended value.
	/*! @param[in] i joint offset relative to either ArmOffset (i.e. one of TPROffset_t) or 0
	 *  @param[out] i joint offset relative to ArmOffset (i.e. one of TPROffset_t)
	 *  @return true if the intended joint could be ascertained, false otherwise */
	static bool ensureValidJoint(unsigned int& i);

	void incrementGrasp(); //!< Helper function to execute a grasp
	

	bool dirty;                          //!< true if a change has been made since last call to updateJointCmds()
	bool  hold;                          //!< if set to true, the posture will be kept active; otherwise joints will be marked unused after each posture is achieved (as if the posture was pruned); set through setHold()
	float tolerance;                     //!< when autopruning, if the maxdiff() of this posture and the robot's current position is below this value, isAlive() will be false, defaults to 0.01 (5.7 degree error)
	bool completionReported;                  //!< true if the most recent movement request has completed
	unsigned int targetTimestamp;        //!< time at which the completionReported flag was set
	unsigned int timeout;                //!< number of milliseconds to wait before giving up on a target that should have already been reached, a value of -1U will try forever
	float armTargets[NumArmJoints];    //!< stores the target value of each joint
	OutputCmd armCmds[NumArmJoints];   //!< stores the last values we sent from updateOutputs
	float maxSpeed[NumArmJoints];       //!< initialized from Config::motion_config, but can be overridden by setMaxSpeed(); rad per frame
	unsigned int pulseOnPeriod; //!< number of milliseconds to keep servo power on, when pulsing
	unsigned int pulseOffPeriod; //!< number of milliseconds to keep servo power off, when pulsing
	unsigned int pulseStartTime; //!< when the current pulse cycle started
	signed int desiredLoad;	//!< how tightly to grasp an object (negative is for tighter loads; -280 is a good value)
	float angleIncrement;	//!< How fast to close the gripper (0.05 by default)
	unsigned int idleCycles;	//!< How long to wait before updating the gripper angle by #angleIncrement (8 by default)
};

#endif
