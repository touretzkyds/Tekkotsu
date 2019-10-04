//-*-c++-*-
#ifndef INCLUDED_PIDMC_h_
#define INCLUDED_PIDMC_h_

#include "MotionCommand.h"
#include "MotionManager.h"
#include "OutputPID.h"
#include "Events/EventBase.h"

//! A nice little MotionCommand for manually manipulating the PID values
/*! This will, by default, autoprune after its PID values have been set.
 *  
 *  Although this doesn't handle directly sending PID commands to the
 *  system (the MotionManager::updatePIDs() does that) a little
 *  documentation on how Tekkotsu ad OPEN-R handle the PIDs might be
 *  nice.
 *
 *  In Tekkotsu, each of P,I, and D are a single floating point
 *  number.  However, OPEN-R has, in essence, a limited precision
 *  floating point number.  Each value is broken into a gain
 *  (mantissa) and shift (exponent).  The shifts appear to be 4 bit
 *  values, and are inverted.  In other words, x = g/(1<<(0x10-s)), or
 *  @f$ x = \frac{g}{2^{16-s}} @f$ The gain is probably 6-8 bits of
 *  resolution.
 *
 *  On the ERS-2xx series, each joint is completely independent.  One
 *  caveat is that the shift value 0x0A gives a warning
 *  (AGRMSDriver::SetGain() : 0x0A IS USED FOR GAIN SHIFT VALUE.) for
 *  some unknown reason.
 *
 *  On the ERS-7, all the joints share the last set shift values, so
 *  a global set of shifts must be enforced.  This of course, pretty
 *  much kills the whole point of having the shifts.
 *
 *  To understand the conversion from Tekkotsu format to the
 *  OPEN-R format, see MotionManager::updatePIDs().
 *  
 *  A final note: the OPENR::SetJointGain function seems to be
 *  a rather costly function call.  You should probably try to avoid
 *  setting PIDs at too high a frequency.
 */
class PIDMC : public MotionCommand {
public:
	//!Constructor, uses default PIDs and 0 weight for all
	PIDMC() : MotionCommand(), dirty(false), completionReported(false) {
		setDefaults(0);
		dirty=false;
	}
	//!Constructor, sets general power level of all 
	PIDMC(float powerlevel, float w=1) : MotionCommand(), dirty(false), completionReported(false) {
		setAllPowerLevel(powerlevel,w);
	}
	//!Constructor, sets general power level of a range of joints, uses default and 0 weight for others, see setRangePowerLevel()
	/*! @param low the first joint to apply @a powerlevel to
	 *  @param high one-past last joint to apply @a powerlevel to (i.e. exclusive upper limit)
	 *  @param powerlevel scaling factor for all of the default PID parameters
	 *  @param w MotionManager weight for averaging conflicting commands
	 *
	 *  Note that if you want to set a single joint @e i, then @a low = @e i, and @a high = @e i + 1 because high is an exclusive upper limit*/
	PIDMC(unsigned int low, unsigned int high, float powerlevel, float w=1) : 
	  MotionCommand(), dirty(false), completionReported(false) {
		setRangePowerLevel(PIDJointOffset,low,1.f,0.f);
		setRangePowerLevel(low,high,powerlevel,w);
		setRangePowerLevel(high,PIDJointOffset+NumPIDJoints,1.f,0.f);
	}

	//!Destructor
	virtual ~PIDMC() {}

	//!Inherited
	//@{
	virtual int updateOutputs() {
	  for(unsigned int i=0; i<NumPIDJoints; i++)
	    motman->setOutput(this,i+PIDJointOffset,PIDs[i]);
	  int wasDirty=dirty;
	  dirty=false;
	  if ( ! completionReported ) {
	    postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
	    completionReported = true;
	  }
	  return wasDirty;
	}
	virtual int isDirty() { return dirty; }
	virtual int isAlive() { return dirty; }

	void setDirty() { dirty = true; }

	//! marks this as dirty each time it is added
	virtual void doStart() {
	  MotionCommand::doStart();
	  completionReported = false;
	  dirty=true;
	}
	//@}

	//!Sets the PIDs to the defaults specified in RobotInfo
	void setDefaults(float weight=1) {
		setAllPowerLevel(1.f,weight);
	}

	//!Sets the PIDs to a percentage of default for a given joint, and sets weight
	void setJointPowerLevel(unsigned int i, float p, float w=1) {
		i-=PIDJointOffset;
		for(unsigned int j=0;j<3;j++)
			PIDs[i].pid[j]=DefaultPIDs[i][j]*p;
		PIDs[i].weight=w;
		dirty=true;
	}

	//!Sets the PIDs to a percentage of default for all joints
	void setAllPowerLevel(float p, float w=1) {
		for(unsigned int i=0; i<NumPIDJoints; i++) {
			for(unsigned int j=0;j<3;j++)
				PIDs[i].pid[j]=DefaultPIDs[i][j]*p;
			PIDs[i].weight=w;
		}
		dirty=true;
	}

	//!Sets a range of joints' PIDs to a given power level and weight
	/*! @param low the first joint to apply power level @a p to
	 *  @param high one-past last joint to apply power level @a p to (i.e. exclusive upper limit)
	 *  @param p scaling factor for all of the default PID parameters
	 *  @param w MotionManager weight for averaging conflicting commands
	 *
	 *  Note that if you want to set a single joint @e i with this function (as opposed to setJointPowerLevel() which is designed for that...)
	 *  then you would need to pass @a low = @e i, and @a high = @e i + 1 because high is an exclusive upper limit*/
	void setRangePowerLevel(unsigned int low, unsigned int high, float p, float w=1) {
		low-=PIDJointOffset;
		high-=PIDJointOffset;
		for(unsigned int i=low; i<high; i++) {
			for(unsigned int j=0;j<3;j++)
				PIDs[i].pid[j]=DefaultPIDs[i][j]*p;
			PIDs[i].weight=w;
		}
		dirty=true;
	}


	//!Use this to set the PID value and weight
	void setPID(unsigned int i, const OutputPID& pid) {
		i-=PIDJointOffset;
		PIDs[i]=pid;
		dirty=true;
	}

	//!Use this if you want to double check the PID you set
	OutputPID& getPID(unsigned int i) {
		return PIDs[i-PIDJointOffset];
	}

	//!Use this if you want to double check the PID you set
	const OutputPID& getPID(unsigned int i) const {
		return PIDs[i-PIDJointOffset];
	}

protected:
	//! returns true if the output i is a PID joint
	static inline bool isPID(unsigned int i) {
		return ((int)i>=(int)PIDJointOffset && i<PIDJointOffset+NumPIDJoints); //casting to int just to get rid of compiler warning.. sigh
	}

	bool dirty; //!< true if there are changes that have not been picked up by Motion
	bool completionReported; //!< true if we've reported completion of this motion command
	OutputPID PIDs[NumPIDJoints]; //!< the PIDs being requested
};

/*! @file
 * @brief Defines PIDMC, a nice little MotionCommand for manually manipulating the PID values
 * @author ejt (Creator)
 */

#endif
