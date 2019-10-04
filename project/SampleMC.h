//-*-c++-*-
#ifndef INCLUDED_SampleMC_h_
#define INCLUDED_SampleMC_h_

#include "Shared/WorldState.h"
#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"

//!a simple MotionCommand which mirrors the legs
class SampleMC : public MotionCommand {
public:
	// Constructor
	SampleMC() : MotionCommand(), source(0) {
		// by default, mirror the robot's left front leg
		setSource(LFrLegOrder);
	}

	// Abstract functions:
	// These must be defined by every MotionCommand
	virtual int updateOutputs() {
		// update positions of all leg joints
		for(unsigned int i=0; i<NumLegJoints; i++) {
			unsigned int joint=i%JointsPerLeg; // find corresponding joint in source leg
			float source_pos=state->outputs[source+joint]; // get its current position
			// assign it to current joint
			motman->setOutput(this,LegOffset+i,source_pos);
		}
		
		// keep power to source leg turned off so it's back-drivable
		for(unsigned int i=0; i<JointsPerLeg; i++) {
			//zeros for the PID value 'turn off' the joint
			motman->setOutput(this,source+i,OutputPID(0,0,0));
		}
		
		//by convention, return the number of dirty joints (or 1 if unknown)
		return NumLegJoints;
	}
	virtual int isDirty() {
		//just assume the source leg is always
		//moving, so we're always dirty
		return true;
	}
	virtual int isAlive() {
		//there's not really an 'end condition' for this
		//motion, so we're always alive
		return true;
	}

	// "Local" Functions:
	// These provide customization to the task at hand (paw?)
	virtual void setSource(LegOrder_t leg) {
		source=leg*JointsPerLeg+LegOffset;
	}
	virtual LegOrder_t getSource() {
		return static_cast<LegOrder_t>((source-LegOffset)/JointsPerLeg);
	}

protected:
	unsigned int source;               /* the joint offset of the leg to mirror */
};

/*! @file
 * @brief Defines the SampleMC, a simple MotionCommand which mirrors the legs
 * @author ejt (Creator)
 */

#endif
