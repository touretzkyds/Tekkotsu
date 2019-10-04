//-*-c++-*-
#ifndef INCLUDED_DynamicRobotState_h_
#define INCLUDED_DynamicRobotState_h_

#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"

//! This class provides a dynamic storage and serialization of robot state, such as output positions, buttons, and sensors
/*! WorldState provides static storage for sensors, combined with PostureEngine for output values.
 *  In addition SensorState (from the hardware abstraction layer's DataSource) also serves this role.
 *  However all of these are based on the RobotInfo's declaration of offsets and array lengths, whereas
 *  this class relies on named entries which may or may not map to the current model */
class DynamicRobotState : public plist::Dictionary {
public:
	//! constructor
	DynamicRobotState() : outputs(), buttons(), sensors(), torques(), framePositions(), frameOrientations() { init(); }
	//! constructor
	DynamicRobotState(const DynamicRobotState& drs) : outputs(drs.outputs), buttons(drs.buttons), sensors(drs.sensors), torques(drs.torques), framePositions(drs.framePositions), frameOrientations(drs.framePositions) { init(); }
	
	plist::DictionaryOf<plist::Primitive<float> > outputs; //!< values for robot effectors, e.g. joint angles
	plist::DictionaryOf<plist::Primitive<float> > buttons; //!< values for buttons and generally sensors which should generate an event when a threshold is crossed
	plist::DictionaryOf<plist::Primitive<float> > sensors; //!< values for other sensors
	plist::DictionaryOf<plist::Primitive<float> > torques; //!< force information corresponding to #outputs

	plist::DictionaryOf<plist::Point> framePositions; //!< reference frame locations
	plist::DictionaryOf<plist::Point> frameOrientations; //!< reference frame orientations (as axis component of quaternion, use fmat::Quaternion::fromAxis(x) to "rehydrate")

protected:
	//! adds members as plist entries for serialization
	void init() {
		addEntry("Outputs",outputs);
		addEntry("Buttons",buttons);
		addEntry("Sensors",sensors);
		addEntry("Torques",torques);
		addEntry("FramePositions",framePositions);
		addEntry("FrameOrientations",frameOrientations);
		setLoadSavePolicy(FIXED,SYNC);
	}
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
