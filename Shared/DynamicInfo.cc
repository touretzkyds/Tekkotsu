#include "Shared/DynamicInfo.h"

namespace DynamicInfo {

	const char* TargetName="Dynamic";

	unsigned NumWheels=0;
	
	unsigned JointsPerArm=0;
	unsigned NumArms=0;
	unsigned NumArmJoints=0;
	
	unsigned JointsPerLeg=0;
	unsigned NumLegs=0;
	unsigned NumLegJoints=0;
	unsigned NumHeadJoints=0;
	unsigned NumTailJoints=0;
	unsigned NumMouthJoints=0;
	unsigned NumEarJoints=0;
	unsigned NumButtons=0;
	unsigned NumSensors=0;
	unsigned NumLEDs=0;
	unsigned NumFacePanelLEDs=0;
	
	unsigned NumPIDJoints=0;
	unsigned NumOutputs=0;
	unsigned NumReferenceFrames=0;

	unsigned PIDJointOffset=0;
	unsigned LEDOffset=0;
	unsigned BaseFrameOffset=0;
	
	std::vector<const char *> buttonNames;
	std::vector<const char *> sensorNames;
	std::vector<const char *> outputNames;
	
	DynamicCapabilities capabilities;
	
	std::vector<DynamicInfoRow<float,3> > DefaultPIDs;
		
	std::vector<float> MaxOutputSpeed;

	std::vector<DynamicInfoRow<float,2> > outputRanges;

	std::vector<DynamicInfoRow<float,2> > mechanicalLimits;
}
	
/*! @file
 * @brief Defines RobotInfo namespace for 'dynamic' models, gives some information about the robot's capabilities, such as joint counts, offsets, names and PID values
 * @author ejt (Creator)
 */
