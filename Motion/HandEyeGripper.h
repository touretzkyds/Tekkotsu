#ifndef INCLUDED_HandEyeGripper_h
#define INCLUDED_HandEyeGripper_h

#include <math.h>

#include "Shared/WorldState.h"

class HandEyeGripper {
	public:
	void OpenClose(int OriJoint, float gripperValue, float *gripperServo) {
#ifdef TGT_HAS_ARMS
		float negscale = -1;
		float posscale = -1;
		unsigned int fingerOffset = 1;
#ifdef TGT_HANDEYEZ
		negscale = -0.5f;
		posscale = -0.5f;
#endif		
		
		for(unsigned int counter = ArmOffset + OriJoint + fingerOffset; counter < ArmOffset + NumArmJoints; counter++) {
			if (gripperValue < 0) {
				gripperServo[counter] = (gripperValue+1)/2 * negscale;
			}
			else {
				gripperServo[counter] = (gripperValue+1)/2 * posscale;
			}
		}
#endif
	}
};

#endif
