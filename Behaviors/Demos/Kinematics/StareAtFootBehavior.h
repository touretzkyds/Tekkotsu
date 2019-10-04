//-*-c++-*-
#ifndef INCLUDED_StareAtFootBehavior_h_
#define INCLUDED_StareAtFootBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Shared/RobotInfo.h"
#include "Motion/HeadPointerMC.h"
#include "Motion/MotionPtr.h"

//! Uses kinematics to track the foot with the camera; on the Aibo you can press the paw buttons to switch feet
class StareAtFootBehavior : public BehaviorBase {
public:
	//! constructor
	StareAtFootBehavior()
		: BehaviorBase("Stare At Foot"), lastLeg(0), pointer()
	{ }

	virtual void doStart();
	virtual void doEvent();

#ifdef TGT_IS_AIBO
	static std::string getClassDescription() { return "Uses kinematics to track the foot with the camera; press the paw buttons to switch feet"; }
#else
	static std::string getClassDescription() { return "Uses kinematics to track the foot with the camera; press any button to increment target foot"; }
#endif
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	const char* getInterestPointName() const; // looks up an appropriate interest point relative to #lastLeg (i.e. a Toe on Aibo, center of foot otherwise)
	
	unsigned int lastLeg; //!< index of last leg to have it's button pressed, i.e. the one we are looking at
	MotionPtr<HeadPointerMC> pointer; //!< the HeadPointerMC we are using to do the looking
};

/*! @file
 * @brief Describes StareAtFootBehavior, which uses kinematics to track the foot with the camera; on the Aibo you can press the paw buttons to switch feet
 * @author ejt (Creator)
 */

#endif
