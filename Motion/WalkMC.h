//-*-c++-*-
#ifndef INCLUDED_WalkMC_h
#define INCLUDED_WalkMC_h

#include "Shared/RobotInfo.h"

// will be undef'd if none of the cases below apply
#define TGT_HAS_WALK

#if defined(TGT_HAS_WHEELS)
#  include "WheeledWalkMC.h"
#  define TGT_HAS_WHEELEDWALK
typedef WheeledWalkMC WalkMC;

#elif defined(TGT_IS_AIBO)
// could use UPennWalkMC instead...
#  include "CMPackWalkMC.h"
#  define TGT_HAS_CMPACKWALK
typedef CMPackWalkMC WalkMC;

#elif defined(TGT_HAS_LEGS)
#  include "XWalkMC.h"
#  define TGT_HAS_XWALK
typedef XWalkMC WalkMC;

#else
#undef TGT_HAS_WALK

// Or supply a no-op... but just asking for more bugs down the line
// Better to fail-fast if the robot is stationary, don't have to pretend we are able to move

/*
 #  if !defined(WALKMC_NO_WARN_NOOP)
#    warning Target robot model has neither legs nor wheels -- WalkMC is a no-op
#  endif
#  define TGT_HAS_NO_WALK
#  include "MotionCommand.h"
class WalkMC : public MotionCommand {
public:
	virtual int updateOutputs() { return 0; }
	virtual int isDirty() { return false; }
	virtual int isAlive() { return false; }
	void setTargetVelocity(float, float, float) {}
	void setTargetVelocity(float, float, float, float) {}
	void setTargetDisplacement(float, float, float, float) {}
	void zeroVelocities() {}
	size_t loadFile(const char * f) { return 0; }
	size_t saveFile(const char * f) const { return 0; }
};
*/

#endif

/*! @file
 * @brief Describes WalkMC, a wrapper for the appropriate locomotion mechanism for the current robot (including wheels)
 * @author ejt (Created)
 */

#endif
