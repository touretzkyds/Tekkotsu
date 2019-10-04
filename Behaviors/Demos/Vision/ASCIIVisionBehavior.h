//-*-c++-*-
#ifndef INCLUDED_ASCIIVisionBehavior_h_
#define INCLUDED_ASCIIVisionBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"

//! streams low-resolution ASCII-art of the camera image to sout
class ASCIIVisionBehavior : public BehaviorBase {
public:
	//! constructor
	ASCIIVisionBehavior() : BehaviorBase("ASCIIVisionBehavior") {}

	static const unsigned int charMapSize=18;  //!< the number of available characters for levels of "gray"
	static const char charMap[charMapSize+1]; //!< the included characters sorted in order of darkness - could be improved... (less is more sometimes)

	virtual void doStart() { erouter->addListener(this,EventBase::visRawCameraEGID); }

	virtual void doEvent();

	static std::string getClassDescription() { return "Streams low-resolution ASCII-art of the camera image to sout"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Describes ASCIIVisionBehavior, which streams low-resolution ASCII-art of the camera image to sout
 * @author ejt (Creator)
 */

#endif
