//-*-c++-*-
#ifndef INCLUDED_RemoteControllerMC_h
#define INCLUDED_RemoteControllerMC_h

#include "MotionCommand.h"
#include "OutputCmd.h"
#include "Shared/RobotInfo.h"

//! This class is used for setting all outputs to a certain set of values (not the gains, just the joint positions)
/*! This is PostureMC's little brother.  Not quite so full of features, but straightforward and easy to understand. (hopefully)
 *  This is about as simple of a motion command as you'll find, so it might make for good sample code. */
class RemoteControllerMC : public MotionCommand {
public:
	//! constructor, defaults all joints at 0
	RemoteControllerMC() : MotionCommand() {}
	//! destructor
	virtual ~RemoteControllerMC() {}
	
	//! Updates all of the outputs to their current positions, every time
	/*! Any outputs which we don't set would be marked unused and be moved by a lower-priority motion command */
	virtual int updateOutputs() {
		for (unsigned int i=0; i<NumOutputs; i++)
			motman->setOutput(this, i, cmds[i]);
		return NumOutputs;
	}
	
	virtual int   isAlive() { return true; } //!< always true
	
	OutputCmd cmds[NumOutputs]; //!< current vector of positions
};

/*! @file
 * @brief Describes RemoteControllerMC, a class used for setting all outputs to a certain set of values (not the gains, just the joint positions)
 * @author alokl (Creator)
 */

#endif

