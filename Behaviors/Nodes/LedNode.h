//-*-c++-*-
#ifndef INCLUDED_LedNode_h_
#define INCLUDED_LedNode_h_

#include "MCNode.h"
#include "Motion/LedMC.h"

//! StateNode that executes a LedMC motion command and posts a status event upon completion
/*! Extends MCNode slightly so that each time the LedMC is accessed, any flash commands are reset.
 *  This allows a flash to be triggered each time the node starts */
class LedNode : public MCNode<LedMC> {
public:
	//! default constructor, use type name as instance name
	LedNode() : MCNode<LedMC>(), lastAccess(0) {}
	
	//! constructor, take an instance name
	LedNode(const std::string& nm) : MCNode<LedMC>(nm), lastAccess(0) {}
	
	static std::string getClassDescription() { return "Displays a pattern on the LEDs for as long as the state is active"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	//! extends MCNode implementation so that each time the LedMC is accessed, any flash commands are reset.
	virtual SharedObject<LedMC>& getPrivateMC() {
		unsigned int curtime=get_time();
		bool isFirstCreation=(mc==NULL);
		SharedObject<LedMC>& lobj=MCNode<LedMC>::getPrivateMC();
		if(!isFirstCreation)
			lobj->extendFlash(curtime-lastAccess);
		lastAccess=curtime;
		return lobj;
	}
	
	unsigned int lastAccess; //!< stores time of last call to getPrivateMC() for resetting flash commands
};

//! Simpler version of LedNode that turns on the specified LEDs on entry and explicitly turns them off again on exit.
class LedActivate : public LedNode {
public:
  //! constructor
  LedActivate(const std::string &nm, LEDBitMask_t mask) : LedNode(nm), theMask(mask) {
    SharedObject<LedMC>& lobj=getPrivateMC();
    lobj->setWeights(AllLEDMask,0);
    lobj->setWeights(theMask, 1);
    lobj->set(theMask, 1);
}

  virtual void stop() { // explicitly turn off the LEDs we turned on before
    LedNode::stop();
    SharedObject<LedMC> douser;
    douser->setWeights(AllLEDMask,0);
    douser->setWeights(theMask, 1);
    douser->set(theMask, 0);
    motman->addPrunableMotion(douser);
  }

protected:
  LEDBitMask_t theMask;
};



/*! @file
 * @brief Defines LedNode and LedActivate, StateNodes that run a LedMC motion command and post a status event upon completion
 * @author dst (Creator)
 */

#endif
