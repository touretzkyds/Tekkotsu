//-*-c++-*-
#ifndef INCLUDED_ArmNode_h_
#define INCLUDED_ArmNode_h_

#include "MCNode.h"
#include "Motion/ArmMC.h"

// You don't actually need to declare extern strings in order to use
// MCNode, but it's nice...  If you left the name and description
// off, it would substitute MCNode's default values, but that would
// yield rather ambiguous debugging output

//!default name for ArmNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of ArmNode.cc) to avoid file bloat */
extern const char defArmNodeName[];
//!default description for ArmNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of ArmNode.cc) to avoid file bloat */
extern const char defArmNodeDesc[];

//! A simple StateNode that executes a ArmMC motion command
class ArmNode : public MCNode<ArmMC,defArmNodeName,defArmNodeDesc,true> {
public:
	//! default constructor, use type name as instance name
	ArmNode() : MCNode<ArmMC,defArmNodeName,defArmNodeDesc,true>() {}
	
	//! constructor, take an instance name
	ArmNode(const std::string& nodename) : MCNode<ArmMC,defArmNodeName,defArmNodeDesc,true>(nodename) {}
};


/*! @file
 * @brief Defines ArmNode, a simple MCNode that runs an ArmMC motion command and throws a status event upon completion
 * @author dst (Creator)
 * @author ejt (Rewrote using MCNode)
 */

#endif

