//-*-c++-*-
#ifndef INCLUDED_HeadPointerNode_h_
#define INCLUDED_HeadPointerNode_h_

#include "MCNode.h"
#include "Motion/HeadPointerMC.h"

// You don't actually need to declare extern strings in order to use
// MCNode, but it's nice...  If you left the name and description
// off, it would substitute MCNode's default values, but that would
// yield rather ambiguous debugging output

//!default name for HeadPointerNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of HeadPointerNode.cc) to avoid file bloat */
extern const char defHeadPointerNodeName[];
//!default description for HeadPointerNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of HeadPointerNode.cc) to avoid file bloat */
extern const char defHeadPointerNodeDesc[];

//! A simple StateNode that executes a HeadPointerMC motion command
class HeadPointerNode : public MCNode<HeadPointerMC,defHeadPointerNodeName,defHeadPointerNodeDesc,true> {
public:
	//! default constructor, use type name as instance name
	HeadPointerNode() : MCNode<HeadPointerMC,defHeadPointerNodeName,defHeadPointerNodeDesc,true>() {}
	
	//! constructor, take an instance name
	HeadPointerNode(const std::string& nm) : MCNode<HeadPointerMC,defHeadPointerNodeName,defHeadPointerNodeDesc,true>(nm) {}
};


/*! @file
 * @brief Defines HeadPointerNode, a simple StateNode that runs a HeadPointerMC motion command and throws a status event upon completion
 * @author dst (Creator)
 * @author ejt (Rewrote using MCNode)
 */

#endif
