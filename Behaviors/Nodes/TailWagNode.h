//-*-c++-*-
#ifndef INCLUDED_TailWagNode_h_
#define INCLUDED_TailWagNode_h_

#include "MCNode.h"
#include "Motion/TailWagMC.h"

// You don't actually need to declare extern strings in order to use
// MCNode, but it's nice...  If you left the name and description
// off, it would substitute MCNode's default values, but that would
// yield rather ambiguous debugging output

//!default name for TailWagNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defTailWagNodeName[];
//!default description for TailWagNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defTailWagNodeDesc[];

//! A simple StateNode that executes a TailWagMC motion command
class TailWagNode : public MCNode<TailWagMC,defTailWagNodeName,defTailWagNodeDesc,false> {
public:
	//! default constructor, use type name as instance name
	TailWagNode() : MCNode<TailWagMC,defTailWagNodeName,defTailWagNodeDesc,false>() {}
	
	//! constructor, take an instance name
	TailWagNode(const std::string& nm) : MCNode<TailWagMC,defTailWagNodeName,defTailWagNodeDesc,false>(nm) {}
};

/*! @file
 * @brief Defines TailWagNode, a simple StateNode that runs a TailWagMC motion command
 * @author dst, ejt (Creators)
 */

#endif
