//-*-c++-*-
#ifndef INCLUDED_LGNode_h_
#define INCLUDED_LGNode_h_

#include "Behaviors/StateNode.h"
#include "Wireless/LGmixin.h"

//! A simple StateNode that provides the Looking Glass display methods via LGmixin
class LGNode : public StateNode, public LGmixin {
public:
	//! constructor
	LGNode() : StateNode(), LGmixin() {}

	//! constructor, specify instance name
	LGNode(const std::string& nodename) : StateNode(nodename), LGmixin() {}

};

#endif
