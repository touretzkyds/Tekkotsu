#ifndef INCLUDED_PIDNode_h_
#define INCLUDED_PIDNode_h_

#include "MCNode.h"
#include "Motion/PIDMC.h"

//! Default name for PIDNodes (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of PITNode.cc) to avoid file bloat */
extern const char defPIDNodeName[];

//!default description for PIDNode (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc (instead of PIDNode.cc) to avoid file bloat */
extern const char defPIDNodeDesc[];

//! A simple StateNode that executes a PIDMC motion command
class PIDNode : public MCNode<PIDMC,defPIDNodeName,defPIDNodeDesc,true> {
 public:
	//! constructor, take an instance name
	PIDNode(const std::string& nm) : MCNode<PIDMC,defPIDNodeName,defPIDNodeDesc,true>(nm),
																	 low(0), high(0), powerlevel(0), weight(0) {}

	//! constructor, take PIDMC arguments; note that @high should be one past the last joint to modify
	PIDNode(const std::string& nm, unsigned int _low, unsigned int _high, float _powerlevel, float _weight=1) :
		MCNode<PIDMC,defPIDNodeName,defPIDNodeDesc,true>(nm),
		low(_low), high(_high), powerlevel(_powerlevel), weight(_weight) {}

	virtual void preStart() {
		MCNode<PIDMC,defPIDNodeName,defPIDNodeDesc,true>::preStart();
		getMC()->setRangePowerLevel(PIDJointOffset, low, 0.f, 0.f);
		getMC()->setRangePowerLevel(low, high, powerlevel, weight);
		getMC()->setRangePowerLevel(high, PIDJointOffset+NumPIDJoints, 0.f, 0.f);
	}

private:
	unsigned int low;
	unsigned int high;
	float powerlevel;
	float weight;

};

#endif
