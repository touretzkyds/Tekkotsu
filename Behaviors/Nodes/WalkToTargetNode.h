//-*-c++-*-
#ifndef INCLUDED_WalkToTargetNode_h_
#define INCLUDED_WalkToTargetNode_h_

#include "Behaviors/StateNode.h"
#include "Motion/MotionManager.h"

//! a state node for walking towards a visual target
class WalkToTargetNode : public StateNode {
public:
	//!constructor, pass VisionObjectSourceID_t
	WalkToTargetNode(unsigned int obj)
		: StateNode(),tracking(obj),
			walker_id(MotionManager::invalid_MC_ID), headpointer_id(MotionManager::invalid_MC_ID) 
	{}
	
	//!constructor, pass instance name and VisionObjectSourceID_t
	WalkToTargetNode(const std::string& nodename, unsigned int obj)
		: StateNode(nodename),tracking(obj),
			walker_id(MotionManager::invalid_MC_ID), headpointer_id(MotionManager::invalid_MC_ID) 
	{}
	
	static std::string getClassDescription() { return "walks towards a visual target, using some basic logic for moving the head to track it"; }
	virtual std::string getDescription() const { return getClassDescription(); }

	virtual Transition* newDefaultLostTrans(StateNode* dest);  //!< returns a suggested transition for detecting "lost" condition, but you don't have to use it
	virtual Transition* newDefaultCloseTrans(StateNode* dest); //!< returns a suggested transition for detecting "close to target" condition, but you don't have to use it

protected:
	virtual void doStart();
	virtual void doStop();
	//! uses head to watch ball, walks towards it
	virtual void doEvent();
	
	unsigned int tracking; //!< the object being tracked
	MotionManager::MC_ID walker_id; //!< so we can walk
	MotionManager::MC_ID headpointer_id; //!< so we can point the head at the object

private:
	WalkToTargetNode(const WalkToTargetNode&); //!< don't call this
	WalkToTargetNode operator=(const WalkToTargetNode&); //!< don't call this
};

/*! @file
 * @brief Describes WalkToTargetNode, a state node for walking towards a visual target
 * @author ejt (Creator)
 */

#endif
