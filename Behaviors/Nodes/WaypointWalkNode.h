//-*-c++-*-
#ifndef INCLUDED_WaypointWalkNode_h_
#define INCLUDED_WaypointWalkNode_h_

#include "MCNode.h"
#include "Motion/MotionManager.h"
#define WALKMC_NO_WARN_NOOP
#include "Motion/WaypointWalkMC.h"
#undef WALKMC_NO_WARN_NOOP
#include "Events/EventRouter.h"
#include "Events/LocomotionEvent.h"
#include "DualCoding/VRmixin.h"

//!default name for WaypointEngineNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defWaypointWalkNodeName[];
//!default description for WaypointWalkNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defWaypointWalkNodeDesc[];

//! A StateNode for doing a waypoint walk, use the template parameter to specify a custom walk MC, or use the ::WaypointWalkNode typedef to accept the "default" walk
template<typename W, const char* mcName=defWaypointWalkNodeName, const char* mcDesc=defWaypointWalkNodeDesc>
class WaypointEngineNode : public MCNode<W,mcName,mcDesc> {
public:
	
	//! constructor
	WaypointEngineNode() : MCNode<W,mcName,mcDesc>() {}
	
	//! constructor
	WaypointEngineNode(const std::string& name) : MCNode<W,mcName,mcDesc>(name) {}
	
	//!destructor
	~WaypointEngineNode() {}
	
	virtual void postStart() {
		// std::cout << "WaypointwalkNode id=" << MCNode<W,mcName,mcDesc>::getMC_ID() << "="
		//           << MCNode<W,mcName,mcDesc>::getMC()->getID() << " start" << std::endl;
		MCNode<W,mcName,mcDesc>::postStart();
		if ( MCNode<W,mcName,mcDesc>::isActive() ) {
		  erouter->addListener(this, EventBase::locomotionEGID, MCNode<W,mcName,mcDesc>::getMC_ID(), EventBase::statusETID);
		  MCNode<W,mcName,mcDesc>::getMC()->go();
		}
		erouter->addTimer(this, 9999, 2000, true);

		DualCoding::VRmixin::isWalkingFlag = true;
	}
	
	virtual void stop() {
		{ MMAccessor<W> wp_walker = MCNode<W,mcName,mcDesc>::getMC();
		  wp_walker->pause(); // prevent cycle() from running and setting the velocity again
		  wp_walker->zeroVelocities();
		}
		// std::cout << "WaypointwalkNode id=" << MCNode<W,mcName,mcDesc>::getMC_ID() << "="
		//           << MCNode<W,mcName,mcDesc>::getMC()->getID() << " stop" << std::endl;
		DualCoding::VRmixin::autoRefreshSketchWorld();
		MCNode<W,mcName,mcDesc>::stop();

		DualCoding::VRmixin::isWalkingFlag = false;
	}

protected:
	//! constructor
	WaypointEngineNode(const std::string& className, const std::string& instanceName) : 
	MCNode<W,mcName,mcDesc>(className,instanceName) {}
	
	void doEvent() {
		if ( MCNode<W,mcName,mcDesc>::event->getGeneratorID() == EventBase::timerEGID && MCNode<W,mcName,mcDesc>::event->getSourceID() == 9999 )
			DualCoding::VRmixin::autoRefreshSketchWorld();
		else if ( static_cast<const LocomotionEvent&>(*MCNode<W,mcName,mcDesc>::event).isStop() )
			MCNode<W,mcName,mcDesc>::postStateCompletion();
	}
	
};

//! the prototypical WaypointWalkNode, using a WaypointWalkMC
typedef WaypointEngineNode<WaypointWalkMC> WaypointWalkNode;

/*! @file
 * @brief Describes WaypointEngineNode,  a StateNode for doing a waypoint walk; use the template parameter to specify a custom waypoint walk MC, or use the WaypointWalkNode typedef to accept the "default" walk
 * @author dst (Creator)
 */

#endif
