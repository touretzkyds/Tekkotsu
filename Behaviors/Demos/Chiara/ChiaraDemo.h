//-*-c++-*-
#ifndef INCLUDED_ChiaraDemo_h_
#define INCLUDED_ChiaraDemo_h_

#include "Behaviors/StateNode.h"
#include "Events/EventRouter.h"
#include "Motion/XWalkMC.h"
#include "Motion/MotionManager.h"
#include "IPC/SharedObject.h"

//! DESCRIPTION
class ChiaraDemo : public StateNode {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
public:
	//! default constructor, use type name as instance name
	ChiaraDemo()
		: StateNode(), walk()
	{}

	//! constructor, take an instance name
	ChiaraDemo(const std::string& nm)
		: StateNode(nm), walk()
	{}
	
	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	virtual void setup();
	virtual void teardown() {
		StateNode::teardown();
		motman->removeMotion(walk->getID());
	}
	
	static std::string getClassDescription() { return "DESCRIPTION"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	SharedObject<XWalkMC> walk;
	

	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	ChiaraDemo(const ChiaraDemo&); //!< don't call (copy constructor)
	ChiaraDemo& operator=(const ChiaraDemo&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines ChiaraDemo, which DESCRIPTION
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
