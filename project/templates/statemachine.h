//-*-c++-*-
#ifndef INCLUDED_CLASSNAME_h_
#define INCLUDED_CLASSNAME_h_

/* StateNodes are recursive data structures, can be used as either a leaf node
 * or a machine.  This template is the suggested form for a state machine, which
 * breaks a task down into one or more subnodes.
 *
 * You are of course welcome to combine the abilities of a leaf node (actual
 * execution) and a state machine (which delegates to subnodes) —
 * simply add DoStart() and processEvent(), as seen in statenode.h
 *
 * Also note there is a short hand 'fsm' notation to make state machines
 * much quicker to write: http://wiki.tekkotsu.org/index.php/State_Machines
 */

// Replace YOURNAMEHERE, CLASSNAME, and DESCRIPTION as appropriate, and go to town!


#include "Behaviors/StateNode.h"

//! DESCRIPTION
class CLASSNAME : public StateNode {

	// ****************************
	// ******* CONSTRUCTORS *******
	// ****************************
public:
	//! default constructor, uses type name as instance name
	CLASSNAME() : StateNode() {}

	//! constructor, takes an instance name
	CLASSNAME(const std::string& nm) : StateNode(nm) {}

	//! destructor, check call to teardown -- only actually necessary if you override teardown()
	~CLASSNAME() {
		if(issetup) 
			teardown();
	}

	
	// ****************************
	// ********* METHODS **********
	// ****************************
	
	//! This function should wire together any subnodes which you may desire
	virtual void setup() {
		StateNode::setup(); // good form to call superclass here
		// <your setup code here>
		startnode=/*...*/;
	}

	//! You may not need this function if the only memory allocated in
	//! setup() was subnodes and transitions
	virtual void teardown() {
		// <your teardown code here>
		StateNode::teardown(); // may delete subnodes (required)
	}

	static std::string getClassDescription() { return "DESCRIPTION"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// ****************************
	// ********* MEMBERS **********
	// ****************************
protected:
	// <class members go here>


	// ****************************
	// ********** OTHER ***********
	// ****************************
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	CLASSNAME(const CLASSNAME&); //!< don't call (copy constructor)
	CLASSNAME& operator=(const CLASSNAME&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines CLASSNAME, which DESCRIPTION
 * @author YOURNAMEHERE (Creator)
 */

#endif
