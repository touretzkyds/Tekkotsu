//-*-c++-*-
#ifndef INCLUDED_CLASSNAME_h_
#define INCLUDED_CLASSNAME_h_

/* Transitions are just another behavior subclass, the main addition
 * being the Transition base class provides a fire() function for
 * you to call when your transition decides it is appropriate. */

// Replace YOURNAMEHERE, CLASSNAME, and DESCRIPTION as appropriate, and go to town!


#include "Behaviors/Transition.h"

//! DESCRIPTION
class CLASSNAME : public Transition {
public:
	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
	
	//! default constructor, use type name as instance name
	CLASSNAME(StateNode* destination)
		: Transition("CLASSNAME",destination)
	{}

	//! constructor, take an instance name
	/* usually transitions aren't named (they can autogenerate a name), so feel free to delete this one */
	CLASSNAME(const std::string& nm, StateNode* destination)
		: Transition("CLASSNAME",nm,destination)
	{}
	
	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //

	virtual void doEvent() {
		// <your event processing code here>
		// Call fire() (a method of the Transition base class) whenever the
		// environment satisifies whatever condition this Transition is supposed to
		// monitor for.  This triggers all the machinery to cause the transition.
	}

	static std::string getClassDescription() { return "DESCRIPTION"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	//! Just like a behavior, called when it's time to start doing your thing
	virtual void doStart() {
		// <your startup code here>
		// e.g. erouter->addListener(this, ... );
		// (Event IDs are found in Events/EventBase.h)

		// Don't call DoStop() yourself (as behaviors can do) -- let the source
		// StateNode do it when a transition (perhaps this one) fires
	}
	
	//! Just like a behavior, called when it's time to stop doing your thing
	virtual void doStop() {
		// <your shutdown code here> (if any)
	}


protected:
	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
	
	// <class members go here>


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
