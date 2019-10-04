#ifndef KODU_ACTION_H_
#define KODU_ACTION_H_

// C++ Library
#include <iostream>
#include <string>

// Kodu Library
#include "Kodu/Primitives/KoduPrimitive.h"
#include "Kodu/General/GeneralMacros.h"

// KoduState
#include "Kodu/General/KoduState.h"

namespace Kodu {
        
	//! Kodu Action (derived from Kodu Behavior)
	class KoduAction : public KoduPrimitive {
	public:
		//! The different action types
		enum ActionTypes {
			AT_DO_NOTHING = 0,
			AT_DROP,
			AT_GRAB,
			AT_GIVE,
			AT_MOTION,
			AT_PAGE_SWITCH,
			AT_PLAY,
			AT_SAY,
			AT_SCORING
		};

		//! Constructor
		KoduAction(const std::string& kActionName, ActionTypes actType, bool actionCanUseOnceMod, bool runOnce) :
			KoduPrimitive(kActionName), actionType(actType), onceModEnabled(actionCanUseOnceMod && runOnce), state()
			{ }
        
		//! Copy constructor
		explicit KoduAction(const KoduAction& kAction) : KoduPrimitive(kAction),
			actionType(kAction.actionType), onceModEnabled(kAction.onceModEnabled), state(kAction.state)
			{ }

		//! Destructor
		virtual ~KoduAction () {}
	 // no explicit implementation
        
	//! Assignment operator
	KoduAction& operator=(const KoduAction& kAction) {
		if (this != &kAction) {
			KoduPrimitive::operator=(kAction);
			actionType = kAction.actionType;
			onceModEnabled = kAction.onceModEnabled;
			state = kAction.state;
		}
		return *this;
	}

	//! Returns whether or not the action can ran
	bool canRun() const {
		return KoduPrimitive::agentCanUsePrimitive();
	}

	//! Checks if the once modifier is enabled
	bool onceModIsEnabled() const {
		return onceModEnabled;
	}

	//! Used to reinitialize certain variables (e.g. when switching to another page)
	virtual void reinitialize() {
		KoduPrimitive::reinitialize();
	}
	/*
	//! Sets the action can run variable
	void setActionCanRun(bool bVal) {
	actionCanRun = bVal;
	}
	*/
	//! Returns the action type
	ActionTypes getActionType() const { return actionType; }

	//! Prints the attributes for a particular behavior
	virtual void printAttrs() const {
		KoduPrimitive::printAttrs();
		PRINT_ATTRS("Once modifier enabled", onceModEnabled);
	}
        
	void setKoduState(const KoduState& curstate){
		state = curstate;
	}
 
 private:
	ActionTypes actionType; //!< states the action type
	bool onceModEnabled;    //!< states if the once modifier is enabled (depends on action type too)
	KoduState state;
};

} // end of Kodu namespace

#endif // end of KODU_ACTION_H_
