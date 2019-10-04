//-*-c++-*-
#ifndef INCLUDED_CompletionTrans_h_
#define INCLUDED_CompletionTrans_h_

#include "Behaviors/StateNode.h"
#include "Behaviors/Transition.h"
#include "Events/EventRouter.h"

//! causes a transition when at least @e n sources have signalled completion;  @e n = 0 means "all" (default)
class CompletionTrans : public Transition {
protected:
	int minsrcs; //!< the minimum number of sources which must signal completion before this transition will fire
	bool *completions;  //!< pointer to array for recording completion events for all sources
	EventBase savedEvent; //!< Copy of current event to be used after timer expires
	
public:
	//! constructor, pass @a destination and the minimum number of sources which must signal completion before this transition will fire
	CompletionTrans(StateNode* destination, int n=0) :
	  Transition(destination), minsrcs(n), completions(NULL), savedEvent() {};
	
	//! constructor, pass @a name, @a destination and the minimum number of times the source must signal completion beyond the first (@a n)
	CompletionTrans(const std::string& name, StateNode* destination, int n=0) :
	  Transition(name,destination), minsrcs(n), completions(NULL), savedEvent() {};
	
	//! starts listening
	virtual void postStart();
	
	//! stops listening
	virtual void stop();
	
	//! record completions, and fire the transition if enough sources have completed
  virtual void doEvent();
	
protected:
	//!@name Dummy functions to satisfy the compiler
	CompletionTrans(const CompletionTrans&);  //!< don't call this
	CompletionTrans& operator=(const CompletionTrans&);  //!< don't call this
	//@}
	
};

/*! @file
 * @brief Defines Completiontrans, which causes a transition if all sources have signalled completion
 * @author dst (Creator)
 */

#endif
