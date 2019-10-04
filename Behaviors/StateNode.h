//-*-c++-*-
#ifndef INCLUDED_StateNode_h_
#define INCLUDED_StateNode_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/DataEvent.h"
#include "Events/EventRouter.h"
#include <vector>
#include <string>

class Transition;

//! Recursive data structure - both a state machine controller as well as a node within a state machine itself
/*!
 *  Override setup() to build your own Transition and StateNode network if you want
 *  this node to contain a state machine.
 *
 *  Override doStart() / doStop() as you would a normal BehaviorBase subclass to
 *  have this node add some functionality of its own.
 *  
 *  You can override setup to create a sub-network, as well as overriding doStart and doStop, in the same class.
 *  
 *  See also the <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/state.shtml">tutorial page on State Machines</a>.
 *  
 *  There are two StateNode templates in <a href="http://cvs.tekkotsu.org/cgi/viewcvs.cgi/Tekkotsu/project/templates/"><i>project</i><tt>/templates/</tt></a>:
 *  - <a href="http://cvs.tekkotsu.org/cgi/viewcvs.cgi/Tekkotsu/project/templates/statenode.h?rev=HEAD&content-type=text/vnd.viewcvs-markup">statenode.h</a>
 *    is intended for leaf nodes, which directly implement the execution of a task.
 *  - <a href="http://cvs.tekkotsu.org/cgi/viewcvs.cgi/Tekkotsu/project/templates/statemachine.h?rev=HEAD&content-type=text/vnd.viewcvs-markup">statemachine.h</a>
 *    is intended for nodes which contain a network of transitions and subnodes, which together solve the task.
 */
class StateNode  : public BehaviorBase {
	friend class Transition;
public:
	//!constructor, class name from typeid is used as instance name
	StateNode()
		: BehaviorBase(), parent(NULL), transitions(), issetup(false), 
		  retain(true), startedTime(0), nodes(), startnode(NULL), speechText()
	{}
	//!constructor, pass a name to use
	StateNode(const std::string& name)
		: BehaviorBase(name), parent(NULL), transitions(), issetup(false), 
		  retain(true), startedTime(0), nodes(), startnode(NULL), speechText()
	{}

	//!destructor, removes references to its outgoing transitions (be careful of incoming ones - they're still around!), and calls removeReference() on subnodes
	virtual ~StateNode();

	//!Adds the specified StateTransition to the transition table
	virtual Transition* addTransition(Transition* trans);

	//!Returns the std::vector of transitions so you can modify them yourself if need be
	std::vector<Transition*>& getTransitions() { return transitions; }

	//!Returns the const std::vector of transitions so you can read through them if need be
	const std::vector<Transition*>& getTransitions() const { return transitions; }

	//!Adds a StateNode to #nodes so it can be automatically dereferenced later, returns what it's passed (for convenience), calls addReference() on @a node.  Also sets the node's parent to @c this if it is null.
	virtual StateNode* addNode(StateNode* node);
	//!Adds a StateNode to #nodes so it can be automatically dereferenced later, returns what it's passed (for convenience), calls addReference() on @a node.  Also sets the node's parent to @c this if it is null.
	template<class T> T* addNode(T* node) { addNode(dynamic_cast<StateNode*>(node)); return node; }

	//!Returns the std::vector of sub-nodes so you can modify them yourself if need be
	std::vector<StateNode*>& getNodes() { return nodes; }

	//!Returns the const std::vector of sub-nodes so you can read through them if need be
	const std::vector<StateNode*>& getNodes() const { return nodes; }

	//!Sets the retain flag - if not retained, will removeReference() subnodes upon stop() and recreate them on start (by calling setup()) - may be too expensive to be worth saving memory...
	void setRetain(bool f) { retain=f; }

	//!Transitions should call this when entering the state, so it can enable its transitions
	virtual void start();

	//!This is called by start() when it should setup the network of subnodes (if any)
	virtual void setup() {}

	//!Transitions should call this when leaving the state, so it can disable its transitions
	virtual void stop();
	
	//!This is called by stop() when you should destruct subnodes
	/*!Default implementation will take care of the subnodes and their
	 * transitions, you only need to worry about any *other* memory
	 * which may have been allocated.  If none, you may not need
	 * implement this function at all. */
	virtual void teardown();

	//!returns pointer to #parent as a generic StateNode
	virtual StateNode* getParent() const { return parent; }

	//! returns a correctly typed pointer to ancestor of the specified type; user by $reference
	template<class T> T* getAncestor() const;

	//! returns correctly typed pointer to #parent for access to its members
	template<class T> T* parentAs() const { return dynamic_cast<T*>(getParent()); }

	//! returns node with specified name that is a child of this node
	virtual StateNode* getChild(const std::string& name) const;

	//! returns node with specified name that is a child of #parent, or of parent's parent, etc.
	virtual StateNode* getSibling(const std::string& name) const;

	//! Set text to speak on node start
	virtual void setSpeechText(const std::string &text) { speechText=text; }

	//! Extract data from a signal sent to this state via Transition::fire(const EventBase&)
	template<typename T>
	static const T extractSignal(const EventBase *ev) {
	  const DataEvent<T> *datev = dynamic_cast<const DataEvent<T>*>(ev);
	  if ( datev == NULL )
	    return T();
	  else
	    return datev->getData();
	}

	//! Try to extract data from a signal sent to this state via Transition::fire(const EventBase&)
	template<typename T>
	static const T* tryExtractSignal(const EventBase *ev) {
	  const DataEvent<T> *datev = dynamic_cast<const DataEvent<T>*>(ev);
	  if ( datev == NULL )
	    return NULL;
	  else
	    return &(datev->getData());
	}

	//! Values used in a SignalTrans for postStateSuccess and postStateFailure
	enum SuccessOrFailure { failureSignal=0, successSignal=1 };

protected:
	//!will post an activation event through stateMachineEGID, used when doStart() is called
	virtual void postStateStart();

	//!will post an deactivation event through stateMachineEGID, used when doStop() is called
	/* @param duration the value to use for EventBase::duration -- nice but not generally used */
	virtual void postStateStop();

	//!will post a status event through stateMachineEGID to signal "completion" of the node
	/*! "completion" is defined by your subclass - will mean different things to different
	 *  nodes depending on the actions they are performing.  So call this yourself if there
	 *  is a natural ending point for your state.
	 * @param magnitude the value to use for EventBase::magnitude -- generally is 1 for status events, but since this is to signal completion, 0 (the default) may be more appropriate; if your node is doing something repetitive however, 1 (or the loop count) may be better */
	virtual void postStateCompletion(float magnitude=0);

	//! posts a DataEvent<SuccessOrFailure>(failure) indicating failure of the node; meaning of failure is used-defined
	virtual void postStateFailure() { postStateSignal<SuccessOrFailure>(failureSignal); }

	//! posts a DataEvent<SuccessOrFailure>(success) indicating success of the node; meaning of success is used-defined
	virtual void postStateSuccess() { postStateSignal<SuccessOrFailure>(successSignal); }

	//! Causes the parent state machine to complete; use this to return from a nested state machine
	virtual void postParentCompletion() { getParent()->postStateCompletion(); }

	//! Causes the parent state machine to fail; use this to return failure from a nested state machine
	virtual void postParentFailure() { getParent()->postStateFailure(); }

	//! Causes the parent state machine to succeed; use this to return success from a nested state machine
	virtual void postParentSuccess() { getParent()->postStateSuccess(); }

	//! Causes the parent state machine to post a DataEvent through stateSignalEGID.  @a value is optional.
	template<typename T> void postParentSignal(const T& value=T()) {
	  getParent()->postStateSignal<T>(value);
	}

	//! Posts a DataEvent through stateSignalEGID that can be picked up by a SignalTrans.  @a value is optional.
	template<typename T> void postStateSignal(const T& value=T()) {
		erouter->postEvent(DataEvent<T>(value, EventBase::stateSignalEGID,(size_t)this, EventBase::statusETID,0,getName(),0));
	}

	//Node Stuff:
	//! pointer to the machine that contains this node
	StateNode* parent;
	//! a vector of outgoing transitions
	std::vector<Transition*> transitions;
	
	//Machine Stuff:
	//! this is set to true if the network has been setup but not destroyed (i.e. don't need to call setupSubNodes again)
	bool issetup;
	//! this is set to true if the network should be retained between activations.  Otherwise it's dereferenced upon doStop(). (or at least removeReference() is called on subnodes)
	bool retain;
	//! the timestamp of last call to start()
	unsigned int startedTime;
	//! vector of StateNodes, just so they can be dereferenced again on doStop() (unless retained) or ~StateNode()
	std::vector<StateNode*> nodes;
	
	//! starting node (should be a member of @a nodes)
	StateNode* startnode;

	//! Text to speak on node start (for debugging)
	std::string speechText;

private:
	StateNode(const StateNode& node); //!< don't call this
	StateNode operator=(const StateNode& node); //!< don't call this
	virtual void DoStartEvent(struct __USE_doStart_AND_CHECK_event_NOT_DoStartEvent__&) {} //!< temporary to produce warnings to help people update
};

template<class T> T* StateNode::getAncestor() const {
  const StateNode *p = this;
  while ( (p = p->getParent()) != NULL ) {
    if ( const T* ancestor = dynamic_cast<const T*>(p) )
      return const_cast<T*>(ancestor);
  }
  std::cerr << "StateNode::getAncestorAs() failed to find ancestor of requested type\n";
  return NULL;
}
  


/*! @file
 * @brief Describes StateNode, which is both a state machine controller as well as a node within a state machine itself
 * @author ejt (Creator)
 */

#endif
