#include "StateNode.h"
#include "Transition.h"
#include "Events/EventRouter.h"
#include "Sound/SoundManager.h"
#include "Wireless/Wireless.h"
#include "Shared/debuget.h"

StateNode::~StateNode() {
	ASSERT(!isActive(), "Destructing while active?")
	for(std::vector<Transition*>::iterator it=transitions.begin(); it!=transitions.end(); it++)
		(*it)->removeReference();
	if(issetup) {
		teardown();
		if(issetup) {
			serr->printf("WARNING %s doesn't seem to call StateNode::teardown() in its\n"
			             "        implementation of the function: issetup=%d, nodes.size()=%lu\n"
			             "        Attempting to recover...\n",getClassName().c_str(),issetup,(unsigned long)nodes.size());
			StateNode::teardown();
		}
	}
}

Transition* StateNode::addTransition(Transition* trans) {
	transitions.push_back(trans);
	trans->addReference();
	trans->addSource(this);
	return trans;
}

StateNode* StateNode::addNode(StateNode* node) {
	nodes.push_back(node);
	node->addReference();
	if ( node->parent == NULL )
		node->parent = this;
	return node;
}

StateNode* StateNode::getChild(const std::string& name) const {
	  for (std::vector<StateNode*>::const_iterator it = nodes.begin(); it != nodes.end(); it++)
	    if ( name == (*it)->getName() ) return *it;
	  return NULL;
}

StateNode* StateNode::getSibling(const std::string& name) const {
	  if ( parent == NULL )
	    return parent;
	  for (std::vector<StateNode*>::const_iterator it = parent->nodes.begin(); it != parent->nodes.end(); it++)
	    if ( name == (*it)->getName() ) return *it;
	  return parent->getSibling(name);
}

struct ScopeReference {
	ScopeReference(ReferenceCounter& rc) : ref(rc) { ref.addReference(); }
	~ScopeReference() { ref.removeReference(); }
	ReferenceCounter& ref;
};

/*! This could be implemented slightly more simply as a pre/postStart() pair, but
 *  keeping our specialization in start() itself allows us to keep pre/postStart empty
 *  (thus more robust to subclass overrides forgetting to call their superclass version,
 *  but also avoids any potential subclass mis-handling of transition fire while in preStart) */
void StateNode::start() {
	if(started)
		return;
	started=true; // use started to watch for immediate transition
	ScopeReference ref(*this); // keep a self-reference for this scope, releases automatically on throw/return
	postStateStart();
#ifdef PLATFORM_APERIOS
	if ( !speechText.empty() )
		sout->printf("Speak: %s\n",speechText.c_str());
#else
	if ( !speechText.empty() )
		sndman->speak(speechText);
#endif
	if ( parent == NULL && transitions.size() > 0 )
		serr->printf("WARNING StateNode '%s' has transitions but no parent; you probably forgot to call addNode().\n",getName().c_str());
	if ( ! issetup ) {
		setup();
		issetup = true;
	}
	// Must start the transitions before we start the behavior, because the behavior could
	// post something that triggers a transition.  But if a transition fires and tries to
	// shut the behavior down, we must detect that and not try to start the behavior afterward.
	for(std::vector<Transition*>::iterator it=transitions.begin(); it!=transitions.end(); it++) {
		if ( !(*it)->isActive()  )
			(*it)->start();
		if(!isActive()) //a transition fired upon its start
			return;
	}
	started=false; // reset flag otherwise BehaviorBase::start() will be a no-op
	BehaviorBase::start();
	if ( isActive() && startnode )
		startnode->start();
}

void StateNode::stop() {
	for(std::vector<Transition*>::iterator it=transitions.begin(); it!=transitions.end(); it++) {
		if((*it)->isActive())
			(*it)->stop();
	}
	for(std::vector<StateNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++)
		if((*it)->isActive())
			(*it)->stop();
	if(!retain && issetup) {
		teardown();
		if(issetup) {
			serr->printf("WARNING %s doesn't seem to call StateNode::teardown() in its\n"
			             "        implementation of the function: issetup=%d, nodes.size()=%lu\n"
			             "        Attempting to recover...\n",getClassName().c_str(),issetup,(unsigned long)nodes.size());
			StateNode::teardown();
		}
	}
	postStateStop();
	BehaviorBase::stop();
}

void StateNode::teardown() {
	for(std::vector<StateNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++)
		(*it)->removeReference();
	startnode = NULL;
	nodes.clear();
	issetup=false;
	/*std::cout << "Teardown!!!!!!!!" << std::endl;*/
}

void StateNode::postStateStart() {
	erouter->postEvent(EventBase::stateMachineEGID,reinterpret_cast<size_t>(this),EventBase::activateETID,0,getName(),1);
}

void StateNode::postStateStop() {
	erouter->postEvent(EventBase::stateMachineEGID,reinterpret_cast<size_t>(this),EventBase::deactivateETID,get_time()-startedTime,getName(),0);
}

void StateNode::postStateCompletion(float magnitude/*=0*/) {
	erouter->postEvent(EventBase::stateMachineEGID,reinterpret_cast<size_t>(this),EventBase::statusETID,get_time()-startedTime,getName(),magnitude);
}


/*! @file 
 * @brief Describes StateNode, which is both a state machine controller as well as a node within a state machine itself
 * @author ejt (Creator)
 */

