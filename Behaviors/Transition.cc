#include "Transition.h"
#include "StateNode.h"
#include "Wireless/Wireless.h"
#include "Sound/SoundManager.h"
#include "Events/EventRouter.h"
#include "Shared/debuget.h"

std::list<Transition*> Transition::firing;

Transition::~Transition() {
	ASSERT(!isActive(),"Transition " << getName() << " destructing while active (forgot stop()?)");
	ASSERT(std::find(firing.begin(),firing.end(),this)==firing.end(),"Transition " << getName() << " still active in firing queue!");
	if(eventData) {
		delete eventData;
		eventData = NULL;
	}
}

void Transition::stop() {
	eventData = NULL;
	firing.remove(this);
	BehaviorBase::stop();
}

void Transition::fire() {
	ASSERT(eventData==NULL, getName() << " firing with event still buffered")
	if(std::find(firing.begin(), firing.end(), this) == firing.end())
		firing.push_back(this);
	else {
		std::cout << "Trying to fire " << getName() << " but it's already on the call stack." << std::endl;
		showFiring();
	}
	
	if(firing.front()==this)  // only do firing from root of call stack...
		fireDownCallStack();
}

void Transition::fire(const EventBase& ev) {
	ASSERT(eventData==NULL, getName() << " firing with previous event already buffered, overwriting...");
	if ( eventData != NULL )
		std::cout << "Previous=" << eventData->getName() << " @ " << event->getTimeStamp()
							<< "    New=" << ev.getName() << " @ " << ev.getTimeStamp() << std::endl;
	eventData = ev.clone();
	if(std::find(firing.begin(), firing.end(), this) == firing.end())
		firing.push_back(this);
	else {
		std::cout << "Trying to fire " << getName() << " but it's already on the call stack." << std::endl;
		showFiring();
	}
	
	if(firing.front()==this)  // only do firing from root of call stack...
		fireDownCallStack();
}

void Transition::fireDownCallStack() {
	while(!firing.empty())
		firing.front()->doFire();
}

void Transition::doFire() {
	ASSERTRET(isActive(),"doFire from " << getName() << " but not active");
	
	addReference(); //just in case a side effect of this transition is to dereference the transition, we don't want to be deleted while still transitioning
	
	if(soundFile.size()!=0)
		sndman->playFile(soundFile);
#ifdef PLATFORM_APERIOS
	if ( !speechText.empty() )
		sout->printf("Speak: %s\n",speechText.c_str());
#else
	if ( !speechText.empty() )
		sndman->speak(speechText);
#endif
	
	erouter->postEvent(EventBase::stateTransitionEGID,
										 reinterpret_cast<size_t>(this),
										 EventBase::activateETID,0,getName(),1);
	
	//HACK - This prevents recursive transitions from thinking they're at the top of the call stack
	firing.push_front(NULL);
	
	const EventBase* eventDataSave = eventData; // save on stack because Transition::stop() will clear it
	for(unsigned int i=0; i<srcs.size(); i++) {
		try {
			if(srcs[i]->isActive()) //It's usually a bad idea to call stop/start when it's already stopped/started...
				srcs[i]->stop();
		} catch(const std::exception& ex) {
			std::cout << "Exception '" << ex.what() << "' thrown during transition "
								<< getName() << ", deactivating source node " << srcs[i]->getName() << std::endl;
		} catch(...) {
			std::cout << "Exception thrown during transition " << getName()
								<< ", deactivating source node " << srcs[i]->getName() << std::endl;
			firing.pop_front();
			removeReference();
			throw;
		}
	}
	// sets BehaviorBase::event for duration of start call and then restores it
	for(unsigned int i=0; i<dsts.size(); i++) {
		const EventBase* prevEvent = dsts[i]->event; // in case of recursive events triggered by doEvent
		dsts[i]->event=eventDataSave;
		try {
			if(!dsts[i]->isActive())
				dsts[i]->start();
		} catch(const std::exception& ex) {
			std::cout << "Exception '" << ex.what() << "' thrown during transition "
								<< getName() << ", activating destination node " << dsts[i]->getName() << std::endl;
		} catch(...) {
			std::cout << "Exception thrown during transition " << getName()
								<< ", activating destination node " << dsts[i]->getName() << std::endl;
			firing.pop_front();
			removeReference();
			throw;
		}
		dsts[i]->event=prevEvent;
	}

	delete eventDataSave;
	firing.pop_front();  // get rid of the NULL blocking recursive transitions
	erouter->postEvent(EventBase::stateTransitionEGID,
										 reinterpret_cast<size_t>(this),
										 EventBase::deactivateETID,0,getName(),0);
	
	if ( isActive() ) {
		if ( firing.size()>0 ) { // might be 0 if self-transition
			ASSERTIF(firing.front()==this, "doFire from " << getName() << " but it's not my turn, should be " << firing.front()->getName()) {
				firing.pop_front();
			}
		}
	} else {
		ASSERT(std::find(firing.begin(),firing.end(),this)==firing.end(),"Transition still in firing queue following doFire, subclass overridden stop() and forgot to call Transition::stop()?");
	}

	removeReference();
}

void Transition::showFiring() {
	std::cout << "firing =";
	for (std::list<Transition*>::const_iterator it = firing.begin();
			 it != firing.end(); it++) {
		std::cout << " ";
		if ( *it == NULL )
			std::cout << "0";
		else
			std::cout << (*it)->instanceName;
	}
	std::cout << std::endl;
}

std::string Transition::getName() const {
	if(instanceName != getClassName()) {
		return instanceName;
	} else {
		std::string ans;
		if ( srcs.size() == 1 )
		  ans = srcs[0]->getName();
		else {
		  ans = '{';
		  for(unsigned int i=0; i<srcs.size(); i++)
		    ans += srcs[i]->getName() + (i<srcs.size()-1?',':'}');
		}
		ans += ">==" + instanceName + "==>";
		if ( dsts.size() == 1 )
		  ans += dsts[0]->getName();
		else {
		  ans += '{';
		  for(unsigned int i=0; i<dsts.size(); i++)
			ans += dsts[i]->getName() + (i<dsts.size()-1?',':'}');
		}
		return ans;
	}
}

/*! @file
 * @brief Implements Transition, represents a transition between StateNodes.
 * @author ejt (Creator)
 */

