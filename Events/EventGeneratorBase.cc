#include "EventGeneratorBase.h"
#include "Events/EventRouter.h"

using namespace std;

void
EventGeneratorBase::doStart() {
	BehaviorBase::doStart();
	if(autoListen && getGeneratorID()!=EventBase::numEGIDs) {
		erouter->addListener(this,EventBase::erouterEGID,getGeneratorID());
		if(hasListeners())
			addSrcListener();
	}
}

void
EventGeneratorBase::doStop() {
	erouter->removeListener(this);
	isListening=false;
	BehaviorBase::doStop();
}

void
EventGeneratorBase::processEvent(const EventBase& ev) {
	if(!autoListen || getGeneratorID()==EventBase::numEGIDs) {
		BehaviorBase::processEvent(ev);
		return;
	}
	if(ev.getGeneratorID()!=EventBase::erouterEGID) {
		BehaviorBase::processEvent(ev); // trap erouter events for ourselves
	} else {
		if(hasListeners()) {
			addSrcListener();
		} else {
			removeSrcListener();
		}
	}
}

void
EventGeneratorBase::setAutoListen(EventBase::EventGeneratorID_t gid) {
	removeSrcListener();
	autoListen=true;
	srcGenID=gid;
	specificity=GENERATOR;
	if(gid==EventBase::numEGIDs) {
		autoListen=false;
		return;
	}
	if(isActive())
		erouter->addListener(this,EventBase::erouterEGID,getGeneratorID());
	if(hasListeners())
		addSrcListener();
}

void
EventGeneratorBase::setAutoListen(EventBase::EventGeneratorID_t gid, size_t sid) {
	removeSrcListener();
	autoListen=true;
	srcGenID=gid;
	srcSourceID=sid;
	specificity=SOURCE;
	if(gid==EventBase::numEGIDs) {
		autoListen=false;
		return;
	}
	if(isActive())
		erouter->addListener(this,EventBase::erouterEGID,getGeneratorID());
	if(hasListeners())
		addSrcListener();
}

void
EventGeneratorBase::setAutoListen(EventBase::EventGeneratorID_t gid, size_t sid, EventBase::EventTypeID_t tid) {
	removeSrcListener();
	autoListen=true;
	srcGenID=gid;
	srcSourceID=sid;
	srcTypeID=tid;
	specificity=TYPE;
	if(gid==EventBase::numEGIDs || tid==EventBase::numETIDs) {
		autoListen=false;
		return;
	}
	if(isActive())
		erouter->addListener(this,EventBase::erouterEGID,getGeneratorID());
	if(hasListeners())
		addSrcListener();
}

void
EventGeneratorBase::unsetAutoListen() {
	removeSrcListener();
	autoListen=false;
}

bool
EventGeneratorBase::hasListeners() const {
	return erouter->hasListeners(getGeneratorID(),getSourceID());
}

void
EventGeneratorBase::addSrcListener() {
	if(isListening || getListenGeneratorID()==EventBase::numEGIDs)
		return;
	isListening=true;
	switch(specificity) {
	case GENERATOR:
		erouter->addListener(this,getListenGeneratorID());
		break;
	case SOURCE:
		erouter->addListener(this,getListenGeneratorID(),getListenSourceID());
		break;
	case TYPE:
		if(getListenTypeID()==EventBase::numETIDs)
			return;
		erouter->addListener(this,getListenGeneratorID(),getListenSourceID(),getListenTypeID());
		break;
	}
}

void
EventGeneratorBase::removeSrcListener() {
	if(!isListening)
		return;
	isListening=false;
	switch(specificity) {
	case GENERATOR:
		erouter->removeListener(this,getListenGeneratorID());
		break;
	case SOURCE:
		erouter->removeListener(this,getListenGeneratorID(),getListenSourceID());
		break;
	case TYPE:
		erouter->removeListener(this,getListenGeneratorID(),getListenSourceID(),getListenTypeID());
		break;
	}
}



/*! @file
 * @brief 
 * @author ejt (Creator)
 */

