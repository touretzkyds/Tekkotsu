#include "BehaviorSwitchControl.h"
#include "Events/TextMsgEvent.h"

ControlBase * BehaviorSwitchControlBase::takeInput(const std::string& msg) {
	if(options.size()>0)
		return ControlBase::takeInput(msg);
	if(!isRunning())
		startmine();
	mybeh->processEvent(TextMsgEvent(msg,1));
	return NULL;
}

void BehaviorSwitchControlBase::setGroup(BehaviorGroup* bg) {
	if(behgrp!=NULL) {
		behgrp->members.erase(this);
		behgrp->removeReference();
	}
	behgrp=bg;
	if(behgrp!=NULL) {
		behgrp->addReference();
		behgrp->members.insert(this);
		if(mybeh!=NULL && mybeh->isActive()) {
			if(behgrp->curBehavior!=NULL) {
				behgrp->curBehavior->stop();
				notifyGroupMembers();
			}
			behgrp->curBehavior=mybeh;
		}
	}
}

ControlBase * BehaviorSwitchControlBase::activate(MC_ID display, Socket * gui) {
	if(slotsSize()==0) {
		toggle();
		return NULL;
	} else
		return ControlBase::activate(display,gui);
}

std::string BehaviorSwitchControlBase::getName() const {
	if(mybeh==NULL)
		return ControlBase::getName();
	return (mybeh->isActive()?'#':'-')+mybeh->getName();
}

std::string BehaviorSwitchControlBase::getDescription() const {
	if(mybeh==NULL)
		return ControlBase::getDescription();
	return "Class "+mybeh->getClassName()+": "+mybeh->getDescription();
}

bool BehaviorSwitchControlBase::isRunning() const {
	if(mybeh==NULL) //not created or has been destroyed, definitely not running
		return false;
	// so, beh has been created (but may have been stopped by another in the group)
	return mybeh->isActive(); //just check active flag (is valid object, we would have set it to NULL if we stopped it ourselves)
}

void BehaviorSwitchControlBase::stopother() {
	if(behgrp==NULL) {
		if(mybeh!=NULL) {
			if(mybeh->isActive())
				mybeh->stop();
			behaviorStopped();
		}
	} else if(behgrp->curBehavior!=NULL) {
		if(behgrp->curBehavior->isActive())
			behgrp->curBehavior->stop();
		notifyGroupMembers();
		behgrp->curBehavior=NULL;
	}
}

void BehaviorSwitchControlBase::startmine() {
	if(behgrp!=NULL)
		behgrp->curBehavior=mybeh;
	mybeh->start();
}

void BehaviorSwitchControlBase::notifyGroupMembers() {
	for(std::set<BehaviorSwitchControlBase*>::iterator it=behgrp->members.begin(); it!=behgrp->members.end(); ++it)
		if((*it)->mybeh==behgrp->curBehavior)
			(*it)->behaviorStopped();
}

/*! @file
 * @brief Implements BehaviorSwitchControlBase - a control for turning behaviors on and off
 * @author ejt (Creator)
 */

