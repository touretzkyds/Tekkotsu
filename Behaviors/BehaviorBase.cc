#include "BehaviorBase.h"
#include "Events/EventRouter.h"
#include "IPC/SharedObject.h"
#include "Shared/string_util.h"
#include "Motion/MotionManager.h"
#include "Shared/MarkScope.h"

BehaviorBase::BehaviorBase()
	: ReferenceCounter(), EventListener(), autoMotions(), started(false), instanceName(), event(NULL)
{
	getRegistryInstance().insert(this);
}

BehaviorBase::BehaviorBase(const std::string& name)
	: ReferenceCounter(), EventListener(), autoMotions(), started(false), instanceName(name), event(NULL)
{
	getRegistryInstance().insert(this);
}

BehaviorBase::BehaviorBase(const BehaviorBase& b)
	: ReferenceCounter(b), EventListener(b), autoMotions(), started(b.started), instanceName(b.instanceName), event(NULL)
{
	getRegistryInstance().insert(this);
}

BehaviorBase&
BehaviorBase::operator=(const BehaviorBase& b) {
	ReferenceCounter::operator=(b);
	EventListener::operator=(b);
	started=b.started;
	instanceName=b.instanceName;
	return *this;
}

BehaviorBase::~BehaviorBase() {
	setAutoDelete(false);
	if(started)
		std::cerr << "Behavior " << getName() << " deleted while running: use 'removeReference', not 'delete'" << std::endl;
	if(erouter!=NULL)
		erouter->removeListener(this);
	autoMotions.clear(); // this actually removes the motions via MonitorMotion's destructors
	getRegistryInstance().erase(this);
}

void
BehaviorBase::start() {
	//std::cout << getName() << " started " << this << std::endl;
	if(!started) {
		started=true;
		addReference();
		try {
			preStart();
			if(started) // re-verify flag in case the subclass called stop()
				doStart();
			if(started)
				postStart();
		} catch(...) {
			// abort our reference... don't delete on dereference
			bool prevAD = getAutoDelete();
			setAutoDelete(false);
			removeReference();
			setAutoDelete(prevAD);
			throw;
		}
	}
}

void
BehaviorBase::stop() {
	//std::cout << getName() << " stopped " << this << std::endl;
	if(started) {
		doStop();
		autoMotions.clear(); // this actually removes the motions via MonitorMotion's destructors
		started=false;
		erouter->remove(this);
		removeReference();
	}
}

void BehaviorBase::processEvent(const EventBase& curEvent) {
	const EventBase* prevEvent = event; // in case of recursive events triggered by doEvent
	event=&curEvent;
	if(prevEvent==event) { // prevent recursive looping on the same event (is this good or no?)
		std::cerr << "Warning: blocking recursive event posting of " << event->getDescription() << " to behavior " << getName() << std::endl;
	} else {
		doEvent();
		event=prevEvent;
	}
}

void BehaviorBase::doEvent() {
	if(event->getGeneratorID()==EventBase::textmsgEGID && event->getSourceID()==1)
		erouter->postEvent(*event);
}

std::string BehaviorBase::getClassName() const {
	return string_util::demangle(typeid(*this).name());
}

std::set<BehaviorBase*>&
BehaviorBase::getRegistryInstance() {
	static std::set<BehaviorBase*> registry;
	return registry;
}

std::string
BehaviorBase::humanifyClassName(const std::string& name) {
	std::string name2 = /*(string_util::endsWith(name,"Behavior") && name.size()>10) ? name.substr(0,name.size()-8) :*/ name;
	std::string name3;
	unsigned int rightEdge = name2.size();
	if ( rightEdge == 0 ) return name3;
	while ( !isalpha(name2[rightEdge-1]) && (--rightEdge > 0) );
	for(unsigned int i=0; i<rightEdge; ++i) {
		/*! a rough heuristic for inserting spaces... before any non-lowercase letter (i.e. caps or nums) where previous or next is lower, but not within a digit string or before a final digit string
		 - ASCIIVision → ASCII Vision
		 - Aibo3DController → Aibo3D Controller
		 - EStopController → EStop Controller
		 - Test32 → Test32
		 - FlashIPAddr → Flash IP Addr
		 */
	if(!islower(name2[i]) && i>1 && 
	   ( (i+1<rightEdge && islower(name2[i+1]) && isalpha(name2[i-1])) || islower(name2[i-1]) ))
			name3.append(1,' ');
		name3.append(1,name2[i]);
	}
	name3.append(name2.substr(rightEdge));
	return name3;
}

// this version calls out to the other addMotion (instead of using optional argument)
// to avoid dependence on MotionManager.h just for the kStdPriority
BehaviorBase::MC_ID
BehaviorBase::addMotion(const SharedObjectBase& mc, Prunability_t prune/*=PERSISTENT*/) {
	return addMotion(mc,prune,MotionManager::kStdPriority);
}

BehaviorBase::MC_ID
BehaviorBase::addMotion(const SharedObjectBase& mc, Prunability_t prune, float priority/*=MotionManager::kStdPriority*/) {
	MotionManager::MC_ID mcid = MotionManager::invalid_MC_ID;
	if(prune==PERSISTENT)
		mcid = motman->addPersistentMotion(mc, priority);
	else if(prune==PRUNABLE)
		mcid = motman->addPrunableMotion(mc, priority);
	else
		throw std::runtime_error("Invalid prunability value for BehaviorBase::addMotion");
	if(mcid!=MotionManager::invalid_MC_ID)
		autoMotions[mcid].monitor(*this,mcid,mc.getRegion());
	return mcid;
}

void
BehaviorBase::removeMotion(MotionManager::MC_ID mcid) {
	autoMotions.erase(mcid); // removes motion via MonitorMotion destructor
}

BehaviorBase::MonitorMotion::~MonitorMotion() {
	if(mcid!=MotionManager::invalid_MC_ID) {
		erouter->removeListener(this,EventBase::motmanEGID,mcid,EventBase::deactivateETID);
		motman->removeMotion(mcid);
	}
	if(mcregion!=NULL)
		mcregion->RemoveReference();
}
void BehaviorBase::MonitorMotion::monitor(BehaviorBase& parent, MotionManager::MC_ID mc_id, RCRegion* region) {
	owner=&parent; mcid=mc_id; mcregion=region;
	if(mcregion!=NULL)
		mcregion->AddReference();
	if(mcid!=MotionManager::invalid_MC_ID)
		erouter->addListener(this,EventBase::motmanEGID,mcid,EventBase::deactivateETID);
}
void BehaviorBase::MonitorMotion::processEvent(const EventBase&) {
	// we have received notification that the motion was removed/pruned
	MotionManager::MC_ID tmp = mcid; // backup original
	mcid=MotionManager::invalid_MC_ID; // clear mcid *before* erasing the entry, so the destructor won't try to remove the mcid a second time
	mcregion->RemoveReference();
	mcregion=NULL;
	owner->autoMotions.erase(tmp);
	// by erasing the map entry, this instance should now be destructed!
}


/*! @file
 * @brief Implements BehaviorBase from which all Behaviors should inherit
 * @author ejt (Creator)
 */

