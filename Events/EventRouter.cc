#include "EventRouter.h"
#include "Shared/Profiler.h"
#include "Behaviors/BehaviorBase.h"
#include "Shared/ProjectInterface.h"
#include <algorithm>
#include "Events/TimerEvent.h"
#include "EventTranslator.h"
#include "Shared/string_util.h"
#include "Shared/debuget.h"

#ifndef TGT_IS_DYNAMIC
#  include "Events/RemoteRouter.h"
#  include "Events/EventProxy.h"
#endif

#include <sstream>

#ifndef PLATFORM_APERIOS
#  include "IPC/Thread.h"
#  include "Shared/MarkScope.h"
#  include "IPC/ThreadedMessageQueue.h"
#endif

EventRouter * erouter=NULL;

EventRouter::EventRouter() :
#ifndef TGT_IS_DYNAMIC
	proxies(), rrouters(), sck(NULL), nextProxyPort(defaultPort+1),
#endif
	timers(), trappers(), listeners(), postings()
#ifndef PLATFORM_APERIOS
	, eventQueue(new ThreadedMessageQueue<EventBase*>)
#endif
{
	for(unsigned int i=0; i<ProcessID::NumProcesses; ++i) {
		forwards[i]=NULL;
	}
}

EventRouter::~EventRouter() {
#ifndef PLATFORM_APERIOS
	delete eventQueue;
	eventQueue=NULL;
#endif
	reset();
	removeAllTimers();
	for(unsigned int i=0; i<ProcessID::NumProcesses; ++i) {
		delete forwards[i];
		forwards[i]=NULL;
	}

#ifndef TGT_IS_DYNAMIC
	//Delete all the event proxies
	//printf("Deleting %zu EventProxies and %zu RemoteRouters\n", proxies.size(), rrouters.size());
	for (std::list<EventProxy *>::iterator pi = proxies.begin(); pi != proxies.end(); pi++)
		delete *pi;
	
	//Delete the remote routers
	for (std::map<int, RemoteRouter *>::iterator mi = rrouters.begin(); mi != rrouters.end(); mi++)
		delete (*mi).second;
#endif
	
}

#ifndef PLATFORM_APERIOS
void EventRouter::queueEvent(EventBase* e) { eventQueue->send(e); }

void EventRouter::requeueEvent(EventBase* e) { eventQueue->remove(SameID(e)); queueEvent(e); }
#endif


//! @todo handle recursive calls
void EventRouter::processTimers() {
  // std::cout << "processTimers..." << std::flush;
	unsigned int curtime=get_time();
	TimerEntry curTimer(curtime);
	timer_it_t last_it=upper_bound(timers.begin(),timers.end(),&curTimer,TimerEntryPtrCmp());
	std::vector<TimerEntry*> process(timers.begin(),last_it); //copy these out for safe keeping
	for(timer_it_t it=process.begin(); it!=process.end(); it++) //increment the timers we're processing
		if(!(*it)->repeat)
			(*it)->next=(unsigned int)-1;
		else if((*it)->delay==0)
			(*it)->next=curtime+1;
		else while((*it)->next<=curtime)
			(*it)->next+=(*it)->delay;
	sort(timers.begin(),last_it,TimerEntryPtrCmp()); //re-sort the timers we're processing (at the beginning of timers)
	inplace_merge(timers.begin(),last_it,timers.end(),TimerEntryPtrCmp()); //now do a merge of the sorted processed stuff and the rest of the list (which is still sorted)
	//	if(process.size()>0) chkTimers();
	for(timer_it_t it=process.begin(); it!=process.end(); it++) { // process the timers we say we're going to, can no longer assume anything about the state of the world
		if(find(timers.begin(),timers.end(),*it)==timers.end())
			continue; // the timer has been removed during processesing of a previous timer...
		TimerEvent e((*it)->el,EventBase::timerEGID,(*it)->sid,EventBase::statusETID,(*it)->next-(*it)->delay);
		try {
			(*it)->el->processEvent(e);
		} catch(const std::exception& ex) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>((*it)->el))
				msg+="listener "+beh->getName();
			else
				msg+="unnamed EventListener";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),&ex))
				throw;
		} catch(...) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>((*it)->el))
				msg+="listener "+beh->getName();
			else
				msg+="unnamed EventListener";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),NULL))
				throw;
		}
		postEvent(e);
	}
	//	if(process.size()>0) chkTimers();
	static const TimerEntry deadTimer((unsigned int)-1); // matches all the dead ones as set in the incrementation phase
	last_it=lower_bound(timers.begin(),timers.end(),&deadTimer,TimerEntryPtrCmp()); //find the beginning of all the non-repeating timers we're clearing
	for(timer_it_t it=last_it; it!=timers.end(); it++) // delete all of them
		delete *it;
	timers.erase(last_it,timers.end()); //and then remove them from the timer list
	//	if(process.size()>0) chkTimers();
	//		cout << "done" << endl;
}

/*! timers are unique by EventListener and source ID - can't have two timers for the same el and sid\n
 *  a delay of 0 with repeating will cause an event to be sent at every opportunity, use sparingly\n
 *  a delay of -1U will call removeTimer() if it already exists, otherwise is ignored\n
 *
 *  @param el the EventListener to send the timer event to
 *  @param sid the source ID to use on that event (if you need to send more info, send a pointer to a struct of your devising, typecasted as int)
 *  @param delay the delay between the first (and future) calls
 *  @param repeat set to true if you want to keep receiving this event, otherwise it will only send once */
void EventRouter::addTimer(EventListener* el, size_t sid, unsigned int delay, bool repeat) {
	if(delay==-1U) {
		removeTimer(el,sid);
		return;
	}
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		if((*it)->el==el && (*it)->sid==sid) {
			(*it)->Set(delay,repeat);
			// now put that timer back into the correct place in the ordering (think before touching this! ;)
			if(it!=timers.begin() && (*it)->next<(*(it-1))->next)
				rotate(upper_bound(timers.begin(),it,*it,TimerEntryPtrCmp()),it,it+1);
			else if(it+1!=timers.end() && (*it)->next>(*(it+1))->next)
				rotate(it,it+1,lower_bound(it+1,timers.end(),*it,TimerEntryPtrCmp()));
			return;
		}
	//didn't find a pre-existing one
	TimerEntry * add=new TimerEntry(el,sid,delay,repeat);
	timers.insert(lower_bound(timers.begin(),timers.end(),add,TimerEntryPtrCmp()),add);
	//	chkTimers();
}

void EventRouter::removeTimer(const EventListener* el) {
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		if((*it)->el==el) {
			delete *it;
			*it=NULL;
		}
	timers.erase(std::remove(timers.begin(),timers.end(),(const TimerEntry*)NULL),timers.end());
}

void EventRouter::removeTimer(const EventListener* el, size_t sid) {
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		if((*it)->el==el && (*it)->sid==sid) {
			delete *it;
			timers.erase(it);
			return;
		}
}

void EventRouter::removeAllTimers() {
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		delete *it;
	timers.erase(timers.begin(),timers.end());
}

const EventRouter::TimerEntry* EventRouter::getNextTimerInfo(const EventListener* el) {
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		if((*it)->el==el)
			return *it;
	return NULL;
}

const EventRouter::TimerEntry* EventRouter::getNextTimerInfo(const EventListener* el, size_t sid) {
	for(timer_it_t it=timers.begin(); it!=timers.end(); it++)
		if((*it)->el==el && (*it)->sid==sid)
			return *it;
	return NULL;
}

void EventRouter::addListener(EventListener* el, EventBase::EventGeneratorID_t egid) {
	bool hadListener=hasListeners(egid);
	listeners.addMapping(el,egid); 
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::addListener(EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid) {
	bool hadListener=hasListeners(egid);
	for(unsigned int et=0; et<EventBase::numETIDs; et++)
		listeners.addMapping(el,egid,sid,(EventBase::EventTypeID_t)et);
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::addListener(EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	bool hadListener=hasListeners(egid);
	listeners.addMapping(el,egid,sid,etid);
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::addListener(EventListener* el, const EventBase& e) {
	bool hadListener=hasListeners(e.getGeneratorID());
	listeners.addMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID());
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::activateETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::statusETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
}

#ifndef TGT_IS_DYNAMIC
//Start of remote event code------------------
void EventRouter::addRemoteListener(EventListener* el, int host,
									EventBase::EventGeneratorID_t egid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	addListener(el, egid);
	rr.addListener(egid);
}

void EventRouter::addRemoteListener(EventListener* el, int host,
									EventBase::EventGeneratorID_t egid, size_t sid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	addListener(el, egid, sid);
	rr.addListener(egid, sid);
}

void EventRouter::addRemoteListener(EventListener* el, int host, const EventBase& e){
	addRemoteListener(el, host, e.getGeneratorID(), e.getSourceID(), e.getTypeID());
}

void EventRouter::addRemoteListener(EventListener* el, int host,
									EventBase::EventGeneratorID_t egid, size_t sid,
									EventBase::EventTypeID_t etid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	addListener(el, egid, sid, etid);
	rr.addListener(egid, sid, etid);
}


void EventRouter::removeRemoteListener(const EventListener* el, int host,
									   EventBase::EventGeneratorID_t egid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	removeListener(el, egid);
	rr.removeListener(egid);
}

void EventRouter::removeRemoteListener(const EventListener* el, int host,
									   EventBase::EventGeneratorID_t egid, size_t sid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	removeListener(el, egid, sid);
	rr.removeListener(egid, sid);	
}
    
void EventRouter::removeRemoteListener(const EventListener* el, int host,
									   const EventBase& e) {
	removeRemoteListener(el, host, e.getGeneratorID(), e.getSourceID(), e.getTypeID());	
}

void EventRouter::removeRemoteListener(const EventListener* el, int host,
									   EventBase::EventGeneratorID_t egid, size_t sid,
									   EventBase::EventTypeID_t etid) {
	RemoteRouter &rr = remoteRouterForHost(host);
	removeListener(el, egid, sid, etid);
	rr.removeListener(egid, sid, etid);
}


void EventRouter::requestRemoteStateUpdates(int host, RemoteState::StateType type,
											unsigned int interval) {
	RemoteRouter &rr = remoteRouterForHost(host);
	rr.requestStateUpdates(type, interval);
}

void EventRouter::stopRemoteStateUpdates(int host, RemoteState::StateType type) {
	RemoteRouter &rr = remoteRouterForHost(host);
	rr.stopStateUpdates(type);
}

RemoteRouter &EventRouter::remoteRouterForHost(int host) {
	RemoteRouter *rr = rrouters[host];
	if (rr) {
		printf("Returning existing remote router for host %s\n", string_util::intToStringIP(host).c_str());
		return *rr;
	} else {
		rrouters[host] = rr = new RemoteRouter(host);
		printf("Returning new remote router for host %s\n", string_util::intToStringIP(host).c_str());
		return *rr;
	}
}

/* This is called in MMCombo.cc on startup. */
bool EventRouter::serveRemoteEventRequests() {
	if (sck)
		return false;
	sck = wireless->socket(Socket::SOCK_STREAM);
	wireless->setReceiver(sck, this);
	wireless->setDaemon(sck, true);
	wireless->listen(sck, EventRouter::defaultPort);
	return true;
}

int EventRouter::processData(char* /* data*/, int bytes) {
	if (bytes != sizeof(int)) {
		std::cerr << "Unknown data received" << std::endl;
		return -1;
	}
	
	int nextPort = nextProxyPort++;
	std::cout << "Starting EventProxy on port " << nextPort
		<< " for host " << string_util::intToStringIP(sck->getPeerAddress()) << std::endl;
	proxies.push_back(new EventProxy(nextPort));

	//Send the port to the RemoteRouter
	sck->write((byte *)&nextPort, sizeof(int));

	//Start listening again
	wireless->close(sck);
	
	return 0;
}

//End of remote event code
#endif

void EventRouter::removeListener(const EventListener* el) {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++) {
		EventBase::EventGeneratorID_t egid=(EventBase::EventGeneratorID_t)eg;
		if(!listeners.removeMapping(el,egid))
			continue; //nothing was removed, don't want to clean up or throw an event
		listeners.clean(egid);
		if(!hasListeners(egid))
			postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
		else
			postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
	}
}
void EventRouter::removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid) {
	if(!listeners.removeMapping(el,egid))
		return; //nothing was removed, don't want to clean up or throw an event
	listeners.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid) {
	unsigned int removed=0;
	for(unsigned int et=0; et<EventBase::numETIDs; et++)
		removed+=listeners.removeMapping(el,egid,sid,(EventBase::EventTypeID_t)et);
	if(!removed)
		return; //nothing was removed, don't want to clean up or throw an event
	listeners.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	if(!listeners.removeMapping(el,egid,sid,etid))
		return; //nothing was removed, don't want to clean up or throw an event
	listeners.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::removeListener(const EventListener* el, const EventBase& e) {
	if(!listeners.removeMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID()))
		return; //nothing was removed, don't want to clean up or throw an event
	listeners.clean(e.getGeneratorID());
	if(!hasListeners(e.getGeneratorID()))
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::deactivateETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::statusETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
}

void EventRouter::addTrapper(EventTrapper* el, const EventBase& e) {
	bool hadListener=hasListeners(e.getGeneratorID());
	trappers.addMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID());
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::activateETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::statusETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
}
/*! Note that since timers are not broadcast, they cannot be trapped.  Only the EventListener which requested the timer will receive that timer. */
void EventRouter::addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid) {
	bool hadListener=hasListeners(egid);
	trappers.addMapping(el,egid);
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
/*! Note that since timers are not broadcast, they cannot be trapped.  Only the EventListener which requested the timer will receive that timer. */
void EventRouter::addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid) {
	bool hadListener=hasListeners(egid);
	for(unsigned int et=0; et<EventBase::numETIDs; et++)
		trappers.addMapping(el,egid,sid,(EventBase::EventTypeID_t)et);
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
/*! Note that since timers are not broadcast, they cannot be trapped.  Only the EventListener which requested the timer will receive that timer. */
void EventRouter::addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	bool hadListener=hasListeners(egid);
	trappers.addMapping(el,egid,sid,etid);
	if(!hadListener)
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::activateETID,0,EventBase::EventGeneratorNames[egid],1));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}

/*! Note that since timers are not broadcast, they cannot be trapped.  Only the EventListener which requested the timer will receive that timer. */
void EventRouter::addTrapper(EventTrapper* el) {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++)
		addTrapper(el,(EventBase::EventGeneratorID_t)eg);
}


void EventRouter::removeTrapper(const EventTrapper* el, const EventBase& e) {
	if(!trappers.removeMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID()))
		return; //nothing was removed, don't want to clean up or throw an event
	trappers.clean(e.getGeneratorID());
	if(!hasListeners(e.getGeneratorID()))
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::deactivateETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,e.getGeneratorID(),EventBase::statusETID,0,EventBase::EventGeneratorNames[e.getGeneratorID()],1));
}
void EventRouter::removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid) {
	if(!trappers.removeMapping(el,egid))
		return; //nothing was removed, don't want to clean up or throw an event
	trappers.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid) {
	int removed=0;
	for(unsigned int et=0; et<EventBase::numETIDs; et++)
		removed+=trappers.removeMapping(el,egid,sid,(EventBase::EventTypeID_t)et);
	if(!removed)
		return; //nothing was removed, don't want to clean up or throw an event
	trappers.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}
void EventRouter::removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	if(!trappers.removeMapping(el,egid,sid,etid))
		return; //nothing was removed, don't want to clean up or throw an event
	trappers.clean(egid);
	if(!hasListeners(egid))
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::deactivateETID,0,EventBase::EventGeneratorNames[egid],0));
	else
		postEvent(EventBase(EventBase::erouterEGID,egid,EventBase::statusETID,0,EventBase::EventGeneratorNames[egid],1));
}

void EventRouter::removeTrapper(const EventTrapper* el) {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++)
		removeTrapper(el,(EventBase::EventGeneratorID_t)eg);
}

void EventRouter::chkTimers() {
  unsigned int last=0;
  for(timer_it_t it=timers.begin(); it!=timers.end(); it++) {
    if(last>(*it)->next) {
      std::cout << "Out of order ";
      dispTimers();
      return;
    }
    last=(*it)->next;
  }
}

//! just for debugging
void EventRouter::dispTimers() {
  std::cout << "timers at " << get_time() << " :\t";
  unsigned int last=0;
  for(timer_it_t it=timers.begin(); it!=timers.end(); it++) {
    if(last>(*it)->next)
      std::cout << "##";
    BehaviorBase* beh = dynamic_cast<BehaviorBase*>((*it)->el);
    if ( beh != NULL )
      std::cout << beh->getName() << "@";
    std::cout << (last=(*it)->next) << '\t';
  }
  std::cout << std::endl;
}


void EventRouter::setForwardingAgent(ProcessID::ProcessID_t proc, EventTranslator* trans) {
	delete forwards[proc];
	forwards[proc]=trans;
}

void EventRouter::processEvent(const EventBase& e) {
	// check for forwarding *before* the lock
	ProcessID::ProcessID_t pid=ProcessID::getID();
	if(pid!=ProcessID::NumProcesses && forwards[pid]!=NULL) {
		if(forwards[pid]->trapEvent(e))
			return;
	}
	
#ifndef PLATFORM_APERIOS
	static Thread::Lock lk;
	MarkScope autolock(lk);
#endif
	PostingStatus ps(trappers,listeners,e);
	postings.push(&ps);
	while(postings.size()>0) {
#ifdef DEBUG
		size_t presize=postings.size();
		postings.front()->process();
		ASSERT(postings.size()==0 || postings.size()==presize,"partial queue completion?");
#else
		postings.front()->process();
#endif
		if(postings.size()>0) // in case a sub-post took over and finished off the queue
			postings.pop();
	}
}

void EventRouter::PostingStatus::process() {
	while(tit!=t.end()) {
		// increment before processing so if a new post is done during the processing, we pick up on the *next* entry
		EventTrapper * et=*tit++;
		if(!trappers.verifyMapping(et,e))
			continue;
		try {
			if(et->trapEvent(e))
				return;
		} catch(const std::exception& ex) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>(et))
				msg+="trapper "+beh->getName();
			else
				msg+="unnamed EventTrapper";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),&ex))
				throw;
		} catch(...) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>(et))
				msg+="trapper "+beh->getName();
			else
				msg+="unnamed EventTrapper";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),NULL))
				throw;
		}
	}
	while(lit!=l.end()) {
		// increment before processing so if a new post is done during the processing, we pick up on the *next* entry
		EventListener * el=*lit++;
		if(!listeners.verifyMapping(el,e))
			continue;
		try {
			el->processEvent(e);
		} catch(const std::exception& ex) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>(el))
				msg+="listener "+beh->getName();
			else
				msg+="unnamed EventListener";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),&ex))
				throw;
		} catch(...) {
			std::string msg="Occurred while processing event "+e.getName()+" by ";
			if(BehaviorBase * beh=dynamic_cast<BehaviorBase*>(el))
				msg+="listener "+beh->getName();
			else
				msg+="unnamed EventListener";
			if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,msg.c_str(),NULL))
				throw;
		}
	}
}

EventRouter::EventMapper::EventMapper() {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++)
		for(unsigned int et=0; et<EventBase::numETIDs; et++)
			filteredevents[eg][et]=NULL;
}

void EventRouter::EventMapper::addMapping(void* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	if(filteredevents[egid][etid]==NULL) //if this is the first subscriber to this EGID and ETID
		filteredevents[egid][etid]=new SIDtoListenerVectorMap_t(); 
	SIDtoListenerVectorMap_t::iterator it=filteredevents[egid][etid]->find(sid); // now find subscribers to the source id as well
	std::vector<void*>* elv=NULL;
	if(it==filteredevents[egid][etid]->end()) { // if this is the first subscriber to the source ID
		std::pair<const size_t,std::vector<void*> > p(sid,std::vector<void*>());
		//		p.first=sid; //p.second is a vector, only needs to be constructed
		filteredevents[egid][etid]->insert(p);
		elv=&(*filteredevents[egid][etid]->find(sid)).second;
	} else {
		elv=&(*it).second;
	}
	elv->push_back(el); // now that everything's set up, we can add the listener
}

bool EventRouter::EventMapper::removeMapping(const void* el, EventBase::EventGeneratorID_t egid) {
	// remove listener from allevents
	size_t numlist=allevents[egid].size();
	allevents[egid].erase(std::remove(allevents[egid].begin(),allevents[egid].end(),el),allevents[egid].end());
	bool hadListener=allevents[egid].size()!=numlist;
	
	// now remove listener from all of the filtered events
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) { // if there are subscribers to this egid/etid
			SIDtoListenerVectorMap_t::iterator mapit=mapping->begin();
			for(mapit=mapping->begin(); mapit!=mapping->end(); mapit++) {// go through each sourceID, delete EL
				std::vector<void*> * v=&(*mapit).second;
				std::vector<void*>::iterator last=std::remove(v->begin(),v->end(),el);
				if(last!=v->end()) {
					hadListener=true;
					v->erase(last,v->end());
				}
			}
		}
	}
	return hadListener;
}

bool EventRouter::EventMapper::removeMapping(const void* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	bool hadListener=false;
	SIDtoListenerVectorMap_t* mapping=filteredevents[egid][etid];
	if(mapping!=NULL) { // if there are subscribers to this egid/etid
		SIDtoListenerVectorMap_t::iterator mapit=mapping->find(sid);
		if(mapit!=mapping->end()) {
			std::vector<void*> * v=&(*mapit).second;
			std::vector<void*>::iterator last=std::remove(v->begin(),v->end(),el);
			if(last!=v->end()) {
				hadListener=true;
				v->erase(last,v->end());
			}
		}
	}
	return hadListener;
}

void EventRouter::EventMapper::clean() {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++)
		clean((EventBase::EventGeneratorID_t)eg);
}
void EventRouter::EventMapper::clean(EventBase::EventGeneratorID_t egid) {
	// first, remove any empty sid vectors from all the mappings
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) { // if there are subscribers to this egid/etid
			SIDtoListenerVectorMap_t::iterator mapit=mapping->begin();
			bool done=false;
			while(!done) {
				done=true;
				for(mapit=mapping->begin(); mapit!=mapping->end(); mapit++) { // go through each sourceID vector
					if((*mapit).second.size()==0) {
						mapping->erase(mapit);
						done=false;
						break;
					}
				}
			}
		}
	}
	// now remove any empty mappings
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) { // if there are subscribers to this egid/etid
			if(mapping->size()==0) {
				delete mapping;
				filteredevents[egid][et]=NULL;
			}
		}
	}
}

void EventRouter::EventMapper::clear() {
	for(unsigned int eg=0; eg<EventBase::numEGIDs; eg++) {
		for(unsigned int et=0; et<EventBase::numETIDs; et++) {
			SIDtoListenerVectorMap_t* mapping=filteredevents[eg][et];
			if(mapping!=NULL) { // don't beat a dead horse!
				mapping->erase(mapping->begin(),mapping->end());
				delete mapping;
				filteredevents[eg][et]=NULL;
			}
		}
	}
}

bool EventRouter::EventMapper::hasMapping(EventBase::EventGeneratorID_t egid) const {
	if(allevents[egid].size()>0)
		return true;
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		const SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) {
			SIDtoListenerVectorMap_t::const_iterator mapit=mapping->begin();
			for(mapit=mapping->begin(); mapit!=mapping->end(); mapit++)
				if((*mapit).second.size()>0)
					return true;
		}
	}
	return false;
}

bool EventRouter::EventMapper::hasMapping(EventBase::EventGeneratorID_t egid, size_t sid) const {
	if(allevents[egid].size()>0)
		return true;
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		const SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) {
			SIDtoListenerVectorMap_t::const_iterator mapit=mapping->find(sid);
			if(mapit!=mapping->end() && (*mapit).second.size()>0)
				return true;
		}
	}
	return false;
}

bool EventRouter::EventMapper::hasMapping(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const {
	if(allevents[egid].size()>0)
		return true;
	const SIDtoListenerVectorMap_t* mapping=filteredevents[egid][etid];
	if(mapping!=NULL) {
		SIDtoListenerVectorMap_t::const_iterator mapit=mapping->find(sid);
		if(mapit!=mapping->end())
			return ((*mapit).second.size()>0);
	}
	return false;
}

template<class T>
void EventRouter::EventMapper::getMapping(const EventBase& e, std::vector<T*>& ls) const {
	// first get all the filtered subscribers (tricky!)
	const std::vector<void*>* elv=NULL;
	const SIDtoListenerVectorMap_t* sidtovm=filteredevents[e.getGeneratorID()][e.getTypeID()];
	if(sidtovm!=NULL) { // if there's a map (at least one EL is filtering on this EGID and ETID)
		SIDtoListenerVectorMap_t::const_iterator mapit=sidtovm->find(e.getSourceID()); // find listening for this source id
		if(mapit!=sidtovm->end()) { // if there's at least one is filtering on this sourceID as well
			elv=&(*mapit).second; // now go through them all
			for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
				ls.push_back(static_cast<T*>(*elit));
		}
	}
	// now get the 'all events' subscribers
	elv=&allevents[e.getGeneratorID()];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		ls.push_back(static_cast<T*>(*elit));
}

bool EventRouter::EventMapper::verifyMapping(const void * listener, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const {
	// first check the 'all events' subscribers
	const std::vector<void*>* elv=&allevents[egid];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		if(*elit==listener)
			return true;
	
	// then check all the filtered subscribers (tricky!)
	const SIDtoListenerVectorMap_t* sidtovm=filteredevents[egid][etid];
	if(sidtovm!=NULL) { // if there's a map (at least one EL is filtering on this EGID and ETID)
		const SIDtoListenerVectorMap_t::const_iterator mapit=sidtovm->find(sid); // find listening for this source id
		if(mapit!=sidtovm->end()) { // if there's at least one is filtering on this sourceID as well
			elv=&(*mapit).second; // now go through them all
			for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
				if(*elit==listener)
					return true;
		}
	}

	// if we haven't found it, doesn't exist:
	return false;
}

bool EventRouter::EventMapper::verifyMappingAll(const void* listener, EventBase::EventGeneratorID_t egid) const {
	const std::vector<void*>* elv=&allevents[egid];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		if(*elit==listener)
			return true;
	// if not in the all listeners, can't be listening for *every* source id
	return false;
}

bool EventRouter::EventMapper::verifyMappingAny(const void* listener, EventBase::EventGeneratorID_t egid) const {
	// first check the 'all events' subscribers
	const std::vector<void*>* elv=&allevents[egid];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		if(*elit==listener)
			return true;
	
	// then check all the filtered subscribers (tricky!)
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		const SIDtoListenerVectorMap_t* sidtovm=filteredevents[egid][et];
		if(sidtovm!=NULL) { // if there's a map (at least one EL is filtering on this EGID and ETID)
			SIDtoListenerVectorMap_t::const_iterator mapit=sidtovm->begin(); // for each of the source ids
			for(;mapit!=sidtovm->end();mapit++) {
				elv=&(*mapit).second; // now go through them all
				for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
					if(*elit==listener)
						return true;
			}
		}
	}

	// if we haven't found any, none exist:
	return false;
}

bool EventRouter::EventMapper::verifyMappingAll(const void* listener, EventBase::EventGeneratorID_t egid, size_t sid) const {
	// first check the 'all events' subscribers
	const std::vector<void*>* elv=&allevents[egid];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		if(*elit==listener)
			return true;
	
	// then check all the filtered subscribers (tricky!)
	// must be found in ALL etids
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		const SIDtoListenerVectorMap_t* sidtovm=filteredevents[egid][et];
		if(sidtovm==NULL)
			return false;
		// there's a map (at least one EL is filtering on this EGID and ETID)
		const SIDtoListenerVectorMap_t::const_iterator mapit=sidtovm->find(sid); // find listening for this source id
		if(mapit==sidtovm->end())
			return false;
		// there's at least one is filtering on this sourceID as well
		elv=&(*mapit).second; // now go through them all
		std::vector<void*>::const_iterator elit=elv->begin();
		while(elit!=elv->end() && *elit!=listener)
			elit++;
		if(elit==elv->end())
			return false;
		//if we didn't return false, we found a match... continue checking other ETIDs
	}

	// we only got here if we *did* find listener in each of the ETIDs
	return true;
}

bool EventRouter::EventMapper::verifyMappingAny(const void* listener, EventBase::EventGeneratorID_t egid, size_t sid) const {
	// first check the 'all events' subscribers
	const std::vector<void*>* elv=&allevents[egid];
	for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
		if(*elit==listener)
			return true;
	
	// then check all the filtered subscribers (tricky!)
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		const SIDtoListenerVectorMap_t* sidtovm=filteredevents[egid][et];
		if(sidtovm!=NULL) { // if there's a map (at least one EL is filtering on this EGID and ETID)
			SIDtoListenerVectorMap_t::const_iterator mapit=sidtovm->find(sid); // find listening for this source id
			if(mapit!=sidtovm->end()) { // if there's at least one is filtering on this sourceID as well
				elv=&(*mapit).second; // now go through them all
				for(std::vector<void*>::const_iterator elit=elv->begin(); elit!=elv->end(); elit++)
					if(*elit==listener)
						return true;
			}
		}
	}

	// if we haven't found it, doesn't exist:
	return false;
}

/*! @file
 * @brief Implements EventRouter class, for distribution and trapping of events to listeners
 * @author ejt (Creator)
 */







// Use hasListeners(*) it's faster, i doubt anyone would really care how many... (but just in case...)
/*
unsigned int EventRouter::numListeners(EventBase::EventGeneratorID_t egid) {
	unsigned int ans=allevents[egid].size();
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) {
			SIDtoListenerVectorMap_t::iterator mapit=mapping->begin();
			for(mapit=mapping->begin(); mapit!=mapping->end(); mapit++)
				ans+=(*mapit).second.size();
		}
	}
	return ans;
}

bool EventRouter::numListeners(EventBase::EventGeneratorID_t egid, size_t sid) {
	size_t ans=allevents[egid].size();
	for(unsigned int et=0; et<EventBase::numETIDs; et++) {
		SIDtoListenerVectorMap_t* mapping=filteredevents[egid][et];
		if(mapping!=NULL) {
			SIDtoListenerVectorMap_t::iterator mapit=mapping->find(sid);
			if(mapit!=mapping->end())
				ans+=(*mapit).second.size();
		}
	}
	return false;
}

unsigned int EventRouter::numListeners(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) {
	unsigned int ans=allevents[egid].size();
	SIDtoListenerVectorMap_t* mapping=filteredevents[egid][etid];
	if(mapping!=NULL) {
		SIDtoListenerVectorMap_t::iterator mapit=mapping->find(sid);
		if(mapit!=mapping->end())
			ans+=(*mapit).second.size();
	}
	return ans;
}
*/
