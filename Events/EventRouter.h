//-*-c++-*-
#ifndef INCLUDED_EventRouter_h
#define INCLUDED_EventRouter_h

#include <string>
#include <vector>
#include <queue>
#include <map>
#include <list>
#include <algorithm>
#include "EventListener.h"
#include "EventTrapper.h"
#include "Shared/get_time.h"
#include "Shared/attributes.h"
#include "IPC/ProcessID.h"
#include <iostream>
#include "Shared/RobotInfo.h"
#ifndef TGT_IS_DYNAMIC
#  include "Shared/RemoteState.h"
#  include "Wireless/SocketListener.h"
#  include "Wireless/Socket.h"
#endif

class RemoteRouter;
class EventTranslator;
class EventProxy;
template<class T> class ThreadedMessageQueue;

//! This class will handle distribution of events as well as management of timers
/*! Classes must inherit from EventListener and/or EventTrapper in order to
 *  receive events.
 *
 *  Use the global ::erouter instance of EventRouter to both send (post) and receive (subscribe/listen) to
 *  events.  (except if you are posting from within a MotionCommand, in
 *  which case you should use MotionCommand::postEvent() so that it can correctly
 *  handle inter-process communication issues under Aperios (the Aibo's OS) -- under a
 *  Unix-based OS, this isn't necessary.)  Events posted in non-Main processes
 *  will be forwarded to Main for processing.
 *
 *  Event processing is a serialized operation, meaning only one event is ever being
 *  processed at a time, and by one listener at a time.  The EventRouter contains its own
 *  thread lock, so if two threads post events at the same time, the EventRouter
 *  will handle ensuring mutual exclusion.  Listeners are free to spawn their own
 *  threads to handle processing or posting events to avoid being dependent on
 *  other listeners' processing times.  (Note: threads are not available on the Aibo,
 *  so listeners which wish to run on the Aibo are stuck with a "cooperative"
 *  multitasking model)
 *
 *  The events available for processing are listed in EventBase::EventGeneratorID_t.
 *  Each generator ID's documentation specifies how to interpret the source ID field, and whether
 *  you can expect events with that generator ID to be of a subclass of EventBase,
 *  such as TextMsgEvent or LocomotionEvent.  Many generators send plain
 *  EventBase instances.
 *
 *  When multiple listeners are subscribed, the order in which an event is
 *  distributed among them is:
 *  -# "Specific" listeners: any listener which specifies a particular source id.
 *     (doesn't matter if they specified type id as well)
 *    - older listeners get events before more recently added listeners ("FIFO")
 *  -# "General" listeners: those that subscribe to an entire generator
 *    - older listeners get events before more recently added listeners ("FIFO")
 *
 *  ...but if you're relying on that ordering, there probably should be a cleaner
 *  way to do whatever you're doing.
 *
 *  TimerEvents are generally only sent to the generator which requested them.  So if 
 *  EventListener @e A requests a timer (see addTimer()) with ID 0 at two second intervals,
 *  and @e B requests a timer with ID 0 at three second intervals,
 *  each will still only receive the timers they requested - no cross talk.
 *  The timer generator is unique in this regard, which is why it is built in
 *  as an integral component of the EventRouter.  However, EventListener @e A
 *  <b>can</b> receive @e B's timers if it specifically wants to, via addListener().  See "Timers" below.
 *
 *  If an EventListener/EventTrapper subscribes to the same event source multiple
 *  times, it will receive multiple copies of the event.  However, the first call
 *  to removeListener for a source will remove all subscriptions to that source.\n
 *  Example: EventListener @e A subscribes to (buttonEGID,*,*), and twice to
 *  (buttonEGID,0,*).
 *    - If button 0 is pressed, @e A will get three copies of the event.
 *    - If button 1 is pressed, @e A will get one copy.
 *    - If removeListener(&A,buttonEGID) is called, the (buttonEGID,*,*) is
 *      removed, <em>as well as</em> both of (buttonEGID,0,*).
 *    - If removeListener(&A,buttonEGID,0) is called, both of (buttonEGID,0,*)
 *      are removed, but (buttonEGID,*,*) would be untouched.
 *
 *  <h3>Timers</h3>
 *  addTimer() allows you to request an TimerEvent to be sent at some later point in time,
 *  possibly on a repeating basis.  Timers are specific to the behavior which requests
 *  them, and you @e do @e not (and usually should not) call addListener() in order to receive a timer
 *  event.
 *
 *  There is an important different between addTimer() and #addListener(timerEGID,...)!
 *  addTimer will "create" the timer, and will send the timer to the listener
 *  which created it when the timer expires.  This means that as long as the listener in
 *  question does @e not call addListener(timerEGID), it will @e only receive its own timers.
 *  In other words, with this usage there is no confusion with timer cross-talk between
 *  listeners, because each listener is only receiving its own timers.
 *
 *  However, if a listener calls addListener(timerEGID), it will begin receiving @e all timer events
 *  from throughout the system.  This allows you to have one behavior "eavesdrop" on
 *  another's timers.  In order to determine which listener requested/created the timer,
 *  you can use the TimerEvent::getTarget() value.
 *
 *  So beware that if you call both addTimer() and addListener(foo,timerEGID), 'foo' will get
 *  two calls to processEvent() for its own timers, and one call for all other timers, and will
 *  have to know to call TimerEvent::getTarget() to distinguish its timers from other
 *  listener's timers (if it cares about the difference...)
 *
 *  Timers are sent to the requesting listener before being broadcast -- EventTrappers cannot
 *  filter a listener's own timers, but can prevent the timer from being broadcast to other listeners.
 *
 *  <h3>Event processing examples:</h3>
 *
 *  Posting events:
 *  @codeEventProxy
 *  //method A: basic event posting (EventBase instance is sent)
 *  erouter->postEvent(EventBase::aiEGID, 1234, EventBase::statusETID);
 *
 *  //method B: specific event instance is posted (have to use this style to post a subclass)
 *  TextMsgEvent txt("hello world");
 *  erouter->postEvent(txt);
 *  // or can be done in one line:
 *  erouter->postEvent(TextMsgEvent("hello world"))
 *  @endcode
 *
 *  Receiving events:
 *  @code
 *  //given an EventListener subclass:
 *  class YourListener : public EventListener {
 *  public:
 *    virtual void processEvent(const EventBase& e) {
 *      std::cout << "Got: " << e.getName() << std::endl;
 *    }
 *  };
 *
 *  YourListener yourList;
 *
 *  // subscribes it to all EventBase::aiEGID events:
 *  erouter->addListener(&yourList, EventBase::aiEGID);
 *
 *  // subscribes only to button activity from the head, but not other buttons:
 *  erouter->addListener(&yourList, EventBase::buttonEGID, ERS7Info::HeadButOffset);
 *  @endcode
 *  Typically in a BehaviorBase subclass, you would just specify 'this' instead of '&yourList'.
 *
 *  <h3>Timer processing examples:</h3>
 *
 *  Requesting/Creating timers:
 *  @code
 *  YourListener yourList; // (any EventListener subclass)
 *
 *  // sends a timer with source ID 123 every 5 seconds to yourList:
 *  erouter->addTimer(&yourList, 123, 5000);
 *
 *  // sends a timer with ID 456 after 1 second, *no repeat, one time only*
 *  erouter->addTimer(&yourList, 456, 1000, false);
 *
 *  // cancels the first timer
 *  erouter->removeTimer(&yourList, 123);
 *
 *  // postpone/update the second timer's settings (*doesn't* create two timers with the same ID)
 *  erouter->addTimer(&yourList, 456, 2500, false);
 *
 *  @endcode
 *  Again, typically in a BehaviorBase subclass, you would just specify 'this' instead of '&yourList'.
 *
 *  @see EventBase::EventGeneratorID_t for a complete listing of all generators,
 *  as well as instructions on how to add new generators.
 *  @see Tutorials:
 *    - <a href="../FirstBehavior2.html">Steps 3, 4, & 5 of Tekkotsu's First Behavior Tutorial</a>
 *    - <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/events.shtml">David Touretzky's Events Chapter</a>
 *    - <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/behaviors.pdf">CMU's Cognitive Robotics course slides</a>
 */
class EventRouter : public EventListener
#ifndef TGT_IS_DYNAMIC
, public SocketListener 
#endif
{
 public:
	EventRouter(); //!< Constructs the router
	virtual ~EventRouter(); //!< just calls reset and removeAllTimers()

	void reset() { listeners.clear(); trappers.clear(); removeAllTimers(); } //!< erases all listeners, trappers and timers, resets EventRouter
	
	
	//!@name Posting/Processing Events
	
	/*!@brief recommended to create and post an event using current buffer setting */
	void postEvent(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid, unsigned int dur=0) { processEvent(EventBase(egid,sid,etid,dur)); }
	void postEvent(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid, unsigned int dur, const std::string& n, float m) { processEvent(EventBase(egid,sid,etid,dur,n,m)); }
	//! posts the specified event, but doesn't delete it at the end -- equivalent to processEvent(e)
	void postEvent(const EventBase& e) { processEvent(e); }
	
#ifndef PLATFORM_APERIOS
	//! Uses a ThreadedMessageQueue to process an event at a later time, as opposed to postEvent() which will process the event immediately.
	/*! If an event generator is running in a background thread (e.g. DeviceDriver or a behavior thread), it can use this function to
	 *  post events instead of grabbing the behaviorLock and calling postEvent directly.  However, beware building up a backlog
	 *  of stale events, you may prefer requeueEvent() to ensure only the most up-to-date data is in the queue
	 *  The event will be processed after all current event processing is completed, similar to a 0-ms timer subscription.
	 *  The event will be deleted after processing.*/
	void queueEvent(EventBase* e);
	
	//! Places @a e in the same queue as queueEvent(), but removes any other events with the same generator and source first.
	/*! For sources only the current value is of significant interest (e.g. sensor readings) this avoids
	 *  the possibility of a backlog of stale data forming in the case the Main thread blocks and falls
	 *  behind on processing incoming data */
	void requeueEvent(EventBase* e);
#endif

	//! determines if timers need to be posted, and posts them if so.
	/*! Call this often to ensure accurate timers. */
	void processTimers();
	//! sends event to its trappers & listeners, but doesn't delete the event at the end (see also postEvent())
	/*! this posting method is supplied to allow an EventRouter to behave as a listener as 
	 *  well -- the 'routers' really can form a sort of network, if desired.  postEvent() is
	 *  probably a more memnomic interface to use in direct function calls however,
	 *  so that is the one you should call. */
	void processEvent(const EventBase& e);
	
	//! returns the forwarding agent for a given process/thread group (see #forwards)
	EventTranslator* getForwardingAgent(ProcessID::ProcessID_t proc) const { return forwards[proc]; }
	//! sets the forwarding agent for a given process/thread group (see #forwards)
	void setForwardingAgent(ProcessID::ProcessID_t proc, EventTranslator* trans);
	
#ifndef PLATFORM_APERIOS
	//! Returns the event queue, to be processed by a callback thread between other behavior processing
	/*! This is similar to the forwarding agents which handles inter-process events (e.g. from Motion to Main)
	 *  but in threaded environments (e.g. non-AIBO) we can use this more efficient mechanism.  As we drop
	 *  AIBO support we can transfer #forwards to use queueEvent() instead.
	 *
	 *  To allow future expansion, please use queueEvent() instead of calling
	 *  ThreadedMessageQueue::send directly via this accessor. */
	ThreadedMessageQueue<EventBase*>& getEventQueue() { return *eventQueue; }
#endif
	//@}

	
	//!@name Listener/Trapper Recall
	
	//! returns true if the specified listener/trapper would receive any events that match the specified criteria
	bool isListeningAny(const EventListener* el, EventBase::EventGeneratorID_t egid) const { return listeners.verifyMappingAny(el,egid); }
	bool isListeningAny(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid) const { return listeners.verifyMappingAny(el,egid,sid); }
	bool isListeningAll(const EventListener* el, EventBase::EventGeneratorID_t egid) const { return listeners.verifyMappingAll(el,egid); }
	bool isListeningAll(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid) const { return listeners.verifyMappingAll(el,egid,sid); }
	bool isListening(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const { return listeners.verifyMapping(el,egid,sid,etid); }
	bool isListening(const EventListener* el, const EventBase& e) const { return listeners.verifyMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID()); }
	bool isTrappingAny(const EventTrapper* el, EventBase::EventGeneratorID_t egid) const { return trappers.verifyMappingAny(el,egid); }
	bool isTrappingAny(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid) const { return trappers.verifyMappingAny(el,egid,sid); }
	bool isTrappingAll(const EventTrapper* el, EventBase::EventGeneratorID_t egid) const { return trappers.verifyMappingAll(el,egid); }
	bool isTrappingAll(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid) const { return trappers.verifyMappingAll(el,egid,sid); }
	bool isTrapping(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const { return trappers.verifyMapping(el,egid,sid,etid); }
	bool isTrapping(const EventTrapper* el, const EventBase& e) const { return trappers.verifyMapping(el,e.getGeneratorID(),e.getSourceID(),e.getTypeID()); }
	//@}
	
	
	//!@name Listener/Trapper Detection
	
	/*!@brief <b>counts both listeners and trappers</b>, so generators can tell if it even needs to bother generating an event...*/
	/* Generators can also subscribe to the EventBase::erouterEGID event stream if
	 * they wish to be notified when they gain or lose listeners (particularly the
	 * first or last).\n
	 * ... if a tree falls in a forest, and there's no one around to see it, does
	 * it make a sound?\n
	 * ... if Vision sees a ball in an image, and there's no listeners, does it
	 * make an event? ;) */
	bool hasListeners(EventBase::EventGeneratorID_t egid) { return trappers.hasMapping(egid) || listeners.hasMapping(egid); }
	bool hasListeners(EventBase::EventGeneratorID_t egid, size_t sid) { return trappers.hasMapping(egid,sid) || listeners.hasMapping(egid,sid); }
	bool hasListeners(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) { return trappers.hasMapping(egid,sid,etid) || listeners.hasMapping(egid,sid,etid); }
	bool hasListeners(const EventBase& e) { return hasListeners(e.getGeneratorID(),e.getSourceID(),e.getTypeID()); }
	//@}

	
	//!@name Timer Management
	
	//! adds a timer if it doesn't exist, or resets the timer if it already exists.
	void addTimer(EventListener* el, size_t sid, unsigned int delay, bool repeat=true);
	void addTimer(EventListener* el, const EventBase& e, bool repeat=true) { addTimer(el,e.getSourceID(),e.getDuration(),repeat); } //!< calls the other addTimer() with the event's source id and duration, doesn't check to see if the generator is timerEGID
	
	//! clears all pending timers for listener @a el; see remove()
	void removeTimer(const EventListener* el);
	void removeTimer(const EventListener* el, size_t sid); //!< clears any pending timers with source id @a sid for listener @a el
	void removeTimer(const EventListener* el, EventBase& e) { if(e.getGeneratorID()==EventBase::timerEGID) removeTimer(el,e.getSourceID()); } //!< clears any pending timers with source id matching that of @a e, but only if @a e has generator timerEGID
	void removeAllTimers(); //!< clears all timers for all listeners
	
	unsigned int getNextTimer() { return (timers.size()==0 ? -1U : timers.front()->next); } //!< returns time of next timer activation
	
	struct TimerEntry;
	//! Returns information of next timer activation (regardless of sid) for an event listener, NULL if none
	/*! Don't try to use this to edit the timer entry, just call addTimer() again to reset the fields */
	const TimerEntry* getNextTimerInfo(const EventListener* el);
	//! Returns information of next activation for a specific event listener timer entry, NULL if none
	/*! Don't try to use this to edit the timer entry, just call addTimer() again to reset the fields */
	const TimerEntry* getNextTimerInfo(const EventListener* el, size_t sid);
	//@}

	
	//!@name Listener Management
	
	//! Adds a listener for all events from a given event generator
	void addListener(EventListener* el, EventBase::EventGeneratorID_t egid);
	void addListener(EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid); //!< Adds a listener for all types from a specific source and generator
	void addListener(EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid); //!< Adds a listener for a specific source id and type from a given event generator
	void addListener(EventListener* el, const EventBase& e); //!< Uses the generator, source, and type fields of @a e to add a listener for that specific tuple

#ifndef TGT_IS_DYNAMIC
	//!@name Remote Event/State code

	//! Request remote events to be sent to this robot, works like the regular addListeners
    void addRemoteListener(EventListener* el, int host, EventBase::EventGeneratorID_t egid);
    void addRemoteListener(EventListener* el, int host, EventBase::EventGeneratorID_t egid, size_t sid);
    void addRemoteListener(EventListener* el, int host, const EventBase& e);
    void addRemoteListener(EventListener* el, int host, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid);

	//! Stop getting remote events from the given robot
    void removeRemoteListener(const EventListener* el, int host, EventBase::EventGeneratorID_t egid);
    void removeRemoteListener(const EventListener* el, int host, EventBase::EventGeneratorID_t egid, size_t sid);
    void removeRemoteListener(const EventListener* el, int host, const EventBase& e);
    void removeRemoteListener(const EventListener* el, int host, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid);

	//! Request remote state updates from the remote robot, every interval ms
	void requestRemoteStateUpdates(int host, RemoteState::StateType type, unsigned int interval = 500);

	//! Stop getting remote state updates
	void stopRemoteStateUpdates(int host, RemoteState::StateType type);

	//! This is called once on startup; it tells the EventRouter to listen for incoming requests
	bool serveRemoteEventRequests();

	//! This handles incomiung connection requests by starting a new EventProxy
	int processData(char *data, int bytes);

	static const int defaultPort = 2424;
	
protected:

	RemoteRouter &remoteRouterForHost(int host);

	std::list<EventProxy *> proxies;

	std::map<int, RemoteRouter *> rrouters;
	Socket *sck;
	int nextProxyPort;
#endif

	//@}
public:

	
	//! stops sending ALL events to the listener -- does not remove pending timers however (may need to call removeTimer(el) as well); see remove()
	void removeListener(const EventListener* el); 
	//! stops sending specified events from the generator to the listener.
	void removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid);
	void removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid); //!< stops sending specified events from the generator to the listener.
	void removeListener(const EventListener* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid); //!< stops sending specified events from the generator to the listener.
	void removeListener(const EventListener* el, const EventBase& e); //!< Uses the generator, source, and type fields of @a e to remove a listener for that specific tuple
	
	//! stops all events and timers, shorthand for removeListener(el) and removeTimer(el); Note that trappers are separate, removeTrapper() is @e not called
	void remove(const EventListener* el) { removeListener(el); removeTimer(el); }

	//@}
	

	//!@name Trapper Management
	
	//! Adds a trapper for a specific source id and type from a given event generator
	/*! Note that only the broadcasted timers can be trapped.  The EventListener which requested the timer will receive that timer before any trapping is done. */
	void addTrapper(EventTrapper* el, const EventBase& e);
	void addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid); //!< Adds a trapper for all events from a given event generator
	void addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid);  //!< Adds a trapper for all types from a specific source and generator
	void addTrapper(EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid); //!< Adds a trapper for a specific source id and type from a given event generator

	void addTrapper(EventTrapper* el); //!< adds a trapper for ALL events

	//! stops sending specified events from the generator to the trapper.
	void removeTrapper(const EventTrapper* el, const EventBase& e);
	void removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid); //!< stops sending specified events from the generator to the trapper.
	void removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid); //!< stops sending specified events from the generator to the trapper.
	void removeTrapper(const EventTrapper* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid); //!< stops sending specified events from the generator to the trapper.

	void removeTrapper(const EventTrapper* el); //!< stops sending ALL events to the trapper
	//@}

	//! Contains all the information needed to maintain a timer by the EventRouter
	struct TimerEntry {
		//! constructs an entry using the given value for next - useful for with TimerEntryPtrCmp
		explicit TimerEntry(unsigned int nxt) : el(NULL), sid(0), delay(0), next(nxt), repeat(false) {}
		//! constructs with the given values, sets next field automatically; see next
		TimerEntry(EventListener* e, size_t s, unsigned int d, bool r) : el(e), sid(s), delay(d), next(get_time()+delay), repeat(r) {}
		//! just does the default, i'm just being explicit since there's a pointer (no deep copy!)
		TimerEntry(const TimerEntry& t) : el(t.el), sid(t.sid), delay(t.delay), next(t.next), repeat(t.repeat) {}
		//! just does the default, i'm just being explicit since there's a pointer (no deep copy!)
		TimerEntry& operator=(const TimerEntry& t) { el=t.el; sid=t.sid; delay=t.delay; next=t.next; repeat=t.repeat; return *this; }
		//! will reset timer
		/*! @param d the time from now when the timer should go off (in milliseconds)
		 *  @param r true if the timer should automatically repeat */
		void Set(unsigned int d, bool r) { delay=d; repeat=r; next=get_time()+delay; }
		EventListener* el;  //!< the listener to fire at
		size_t sid;   //!< the source id to fire with
		unsigned int delay; //!< the delay until firing
		unsigned int next;  //!< the time at which this timer will go off next
		bool repeat;        //!< if true, will reset after firing, else will be deleted
	};

protected:
	/*! @brief Used by STL to sort the timer list in order of activation time
	 *  @see EventRouter::timers */
	class TimerEntryPtrCmp {
	public:
		//! Used by STL to sort the timer list in order of activation time; see timers
		/*! Since we remove NULLs before sorting, shouldn't need to check here (and I want to know if i'm wrong)
		 *  @return (a->next<b->next) */
		bool operator()(const TimerEntry* const a, const TimerEntry* const b) const { return (a->next<b->next); }
	};
	typedef std::vector<TimerEntry*>::iterator timer_it_t; //!< makes code more readable
	std::vector<TimerEntry*> timers;         //!< the list of timer entries being maintained, kept sorted by time they go active

public:
	//! just for debugging
	void chkTimers();

	//! just for debugging
	void dispTimers();

protected:
	//! Does the actual storage of the mapping between EventBase's and the EventListeners/EventTrappers who should receive them
	/*! Actually only stores void*'s, so it's more general than just Listeners or Trappers */
	class EventMapper {
	public:
		//! constructor
		EventMapper();

		void addMapping(void* el, EventBase::EventGeneratorID_t egid) { allevents[egid].push_back(el); } //!< Adds a listener for all events from a given event generator
		void addMapping(void* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid); //!< Adds a listener for a specific source id and type from a given event generator

		//! Removes a listener for all events from a given event generator, returns true if something was actually removed
		/*! Doesn't necessarily remove the vector or mapping if this was the last listener, use clean() to do that */
		bool removeMapping(const void* el, EventBase::EventGeneratorID_t egid); 

		//! Removes a listener for a specific source id and type from a given event generator, returns true if something was actually removed
		/*! Doesn't necessarily remove the vector or mapping if this was the last listener, use clean() to do that */
		bool removeMapping(const void* el, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid);

		void clean(); //!<removes empty data structures for all event generators
		void clean(EventBase::EventGeneratorID_t egid); //!< removes empty data structures associated with a single event generator
		void clear(); //!<Resets the mapping

		//@{
		//! so stuff can tell if it even needs to bother generating an event...
		/*! ... if a tree falls in a forest, and there's no one around to see it, does it make a sound?\n
			  ... if Vision sees a ball in an image, and there's no listeners, does it make an event? ;) \n
			  @return true if it has any listeners, false otherwise */
		bool hasMapping(EventBase::EventGeneratorID_t egid) const;
		bool hasMapping(EventBase::EventGeneratorID_t egid, size_t sid) const;
		bool hasMapping(EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const;
		//@}

		//! builds a list of all listeners which should receive the event, templated to typecast the pointers for you
		/*! @param e the key event
		 *  @param listeners upon return, the resulting list of listeners @a e maps to\n
		 *  @a listeners is not cleared prior to building, new listeners are pushed on end\n
		 *  Results are in the order: all specific matches first, all generator listeners second, in order they were added to the EventMapper.*/
		template<class T>
		void getMapping(const EventBase& e, std::vector<T*>& listeners) const;

		//! Used to make sure that the specified listener exists for the given event
		/*! This is needed because after we call processEvent on a lister, we can't assume
		 *  that no other listeners have been modified - one listener could cause another
		 *  to turn off.  If that has happened, we shouldn't send the event, even if it
		 *  was in the queue originally. */
		bool verifyMapping(const void * listener, const EventBase& e) const { return verifyMapping(listener,e.getGeneratorID(),e.getSourceID(),e.getTypeID()); }
		//! Used to make sure that the specified listener exists for the given event
		/*! This is needed because after we call processEvent on a lister, we can't assume
		 *  that no other listeners have been modified - one listener could cause another
		 *  to turn off.  If that has happened, we shouldn't send the event, even if it
		 *  was in the queue originally. */
		bool verifyMapping(const void * listener, EventBase::EventGeneratorID_t egid, size_t sid, EventBase::EventTypeID_t etid) const;

		//! Needed to complete EventRouter::isListening suite
		/*! Only checks #allevents */
		bool verifyMappingAll(const void * listener, EventBase::EventGeneratorID_t egid) const;
		//! Needed to complete EventRouter::isListening suite
		/*! Checks both #allevents and #filteredevents */
		bool verifyMappingAny(const void * listener, EventBase::EventGeneratorID_t egid) const;

		//! Needed to complete EventRouter::isListening suite
		/*! Checks both #allevents and #filteredevents, must be found in all */
		bool verifyMappingAll(const void * listener, EventBase::EventGeneratorID_t egid, size_t sid) const;
		//! Needed to complete EventRouter::isListening suite
		/*! Checks both #allevents and #filteredevents, can be found in either */
		bool verifyMappingAny(const void * listener, EventBase::EventGeneratorID_t egid, size_t sid) const;

	protected:
		//! a mapping from source IDs (size_t's), each to a vector of pointers to listeners
		/*! main use in filteredevents @see filteredevents */
		typedef std::map<size_t,std::vector<void*>,std::less<size_t> > SIDtoListenerVectorMap_t;
		
		//! an array of vectors of pointers to listeners... in other words, a vector of listener pointers for each generator
		std::vector<void*> allevents[EventBase::numEGIDs];
		//! not for the faint of heart: a matrix of mappings to vectors of pointers to listeners
		SIDtoListenerVectorMap_t* filteredevents[EventBase::numEGIDs][EventBase::numETIDs];

	private:
		EventMapper(const EventMapper&);           //!< this shouldn't be called...
		EventMapper& operator=(const EventMapper&); //!< this shouldn't be called...
	};

	EventMapper trappers;  //!< A mapping of which EventTrapper's should get a chance to trap the event
	EventMapper listeners; //!< A mapping of which EventListener's should receive events

	//! contains information regarding the progress of posting an event
	/*! This allows us to resume and complete the posting of the "current" event before processing a new incoming event */
	class PostingStatus {
	public:
		//! constructor
		PostingStatus(const EventMapper& eventTrappers, const EventMapper& eventListeners, const EventBase& event)
			: trappers(eventTrappers), listeners(eventListeners), t(), tit(), l(), lit(), e(event)
		{ trappers.getMapping(e,t); tit=t.begin(); listeners.getMapping(e,l); lit=l.begin(); }
		//! begins or resumes sending the event #e to trappers and listeners in #t and #l
		void process();
	protected:
		const EventMapper& trappers; //!< the current trapper mapping, used to verify each entry in #t is still valid before processing it
		const EventMapper& listeners; //!< the current listener mapping, used to verify each entry in #l is still valid before processing it
		std::vector<EventTrapper*> t; //!< list of trappers which were subscribed when the PostingStatus instance was constructed
		std::vector<EventTrapper*>::const_iterator tit; //!< current position within #t
		std::vector<EventListener*> l; //!< list of listeners which were subscribed when the PostingStatus instance was constructed
		std::vector<EventListener*>::const_iterator lit; //!< current position within #l
		const EventBase& e; //!< the event being processed
	};
	std::queue<PostingStatus*> postings; //!< stores calls to post() currently in progress -- may grow if one postEvent() triggers another; this allows us to finish processing of the original postEvent() before starting the second.
	
	//! This table will be checked on each processEvent to forward the event to some other destination
	/*! The main reason for including this functionality is in the uni-process model, we don't want
	 *  event postings from real time processes like Motion to block on the event queue processing.
	 *  So with this mechanism we can intercept those events, and queue them in a separate IPC
	 *  mechanism to be picked up by Main later on.
	 *
	 *  This might also be handy for other purposes, such as remote event forwarding over the network...
	 *
	 *  If the EventTranslator's trapEvent returns true, then further processing on the event is skipped.
	 *  (see EventTranslator::setTrapEventValue() )*/
	EventTranslator* forwards[ProcessID::NumProcesses];
	
#ifndef PLATFORM_APERIOS
	//! A queue of events which have been posted from background threads or otherwise desire postponed processing
	/*! This is a more efficient mechanism than #forwards, but only available on threaded platforms (e.g. non-AIBO).
	 *  See queueEvent() and getEventQueue() */
	ThreadedMessageQueue<EventBase*>* eventQueue;
	
	//! Predicate used to remove old events from the same event source from #eventQueue (see requeueEvent())
	struct SameID {
		const EventBase* e;
		explicit SameID(const EventBase* event) : e(event) {}
		bool operator()(const EventBase* x) { return e->getGeneratorID()==x->getGeneratorID() && e->getSourceID()==x->getSourceID(); }
	};
#endif
	
private:
  EventRouter(const EventRouter&); //!< don't call this
  EventRouter& operator=(const EventRouter&); //!< don't call this

};

//! a global router for cross communication, probably the most common usage, although perhaps there may be times you'd rather have multiple event routers for smaller sections
extern EventRouter * erouter;

/*! @file
 * @brief Describes EventRouter class, for distribution and trapping of events to listeners
 * @author ejt (Creator)
 */

#endif
