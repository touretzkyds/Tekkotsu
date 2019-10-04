//-*-c++-*-
#ifndef INCLUDED_EventGeneratorBase_h_
#define INCLUDED_EventGeneratorBase_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventBase.h"

//! A simple convenience class for event generators
/*! Note that you don't need to inherit from this class to be able to
 *  send events!  Any code can send any event any time, just by calling 
 *  one of the EventRouter::postEvent() functions.
 *  
 *  Uses a BehaviorBase base class so that you can start and stop it.
 *
 *  Allows variable settings of the generator id and source id for
 *  outgoing events as well as automatic handling of
 *  listening/unlistening for a single optional event source.  If you
 *  want something more fancy that that though, you'll have to
 *  override doStart/doStop yourself. (or extend/modify this class...)
 *
 */
class EventGeneratorBase : public BehaviorBase {
public:
	// Constructors are all protected - doesn't make sense to
	// instantiate this class directly, you want to use a subclass

	//! destructor - does nothing
	virtual ~EventGeneratorBase() {}
	
	virtual void doStart();
	
	virtual void doStop();
	
	//! if autolistening, will receive EventRouter events concerning our own listeners
	/*! This will automatically reduce overhead by eliminating chains of events thrown
	 *  that don't have any end listeners.  However, this might mean your subclass's
	 *  processEvent will be receiving the events from erouterEGID, and will need
	 *  to call EventGeneratorBase::processEvent() in order to allow them to be used */
	virtual void processEvent(const EventBase& ev);
	

	//! These concern the events which will be thrown to listeners
	//!@name Downstream Settings

	//! return the generator ID that will be broadcast from
	virtual EventBase::EventGeneratorID_t getGeneratorID() const { return myGenID; }
	//! set the generator ID that will be broadcast from (typically it's a bad idea to call this...)
	virtual void setGeneratorID(EventBase::EventGeneratorID_t gid) { myGenID=gid; }

	//! return the source ID that will be broadcast on
	virtual size_t getSourceID() const { return mySourceID; }
	//! set the source ID that will be broadcast on
	virtual void setSourceID(size_t sid) { mySourceID=sid; }

	//! return true if this generator has listeners
	virtual bool hasListeners() const;
	//@}


	//! These help select which events will be received from other generators
	//!@name Upstream Settings

	//! lets you specify what level of filtering should be done
	enum specificity_t {
		GENERATOR, //!< only the generator needs to match, select all sources and types
		SOURCE,    //!< both generator and source need to match, select all types
		TYPE       //!< explicit event tuple; generator, source, and type must all match
	};
	//! returns the current specificity level, to modify this, call the appropriate version of setAutoListen()
	virtual specificity_t getSpecificity() const { return specificity; }

	//! turns on auto listening to make it easier to set up dependancies between vision filters
	virtual void setAutoListen(EventBase::EventGeneratorID_t gid);
	//! turns on auto listening to make it easier to set up dependancies between vision filters
	virtual void setAutoListen(EventBase::EventGeneratorID_t gid, size_t sid);
	//! turns on auto listening to make it easier to set up dependancies between vision filters
	virtual void setAutoListen(EventBase::EventGeneratorID_t gid, size_t sid, EventBase::EventTypeID_t tid);
	//! turns off auto listening
	virtual void unsetAutoListen();

	//! returns the generator ID that will be listened for (not the generator of the FilterBankEvent to be created - that depends on the subclass)
	virtual EventBase::EventGeneratorID_t getListenGeneratorID() const { return srcGenID; }
	//! returns the source ID that will be listened for (not the source of the FilterBankEvent to be created - that depends on the subclass)
	virtual size_t getListenSourceID() const { return srcSourceID; }
	//! returns the type ID that will be listened for (not the type of the FilterBankEvent to be created - that depends on the subclass)
	virtual EventBase::EventTypeID_t getListenTypeID() const { return srcTypeID; }

	//@}

protected:
	//!@name Constructors
	//!
	EventGeneratorBase(const std::string& instancename, EventBase::EventGeneratorID_t mgid, size_t msid)
		: BehaviorBase(instancename), myGenID(mgid), mySourceID(msid), autoListen(false), isListening(false), srcGenID(EventBase::numEGIDs), srcSourceID(), srcTypeID(), specificity()
	{}
	EventGeneratorBase(const std::string& instancename, EventBase::EventGeneratorID_t mgid, size_t msid,EventBase::EventGeneratorID_t srcgid)
		: BehaviorBase(instancename), myGenID(mgid), mySourceID(msid), autoListen(srcgid!=EventBase::numEGIDs), isListening(false), srcGenID(srcgid), srcSourceID(), srcTypeID(), specificity(GENERATOR)
	{}
	EventGeneratorBase(const std::string& instancename, EventBase::EventGeneratorID_t mgid, size_t msid,EventBase::EventGeneratorID_t srcgid, size_t srcsid)
		: BehaviorBase(instancename), myGenID(mgid), mySourceID(msid), autoListen(srcgid!=EventBase::numEGIDs), isListening(false), srcGenID(srcgid), srcSourceID(srcsid), srcTypeID(), specificity(SOURCE)
	{}
	EventGeneratorBase(const std::string& instancename, EventBase::EventGeneratorID_t mgid, size_t msid,EventBase::EventGeneratorID_t srcgid, size_t srcsid, EventBase::EventTypeID_t srctype)
		: BehaviorBase(instancename), myGenID(mgid), mySourceID(msid), autoListen(srcgid!=EventBase::numEGIDs), isListening(false), srcGenID(srcgid), srcSourceID(srcsid), srcTypeID(srctype), specificity(TYPE)
	{}
	//@}

	//! subscribe this generator to its source
	virtual void addSrcListener();

	//! unsubscribe this generator from its source
	virtual void removeSrcListener();

	EventBase::EventGeneratorID_t myGenID; //!< the generator ID to broadcast on
	size_t mySourceID;     //!< the source ID to broadcast on
	bool autoListen;          //!< if true, will automatically start listening for EventBase(genID,sourceID) events
	bool isListening;         //!< true if listening triggered by autoListen
	EventBase::EventGeneratorID_t srcGenID; //!< the generator ID to listen for (typically the source that this filter works on)
	size_t srcSourceID;    //!< the source ID to listen for
	EventBase::EventTypeID_t srcTypeID; //!< the type ID to listen for
	specificity_t specificity; //!< the level of event specificity that is being listened for, so when #autoListen is triggered, we can subscribe to the right level of event stream
};

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
