//-*-c++-*-
#ifndef INCLUDED_BehaviorSwitchControl_h_
#define INCLUDED_BehaviorSwitchControl_h_

#include "ControlBase.h"
#include "Behaviors/BehaviorBase.h"
#include "Shared/ReferenceCounter.h"
#include "Shared/Factories.h"

//! Holds some utility classes and functions for BehaviorSwitchControl which shouldn't be stored in a templated class
class BehaviorSwitchControlBase : public ControlBase {
public:
	//! A simple utility class to allow the BehaviorSwitchControl's to be able to deactivate the current behavior when a new one becomes active
	/*! Most behaviors are either major actions which you'll only want one of active
	 *  at a time, or else their background monitors of some sort, that can run in different
	 *  combinations.  Think radio buttons vs. checkboxes.  This will help you implement the
	 *  "radio button" style... just assign all the behaviors to the same group, they will
	 *  automatically use it to turn the previous behavior off when a new one becomes active.\n
	 *  Pass NULL instead of one of these to get checkbox-style. */
	class BehaviorGroup : public ReferenceCounter {
	public:
		BehaviorGroup() : curBehavior(NULL), members() { } //!< contructor
		~BehaviorGroup() { if(curBehavior!=NULL) curBehavior->stop(); } //!< destructor, will stop the current behavior if it was a one-shot
		BehaviorBase * curBehavior; //!< pointer to current behavior
		std::set<BehaviorSwitchControlBase*> members; //!< set of members of the group
	private:
		BehaviorGroup(const BehaviorGroup&); //!< shouldn't be called
		BehaviorGroup operator=(const BehaviorGroup&); //!< shouldn't be called
	};
	
	
	//! constructor
	BehaviorSwitchControlBase(const std::string& n, BehaviorBase* beh, BehaviorGroup* bg=NULL)
		: ControlBase(n), behgrp(NULL), mybeh(beh)
	{
		if(mybeh!=NULL) {
			mybeh->addReference();
			mybeh->setName(n);
			if(mybeh->isActive())
				mybeh->addReference();
		}
		setGroup(bg);
	}
	
	//! constructor, behavior must not be NULL
	BehaviorSwitchControlBase(BehaviorBase* beh, BehaviorGroup* bg=NULL)
		: ControlBase(beh->getName()), behgrp(NULL), mybeh(beh)
	{
		mybeh->addReference();
		if(mybeh->isActive())
			mybeh->addReference();
		setGroup(bg);
	}

	//! destructor
	virtual ~BehaviorSwitchControlBase() {
		//cout << "~BehaviorSwitchControlBase(): " << getName() << endl;
		stop();
		setGroup(NULL);
		if(mybeh!=NULL)
			mybeh->removeReference();
		mybeh=NULL;
	}

	//! activates the behavior, handy for making start-up behaviors that you can turn off again with the Controller
	/*! If you start twice without stopping (ie it's already running), shouldn't do anything */
	virtual BehaviorSwitchControlBase* start() { if(!isRunning()) { stopother(); startmine(); } return this; }

	//! stops the behavior
	virtual BehaviorSwitchControlBase* stop() { if(isRunning()) stopother(); return this; }

	//! toggles the behavior
	virtual BehaviorSwitchControlBase* toggle() { if(isRunning()) stopother(); else { stopother(); startmine(); } return this; }

	virtual ControlBase * takeInput(const std::string& msg);
	
	//! Reassigns the behavior group, pass NULL to remove from group.
	/*! If this behavior is active and the new group already has an active behavior, the new group's active behavior is stopped first. */
	virtual void setGroup(BehaviorGroup* bg);
	
	//! tells the current behavior (if there is one) to stop then loads its own
	/*! @return NULL unless there are submenus */
	virtual ControlBase * activate(MC_ID display, Socket * gui);

	//! adds a status to the name: - if in memory, # if running
	virtual std::string getName() const;
	
	virtual std::string getDescription() const;
	
	//! Returns true if the associated behavior is running
	virtual bool isRunning() const;
	
protected:
	//! Stops the "other" guy's behavior - if ::behgrp is NULL, stops ourselves
	virtual void stopother();
	
	//! Starts our behavior
	virtual void startmine();
	
	//! updates other members in the group that the current behavior stopped -- do not call if behgrp is NULL
	virtual void notifyGroupMembers();
	
	//! called by notifyGroupMembers if #mybeh was destructed when stopped
	virtual void behaviorStopped() {}

	BehaviorGroup * behgrp; //!< the behavior group this belongs to.  Uses this to track the "current" behavior
	BehaviorBase* mybeh; //!< used to store the behavior.  If retained and non-NULL, will be valid.  However, if not retained, only valid if equals behgrp->curBehavior

private:
	BehaviorSwitchControlBase(const BehaviorSwitchControlBase&); //!< shouldn't copy these
	BehaviorSwitchControlBase operator=(const BehaviorSwitchControlBase&); //!< shouldn't assign these
};




//! Allows proper switching between major behaviors, calling doStart and doStop
template < class B, class Al = typename Factory0Arg<B>::template Factory<B> >
class BehaviorSwitchControl : public BehaviorSwitchControlBase {
public:
	//! constructor, can use this to toggle a single behavior on and off
	BehaviorSwitchControl(const std::string& n, bool retain=false)
		: BehaviorSwitchControlBase(n,NULL,NULL), retained(retain), startref(NULL)
	{}
	//! constructor, if you want to use an already constructed behavior
	BehaviorSwitchControl(B* beh, BehaviorGroup* bg=NULL)
		: BehaviorSwitchControlBase(beh,bg), retained(true), startref(NULL)
	{}
	//! constructor, if you want to use an already constructed behavior, but unretain it if it's stopped (if not retaining, will start @a beh if it's not already started)
	BehaviorSwitchControl(const std::string& n, B* beh, BehaviorGroup* bg=NULL, bool retain=false)
		: BehaviorSwitchControlBase(n,beh,bg), retained(retain), startref(NULL)
	{
		if(!retained) {
			// have to make sure behavior is started to maintain invariants
			if(!mybeh->isActive()) {
				startmine();
			}
			mybeh->removeReference(); //cancels reference from BehaviorSwitchControlBase's constructor
		}
	}
	//! constructor, needs to know what group its in and whether to retain its behavior
	BehaviorSwitchControl(const std::string& n, BehaviorGroup* bg, bool retain=false)
		: BehaviorSwitchControlBase(n,NULL,bg), retained(retain), startref(NULL)
	{}
	
	//! destructor
	virtual ~BehaviorSwitchControl() {
		stop();
		setGroup(NULL);
		if(mybeh!=NULL && retained)
			mybeh->removeReference();
		mybeh=NULL;
	}

	virtual std::string getName() const {
		if(!isValid())
			return ControlBase::getName();
		else
			return BehaviorSwitchControlBase::getName();
	}
	virtual std::string getDescription() const {
		if(!isValid() || mybeh->getDescription().size()==0)
			return B::getClassDescription();
		else
			return BehaviorSwitchControlBase::getDescription();
	}
	
protected:

	virtual void startmine() {
		if(!retained) {
			Al allocator;
			mybeh=allocator();
			mybeh->setName(getName());
		} else {
			if(mybeh==NULL) {
				Al allocator;
				mybeh=allocator();
				mybeh->setName(getName());
				mybeh->addReference();
			}
		}
		startref=mybeh;
		startref->addReference();
		BehaviorSwitchControlBase::startmine();
	}

	//! adds a check to see if behavior has stopped itself -- if so, remove startref
	virtual bool isRunning() const {
		if(BehaviorSwitchControlBase::isRunning())
			return true;
		else if(startref!=NULL) 
			const_cast<BehaviorSwitchControl<B,Al>*>(this)->stopother();
		return false;
	}		
	
	//! Returns true if mybeh is pointing to a valid object
	virtual bool isValid() const {
		if(isRunning())
			return true;
		return retained;
	}

	virtual void behaviorStopped() {
		if(!retained)
			mybeh=NULL;
		if(startref!=NULL) {
			startref->removeReference();
			startref=NULL;
		}
	}
	
	bool retained; //!< true if the behavior should be generated once and retained after doStop.  Otherwise, a new one is generated each time it is started
	BehaviorBase * startref; //!< true if a reference was added (and still current) from calling doStart
	
private:
	BehaviorSwitchControl(const BehaviorSwitchControl&); //!< shouldn't call this
	BehaviorSwitchControl operator=(const BehaviorSwitchControl&); //!<shouldn't call this
};

/*! @file
 * @brief Describes BehaviorSwitchControl<BEH> and its common superclass BehaviorSwitchControlBase - a control for turning behaviors on and off
 * @author ejt (Creator)
 */

#endif
