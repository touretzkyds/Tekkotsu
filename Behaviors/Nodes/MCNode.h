//-*-c++-*-
#ifndef INCLUDED_MCNode_h_
#define INCLUDED_MCNode_h_

#include "Behaviors/StateNode.h"
#include "Motion/MotionManager.h"
#include "Motion/MMAccessor.h"
#include "IPC/SharedObject.h"

//! Common parent class for all the templated MCNode, which is what you want to instantiate.
class MCNodeBase : public StateNode {
public:
  static const char defName[]; //!< the default name for MCNodes -- can be overridden via MCNode's template arguments
  static const char defDesc[]; //!< the default description for MCNodes -- can be overridden via MCNode's template arguments

  //! destructor, free #mc
  virtual ~MCNodeBase() { delete mc; mc=NULL; }
	
  //! Removes the motion command from the motion manager if it was our own creation
  virtual void stop();	
	
  //! Assumes the event is a completion event from the motion, throws a corresponding state node completion event
  virtual void doEvent();
	
  //! Allows you to assign a previously created motion, which might be shared among several MCNodes
  /*! If this node already has an #mc, then it will be freed, removing from MotionManager if necessary */
  virtual void setMC(MotionManager::MC_ID mcid);
	
  //! reveal the MC_ID; if the motion isn't currently active, returns MotionManager::invalid_MC_ID
  virtual MotionManager::MC_ID getMC_ID() { return mc_id; }
	
  //! Return the priority assigned to this node's motion command
  virtual float getPriority() const { return priority; }

  //! Set the priority that will be used when adding this motion command; if the command is already active, change its priority in the MotionManager
  virtual void setPriority(const float p);

  //! Post a failure event and abort the motion command
  /*! This is useful if the user calls some IK method that determines
    that the target is unreachable, and an approximate solution isn't
    acceptable.
   */
  virtual void motionFails();

  static std::string getClassDescription() { return defName; }
  virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
  //! constructor for subclasses
  MCNodeBase(bool expectCompletion)
    : StateNode(), mc(NULL), mc_id(MotionManager::invalid_MC_ID), 
      priority(MotionManager::kStdPriority), mcCompletes(expectCompletion)
  {}

  //! constructor for subclasses which provide instance name
  MCNodeBase(const std::string &node_name, bool expectCompletion=true)
    : StateNode(node_name), mc(NULL), mc_id(MotionManager::invalid_MC_ID), 
      priority(MotionManager::kStdPriority), mcCompletes(expectCompletion)
  {}
	
  //! returns reference to #mc or a new SharedObject<T> if #mc is currently NULL (so it will always return a valid value)
  /*! if a particular motion command needs some initial setup besides the default constructor,
   *  overriding this function is a good opportunity to do so */
  virtual SharedObjectBase& getPrivateMC()=0;

  //! returns true if the motion command being used was created internally via getPrivateMC()
  virtual bool hasPrivateMC() { return mc!=NULL; }

  SharedObjectBase* mc;    //!< MotionCommand used by this node (may be NULL if sharing the MC with other nodes)
  MotionManager::MC_ID mc_id;  //!< id number for the MotionCommand
  float priority; //!< Priority to use when adding this motion commmand to the MotionManager
  bool mcCompletes; //!< if true, will post a completion when the underlying MotionCommand posts a status

private:
  MCNodeBase(const MCNodeBase&); //!< don't call (copy constructor)
  MCNodeBase& operator=(const MCNodeBase&); //!< don't call (assignment operator)
};

//! A generic wrapper for any MotionCommand.  Note that some functions are provided by the MCNodeBase common base class, namely MCNodeBase::setMC() and MCNodeBase::getMC_ID()
template<class T, const char* mcName=MCNodeBase::defName, const char* mcDesc=MCNodeBase::defDesc, bool completes=true>
class MCNode : public MCNodeBase {
public:
  //! default constructor
  MCNode()
    : MCNodeBase(completes)
  {}
	
  //! constructor, take an instance name
  explicit MCNode(const std::string& nm)
    : MCNodeBase(nm,completes)
  {}
	
  //! constructor, take an instance name
  explicit MCNode(const char* nm)
    : MCNodeBase(nm,completes)
  {}
	
  //! destructor
  virtual ~MCNode() {}
	
  //! reveal the MotionCommand through an MMAccessor
  /*! This is a no-op if the motion command hasn't been added to motion manager yet, and enforces mutual exclusion if it has */
  virtual MMAccessor<T> getMC();

  //! Adds the motion command to the motion manager, but stops it from running until the user has a chance to program it
  virtual void preStart();

  //! Adds a listener and then starts the motion command
  virtual void postStart();

  static std::string getClassDescription() { return mcDesc; }
  virtual std::string getDescription() const { return getClassDescription(); }

protected:
  explicit MCNode(bool subCompletes) : MCNodeBase(subCompletes)	{}
	
  //! constructor, take an instance name
  MCNode(const std::string& nm, bool subCompletes) : MCNodeBase(nm,subCompletes) {}
	
  virtual SharedObject<T>& getPrivateMC();
};


// ****************************
// ******* IMPLEMENTATION *******
// ****************************

template<class T, const char* mcName, const char* mcDesc, bool completes>
MMAccessor<T> MCNode<T,mcName,mcDesc,completes>::getMC() {
  if(mc_id==MotionManager::invalid_MC_ID) {
    // motion hasn't been added to motion manager yet; don't try to check it out
    return MMAccessor<T>(*getPrivateMC(),false);
  } else {
    // motion has been added to motion manager, check it out
    return MMAccessor<T>(mc_id);
  }
}

template<class T, const char* mcName, const char* mcDesc, bool completes>
SharedObject<T>& MCNode<T,mcName,mcDesc,completes>::getPrivateMC() {
  if(mc==NULL)
    mc=new SharedObject<T>;
  return dynamic_cast<SharedObject<T>&>(*mc);
}

template<class T, const char* mcName, const char* mcDesc, bool completes>
void MCNode<T,mcName,mcDesc,completes>::preStart() {
    StateNode::preStart();
    getMC()->stop();  // prevent generation of premature completion events
}

template<class T, const char* mcName, const char* mcDesc, bool completes>
void MCNode<T,mcName,mcDesc,completes>::postStart() {
    StateNode::postStart();
    if(mc_id==MotionManager::invalid_MC_ID)
      mc_id = motman->addPersistentMotion(getPrivateMC(),priority);
    else {
      getMC()->start();
      motman->setPriority(mc_id,priority);
    }
    erouter->addListener(this,EventBase::motmanEGID,mc_id,EventBase::statusETID);
    getMC()->setDirty();  // in case addPersistentMotion caused an immediate completion that we missed
}

/*! @file
 * @brief Defines MCNode, which provides generic wrapper for any MotionCommand
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
