//-*-c++-*-
#ifndef INCLUDED_RandomTrans_h_
#define INCLUDED_RandomTrans_h_

#include "Behaviors/Transitions/NullTrans.h"
#include "Events/EventRouter.h"

//! A transition that fires immediately, randomly choosing one destination node to activate.

/*! This class should be extended in the following way:
 *
 *  setHistoryLength(int n) to maintain a history of recent choices that are
 *  not to be reused, i.e., drawing without replacement
 */

class RandomTrans : public NullTrans {
public:
  //! constructor
  RandomTrans(StateNode* destination, float weight=1);
	
  //! constructor
  RandomTrans(const std::string& name, StateNode* destination, float weight=1);

  //! Add a destination node with a specified weight (defaults to 1.0)
  virtual void addDestination(StateNode* destination, float const weight);

	// This is redundant but needed to shadow Transition::addDestination
  virtual void addDestination(StateNode* destination) { addDestination(destination, 1); }

  //! Firing this type of transition activates one destination node at random, instead of all destinations.
  virtual void fire();
  using Transition::fire;

protected:
	//! constructor, only to be called by subclasses (which need to specify their own @a classname)
  RandomTrans(const std::string &classname, const std::string &instancename, 
	    StateNode* destination, float weight=1);

private:
  std::vector<float> weights; //!< the probably of selection for each source
  void addWeight(float weight); //!< adds a weight entry to the back of the queue
};

#endif
