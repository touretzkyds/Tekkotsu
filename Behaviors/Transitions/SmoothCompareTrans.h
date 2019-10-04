//-*-c++-*-
#ifndef INCLUDED_SmoothCompareTrans_h_
#define INCLUDED_SmoothCompareTrans_h_

#include "Behaviors/Transitions/CompareTrans.h"

//! A subclass of CompareTrans, which provides monitoring of exponentially weighted averages to a threshold
/*! Has the additional requirement that template types must supply operator*=(float)
 *  and operator+=(T) for the weighted average
 *
 *  The gamma parameter is how much to weight the preceeding value - 1 will cause it to never update, 0 will cause it to only look at most recent.
 *  So, the lower the value, the faster it is to switch, but more prone to noise
 */
template<class T>
class SmoothCompareTrans : public CompareTrans<T> {
public:
	//! constructor, see SmoothCompareTrans class notes for information
	SmoothCompareTrans(StateNode* destination, const T* monitor, typename SmoothCompareTrans<T>::Test_t test, const T& value, const EventBase& poll, float gammap=0)
		: CompareTrans<T>(destination,&avg,test,value,poll), avg(*monitor), realmon(monitor),
		burnin((unsigned int)(1/(1-gammap))), tests(0), g(gammap)
	{ }

	//! constructor, see SmoothCompareTrans class notes for information
	SmoothCompareTrans(const std::string& name, StateNode* destination, const T* monitor, typename SmoothCompareTrans<T>::Test_t test, const T& value, const EventBase& poll, float gammap=0)
		: CompareTrans<T>(name,destination,&avg,test,value,poll), avg(*monitor), realmon(monitor),
		burnin((unsigned int)(1/(1-gammap))), tests(0), g(gammap)
	{ }

	virtual void postStart() {
		avg=*realmon;
		tests=0;
		CompareTrans<T>::postStart();
	}

	//! sets number of tests to perform before allowing a transition; default 1/(1-g)
	void setBurnIn(unsigned int i) {
		burnin=i;
	}

	//! returns number of tests to perform before allowing a transition; default 1/(1-g)
	unsigned int getBurnIn() {
		return burnin;
	}

	//!don't care about the event, just a pulse to check the values
	virtual void doEvent() {
		avg*=g;
		T x(*realmon);
		x*=(1-g);
		avg+=x;
		tests++;
		if(tests>burnin)
			CompareTrans<T>::doEvent();
	}

protected:
	T avg; //!< the current running average
	const T* realmon; //!< pointer to the value being monitored

	unsigned int burnin; //!< number of tests to perform before allowing a transition; default 1/(1-g)
	unsigned int tests; //!< counter of tests made since last doStart()

	//! the gamma value controlling the exponential average, see the class documentation at the top
	float g; 

private:
	SmoothCompareTrans(const SmoothCompareTrans& node); //!< don't call this
	SmoothCompareTrans operator=(const SmoothCompareTrans& node); //!< don't call this
};

/*! @file
 * @brief Defines SmoothCompareTrans, subclass of CompareTrans, which provides monitoring of exponentially weighted averages to a threshold
 * @author ejt (Creator)
 */

#endif
