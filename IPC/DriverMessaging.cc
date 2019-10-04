#include "DriverMessaging.h"

using namespace std; 

namespace DriverMessaging {
	
	typedef std::map<const char* const, std::set<Listener*> > listeners_t;
	listeners_t listeners;

	const char* const LoadPrediction::NAME="LoadPrediction";
	const char* const SensorPriority::NAME="SensorPriority";
	const char* const FixedPoints::NAME="FixedPoints";

	Message::~Message() {}

	void addListener(Listener* l, const char* const t) {
		listeners[t].insert(l);
	}
	
	void removeListener(Listener* l, const char* const t) {
		listeners[t].erase(l);
	}
	
	void removeListener(Listener* l) {
		for(listeners_t::iterator it=listeners.begin(); it!=listeners.end(); ++it)
			it->second.erase(l);
	}
	
	bool hasListener(const char* const t) {
		return (listeners[t].size() > 0);
	}
	
	void postMessage(const Message& d) {
		std::set<Listener*>& live = listeners[d.CLASS_NAME]; // reference to 'live' listener list so we don't have to re-do the map lookup
		std::set<Listener*> ls = live; // make a copy so listeners can come and go during processing
		for(std::set<Listener*>::const_iterator it=ls.begin(); it!=ls.end(); ++it) {
			if(live.find(*it)!=live.end()) {
				(*it)->processDriverMessage(d);
			}
		}
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
