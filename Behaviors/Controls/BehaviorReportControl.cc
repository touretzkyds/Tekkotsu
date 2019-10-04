#include "ControlBase.h"
#include "Behaviors/BehaviorBase.h"
#include "Wireless/Socket.h"
#include <cstdio>

//! Reads the set of currently instantiated behaviors and sends a report to ::sout
class BehaviorReportControl : public ControlBase {
public:
	//! Constructor
	BehaviorReportControl()
		: ControlBase("Behavior Report","Generates a summary of currently instantiated behaviors")
	{}

	//! Prints a report to sout
	virtual ControlBase * activate(MC_ID, Socket *) {
		typedef std::set<BehaviorBase*> registry_t;
		typedef std::vector<BehaviorBase*> behlist_t;
		const registry_t& reg=BehaviorBase::getRegistry();
		behlist_t active,inactive;
		for(registry_t::const_iterator it=reg.begin(); it!=reg.end(); it++) {
			if((*it)->isActive())
				active.push_back(*it);
			else
				inactive.push_back(*it);
		}

		char format[100];
		unsigned int maxlen=0;
		for(behlist_t::const_iterator it=active.begin(); it!=active.end(); it++)
			if((*it)->getName().size()>maxlen)
				maxlen=(*it)->getClassName().size();
		for(behlist_t::const_iterator it=inactive.begin(); it!=inactive.end(); it++)
			if((*it)->getName().size()>maxlen)
				maxlen=(*it)->getClassName().size();
		snprintf(format,100,"  %%-%ds   %%s\n",maxlen);

		sout->printf("** Currently Instantiated Behavior Report **\n");
		sout->printf("%lu active, %lu inactive, %lu total\n\n",(unsigned long)active.size(),(unsigned long)inactive.size(),(unsigned long)reg.size());
		sout->printf("Active Behaviors:\n");
		sout->printf(format,"Class Name","Instance Name");
		sout->printf(format,"------------","---------------");
		for(behlist_t::const_iterator it=active.begin(); it!=active.end(); it++)
			sout->printf(format,(*it)->getClassName().c_str(),(*it)->getName().c_str());
		sout->printf("\n");
		sout->printf("Inactive Behaviors:\n");
		sout->printf(format,"Class Name","Instance Name");
		sout->printf(format,"------------","---------------");
		for(behlist_t::const_iterator it=inactive.begin(); it!=inactive.end(); it++)
			sout->printf(format,(*it)->getClassName().c_str(),(*it)->getName().c_str());
		return NULL;
	}
};

REGISTER_CONTROL(BehaviorReportControl,"Status Reports");

/*! @file
 * @brief Defines BehaviorReportControl, which reads the set of currently instantiated behaviors and sends a report to ::sout
 * @author ejt (Creator)
 */
