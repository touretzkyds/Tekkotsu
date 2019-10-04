//-*-c++-*-
#ifndef INCLUDED_DeviceDriver_h_
#define INCLUDED_DeviceDriver_h_

#include "Shared/InstanceTracker.h"
#include "Shared/plistCollections.h"
#include <set>
class MotionHook;
class DataSource;

//! description of DeviceDriver
class DeviceDriver : public virtual plist::Dictionary {
public:
	//! destructor, removes from registry in case we're deleting it from some other source than registry's own destroy()
	virtual ~DeviceDriver() { getRegistry().destroy(instanceName); }
	
	//! Returns the name of the class (aka its type)
	/*! Suggested implementation is to declare a static string member, set it to the result of
	 *  calling the registry's registerType, and then return that member here */
	virtual std::string getClassName() const=0;
	//! Returns the name of the instance (#instanceName)
	virtual std::string getName() const { return instanceName; }
	
	virtual MotionHook* getMotionSink() { return NULL; }
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) { sources.clear(); }
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) { sources.clear(); }
	
	typedef InstanceTracker<DeviceDriver,std::string,Factory1Arg<DeviceDriver,std::string> > registry_t;
	static registry_t& getRegistry() { static registry_t registry; return registry; }
	
	//! allows LoadDataThreads to be notified when a data source is added or removed
	class SourceListener {
	public:
		virtual ~SourceListener() {}; //!< destructor
		virtual void dataSourcesUpdated()=0; //!< indicates a data source has been added or removed
	};
	
	//! add a listener to #sourceListeners
	virtual void addSourceListener(SourceListener* l) { if(l!=NULL) sourceListeners.insert(l); }
	//! remove a listener from #sourceListeners
	virtual void removeSourceListener(SourceListener* l) { sourceListeners.erase(l); }
	
protected:
	//! constructor, pass the name of the class's type so we can use it in error messages, and a name for the instance so we can register it for MotionHook's to lookup
	DeviceDriver(const std::string& /*classname*/, const std::string& instancename)
	: plist::Dictionary(), instanceName(instancename), sourceListeners()
	{
		setLoadSavePolicy(FIXED,SYNC);
	}

	//! To be called be "deepest" subclass constructor at the end of construction
	/*! Don't want to register until completed construction!  plist::Collection listeners would be
	 *  triggered and might start performing operations on instance while partially constructed */
	virtual void registerInstance() {
		if(DeviceDriver * inst=getRegistry().getInstance(instanceName)) {
			if(inst==this)
				return; // duplicate registration, skip it
			std::cerr << "Warning: registration of DeviceDriver " << getClassName() << " named " << instanceName << " @ " << this
			<< " blocked by previous " << inst->getClassName() << " instance of same name @ " << inst << std::endl;
		}
		if(!getRegistry().registerInstance(getClassName(),instanceName,this))
			std::cerr << "Error: failed to register " << getClassName() << " named " << instanceName << " @ " << this;
		//addEntry(".type",new plist::Primitive<std::string>(className),"Stores the typename of the device driver so it can be re-instantiated on load.\n** Do not edit ** ");
	}
	
	//! calls SourceListener::dataSourcesUpdated() for entries registered in #sourceListeners
	virtual void fireDataSourcesUpdated() {
		std::set<SourceListener*> notify=sourceListeners;
		for(std::set<SourceListener*>::const_iterator it=notify.begin(); it!=notify.end(); ++it) {
			if(sourceListeners.find(*it)!=sourceListeners.end()) // check that it hasn't been removed during course of processing...
				(*it)->dataSourcesUpdated();
		}
	}
	
	const std::string instanceName; //!< holds the name of this instance of CommPort (mainly for error message reporting by the class itself)
	std::set<SourceListener*> sourceListeners; //!< list (of LoadDataThreads) to be notified when a data source is added or removed
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
