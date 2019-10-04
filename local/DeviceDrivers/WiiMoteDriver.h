//-*-c++-*-
#ifndef INCLUDED_WiiMote_h_
#define INCLUDED_WiiMote_h_

#include "local/DeviceDriver.h"
#include "local/MotionHook.h"
#include "local/DataSource.h"

#include "IPC/Thread.h"

#include <bluetooth/bluetooth.h>
#include <wiimote.h>

using namespace std;

// bleh work around hack :(
class WiiMoteDriver;
WiiMoteDriver *driver = 0;

class DataCache:public DataSource{
 public:
  DataCache();
  virtual ~DataCache() {}
  virtual unsigned int nextTimestamp() { return 0; };
  virtual const std::string& nextName() {
		static const std::string noneStr="(none)";
		return noneStr;
	}

  virtual unsigned int getData(const char *& payload, unsigned int& size, unsigned int& timestamp, std::string& name);
  virtual void setDataSourceThread(LoadDataThread* p);

	virtual void updateAccMesg(const wiimote_acc_mesg &mesg);
	virtual void updateBtnMesg(const wiimote_btn_mesg &mesg);

 protected:
	wiimote_acc_mesg acc_mesg;
	wiimote_btn_mesg btn_mesg;

	unsigned int number;

  std::ostringstream state;
  std::string states[2];
  int cstate;

  ThreadNS::Condition updateCond;
  ThreadNS::Lock updateLock;

	short updated;

  virtual void doFreeze() {} //!< user hook for when #frozen is set to true
  virtual void doUnfreeze() {} //!< user hook for when #frozen is set to false

 private:
	DataCache(const DataCache&); //!< do not call
	DataCache& operator=(const DataCache&); //!< do not call
};


//! Provides something
/*!
 *
 *  To do: write something here
 */
class WiiMoteDriver : public virtual DeviceDriver, public MotionHook, public virtual plist::PrimitiveListener {
public:
	//! constructor
	explicit WiiMoteDriver(const std::string& name)
		: DeviceDriver(autoRegisterWiiMoteDriver,name), MotionHook(), data(), wiim(NULL)
	{
		if(driver)
			cerr << "Warning: Createing another WiiMote Driver is not supported!" << endl;

		driver = this;
		this->init();
	}
	
	//! destructor
	virtual ~WiiMoteDriver() {
		close();
		// don't need to delete properties, smart pointer
	}

	virtual std::string getClassName() const { return autoRegisterWiiMoteDriver; }

	virtual MotionHook* getMotionSink() { return dynamic_cast<MotionHook*>(this); }
	virtual void getSensorSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Sensors"] = &data;
	}

	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
	}
	
	virtual void motionStarting() {}
	virtual void motionUpdated(const std::vector<size_t>& changedIndices, const float outputs[][NumOutputs]);
	virtual void motionStopping() {}
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl) { };

	virtual void mesg_callback(int id, int mesg_count, union wiimote_mesg* mesg[]);

	DataCache data;

protected:

	virtual int init();

	//! connect to qwerk
	virtual void connect() { };

	//! closes the current proxies (reuses the communicator instance though)
	virtual void close() {
		// disconnect the wiimote
		wiimote_disconnect(wiim);
		wiim=NULL;
	}
	
	inline int calcLEDValue(unsigned int i,float x) {
		/*
		if(x<=0.0) {
			ledActivation[i]*=.9; //decay activation... resets to keeps LEDs in sync, looks a little better
			return 0;
		} else if(x>=1.0) {
			return 1;
		} else {
			x*=x; // squared to "gamma correct" - we can see a single pulse better than a single flicker - after image and all that
			ledActivation[i]+=x;
			if(ledActivation[i]>=1.0) {
				ledActivation[i]-=1.0;
				return 0;
			} else {
				return 1;
			}
		}
		*/
		return 1;
	}

	wiimote_t *wiim;
private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterWiiMoteDriver;

	WiiMoteDriver(const WiiMoteDriver&); //!< do not call
	WiiMoteDriver& operator=(const WiiMoteDriver&); //!< do not call
};

/*! @file
 * @brief 
 * @author Benson Tsai
 */

#endif

