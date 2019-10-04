//-*-c++-*-
#ifndef INCLUDED_DataSource_h_
#define INCLUDED_DataSource_h_

//#include "Shared/RobotInfo.h" // only needed for some debugging output below
#include "IPC/Thread.h"
#include "IPC/MessageQueue.h"
#include "Shared/debuget.h"
#include "Shared/get_time.h"
#include "Shared/RobotInfo.h"
#include "Shared/plistPrimitives.h"
#include <string>
#include <iostream>
#include <list>

class RCRegion;
struct SensorState;

//! abstract base class for simulator data sources
/*! Each subclass will implement loading data from some piece of hardware or network protocol.
 *
 *  The general flow of calls is:
 *  - constructor - user creates an instance of a DeviceDriver (which creates the data source)
 *  - registerSource() - the user selects the data source via SharedGlobals::StreamSimConfig::sources
 *  - enteringRealtime() - if the simulator enters real time mode
 *  - leavingRealtime() - if the simulator is paused, set to full-speed mode, or begins to shut down
 *  - deregisterSource() - simulator is shutting down or otherwise deleting associated DeviceDriver
 *  - destructor - deleting associated DeviceDriver
 *
 *  If your data source provides sensor data for current output values, you should call
 *  providingOutput() for those outputs when registerSource() is called, and ignoringOutput()
 *  when you are no longer active (deregisterSource() or destructor is called).
 *  This prevents the Motion process from clobbering your readings with its own feedback.
 */
class DataSource {
public:
	DataSource() : regions(), frozen(false), imageQueue(NULL) {} //!< constructor
	DataSource(const DataSource& ds) : regions(ds.regions), frozen(ds.frozen), imageQueue(ds.imageQueue) {} //!< copy constructor, just in case your subclass wants it
	DataSource& operator=(const DataSource&) { return *this; } //!< assignment operator, just in case your subclass wants it
	virtual ~DataSource(); //!< destructor

	//! Returns the simulator time of the next data segment.
	/*! Should be in the future if nothing new since last data segment, otherwise should be the 
	 *  timestamp of the most recent data segment (older segments are skipped), return -1U if there is no more data */
	virtual unsigned int nextTimestamp()=0;
	
	//! Returns a descriptive name of the next data segment for user feedback (e.g. a filename for logged data).
	/*! Just use your class name if you don't have a useful name for individual samples. */
	virtual const std::string& nextName()=0;
	
	//! Called when the simulator is stepping while paused or advancing a frozen data source, return true if successful, or false if no more data is available
	virtual bool advance()=0;
	
	//! Called by simulator if the user pauses a data source; calls doFreeze() or doUnfreeze() as appropriate
	/*! You probably don't want to override this function -- that's what doFreeze() doUnfreeze are for! */
	virtual void setFrozen(bool fr) { if(fr==frozen) return; frozen=fr; if(getTimeScale()>0) { if(frozen) doFreeze(); else doUnfreeze(); } }
	virtual bool getFrozen() const { return frozen; } //!< returns #frozen status
	
	//! if called, indicates a request to restart/reinitialize the data source
	/*! For example, a FileSystemDataSource would go back to the beginning of its list,
	 *  and a network-based source would close and reopen the connection */
	virtual void reset() {}
	
	//! User hook, called when the data source should claim which outputs it provides feedback (providingOuput())
	/*! Does not indicate the data source should start sending updates yet — wait for enteringRealtime() or advance() to be called */
	virtual void registerSource() {}
	
	//! User hook, called when the data source should release its claim on outputs with feedback (ignoringOuput()).
	/*! It would be wise to call this from your destructor as well. */
	virtual void deregisterSource() {}
	
	//! User hook, called when the controller is going to be running in realtime mode, which is probably the normal mode you'd expect.
	/*! You might be in realtime mode, but a debugger breakpoint will still pause things, or thread scheduling could hiccup, so try to be robust.\n
	 *  The argument is a reference to SharedGlobals::timeScale, so the data source can subscribe to changes in
	 *  simulation speed if it can use that information.  (We avoid direct dependency on the tekkotsu simulator
	 *  so this code can be reused for other tools too.) */
	virtual void enteringRealtime(const plist::Primitive<double>& /*simTimeScale*/) { if(!frozen) doUnfreeze(); }
	
	//! User hook, called when leaving realtime mode, which means you have no idea when motionCheck() is going to be called in terms of wall-clock time.
	/*! Argument set to true if entering full speed mode, which indicates everything should run
	 *  at full native "frame rate", and may indicate more data will be processed than normal, CPU speed permitting.
	 *  However, if false, almost certainly indicates updates will be sparse, trigger by user 'step' commands.
	 *  May be called multiple times if changing between full-speed mode and paused
	 *
	 *  A non-realtime mode might be triggered if the user wants to pause the simulator/controller to step through something...
	 *  No guarantees though!  The debugger might catch a breakpoint and stop things, and this won't be called! */
	virtual void leavingRealtime(bool /*isFullSpeed*/) { if(!frozen) doFreeze();  }
	
	//! Called by simulator during initialization to tell DataSources where the array of sensor values are stored (see #sensorState)
	/*! This would need to point into a shared memory region if using multi-process model, hence we can't just
	 *  use process-local static allocation. */
	static void setSensorState(SensorState* senState) { sensorState=senState; }
	
	//! Called by simulator during initialization to tell the DataSource where it should send image data (if this is selected as an active image data source).
	/*! Each image data source (if we get around to supporting such concurrent sources) will have a separate message queue */
	virtual void setImageQueue(MessageQueueBase* mq) { imageQueue=mq; }
	
	//! will be called by initialization code prior to first getData() if client code is going to block on getting the first sensor reading
	static void setNeedsSensor(bool waiting) { requiresFirstSensor=waiting; }
	
	//! will be called by initialization code to provide a pointer to the sensor synchronization framerate #sensorFramerate
	static void setSensorFramerate(const plist::Primitive<float>* senFR) { sensorFramerate = senFR; }
	
	//! This structure should be written into the beginning of each image buffer sent via setImage().
	/*! Using "placement new" avoids doing a memcpy: <code>new (buffer) ImageHeader(...);</code>\n
	 *  If you are storing a visual image to be processed by the pipeline (vs. a laser rangefinder sweep or such), then
	 *  values should be interleaved YUV samples. */
	struct ImageHeader {
		//! full constructor, see corresponding members for documentation on arguments
		ImageHeader(unsigned int sourceID_, int layer_, unsigned int width_, unsigned int height_, unsigned int components_, unsigned int frameNumber_, unsigned int timestamp_, const std::string& name_)
		: sourceID(sourceID_), layer(layer_), width(width_), height(height_), components(components_), frameNumber(frameNumber_), timestamp(timestamp_) { strncpy(name,name_.c_str(),MAX_NAME_LEN); }
		
		unsigned int sourceID; //!< In order to support multiple cameras, this tracks how to identify the cameras within Tekkotsu.  You should allow users to configure the value you store here.
		int layer; //!< the resolution layer for this image, if 0 then automatically chooses the closest in size based on expected values in RobotInfo namespace.  Otherwise should be presented as a configuration item.
		unsigned int width; //!< pixels per row of image
		unsigned int height; //!< pixels per column of image
		unsigned int components; //!< number of color channels, vision pipeline expects 3 for YUV images.
		
		unsigned int frameNumber; //!< serial number per image sent, provides user notification for dropped images if the SharedGlobals::StreamSimConfig::verbose flag is set
		unsigned int timestamp; //!< timestamp image was recorded
		static const unsigned int MAX_NAME_LEN=64; //!< maximum length of #name
		char name[MAX_NAME_LEN]; //!< a user identifiable name for the image... see DataSource::nextName()
	};
	
	//! returns the pending output value, which may not have been processed by the framework yet
	static float getOutputValue(unsigned int i);
	
	//! returns the pending sensor value, which may not have been processed by the framework yet
	static float getSensorValue(unsigned int i);
	
	//! returns the pending button value, which may not have been processed by the framework yet
	static float getButtonValue(unsigned int i);
	
	//! returns the pending duty cycle value, which may not have been processed by the framework yet
	static float getPIDDutyValue(unsigned int i);
		
protected:
	//! subclasses should pass this to a MarkScope instance before they begin calling the static setXXXValue functions to prevent concurrent or partial updates
	static Resource& getSensorWriteLock();
	
	//! subclasses should call this if they provide sensor updates which will contain a measurement of the current position of output @a i.
	/* A DataSource should consider itself providing an output if it will be sending some kind of measurement of
	 *  the current value of an output, and has been assigned to a LoadDataThread
	 *  (i.e. setDataSourceThread() has been called and #thread is non-NULL).\n
	 *  This prevents the motion process from clobbering your readings with its own feedback.  */
	static void providingOutput(unsigned int i);
	
	//! subclasses should call this if they provided sensor updates containing the current position of output @a i, but are not going to do so any longer.
	/*! You don't need to call this if you didn't previously call providingOutput(). */
	static void ignoringOutput(unsigned int i);
	
	//! sets the current value for an output, be sure to register with providingOutput() (and later ignoringOutput()) in order to avoid being clobbered by "blind" feedback from the Motion process
	static void setOutputValue(unsigned int i, float v);
	
	//! sets the current value for a sensor
	static void setSensorValue(unsigned int i, float v);
	
	//! sets the current value for a button
	static void setButtonValue(unsigned int i, float v);
	
	//! sets the current duty cycle value for a servo joint, @a i should be relative to PIDJointOffset
	static void setPIDDutyValue(unsigned int i, float v);
	
	//! Returns a counter which is incremented each time the sensor data is copied into the framework
	/*! You could use this to determine if user code has gotten a chance to see a detection flag of some sort before clearing it again */
	static unsigned int getSensorFrameNumber();
	
	//! Sets the current image, for bulk data sources like cameras.
	/*! You should not modify data in @a region until you pass a new @a region here.
	 *  However even then, the previous region may still be in processing.  Check
	 *  region reference counts to determine when an old region is available for
	 *  recycling, or otherwise dereference them after calling this so they will
	 *  be deallocated after use.
	 *
	 *  Image data should be in 8-bit per channel interleaved format and the data
	 *  should be prepended by an ImageHeader structure so the receiver knows how to
	 *  interpret the data.
	 *
	 *  Recommended to just use getUnusedRegion() to handle region recycling for you. */
	void setImage(RCRegion* region) {
		ASSERTRET(imageQueue!=NULL,"DataSource::setImage called without imageQueue");
		imageQueue->sendMessage(region);
	}

	//! sets the current image, for bulk data sources like cameras
	/*! This version copies the data into an RCRegion and calls setImage(RCRegion*).  For efficiency with
	 *  large blocks of data, if possible you should use getUnusedRegion and write your results directly
	 *   there to call setImage(RCRegion*) instead and save a copy if possible.
	 *
	 *  Image @a data should be in 8-bit per channel interleaved format. */
	void setImage(const ImageHeader& header, const void * data);

	//! Finds a one-reference entry in #regions with capacity at least @a minSize, creating a new one of size @a minSize + @a padding if necessary.
	/*! If the first one-reference entry is too small, it will be freed and a new one created instead. */
	virtual RCRegion* getUnusedRegion(size_t minSize, size_t padding);
	
	std::list<RCRegion*> regions; //!< for efficiency, reuse old buffers via getUnusedRegion() -- oldest at front, most recently used at back
	
	virtual void doFreeze() {} //!< user hook for when #frozen is set to true; advance() will be called by simulator at user discretion.
	virtual void doUnfreeze() {} //!< user hook for when #frozen is set to false; if enteringRealtime() has been called then you should resume sending data.
	bool frozen;	//!< indicates that data is going to be requested "sparsely"; advance() will be called by simulator at user discretion
	
	//! if true, indicates that client code is going to wait for sensor readings returned by getData before executing
	static bool requiresFirstSensor;
	
	//! Assigned by setSensorFramerate() during initialization, points to the synchronization framerate for sensors.
	/*! It is pointless to exceed this framerate for sensor updates, so if you have control over the polling rate, set it to this value. */
	static const plist::Primitive<float>* sensorFramerate;

private:
	static SensorState * sensorState;
	
	//! assigned by setImageQueue(), calls to setImage() will send messages into this queue
	MessageQueueBase* imageQueue;
};


//! A global instance of this structure is registered via DataSource::setSensorState() during launch, so that DataSources know where to put their results
/*! Subclasses of DataSource don't access this structure directly, they should call the static DataSource functions to write its fields.
 *  Image based data sources like cameras will send raw buffers of data via DataSource::setImage, ignoring this structure. */
struct SensorState : public Resource {
	//! constructor
	SensorState();
	
	//! returns true if any of #providedOutputs is greater than zero
	bool hasProvidedOutput() const { for(unsigned int i=0; i<NumOutputs; i++) if(providedOutputs[i]>0) return true; return false; }
	//! returns true if any of #providedOutputs is zero
	bool hasUnprovidedOutput() const { for(unsigned int i=0; i<NumOutputs; i++) if(providedOutputs[i]==0) return true; return false; }
	
	//! Counts the number of sensor data sources which are providing readings for each output
	/*! This isn't a configuration setting per se, but needed so motion process can tell if it
	 *  should provide feedback for each output.  If an output doesn't have any sensor feedback
	 *  (or #override is true), then motion should provide feedback.  If more than one
	 *  sensor is providing the same output, that could be a problem, but not dealt with here.
	 *
	 *  The simulator's initialization routines will pass this to DataSource::setOutputTracker(). */
	unsigned int providedOutputs[NumOutputs];
	
	float outputs[NumOutputs];     //!< maps to WorldState::outputs, assign via DataSource::setOutputValue
	float buttons[NumButtons];     //!< maps to WorldState::buttons, assign via DataSource::setButtonValue
	float sensors[NumSensors];     //!< maps to WorldState::sensors, assign via DataSource::setSensorValue
	float pids[NumPIDJoints][3];   //!< maps to WorldState::pids, only assigned via MotionExecThread
	float pidduties[NumPIDJoints]; //!< maps to WorldState::pidduties, assign via DataSource::setPIDDutyValue
	
	unsigned int timestamp; //!< simulator time of last update to one of the value arrays
	
	//! Serial number for the update, incremented each time Simulator sends an update notification.
	/*! This is not incremented for each DataSource modification because there might be multiple data sources,
	 *  or a data source might make multiple partial updates within each Simulator frame. */
	unsigned int frameNumber;
	
	//! this flag should be set when any of the fields are updated, cleared when releaseResource is called (when timestamp is set instead)
	bool dirty;
	
	//! This may be set to point to the Motion.OverrideSensors configuration setting, causing sensor updates for outputs to be ignored (forcing open loop)
	/*! This is useful if loading sensor data from log, where we want to compute new output positions in simulation, but using the "pure" sensors for input */
	plist::Primitive<bool>* motionOverride;
	
	// ! signals when a sensor field is updated so things like full speed mode and waiting for the first sensor update can work
	/*Thread::Condition updateSignal;*/
	
	//! a function to be called whenever releaseResource() is called and dirty flag is set; called at the end once the resource is immediately available
	void (*resourceSync)();
	
protected:
	virtual void useResource(Data& d) {
		static_cast<Resource&>(lock).useResource(d);
	}
	virtual void releaseResource(Data& d);
	
	//! lock to prevent concurrent access to the structure, but should use the SensorState with MarkScope to set the dirty flag on unlock
	Thread::Lock lock;
	
private:
	SensorState(const SensorState&); //!< don't call
	SensorState& operator=(const SensorState&); //!< don't call
};

inline Resource& DataSource::getSensorWriteLock() { return *sensorState; }
inline void DataSource::setOutputValue(unsigned int i, float v) {
	if(sensorState->motionOverride!=NULL && *sensorState->motionOverride)
		return;
	sensorState->outputs[i]=v;
	sensorState->dirty=true;
}
inline float DataSource::getOutputValue(unsigned int i) { return sensorState->outputs[i]; }

inline void DataSource::setSensorValue(unsigned int i, float v) { sensorState->sensors[i]=v; sensorState->dirty=true; }
inline float DataSource::getSensorValue(unsigned int i) { return sensorState->sensors[i]; }

inline void DataSource::setButtonValue(unsigned int i, float v) { sensorState->buttons[i]=v; sensorState->dirty=true; }
inline float DataSource::getButtonValue(unsigned int i) { return sensorState->buttons[i]; }

inline void DataSource::setPIDDutyValue(unsigned int i, float v) { sensorState->pidduties[i]=v; sensorState->dirty=true; }
inline float DataSource::getPIDDutyValue(unsigned int i) { return sensorState->pidduties[i]; }

inline unsigned int DataSource::getSensorFrameNumber() { return sensorState->frameNumber; }

/*! @file
 * @brief Defines DataSource, an abstract base class for simulator data sources
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
