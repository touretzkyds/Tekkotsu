//-*-c++-*-
#ifndef INCLUDED_CameraDriverV4L2_h_
#define INCLUDED_CameraDriverV4L2_h_

#include "local/DeviceDriver.h"
#include "local/DataSource.h"
#include "Shared/get_time.h"
#include "IPC/CallbackThread.h"

#include <linux/types.h> 
#include <linux/videodev2.h>

#include <map>
#include <vector>

#define NUM_BUFFERS 4
#define MIN_BUFFERS 1

//! description of CameraDriverV4L2
class CameraDriverV4L2 : public virtual DeviceDriver, public virtual plist::PrimitiveListener, public virtual plist::CollectionListener, public DataSource
{
public:
	explicit CameraDriverV4L2(const std::string& name)
		: DeviceDriver(autoRegisterCameraDriverV4L2,name), DataSource(),
		path("/dev/video0"), resolution("640x480"),queryOptions(false),userOptions(),options(),
		thread(&CameraDriverV4L2::threadrun,*this),
		camfd(-1), v_cap(), v_fmt(), v_buf(), v_mem(0),
		streaming(false), downsample(true), frameCount(0), timestamp(0),
		captureBuf(), lock()
	{
		addEntry("Path",path,"Path to the video device, e.g. /dev/video0");
		addEntry("Resolution",resolution,"Image resolution of the final output image, e.g. 640x480."
			"If small enough, image will be captured at twice this resolution and downsampled,"
			"otherwise image will be captured at this resolution and upsampled.");
		addEntry("QueryOptions",queryOptions,"Query the options available for this camera.");
		addEntry("Options",userOptions,"Add entries to this collection to control camera-specific settings.\nSet QueryOptions to true to see list of available options.");
		userOptions.setLoadSavePolicy(SYNC,SYNC);
	}
	
	~CameraDriverV4L2() { deregisterSource(); }
	
	virtual std::string getClassName() const { return autoRegisterCameraDriverV4L2; }
	
	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear(); sources.insert(std::make_pair("Camera",this));
	}
	
	virtual unsigned int nextTimestamp() { return get_time(); }
	virtual const std::string& nextName() { return instanceName; }
	
	virtual bool advance();
	virtual void registerSource();
	virtual void deregisterSource();
	
	//! watches #path, triggers a close() and re-open() if it changes
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	//! watches #userOptions to sync when settings are added or removed
	virtual void plistCollectionEntryAdded(plist::Collection& col, plist::ObjectBase& obj);
	//! watches #userOptions to sync when settings are added or removed
	virtual void plistCollectionEntryRemoved(plist::Collection& col, plist::ObjectBase& obj);
	//! watches #userOptions to sync when settings are added or removed
	virtual void plistCollectionEntriesChanged(plist::Collection& col);

	plist::Primitive<std::string> path;
	plist::Primitive<std::string> resolution;
	plist::Primitive<bool> queryOptions;
	plist::DictionaryOf< plist::Primitive<int> > userOptions;
	std::map<std::string, struct v4l2_queryctrl> options;
	
protected:
	void close_cam();
	void open_cam();
	bool setControl(const std::string& name, int value, bool verbose=false);

	int video_enable();
	int video_disable();
	int select_format();
	int add_control(struct v4l2_queryctrl & queryctrl, bool verbose);
	int query_options(bool verbose);
	int dequeue_buffer();
	std::string v4l2_fourcc_inv(__u32 f);

	void doFreeze();
	void doUnfreeze();
	
	void downsample_yuyv();
	void upsample_yuyv();
	
	void threadrun();
	CallbackThread thread;

	int camfd;

	struct v4l2_capability v_cap;
	struct v4l2_format v_fmt;
	struct v4l2_buffer v_buf;

	struct MemoryMapping {
		void *start;
		size_t length;
	};
	
	// record keeping for mapped memory
	std::vector<MemoryMapping> v_mem;

	bool streaming;
	bool downsample;

	unsigned int frameCount;
	unsigned int timestamp;
	
	std::vector<unsigned char> captureBuf; //!< the buffer to capture the frame into (unconverted)
	Thread::Lock lock; // buffer/img_size lock so we can't change resolution while reading

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterCameraDriverV4L2;
};

/*! @file
* @brief 
* @author Ethan Tira-Thompson (ejt)
* @author Alex Grubb (agrubb1)
*/

#endif
