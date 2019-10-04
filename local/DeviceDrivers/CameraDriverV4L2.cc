#ifdef __linux__

#include "CameraDriverV4L2.h"
#include "Shared/debuget.h"
//#include "Shared/TimeET.h"
#include "Shared/MarkScope.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>

using namespace std; 

const std::string CameraDriverV4L2::autoRegisterCameraDriverV4L2 = DeviceDriver::getRegistry().registerType<CameraDriverV4L2>("Camera");

bool CameraDriverV4L2::advance()
{
	//cout << "CDV4L2: getData called with timestamp " << timestamp << "\n";

	int ret;

	if (camfd < 0)
		return false;

	//std::cout << get_time() << " start" << std::endl;
	MarkScope l(lock);

	// enable streaming if it hasn't been yet
	if (!streaming) {
		if (video_enable()) {
			return false;
		}
	}

	//std::cout << get_time() << " getting sample" << std::endl;
	// clear out single buffer
	ret = dequeue_buffer();
	if (ret < 0 && errno!= EAGAIN) {
		return false;
	}
	
	timestamp = get_time();
	//std::cout << timestamp << " have sample " << ret << std::endl;
	

	if (downsample)
		downsample_yuyv();
	else
		upsample_yuyv();
	
	//std::cout << get_time() << " done" << std::endl;

	++frameCount;
	return true;
}

int CameraDriverV4L2::dequeue_buffer()
{
	int ret;
	
	// dequeue an available buffer
	memset( &v_buf, 0, sizeof (struct v4l2_buffer) );
	v_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v_buf.memory = V4L2_MEMORY_MMAP;
	ret = ioctl( camfd, VIDIOC_DQBUF, &v_buf );
	if (ret) {
		perror("Unable to dequeue buffer");
		return ret;
	}
	
	// print some stuff out for now
	//    cout << "CDV4L2: Buffer " << v_buf.index << " dequeued. Capture time: " << v_buf.timestamp.tv_sec << "s, sequence num "
	//	 << v_buf.sequence << ".\n";
	//    cout << "CDV4L2: Buffer contains " << v_buf.bytesused << " bytes in buffer of length " << v_buf.length << ".\n"; 

	// resize if needed (might move this)
	if (captureBuf.size() != v_buf.length)
		captureBuf.resize(v_buf.length);

	// copy the frame over
	memcpy(&captureBuf[0], v_mem[v_buf.index].start, v_buf.bytesused);
	
	// requeue the buffer
	ret = ioctl( camfd, VIDIOC_QBUF, &v_buf);
	if (ret < 0) {
		perror("Unable to requeue buffer");
		return ret;
	}
	
	return 0;
}

void CameraDriverV4L2::registerSource() {
	open_cam();
	path.addPrimitiveListener(this);
	resolution.addPrimitiveListener(this);
	queryOptions.addPrimitiveListener(this);
	userOptions.addCollectionListener(this);
	for(plist::DictionaryOf< plist::Primitive<int> >::const_iterator it=userOptions.begin(); it!=userOptions.end(); ++it)
		it->second->addPrimitiveListener(this);
}

void CameraDriverV4L2::deregisterSource() {
	userOptions.removeCollectionListener(this);
	for(plist::DictionaryOf< plist::Primitive<int> >::const_iterator it=userOptions.begin(); it!=userOptions.end(); ++it)
		it->second->removePrimitiveListener(this);
	queryOptions.removePrimitiveListener(this);
	resolution.removePrimitiveListener(this);
	path.removePrimitiveListener(this);
	close_cam();
}

void CameraDriverV4L2::doUnfreeze() {
	thread.start();
}

void CameraDriverV4L2::doFreeze() {
	if(thread.isStarted())
		thread.stop().join();
}

void CameraDriverV4L2::threadrun() {
	while(advance())
		Thread::testCurrentCancel();
}


void CameraDriverV4L2::plistValueChanged(const plist::PrimitiveBase& pl)
{
	if (&pl == &path || &pl == &resolution) {
		MarkScope l(lock);
		open_cam();
	}
	else if (&pl == &queryOptions && queryOptions == true) {
		MarkScope l(lock);
		query_options(true);
		queryOptions = false;
	}
	else {
		// first find name in userOptions
		std::string name;
		for(plist::DictionaryOf< plist::Primitive<int> >::const_iterator it=userOptions.begin(); it!=userOptions.end(); ++it) {
			if(&pl == it->second) {
				name = it->first;
				break;
			}
		}
		if(name.size()==0) {
			std::cerr << "Could not find name for modified value" << std::endl;
			return;
		}

		// We verified a proper name was set, to apply reopen camera.
		// we could call setControl directly, but this seems to block
		// for random and significant amounts of time if video capture is active
		MarkScope l(lock);
		close_cam();
		open_cam(); // we only have listeners if camera should be active
	}
}

void CameraDriverV4L2::plistCollectionEntryAdded(plist::Collection& /*col*/, plist::ObjectBase& obj) {
	plist::PrimitiveBase& prim = dynamic_cast<plist::PrimitiveBase&>(obj);
	prim.addPrimitiveListener(this);
	plistValueChanged(prim);
}
void CameraDriverV4L2::plistCollectionEntryRemoved(plist::Collection& /*col*/, plist::ObjectBase& obj) {
	plist::PrimitiveBase& prim = dynamic_cast<plist::PrimitiveBase&>(obj);
	prim.removePrimitiveListener(this);
}
void CameraDriverV4L2::plistCollectionEntriesChanged(plist::Collection& /*col*/) {
	// reapplies all settings, just close and open camera
	MarkScope l(lock);
	close_cam();
	open_cam(); // we only have this listener if camera should be active
}


void CameraDriverV4L2::close_cam()
{
	if(camfd >= 0) {
		if (v_mem.size() > 0) {
			// unmap all buffers up to this point
			for (unsigned int i = 0; i < v_mem.size(); i++) {
				munmap( v_mem[i].start, v_mem[i].length );
			}
			v_mem.resize(0);
		}

		if (streaming)
			video_disable();

		::close(camfd);
		camfd = -1;
	}
}

void CameraDriverV4L2::open_cam()
{
	int ret;

	close_cam();
	if (path.size() == 0)
		return;
	
	// open the camera
	if ((camfd = ::open(path.c_str(), O_RDWR, 0)) == -1) {
		perror ("ERROR opening V4L interface \n");
		return;
	}

	// check capabilities
	memset( &v_cap, 0, sizeof (struct v4l2_capability) );
	ret = ioctl( camfd, VIDIOC_QUERYCAP, &v_cap);
	if (ret < 0) {
		perror("Unable to query capabilities");
		close_cam();
		return;
	}

	if ((v_cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
		cout << "Error opening device " << path << ": video capture not supported.\n";
		close_cam();
		return;
	}

	if (!(v_cap.capabilities & V4L2_CAP_STREAMING)) {
		cout << path << " does not support streaming i/o\n";
		close_cam();
		return;
	}

	// setup the format
	if (select_format()) {
		cout << "Format selection failed.\n";
		close_cam();
		return;
	}

	// setup the buffers for capturing
	struct v4l2_requestbuffers v_rb;
	memset( &v_rb, 0, sizeof (struct v4l2_requestbuffers) );
	v_rb.count = NUM_BUFFERS;
	v_rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v_rb.memory = V4L2_MEMORY_MMAP;
	
	ret = ioctl( camfd, VIDIOC_REQBUFS, &v_rb );
	if (ret < 0) {
		perror("Unable to allocate buffers");
		close_cam();
		return;
	}
	
	// how many buffers did we actually get
	if(v_rb.count!=NUM_BUFFERS)
		cout << "Allocated " << v_rb.count << " buffers for capturing.\n";
	if (v_rb.count < MIN_BUFFERS) {
		cout << "Not enough buffers allocated for capturing.\n";
		close_cam();
		return;
	}
	
	// we need to mmap memory for each buffer
	v_mem.resize( v_rb.count );

	for (unsigned int i = 0; i < v_rb.count; i++) {
		memset( &v_buf, 0, sizeof (struct v4l2_buffer) );
		v_buf.index = i;
		v_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v_buf.memory = V4L2_MEMORY_MMAP;
		ret = ioctl( camfd, VIDIOC_QUERYBUF, &v_buf );
		if (ret < 0) {
			perror("Unable to query buffer");
			goto cleanup;
		}
		v_mem[i].length = v_buf.length;
		v_mem[i].start = mmap (NULL, v_buf.length,
													 PROT_READ | PROT_WRITE, MAP_SHARED,
													 camfd, v_buf.m.offset);
		if (v_mem[i].start == MAP_FAILED) {
			perror("Unable to map buffer");
			goto cleanup;
		}

		continue;

	cleanup:
		// unmap all buffers up to this point
		for (unsigned int j = 0; j < i; j++) {
			munmap( v_mem[j].start, v_mem[j].length );
		}
		v_mem.resize(0);

		// close up everything else
		close_cam();
		return;
	}
	
	for (int i = 0; i < NUM_BUFFERS; ++i) {
		memset (&v_buf, 0, sizeof (struct v4l2_buffer));
		v_buf.index = i;
		v_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v_buf.memory = V4L2_MEMORY_MMAP;
		ret = ioctl (camfd, VIDIOC_QBUF, &v_buf);
		if (ret < 0) {
			perror("Unable to queue buffer");
			close_cam();
			return;
		}
	}

	// query the options for this camera (brightness, etc.)
	query_options(false);

	// set options
	for(plist::DictionaryOf< plist::Primitive<int> >::const_iterator it=userOptions.begin(); it!=userOptions.end(); ++it) {
		setControl(it->first,*it->second,false);
	}

	return;
}

bool CameraDriverV4L2::setControl(const std::string& name, int value, bool verbose/*=false*/) {
	map<std::string, struct v4l2_queryctrl>::const_iterator opt = options.find(name);
	if(opt==options.end()) {
		cerr << instanceName << " does not recognize setting " << name << std::endl;
		return false;
	}
		
	MarkScope l(lock);

	if(verbose)
		cout << "Setting \"" << (const char *)(opt->second.name) << "\" to " << value << "." << endl;
		
	struct v4l2_control control;
	control.id = opt->second.id;
	control.value = value;
	int error = ioctl(camfd, VIDIOC_S_CTRL, &control);
	if (error!=0)
		cerr << "Unable to set control \"" << (const char *)(opt->second.name) << "\"." << endl;

	return (error==0);
}

int CameraDriverV4L2::video_enable()
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;
	
	ret = ioctl (camfd, VIDIOC_STREAMON, &type);
	if (ret < 0) {
		perror("Unable to %s capture: %d.");
		return ret;
	}

	streaming = true;
	
	return 0;
}

int CameraDriverV4L2::video_disable()
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;
	
	ret = ioctl( camfd, VIDIOC_STREAMOFF, &type);
	if (ret < 0) {
		perror("Unable to stop capture:");
		return ret;
	}
	
	streaming = false;

	return 0;
}

string CameraDriverV4L2::v4l2_fourcc_inv(__u32 f)
{
	char tmp[5];
	tmp[4] = 0;
	*((int *)tmp) = f;

	return string(tmp);
}

int CameraDriverV4L2::add_control(struct v4l2_queryctrl & queryctrl, bool v)
{
	struct v4l2_querymenu querymenu;
	struct v4l2_control   control_s;

	control_s.id=queryctrl.id;
	ioctl(camfd, VIDIOC_G_CTRL, &control_s);
	
	stringstream ss;
	ss << "Type: " << queryctrl.type << ", values: [" << queryctrl.minimum << " - " << queryctrl.maximum << "], stepsize "
		 << queryctrl.step << ", default " << queryctrl.default_value << ".";
	
	
	// add control to plist dictionary
	options[(char*)queryctrl.name] = queryctrl;
	
	if (v) {
		cout << "   " << (char*)(queryctrl.name) << ":     " << ss.str() << endl;

		if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
			printf ("    Menu items: ");
			memset (&querymenu, 0, sizeof (querymenu));
			querymenu.id = queryctrl.id;
			for (querymenu.index = queryctrl.minimum;
					 (decltype(queryctrl.maximum))querymenu.index <= queryctrl.maximum;
					 querymenu.index++) {
				if (0 == ioctl (camfd, VIDIOC_QUERYMENU, &querymenu)) {
					printf ("\n    index:%d name:%s", querymenu.index, querymenu.name);
				} else {
					printf ("error getting control menu");
					break;
				}
			}
			printf("\n");
		}
	}

	return 0;
}

int CameraDriverV4L2::query_options(bool v)
{
	if (v)
		cout << "CAMERA DRIVER: Querying Camera Options" << endl;

	if (camfd < 0) {
		if (v)
			cout << "CAMERA DRIVER: No camera connected." << endl;
		return -1;
	}

	// remove old options from the plist dictionary
	options.clear();
	
	if (v)
		cout << endl << "CAMERA DRIVER: Available Frame Formats:" << endl;

	// enumerate frame formats and sizes
	int ret;
	int f = 0;
	struct v4l2_fmtdesc v_fmtdesc;
	while (true) {
		memset( &v_fmtdesc, 0, sizeof(struct v4l2_fmtdesc) );
		v_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v_fmtdesc.index = f;
	
		ret = ioctl( camfd, VIDIOC_ENUM_FMT, &v_fmtdesc );
		if (ret)
			break;
	
		if (v) {
			cout << "  " << v_fmtdesc.description << ":" << endl;
			cout << "    " << "Frame size enumeration disabled." << endl;
		}

		/*
			struct v4l2_frmsizeenum v_frmsize;
			int s = 0;
			while (true) {
			memset( &v_frmsize, 0, sizeof(struct v4l2_frmsizeenum) );
			v_frmsize.pixel_format = v_fmtdesc.pixelformat;
			v_frmsize.index = s;
		
			ret = ioctl( camfd, VIDIOC_ENUM_FRAMESIZES, &v_frmsize );
			if (ret)
			break;
		
			if (v_frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
			cout << "  " << v_frmsize.discrete.width << "x" << v_frmsize.discrete.height << "\n";
			else
			cout << "*****\n";
		
			s++;
			}*/
		f++;
	}

	//enumerate controls
	struct v4l2_queryctrl queryctrl;
	struct v4l2_input*    getinput;
	
	//Name of the device
	getinput=(struct v4l2_input *) calloc(1, sizeof(struct v4l2_input));
	memset(getinput, 0, sizeof(struct v4l2_input));
	getinput->index=0;
	ioctl(camfd, VIDIOC_ENUMINPUT, getinput);
	if (v) {
		cout << endl << "CAMERA DRIVER: Available controls (Type 1=Integer 2=Boolean 3=Menu 4=Button):" << endl;
	}

	// advanced enumeration code from libwebcam, written by quickcamteam:
	// http://www.quickcamteam.net/software/libwebcam
	queryctrl.id = 0 | V4L2_CTRL_FLAG_NEXT_CTRL;
	if (ioctl(camfd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
		if (v) {
			cout << "CAMERA DRIVER: Using advanced method of enumerating controls." << endl;
		}
		// The driver supports the V4L2_CTRL_FLAG_NEXT_CTRL flag, so go ahead with
		// the advanced enumeration way.

		int r;
		queryctrl.id = 0;
		int current_ctrl = queryctrl.id;
		queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		// Loop as long as ioctl does not return EINVAL
		while ((r = ioctl(camfd, VIDIOC_QUERYCTRL, &queryctrl)), r ? errno != EINVAL : 1) {

			// Prevent infinite loop for buggy NEXT_CTRL implementations
			if(r && (int)queryctrl.id <= current_ctrl) {
				// If there was an error but the driver failed to provide us with the ID
				// of the next control, we have to manually increase the control ID,
				// otherwise we risk getting stuck querying the erroneous control.
				current_ctrl++;
				goto next_control;
			}
			current_ctrl = queryctrl.id;

			// Skip failed and disabled controls
			if(r || queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				goto next_control;

			add_control(queryctrl, v);

		next_control:
			queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		}
	} else {
		if (v) {
			cout << "CAMERA DRIVER: Using non-advanced method of enumerating controls." << endl;
		}
		//predefined controls
		memset (&queryctrl, 0, sizeof (queryctrl));
		for (int ctrl_id = V4L2_CID_BASE; ctrl_id < V4L2_CID_LASTP1; ctrl_id++) {
			queryctrl.id = ctrl_id;
			if (0 == ioctl (camfd, VIDIOC_QUERYCTRL, &queryctrl)) {
				if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
					continue;
				
				add_control(queryctrl, v);
				
			} else {
				if (errno == EINVAL)
					continue;
				perror ("error getting base controls");
			}
		}
		
		//driver specific controls
		for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;
				 queryctrl.id++) {
			if (0 == ioctl (camfd, VIDIOC_QUERYCTRL, &queryctrl)) {
				if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
					continue;
				
				add_control(queryctrl, v);
			} else {
				if (errno == EINVAL)
					break;
				perror ("error getting private base controls");
			}
		}
	}
	
	return 0;
}

int CameraDriverV4L2::select_format()
{
	int ret;

	memset(&v_fmt, 0, sizeof (struct v4l2_format));
	v_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v_fmt.fmt.pix.field = V4L2_FIELD_ANY;

	// get the current format
	ret = ioctl(camfd, VIDIOC_G_FMT, &v_fmt);
	if (ret) {
		perror("Unable to get image format");
		return ret;
	}

	// set the mode:
	unsigned int w = 0, h = 0;
	char c;
	stringstream ss(resolution);
	ss >> w >> c >> h;

	v_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	v_fmt.fmt.pix.width = w*2;
	v_fmt.fmt.pix.height = h*2;
	
	cout << "Trying " << v_fmt.fmt.pix.width << "x" << v_fmt.fmt.pix.height << "\n";

	// unable to set to double size, try for upsampled resolution
	if (ioctl(camfd, VIDIOC_S_FMT, &v_fmt) ||
			(v_fmt.fmt.pix.width != w*2) ||
			(v_fmt.fmt.pix.height != h*2)) {

		v_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		v_fmt.fmt.pix.width = w;
		v_fmt.fmt.pix.height = h;

		cout << "Trying " << v_fmt.fmt.pix.width << "x" << v_fmt.fmt.pix.height << "\n";

		ret = ioctl(camfd, VIDIOC_S_FMT, &v_fmt);
		if (ret < 0) {
			perror("Unable to set format.");
			return ret;
		}

		if ((v_fmt.fmt.pix.width != w) ||
				(v_fmt.fmt.pix.height != h)) {
			cout << "Format asked unavailable.\n";
		}

		downsample = false;
		cout << "Image will be upsampled to " << v_fmt.fmt.pix.width << "x"
				 << v_fmt.fmt.pix.height << ".\n";
	}
	else {
		downsample = true;
		cout << "Image will be downsampled to " << v_fmt.fmt.pix.width/2 << "x"
				 << v_fmt.fmt.pix.height/2 << ".\n";
	}

	return 0;
}

void CameraDriverV4L2::downsample_yuyv()
{
	unsigned int layer = 0, components = 3;
	unsigned int width = v_fmt.fmt.pix.width / 2;
	unsigned int height = v_fmt.fmt.pix.height / 2;
	
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion* region = getUnusedRegion(reqSize,0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (buf) ImageHeader(0, layer, width, height, components, frameCount, timestamp, nextName());
	
	unsigned char * src = &captureBuf[0];
	const unsigned int srcStride=v_fmt.fmt.pix.width * 2;
	unsigned char * dst = buf + sizeof(ImageHeader);
	unsigned char * const dstEnd = dst + width*height*components;

	// convert!
	while (dst != dstEnd) {
		unsigned char * const rowEnd = dst + width * components;
		while (dst != rowEnd) {
			int y;
			int u, v;
		
			y=*src;
			y+=*(src+srcStride);
			++src;
		
			u=*src;
			u+=*(src+srcStride);
			++src;
		
			y+=*src;
			y+=*(src+srcStride);
			++src;
		
			v=*src;
			v+=*(src+srcStride);
			++src;
						
			*dst++ = y/4;
			*dst++ = u/2;
			*dst++ = v/2;
		}
		src+=srcStride;
	}
	ASSERTRET(dst-buf==reqSize,"CameraDriverV4L2 bad downsample_yuyv " << reqSize << " vs " << (dst-buf));
	setImage(region);
}

void CameraDriverV4L2::upsample_yuyv()
{
	unsigned int layer = 0, components = 3;
	unsigned int width = v_fmt.fmt.pix.width;
	unsigned int height = v_fmt.fmt.pix.height;
	
	ssize_t reqSize = sizeof(ImageHeader) + width * height * components;
	RCRegion* region = getUnusedRegion(reqSize,0);
	unsigned char * buf = reinterpret_cast<unsigned char*>(region->Base());
	new (buf) ImageHeader(0, layer, width, height, components, frameCount, timestamp, nextName());
	
	// setup pointers
	unsigned char * src = &captureBuf[0];
	unsigned char * dst = buf + sizeof(ImageHeader);
	unsigned char * const dstEnd = dst + width*height*components;

	// convert!
	while (dst != dstEnd) {
		unsigned char * const rowEnd = dst + width * components;
		while (dst != rowEnd) {
			int y1, y2;
			int u, v;
		
			y1=*src;
			++src;
		
			u=*src;
			++src;
		
			y2=*src;
			++src;
		
			v=*src;
			++src;
						
			*dst++ = y1;
			*dst++ = u;
			*dst++ = v;
			*dst++ = y2;
			*dst++ = u;
			*dst++ = v;
		}
	}
	ASSERTRET(dst-buf==reqSize,"CameraDriverV4L2 bad upsample_yuyv " << reqSize << " vs " << (dst-buf));
	setImage(region);
}

/*! @file
 * @brief 
 * @author Alex Grubb (agrubb1) (V4L2 code)
 * @author Ethan Tira-Thompson (ejt) (Framework)
 */

#endif
