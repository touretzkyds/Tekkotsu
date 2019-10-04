#if 0
#ifdef __linux__

#include "CameraDriverV4L1.h"
#include "Shared/debuget.h"
//#include "Shared/TimeET.h"
#include "Shared/MarkScope.h"

#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std; 

const std::string CameraDriverV4L1::autoRegisterCameraDriverV4L1 = DeviceDriver::getRegistry().registerType<CameraDriverV4L1>("CameraV4L1");
const size_t CameraDriverV4L1::HEADER_SIZE = LoadSave::getSerializedSize<unsigned int>()*4;

unsigned int CameraDriverV4L1::getData(const char *& payload, unsigned int& payloadSize, unsigned int& timestamp, std::string& name) {
	//cout << '.' << flush;
	payload=NULL;
	payloadSize=0;
	if(camfd<0)
		return frameCount;

	curBuf.resize(getBufferSize());
	
	// reset non-blocking in case thread was canceled while we were
	// were in a blocking call...
	if(blocking) {
		if( fcntl(camfd,F_SETFL,O_NONBLOCK) != 0) {
			perror("ioctl set non-blocking mode");
			blocking = fcntl(camfd,F_GETFL) & O_NONBLOCK;
		} else
			blocking=false;
	}

	unsigned int t=get_time();
	unsigned int dropped=0;
	int nbytes;
	size_t lastread;
	while(timestamp>t) {
		//clear any backlog
		while( (nbytes=read(camfd, &curBuf[HEADER_SIZE], curBuf.size()-HEADER_SIZE)) >= 0) {
			lastread=nbytes;
			++dropped;
			//cout << "clear " << TimeET() << endl;
		}
		usleep(static_cast<unsigned int>((timestamp-t)*1000/(getTimeScale()>0?getTimeScale():1.f)));
		t=get_time();
	}
	timestamp = t;
	// we might've been asleep for a while, so need to double check a few things...
	if(camfd<0) // in case we shutdown while asleep!
		return frameCount;

	unsigned int width, height, components=3;
	{
		MarkScope l(lock);
		width=getWidth();
		height=getHeight();
		if(curBuf.size()!=getBufferSize()) { // check in case user changed resolution
			curBuf.resize(getBufferSize());
			dropped=0;
		}
	}

	//get most recent image
	while( (nbytes=read(camfd, &curBuf[HEADER_SIZE], curBuf.size()-HEADER_SIZE)) >= 0) {
		lastread=nbytes;
		++dropped;
		//cout << "early " << TimeET() << endl;
	}
	timestamp = get_time();

	char * buf=reinterpret_cast<char*>(&curBuf[0]);
	unsigned int remain=curBuf.size();
	LoadSave::encodeIncT(*layer,buf,remain);
	LoadSave::encodeIncT(width,buf,remain);
	LoadSave::encodeIncT(height,buf,remain);
	LoadSave::encodeIncT(components,buf,remain);

	if(remain<img_size) {
		// user has changed resolution... skip frame
		//cerr << "Not enough space in buffer for image! " << remain << " vs " << img_size << endl;
		return frameCount;
	}
	if(dropped==0 || lastread!=img_size) {
		ASSERTRETVAL(static_cast<size_t>(remain)>=img_size, "Read more than remains in buffer!", frameCount);
		// disable non-blocking io, we want to wait for the next frame
		if( fcntl(camfd,F_SETFL,0) != 0) {
			perror("ioctl set blocking mode");
			blocking = fcntl(camfd,F_GETFL) & O_NONBLOCK;
		} else
			blocking=true;
		//TimeET bt;
		//cout << "block " << bt << ' ';
		nbytes = read(camfd, buf, remain);
		timestamp = get_time();
		//cout << bt.Age() << endl;
		if( fcntl(camfd,F_SETFL,O_NONBLOCK) != 0) {
			perror("ioctl set non-blocking mode");
			blocking = fcntl(camfd,F_GETFL) & O_NONBLOCK;
		} else
			blocking=false;
		if ( nbytes<0 ) {
			perror("Error reading from camera");
			return frameCount;
		}
		lastread=nbytes;
		++dropped;
	}
	
	MarkScope l(lock);
	// check if we had a short read or if the desired image size changed while we were blocked
  if ( lastread!=img_size // short read
	     || (resolution!=0 && img_size!=width*height*6) // size changed (downsample)
	     || (resolution==0 && img_size!=width*height*3/2) // size changed (upsample)
	   )
	{
		// short read is relatively normal -- always get this on the first frame for
		// some reason, and also might have just changed resolution setting and hasn't
		// taken effect yet.
    //cerr << "CameraDriverV4L1 got " << nbytes << " bytes from camera, expected " << img_size << endl;
		return frameCount;
	}

	if(resolution==0)
		interleave_yuv_up();
	else
		interleave_yuv_down();
  
	if ( autobright )
    auto_bright();
	
	curBuf.swap(lastBuf);
	payload = reinterpret_cast<char*>(&lastBuf[0]);
	payloadSize = lastBuf.size();
	name = nextName();
	return frameCount+=dropped;
}


void CameraDriverV4L1::setDataSourceThread(LoadDataThread* th) {
	DataSource::setDataSourceThread(th);
	if(th != NULL) {
		open_cam();
		path.addPrimitiveListener(this);
		resolution.addPrimitiveListener(this);
	} else {
		resolution.removePrimitiveListener(this);
		path.removePrimitiveListener(this);
		close_cam();
	}
}

void CameraDriverV4L1::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl == &path) {
		MarkScope l(lock);
		open_cam();
	} else if(&pl == &resolution) {
		MarkScope l(lock);
		set_resolution();
	}
}

void CameraDriverV4L1::close_cam() {
	if(camfd >= 0) {
		::close(camfd);
		camfd=-1;
	}
}

void CameraDriverV4L1::open_cam() {
	close_cam();
	if(path.size() == 0)
		return;
	
  camfd = ::open(path.c_str(), O_RDWR|O_NONBLOCK, 0);
  if ( camfd < 0 ) {
    perror("Error on open()");
		cerr << "Could not open camera device '" << path << "'" << endl;
		return;
  }
	blocking=false;
  
	set_resolution();

  if ( ioctl(camfd, VIDIOCGPICT, &vid_pic) == -1 )
    perror ("ioctl (VIDIOCGPICT) brightness");
	vid_pic.brightness=128*256; // initialize to middle brightness
  if ( ioctl(camfd, VIDIOCSPICT, &vid_pic) == -1 )
    perror ("ioctl (VIDIOCSPICT) brightness");

	// try to switch to YUV mode so we don't have to do color conversion
	// this is only trying for 4-2-0 planar mode...
	isYUVMode=false;
  ioctl(camfd, VIDIOCGPICT, &vid_pic);
  vid_pic.depth=16;
  vid_pic.palette=VIDEO_PALETTE_YUV420P;
  if ( ioctl(camfd, VIDIOCSPICT, &vid_pic) == -1 )
    perror ("ioctl (VIDIOCSPICT) palette");
  else {
    img_size = vid_win.width * vid_win.height + vid_win.width*vid_win.height/2;
		isYUVMode=true;
  }
  
  /*{
    struct v4l2_control ctrl;
    ctrl.id=V4L2_CID_AUTO_WHITE_BALANCE;
    ctrl.value=0;
    if ( ioctl(camera->fd,VIDIOC_S_CTRL,&ctrl) == -1 )
      perror ("ioctl (VIDIOC_S_CTRL) disable auto white balance");
  } */ 
}

void CameraDriverV4L1::get_cam_info() {
  ioctl(camfd, VIDIOCGCAP, &vid_caps);
  ioctl(camfd, VIDIOCGWIN, &vid_win);
  ioctl(camfd, VIDIOCGPICT, &vid_pic);
}

void CameraDriverV4L1::set_cam_info() {
  if ( ioctl(camfd, VIDIOCSPICT, &vid_pic) == -1 ) {
    perror ("ioctl (VIDIOCSPICT)");
		cout << "refreshing settings..." << endl;
		if ( ioctl(camfd, VIDIOCGPICT, &vid_pic) == -1 )
			perror ("ioctl (VIDIOCGPICT)");
	}
  if ( ioctl(camfd, VIDIOCSWIN, &vid_win) == -1 ) {
    perror ("ioctl (VIDIOCSWIN)");
		cout << "refreshing settings..." << endl;
		if ( ioctl(camfd, VIDIOCGWIN, &vid_win) == -1 )
			perror ("ioctl (VIDIOCGWIN)");
	}
}

void CameraDriverV4L1::set_resolution() {
  get_cam_info();
  int width = vid_caps.maxwidth >> (resolution==0 ? 0 : resolution-1);
  int height = vid_caps.maxheight >> (resolution==0 ? 0 : resolution-1);
  if ( width < vid_caps.minwidth || height < vid_caps.minheight ) {
		cout << "Warning: requested image " << width<<'x'<<height<<" smaller than camera's minimum size, trying ";
    width = vid_caps.minwidth;
    height = vid_caps.minheight;
		cout << width<<'x'<<height << endl;
  }

  vid_win.width = width;
  vid_win.height = height;
  set_cam_info();
  width = vid_win.width; // restore in case of error or adjustment of requested values
  height = vid_win.height; // (camera might only support certain explicit resolutions)
	
	if(isYUVMode) {
    img_size = width * height + width * height / 2;
	} else {
		img_size = width * height * 3;
	}
		
	cout << "Camera image size is " << getWidth() << 'x' << getHeight() << endl;
}

/*! this is run @em after interleaving, so we go through every 3rd byte of the
 *  entire image to sum the y channel */
void CameraDriverV4L1::auto_bright() {
  size_t total = 0;
	unsigned char *pix = &curBuf[HEADER_SIZE];
	const size_t skip = getWidth() * getHeight() / 719; // actually, only take a subsample
	const size_t npix = getWidth() * getHeight() / skip; // we want a regular sample stride, so will actually be a bit less
	const unsigned char *endp = pix + npix*skip*3;
  for (; pix!=endp; pix+=skip*3) // *3 because of interleaving
    total += *pix;
  int average = total / npix;
  if((average <= 120 || average >= 136)) {
		int bright = vid_pic.brightness + (128 - average)*256/4;
		const typeof(vid_pic.brightness) maxBright = numeric_limits<typeof(vid_pic.brightness)>::max();
		const typeof(vid_pic.brightness) minBright = numeric_limits<typeof(vid_pic.brightness)>::min();
		if(bright>maxBright)
			vid_pic.brightness = maxBright;
		else if(bright<minBright)
			vid_pic.brightness = minBright;
		else
			vid_pic.brightness = bright;
		//cout << "Autobrightness " << average << " " << vid_pic.brightness << endl;
    set_cam_info();
  }
}

void CameraDriverV4L1::interleave_yuv_down() {
	unsigned char * buffer = &curBuf[HEADER_SIZE];
	int width = vid_win.width;
	int height = vid_win.height;
	
  //first downsample y channel
  char tmp[width/2];
  int x,y=0;
  for(x=0; x<width; x+=2) {
    short t=buffer[x+y*width];
    t+=buffer[x+y*width+1];
    t+=buffer[x+y*width+width];
    t+=buffer[x+y*width+width+1];
    tmp[x/2] = t/4;
  }
  for(x=0; x<width/2; x++) {
    buffer[x*3] = tmp[x];
  }  
  for(y=2; y<height; y+=2) {
    for(x=0; x<width; x+=2) {
      short t=buffer[x+y*width];
      t+=buffer[x+y*width+1];
      t+=buffer[x+y*width+width];
      t+=buffer[x+y*width+width+1];
      buffer[y*width/4*3+x/2*3] = t/4;
    }
  }
  // now fill in color components
  unsigned char * c = &buffer[width*height];
	buffer = &curBuf[HEADER_SIZE]+1;
	unsigned char * endp = buffer + width*height/4*3;
	while(buffer!=endp) { // u/Cr channel
		*buffer = *c++;
		buffer+=3;
	}
	buffer = &curBuf[HEADER_SIZE]+2;
	endp = buffer + width*height/4*3;
	while(buffer!=endp) { // v/Cb channel
		*buffer = *c++;
		buffer+=3;
	}
}

void CameraDriverV4L1::interleave_yuv_up() {
	unsigned char * buffer = &curBuf[HEADER_SIZE];
	size_t width = vid_win.width;
	size_t height = vid_win.height;
	tmpBuf.resize(getBufferSize());
	memcpy(&tmpBuf[0],&curBuf[0],HEADER_SIZE);
	unsigned char * out = &tmpBuf[HEADER_SIZE];

  //first copy over y channel
  unsigned char * i = buffer;
  unsigned char * o = out;
	unsigned char * e = i + width*height;
	while(i!=e) {
		*o=*i++;
		o+=3;
	}
  
	//now u channel
	o = out+1;
	buffer+=width*height;
	for(size_t y=0; y<height/2; ++y) {
		i = buffer+width/2*y;
		e = i + width/2;
		while(i!=e) {
			*o=*(o+3)=*(o+width*3)=*(o+width*3+3)=*i++;
			o+=6;
		}
		o+=width*3;
	}
  
	//and now v channel
	o = out+2;
	buffer+=width*height/4;
	for(size_t y=0; y<height/2; ++y) {
		i = buffer+width/2*y;
		e = i + width/2;
		while(i!=e) {
			*o=*(o+3)=*(o+width*3)=*(o+width*3+3)=*i++;
			o+=6;
		}
		o+=width*3;
	}
  
	curBuf.swap(tmpBuf);
}
 

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
#endif
