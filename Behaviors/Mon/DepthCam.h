#ifndef INCLUDED_DepthCam_h_
#define INCLUDED_DepthCam_h_

#include "CameraStreamBehavior.h"
#include "Shared/Config.h"

class Socket;
class FilterBankGenerator;
class FilterBankEvent;

class DepthCam : public CameraStreamBehavior {
public:
	//! constructor
	DepthCam();
	
	//! destructor
	~DepthCam() { theOne=NULL; }

#ifdef PLATFORM_APERIOS
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=200000; //!< 200000 bytes for use up to 416x320 + 2*208x160 (double res Y, full res UV on ERS-7)
#else
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=640*480*2 + 1024; //!< size for full 640x480 depth map + 1KB for header
#endif
	static const unsigned int UDP_WIRELESS_BUFFER_SIZE=64*1024; //!< 64KB is the max udp packet size

	virtual void doStart();

	virtual void doStop();

	virtual void doEvent();

	static std::string getClassDescription() {
		char tmp[20];
		snprintf(tmp,20,"%d",*config->vision.depthcam.port); tmp[19]='\0';
		return std::string("Forwards images from camera over port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }

 protected:
	static DepthCam* theOne;

	static int networkCallback(char* buf, int bytes) { return theOne->receiveData(buf,bytes); }

	void setupServer(); //!<setup the server socket (#visDepth)
	void closeServer(); //!<tear down the server socket (#visDepth)

	bool openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer);
	bool writeDepth(const FilterBankEvent& fbke);

       	void closePacket(); //!< closes and sends a packet, does nothing if no packet open

	bool sendCloseConnectionPacket();

	Socket* visDepth;
	char* packet;
	char* cur;
	unsigned int avail;
	unsigned int max_buf;
	unsigned int lastProcessedTime;

 private:
	DepthCam(const DepthCam&);
	DepthCam& operator=(const DepthCam&);
};
/*! @file
 * @brief Describes RawCam, which forwards images from camera over wireless
 * @author ejt (Creator)
 */

#endif
