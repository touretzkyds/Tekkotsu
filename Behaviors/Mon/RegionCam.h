//-*-c++-*-
#ifndef INCLUDED_RegionCam_h_
#define INCLUDED_RegionCam_h_

#include "Behaviors/BehaviorBase.h"
#include "Shared/Config.h"

class Socket;
class FilterBankGenerator;
class FilterBankEvent;

//! Forwards segmented images from camera over wireless

class RegionCam : public BehaviorBase {
public:
	//! constructor
	RegionCam();

	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=85000; //!< 85000 bytes for use up to 416x320 pixels / 8 min expected runs * 5 bytes per run + some padding
	static const unsigned int UDP_WIRELESS_BUFFER_SIZE=64*1024; //!< 64KB is the max udp packet size

	virtual void doStart();

	virtual void doStop();

	virtual void doEvent();

	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",*config->vision.segcam.port);
		return std::string("Forwards regions calculated out of images from camera over port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	void closeServer(); //!<tear down the server socket (#visRegion)
	void setupServer(); //!<setup the server socket (#visRegion)
	
	//! opens a new packet, writes header info; returns true if open, false if otherwise open (check cur==NULL for error)
	/*! see the class documentation for RegionCam for the protocol documentation */
	bool openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer); 
	bool writeRegions(const FilterBankEvent& fbke); //!< writes the region information
	void closePacket(); //!< closes and sends a packet, does nothing if no packet open

	Socket * visRegion; //!< socket to send image stream over
	char * packet; //!< buffer being filled out to be sent
	char * cur; //!< current location in #packet
	unsigned int avail; //!< number of bytes remaining in #packet
	unsigned int max_buf; //!< the buffer size requested from Wireless when the socket was allocated

private:
	RegionCam(const RegionCam&); //!< don't call
	RegionCam& operator=(const RegionCam&); //!< don't call
};

/*! @file
 * @brief Implements RegionCam, which forwards the regions from RegionGenerator over wireless
 * @author harm & niels (Creators)
 */

#endif
