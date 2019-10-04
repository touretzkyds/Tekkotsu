//-*-c++-*-
#ifndef INCLUDED_RawCam_h_
#define INCLUDED_RawCam_h_

#include "CameraStreamBehavior.h"
#include "Shared/Config.h"

class Socket;
class FilterBankGenerator;
class FilterBankEvent;

//! Forwards images from camera over wireless
/*! The format used for serialization is basically defined by the
 *  subclass of FilterBankGenerator being used.  I suggest looking at
 *  that classes's documentation to determine the format used. (Generally
 *  either RawCameraGenerator or JPEGGenerator)
 *
 *  However, RawCam will add a few fields at the beginning of
 *  each packet to assist in processing the image stream.
 *
 *  I emphasize: <i>beginning</i> of each Vision packet, <i>before</i> the FilterBankGenerator header. 
 *  - <@c string:"TekkotsuImage">
 *  - <<tt>Config::vision_config::encoding_t</tt>: rawcam.encoding> <i>(expect single or multiple channels, 0 means color (3 channels), 1 means intensity (1 channel))</i>
 *  - <<tt>Config::vision_config::compression_t</tt>: rawcam.compression> <i>(0==none, 1==jpeg, 2==rle)</i>
 *  - <@c unsigned @c int: width> <i>(this is the width of the largest channel - note different channels can be sent at different resolutions!  Provides cheap "compression" of chromaticity channels)</i>
 *  - <@c unsigned @c int: height> <i>(similarly, height of largest channel)</i>
 *  - <@c unsigned @c int: timestamp> <i>(time image was taken, milliseconds since boot)</i>
 *  - <@c unsigned @c int: framenumber> <i>(incremented for each frame, so we can tell if/when we drop one)</i>
 *
 *  Alternatively, RawCameraGenerator may send a "Close Connection" packet
 *  when the server is shutting down.  This is to help UDP connections, which
 *  otherwise wouldn't realize that they need to start trying to reconnect.
 *  - <@c string:"CloseConnection">
 *  
 *  This is exactly the same protocol that is followed by the SegCamBehavior as well - the same code can parse either stream.
 *
 *  This is a binary protocol -- the fields listed indicate binary values
 *  in the AIBO's byte order (little endian).  Strings are encoded using
 *  the LoadSave::encode(char*,unsigned int, unsigned int) method.
 */ 
class RawCam : public CameraStreamBehavior {
public:
	//! constructor
	RawCam();
	
	//! destructor
	~RawCam() { theOne=NULL; }

#ifdef PLATFORM_APERIOS
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=200000; //!< 200000 bytes for use up to 416x320 + 2*208x160 (double res Y, full res UV on ERS-7)
#elif defined(TGT_IS_CALLIOPE3) || defined(TGT_CAMERABOT)
        static const unsigned int TCP_WIRELESS_BUFFER_SIZE=1280 * 720 * 3 + 1024;
#else
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=901*1024; //!< 900KB for max of full-color 640x480 + 1KB for header
#endif
	static const unsigned int UDP_WIRELESS_BUFFER_SIZE=64*1024; //!< 64KB is the max udp packet size

	virtual void doStart();

	virtual void doStop();

	virtual void doEvent();

	static std::string getClassDescription() {
		char tmp[20];
		snprintf(tmp,20,"%d",*config->vision.rawcam.port); tmp[19]='\0';
		return std::string("Forwards images from camera over port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }

	static unsigned int getSourceLayer(unsigned int chan, unsigned int numLayers); //!< returns the layer which will be used out of the source, based on current ::config settings (i.e. compression, skip, etc)
	static unsigned int getSourceYLayer(unsigned int numLayers); //!< returns the layer which will be used out of the source, based on current ::config settings (i.e. compression, skip, etc)
	static unsigned int getSourceULayer(unsigned int numLayers); //!< returns the layer which will be used out of the source, based on current ::config settings (i.e. compression, skip, etc)
	static unsigned int getSourceVLayer(unsigned int numLayers); //!< returns the layer which will be used out of the source, based on current ::config settings (i.e. compression, skip, etc)

protected:
	static RawCam* theOne; //!< global instance of RawCam acting as server
	//! function for network data to be sent to -- forwards to #theOne's receiveData()
	static int networkCallback(char* buf, int bytes) { return theOne->receiveData(buf,bytes); }

	void closeServer(); //!<tear down the server socket (#visRaw)
	void setupServer(); //!<setup the server socket (#visRaw)
	
	void drawShapesIntoBuffer(const FilterBankEvent &fbke); //!< draw contents of drawShapes vector into camera buffer

	//! opens a new packet, writes header info; returns true if open, false if otherwise open (check cur==NULL for error)
	/*! see the class documentation for RawCam for the protocol documentation */
	bool openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer);
	bool writeColor(const FilterBankEvent& fbke); //!< writes a color image
	bool writeSingleChannel(const FilterBankEvent& fbke); //!< writes a single channel
	void closePacket(); //!< closes and sends a packet, does nothing if no packet open

	//! sends a packet signaling the server is closing the connection (good for UDP connections)
	bool sendCloseConnectionPacket();
		
	Socket * visRaw; //!< socket for sending the image stream
	char * packet; //!< point to the current buffer being prepared to be sent
	char * cur; //!< current location within that buffer
	unsigned int avail; //!< the number of bytes remaining in the buffer
	unsigned int max_buf; //!< the buffer size requested from Wireless when the socket was allocated
	unsigned int lastProcessedTime; //!< the time that the last event was processed
private:
	RawCam(const RawCam&); //!< don't call
	RawCam& operator=(const RawCam&); //!< don't call
};

/*! @file
 * @brief Describes RawCam, which forwards images from camera over wireless
 * @author ejt (Creator)
 */

#endif
