//-*-c++-*-
#ifndef INCLUDED_SegCam_h_
#define INCLUDED_SegCam_h_

#include "CameraStreamBehavior.h"
#include "Shared/Config.h"

class Socket;
class FilterBankGenerator;
class FilterBankEvent;

//! Forwards segmented images from camera over wireless
/*! The format used for serialization is basically defined by the
 *  subclass of FilterBankGenerator being used.  I suggest looking at
 *  that classes's documentation to determine the format used. (Generally
 *  either SegmentedColorGenerator or RLEGenerator)
 *
 *  However, SegCam will add a few fields at the beginning of
 *  each packet to assist in processing the image stream.
 *
 *  I emphasize: <i>beginning</i> of each Vision packet, <i>before</i> the FilterBankGenerator header. 
 *  - <@c string:"TekkotsuImage">
 *  - <<tt>Config::vision_config::encoding_t</tt>: Config::vision_config::ENCODE_SINGLE_CHANNEL> <i>(always just sends a single channel)</i>
 *  - <<tt>Config::vision_config::compression_t</tt>: Config::vision_config::COMPRESS_RLE> <i>(This is misleading - may actually be uncompressed, but this signals it's a segmented color image)</i>
 *  - <@c unsigned @c int: width> <i>(this is the width of the largest channel - note different channels can be sent at different resolutions!  Provides cheap "compression" of chromaticity channels)</i>
 *  - <@c unsigned @c int: height> <i>(similarly, height of largest channel)</i>
 *  - <@c unsigned @c int: timestamp> <i>(time image was taken, milliseconds since boot)</i>
 *  - <@c unsigned @c int: framenumber> <i>(incremented for each frame, so we can tell if/when we drop one)</i>
 *
 *  Alternatively, SegCam may send a "Close Connection" packet
 *  when the server is shutting down.  This is to help UDP connections, which
 *  otherwise wouldn't realize that they need to start trying to reconnect.
 *  - <@c string:"CloseConnection">
 *  
 *  This is exactly the same protocol that is followed by the
 *  RawCam behavior as well - the same code can parse either stream.
 *
 *  However, one odd bit - since the RLEGenerator doesn't save the color
 *  information itself, SegCam will do it instead.  So, if
 *  SegCam is using RLE compression, it will tack a footer at
 *  the end of the packet: (from SegmentedColorGenerator::encodeColors())
 *  - <@c unsigned @c int: num_cols> <i>(number of different colors available)</i>
 *  - for each of num_col:
 *    - <@c char: red> <i>red color to use for display of this index</i>
 *    - <@c char: green> <i>green color to use for display of this index</i>
 *    - <@c char: blue> <i>blue color to use for display of this index</i>
 *
 *  You can tell whether to expect the color footer by the creator string
 *  that follows the SegCam header.  (The compression field listed
 *  is considering segmented color itself a type of compression, whether
 *  or not it's RLE encoded, so you can't use that to tell whether the
 *  data is RLE encoded until you get to the data section.)
 *
 *  This is a binary protocol -- the fields listed indicate binary values
 *  in the AIBO's byte order (little endian).  Strings are encoded using
 *  the LoadSave::encode(char*,unsigned int, unsigned int) method.
 */ 
class SegCam : public CameraStreamBehavior {
public:
	//! constructor
	SegCam();

	//! destructor
	~SegCam() { theOne=NULL; }

#if defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx) || defined(TGT_ERS7)
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=85000; //!< 85000 bytes for use up to 416x320 pixels / 8 min expected runs * 5 bytes per run + some padding
#else
	static const unsigned int TCP_WIRELESS_BUFFER_SIZE=301*1024; //!< 300KB for use up to 640x480 pixels / 8 min expected runs * 5 bytes per run + some padding
#endif
	static const unsigned int UDP_WIRELESS_BUFFER_SIZE=64*1024; //!< 64KB is the max udp packet size

	virtual void doStart();

	virtual void doStop();

	virtual void doEvent();

	static std::string getClassDescription() {
		char tmp[20];
		snprintf(tmp,20,"%d",*config->vision.segcam.port); tmp[19]='\0';
		return std::string("Forwards segmented images from camera over port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	static SegCam* theOne; //!< global instance of SegCam acting as server
	//! function for network data to be sent to -- forwards to #theOne's receiveData()
	static int networkCallback(char* buf, int bytes) { return theOne->receiveData(buf,bytes); }
	
	void closeServer(); //!<tear down the server socket (#visRLE)
	void setupServer(); //!<setup the server socket (#visRLE	)
	
	//! opens a new packet, writes header info; returns true if open, false if otherwise open (check cur==NULL for error)
	/*! see the class documentation for SegCam for the protocol documentation */
	bool openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer); 
	bool writeRLE(const FilterBankEvent& fbke); //!< writes a color image
	bool writeSeg(const FilterBankEvent& fbke); //!< writes a color image
	void closePacket(); //!< closes and sends a packet, does nothing if no packet open

	//! sends a packet signaling the server is closing the connection (good for UDP connections)
	bool sendCloseConnectionPacket();
		
	Socket * visRLE; //!< socket to send image stream over
	char * packet; //!< buffer being filled out to be sent
	char * cur; //!< current location in #packet
	unsigned int avail; //!< number of bytes remaining in #packet
	unsigned int max_buf; //!< the buffer size requested from Wireless when the socket was allocated
	unsigned int lastProcessedTime; //!< the time that the last event was processed

private:
	SegCam(const SegCam&); //!< don't call
	SegCam& operator=(const SegCam&); //!< don't call
};

/*! @file
 * @brief Describes SegCam, which forwards segmented images from camera over wireless
 * @author ejt (Creator)
 */

#endif
