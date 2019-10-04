//-*-c++-*-
#ifndef INCLUDED_ImageStreamDriver_h_
#define INCLUDED_ImageStreamDriver_h_

#include "DataStreamDriver.h"
#include "Shared/ProjectInterface.h"

//! description of ImageStreamDriver
class ImageStreamDriver : public DataStreamDriver {
public:
	explicit ImageStreamDriver(const std::string& name)
		: DeviceDriver(autoRegisterDriver,name), DataStreamDriver(autoRegisterDriver,name), format(FORMAT_JPEG,formatNames), payloadSize(0), sid(ProjectInterface::visRawCameraSID)
	{
		addEntry("Format",format,"The type of format to expect from the comm port.\n"
				 "'YUV' expects interleaved components 'CameraResolutionX' wide and 'CameraResolutionY' high\n"
				 "(defined in target's RobotInfo namespace)");
	}

	virtual std::string getClassName() const { return autoRegisterDriver; }

	virtual void getImageSources(std::map<std::string,DataSource*>& sources) {
		sources.clear();
		sources["Camera"]=this;
	}
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	//! enumerates the available formats for input to the driver
	enum format_t {
		FORMAT_YUV=0, //!< raw interleaved YUV data, @c CameraResolutionX wide by @c CameraResolutionY high (defined in target's RobotInfo namespace)
		FORMAT_PNG, //!< PNG compressed image
		FORMAT_JPEG, //!< JPEG compressed iamge
		FORMAT_TEKKOTSU //!< the format used by the Tekkotsu streaming video
	};
	//! number of entries in format_t
	static const size_t NUM_FORMATS = 4;
	//! human-readable names for the supported formats, see format_t
	static const char * formatNames[NUM_FORMATS+1];
	
	//! The type of format to expect from the comm port.
	/*! 'YUV' expects interleaved components 'CameraResolutionX' wide and 'CameraResolutionY' high (defined in target's RobotInfo namespace) */
	plist::NamedEnumeration<format_t> format;

protected:
	virtual bool readData(std::istream& is);
	RCRegion * readImage(std::istream& is);
	static void copyImage(char * buf, unsigned int width, unsigned int height, unsigned int channels, const char * chan, unsigned int lwidth, unsigned int lheight, unsigned int lchan);
	
	virtual void connect(CommPort* comm);
	virtual void disconnect(CommPort* comm);

	void setSID(unsigned short imageSID) {this->sid = imageSID;}
	
	size_t payloadSize; //!< caches size of last decompressed (yuv) image to assume next image will be the same size
	unsigned short sid; // the source id of the stream

private:
	//! holds the class name, set via registration with the DeviceDriver registry
	static const std::string autoRegisterDriver;
	ImageStreamDriver(const ImageStreamDriver&); //!< no call
	ImageStreamDriver operator=(const ImageStreamDriver&); //!< no call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
