//-*-c++-*-
#ifndef INCLUDED_SerialCommPort_h_
#define INCLUDED_SerialCommPort_h_

#include "FileSystemCommPort.h"
#include <fcntl.h>
#include <termios.h>

//! Provides CommPort interface to serial port devices -- essentially just a FileSystemCommPort, but can apply terminal IO settings
/*! You could use FileSystemCommPort instead of this class, and thus
 *  rely on a prior manual call to stty.  However, other programs (or rebooting)
 *  will reset those parameters, so it's nice to use this class to ensure the
 *  desired settings are reapplied each time the device is opened. */
class SerialCommPort : public FileSystemCommPort {
public:
	//! constructor
	explicit SerialCommPort(const std::string& name)
	: FileSystemCommPort(autoRegisterSerialCommPort,name),
	baudRate(57600), dataBits(8), stopBits(1), parity(NONE,parityNames), sttyConfig(),
	fd(-1)
	{
		if(bauds.size()==0)
			initBauds();
		addEntry("Baud",baudRate,"Communication frequency (bits per second)");
		addEntry("DataBits",dataBits,"Number of data bits to send at a time (5-8)");
		addEntry("StopBits",stopBits,"Number of stop bits to send between data bits (1-2)");
		addEntry("Parity",parity,"Parity bit can be sent for error checking\n"+parity.getDescription());
		addEntry("TTYFlags",sttyConfig,"Additional configuration string to pass to stty\n(may not work when using non-standard baud rates on OS X)");
		baudRate.addPrimitiveListener(this);
		dataBits.addPrimitiveListener(this);
		stopBits.addPrimitiveListener(this);
		parity.addPrimitiveListener(this);
		sttyConfig.addPrimitiveListener(this);
	}
	
	//! destructor
	~SerialCommPort() {
		baudRate.removePrimitiveListener(this);
		dataBits.removePrimitiveListener(this);
		stopBits.removePrimitiveListener(this);
		parity.removePrimitiveListener(this);
		sttyConfig.removePrimitiveListener(this);
	}

	virtual std::string getClassName() const { return autoRegisterSerialCommPort; }
	
	virtual bool open() {
		if(openedCnt==0) { // just do it on initial call
			// open serial port, need to keep it open through the FileSystemCommPort opening
			fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
			setupSerial();
		}
		return FileSystemCommPort::open();
	}
	
	virtual bool close() {
		bool ans = FileSystemCommPort::close();
		if(openedCnt==0) {
			::close(fd);
			fd=-1;
		}
		return ans;
	}
	
	//! watches #sttyConfig, reapplies the settings if changed
	virtual void plistValueChanged(const plist::PrimitiveBase& pl) {
		if(&pl==&sttyConfig) {
			setupSerial();
		} else if(&pl==&baudRate) {
			setupSerial();
		} else if(&pl==&dataBits) {
			if(dataBits<5 || dataBits>8) {
				std::cerr << "Cannot set DataBits to " << dataBits << ", reset to " << dataBits.getPreviousValue() << std::endl;
				dataBits=dataBits.getPreviousValue();
				return;
			}
			setupSerial();
		} else if(&pl==&stopBits) {
			if(stopBits<1 || stopBits>2) {
				std::cerr << "Cannot set StopBits to " << stopBits << ", reset to " << stopBits.getPreviousValue() << std::endl;
				stopBits=stopBits.getPreviousValue();
				return;
			}
			setupSerial();
		} else if(&pl==&parity) {
			setupSerial();
		} else {
			// path or mode changed... if opened, will be called if changing path...
			FileSystemCommPort::plistValueChanged(pl);
		}
	}
	
	plist::Primitive<unsigned int> baudRate;
	plist::Primitive<unsigned int> dataBits;
	plist::Primitive<unsigned int> stopBits;
	enum parity_t { EVEN, ODD, NONE };
	static const char * const parityNames[];
	plist::NamedEnumeration<parity_t> parity;
	plist::Primitive<std::string> sttyConfig; //!< Configuration string to pass to stty
	
protected:
	//! file descriptor for serial port -- needed for tcsetattr and ioctl interfaces
	int fd;
	
	//! performs serial port initialization (if fd is non-negative)
	virtual void setupSerial();
	
	void dispError(const char* where, int ret, int err);

	//! initializes #bauds with all of the symbolic baud rate settings available on host platform
	static void initBauds();

	//! table of valid baud rates
	static std::map<int,speed_t> bauds;
		
	//! holds the class name, set via registration with the CommPort registry
	static const std::string autoRegisterSerialCommPort;
};

/*! @file
* @brief Describes SerialCommPort, which provides CommPort interface to serial port devices -- essentially just a FileSystemCommPort, but can apply terminal IO settings
* @author Ethan Tira-Thompson (ejt) (Creator)
*/

#endif
