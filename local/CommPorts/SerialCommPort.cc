#include "SerialCommPort.h"
#include <sys/ioctl.h>
#include <errno.h>
#ifdef __APPLE__
#  include <IOKit/serial/ioss.h>
#endif
#ifdef __linux__
#  include <linux/serial.h>
#endif

#ifdef __CYGWIN__
// Cygwin doesn't provide cfmakeraw...
// this definition found in port of unix 'script' utility
// by Alan Evans (Alan_Evans AT iwv com), 2002-09-27
// http://marc.info/?l=cygwin&m=103314951904556&w=2
void cfmakeraw(struct termios *termios_p) {
  termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
			  |INLCR|IGNCR|ICRNL|IXON);
  termios_p->c_oflag &= ~OPOST;
  termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  termios_p->c_cflag &= ~(CSIZE|PARENB);
  termios_p->c_cflag |= CS8;
}
#endif

using namespace std;

const char * const SerialCommPort::parityNames[] = { "EVEN", "ODD", "NONE", NULL };
INSTANTIATE_NAMEDENUMERATION_STATICS(SerialCommPort::parity_t);

std::map<int,speed_t> SerialCommPort::bauds;

const std::string SerialCommPort::autoRegisterSerialCommPort = CommPort::getRegistry().registerType<SerialCommPort>("SerialCommPort");

void SerialCommPort::setupSerial() {
	if(fd<0)
		return;
	
	// ** first do a basic setup to initialize our access to the serial port **
	
	// get current termios structure (sys/termios.h)
	struct termios	theTermios;
	int ret = tcgetattr(fd, &theTermios);
	cfmakeraw(&theTermios);
	
	// We'll handle Darwin and Linux after we're done with termios settings
	// these platforms use a separate ioctl call to get non-standard baud rates
#ifndef __APPLE__
	std::map<int,speed_t>::const_iterator baudIt = bauds.find(baudRate);
#  ifdef __linux__
	// For linux, custom/non-standard speed requires B38400 baud be set here (wtf...)
	const speed_t TGTBAUD = baudIt==bauds.end() ? B38400 : baudIt->second;
#  else
	// Unknown platform: we'll just have to stick to cfsetspeed, which probably
	// doesn't handle non-standard rates, but worth a shot :(
	const speed_t TGTBAUD = baudIt==bauds.end() ? baudRate : baudIt->second;
#  endif
	ret = cfsetispeed(&theTermios, TGTBAUD);
	if(ret)
		dispError("cfsetispeed",ret,errno);
	ret = cfsetospeed(&theTermios, TGTBAUD);
	if(ret)
		dispError("cfsetospeed",ret,errno);
#endif
	
	// turn on READ and ignore modem control lines
	theTermios.c_cflag |= CREAD | CLOCAL;
	//theTermios.c_cflag = CIGNORE;
	
	switch (dataBits) {
		case 5:		theTermios.c_cflag |= CS5;		break;
		case 6:		theTermios.c_cflag |= CS6;		break;
		case 7:		theTermios.c_cflag |= CS7;		break;
		case 8:		theTermios.c_cflag |= CS8;		break;
		default: std::cerr << "SerialCommPort: bad DataBits value " << dataBits << std::endl; 
	}
	
	// stop bit(s)?
	if(stopBits==1u)
		theTermios.c_cflag &= ~CSTOPB;
	else if(stopBits==2u)
		theTermios.c_cflag |= CSTOPB;
	else
		std::cerr << "SerialCommPort: bad StopBits value " << stopBits << std::endl; 
	
	// parity?
	switch(parity) {
		case EVEN:
			theTermios.c_cflag |= PARENB; theTermios.c_cflag &= ~PARODD; break;
		case ODD:
			theTermios.c_cflag |= PARENB; theTermios.c_cflag |= PARODD; break;
		case NONE:
			theTermios.c_cflag &= ~PARENB; break;
	}
	// default no flow control:
	theTermios.c_iflag &= ~(IXON | IXOFF);
	theTermios.c_cflag &= ~CRTSCTS;
	
	// apply our settings
	ret = tcsetattr(fd, TCSAFLUSH, &theTermios);
	//ret = ioctl(fd, TIOCSETA, &theTermios); // alternative ?
	if (ret)
		dispError("tcsetattr(TCSAFLUSH)",ret,errno);
	
#ifdef __APPLE__
	// this allows higher (non-standard) baud rates, apparently not supported (on darwin) via termios
	const int TGTBAUD = baudRate;
	ret = ioctl(fd, IOSSIOSPEED, &TGTBAUD); // as opposed to setting it in theTermios ?
	if (ret)
		dispError("ioctl(IOSSIOSPEED)",ret,errno);
#endif

#ifdef __linux__
	if(baudIt==bauds.end()) {
		serial_struct sserial;
		ret = ioctl(fd, TIOCGSERIAL, &sserial);
		if(ret)
			dispError("ioctl(TIOCGSERIAL)",ret,errno);
		else {
			// if we're using a custom speed, set the bit, otherwise clear it
			if(baudIt==bauds.end())
				sserial.flags |= ASYNC_SPD_CUST;
			else
				sserial.flags &= ~ASYNC_SPD_MASK;
			// always request low-latency though (appears to be default anyway)
			sserial.flags |= ASYNC_LOW_LATENCY;
			sserial.custom_divisor = (int)(sserial.baud_base / (float)baudRate + .5);
			ret = ioctl(fd, TIOCSSERIAL, &sserial);
			if(ret)
				dispError("ioctl(TIOCSSERIAL)",ret,errno);
		}
	}
#endif
	
	// this part doesn't seem to be necessary, but better safe than sorry...
	int modem;
	ret = ioctl(fd, TIOCMGET, &modem);
	if (ret!=0) {
		//int err = errno;
		//std::cerr << "Warning: ioctl(TIOCMGET) returned errno " << err << " (" << strerror(err) << ")" << endl;
	} else {
		modem |= TIOCM_DTR;
		ret = ioctl(fd, TIOCMSET, &modem);
		if (ret)
			dispError("ioctl(TIOCMSET)",ret,errno);
	}
	
	// this works on linux, but won't let you set custom speeds on OS X
	// best to leave blank unless you really need it
	if(sttyConfig.size()>0) {
		// ** do additional setup by calling out to stty **
	#ifdef __linux__
		std::string cmd="stty -F "+path+" "+sttyConfig;
	#else /* assume BSD style... use a -f instead of -F (...sigh...) */
		std::string cmd="stty -f "+path+" "+sttyConfig;
	#endif
		switch(::system(cmd.c_str())) {
			case 0:
				break; // good!
			case -1:
				perror("Warning: SerialCommPort could not make system call to stty"); break;
			case 127:
				std::cerr << "Warning: SerialCommPort could not make system call to stty: no shell found" << std::endl; break;
			default:
				std::cerr << "Warning: SerialCommPort stty reported error on configuration string: " << sttyConfig << std::endl; break;
		}
	}
}

void SerialCommPort::dispError(const char* where, int ret, int err) {
	std::cerr << where << " returned " << ret << " (errno " << err << ": " << strerror(err) << ")" << std::endl;
}

void SerialCommPort::initBauds() {
#ifdef B0
	bauds[0]=B0;
#endif
#ifdef B50
	bauds[50]=B50;
#endif
#ifdef B75
	bauds[75]=B75;
#endif
#ifdef B110
	bauds[110]=B110;
#endif
#ifdef B134
	bauds[134]=B134;
#endif
#ifdef B150
	bauds[150]=B150;
#endif
#ifdef B200
	bauds[200]=B200;
#endif
#ifdef B300
	bauds[300]=B300;
#endif
#ifdef B600
	bauds[600]=B600;
#endif
#ifdef B1200
	bauds[1200]=B1200;
#endif
#ifdef B1200
	bauds[1200]=B1200;
#endif
#ifdef B1800
	bauds[1800]=B1800;
#endif
#ifdef B2400
	bauds[2400]=B2400;
#endif
#ifdef B4800
	bauds[4800]=B4800;
#endif
#ifdef B9600
	bauds[9600]=B9600;
#endif
#ifdef B19200
	bauds[19200]=B19200;
#endif
#ifdef B38400
	bauds[38400]=B38400;
#endif
#ifdef B57600
	bauds[57600]=B57600;
#endif
#ifdef B76800
	bauds[76800]=B76800;
#endif
#ifdef B115200
	bauds[115200]=B115200;
#endif
#ifdef B153600
	bauds[153600]=B153600;
#endif
#ifdef B230400
	bauds[230400]=B230400;
#endif
#ifdef B307200
	bauds[307200]=B307200;
#endif
#ifdef B460800
	bauds[460800]=B460800;
#endif
#ifdef B500000
	bauds[500000]=B500000;
#endif
#ifdef B576000
	bauds[576000]=B576000;
#endif
#ifdef B921600
	bauds[921600]=B921600;
#endif
#ifdef B1000000
	bauds[1000000]=B1000000;
#endif
#ifdef B1152000
	bauds[1152000]=B1152000;
#endif
#ifdef B1500000
	bauds[1500000]=B1500000;
#endif
#ifdef B2000000
	bauds[2000000]=B2000000;
#endif
#ifdef B2500000
	bauds[2500000]=B2500000;
#endif
#ifdef B3000000
	bauds[3000000]=B3000000;
#endif
#ifdef B3500000
	bauds[3500000]=B3500000;
#endif
#ifdef B4000000
	bauds[4000000]=B4000000;
#endif
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
