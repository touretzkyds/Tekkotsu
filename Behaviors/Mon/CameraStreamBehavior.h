//-*-c++-*-
#ifndef INCLUDED_CameraStreamBehavior_h_
#define INCLUDED_CameraStreamBehavior_h_

#include "Behaviors/BehaviorBase.h"

class Socket;

//! Base class for camera streaming communication classes, handles upstream communication
/*! This class isn't meant to be run directly -- it just provides common functionality for its subclasses. */
class CameraStreamBehavior : public BehaviorBase {
public:

	virtual void doEvent();

	static std::string getClassDescription() { return "Base class for camera streaming communication classes, handles upstream communication"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
	int receiveData(char* data, unsigned int len); //!< called when new data is available (currently, only to toggle sensor sending)
	
	void sendSensors(); //!< causes current sensor values to be sent through #curSocket (along with video data)

protected:
	//! constructor, protected because you're not intended to instantiate this directly
	/*! @param name the name of the instance and the class
	 *  @param s the subclass's socket, a reference is stored so CameraStreamBehavior will always have access to the current socket */
	CameraStreamBehavior(const std::string& name, Socket*& s)
		: BehaviorBase(name), curSocket(s), sensorListeners(0), lastProcessedTime(0)
	{}

	//! the socket over which to send updates
	Socket*& curSocket;

	//! number of times startSensors has been sent, minus number of times stopSensors has been sent
	unsigned int sensorListeners;

	//! timestamp of last sensor update sent
	unsigned int lastProcessedTime;

private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	CameraStreamBehavior(const CameraStreamBehavior&); //!< don't call (copy constructor)
	CameraStreamBehavior& operator=(const CameraStreamBehavior&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines CameraStreamBehavior, which is the base class for camera streaming communication classes, handles upstream communication
 * @author ejt (Creator)
 */

#endif
