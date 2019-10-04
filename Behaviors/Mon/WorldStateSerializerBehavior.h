//-*-c++-*-
#ifndef INCLUDED_WorldStateSerializer_h
#define INCLUDED_WorldStateSerializer_h

#include "Behaviors/BehaviorBase.h"
#include "Shared/Config.h"
#include <string>

class Socket;

//! Copies WorldState into a buffer for transmission over the network
/*! To determine the communication protocol, just look in the
 *  doEvent() function - it's pretty straightforward binary copy
 *  of values.
 *
 *  Protocol:
 *  - <@c char[]: modelName> (null terminated character array)
 *  - <@c unsigned @c int: timestamp>
 *  - <@c unsigned @c int: framenumber>
 *  - <@c unsigned @c int: ::NumOutputs>
 *  - for each <i>i</i> of ::NumOutputs:
 *    - <@c float: position of output <i>i</i>>
 *  - <@c unsigned @c int: ::NumSensors>
 *  - for each <i>i</i> of ::NumSensors:
 *    - <@c float: value of sensor <i>i</i>>
 *  - <@c unsigned @c int: ::NumButtons>
 *  - for each <i>i</i> of ::NumButtons:
 *    - <@c float: value of button <i>i</i>>
 *  - <@c unsigned @c int: ::NumPIDJoints>
 *  - for each <i>i</i> of ::NumPIDJoints:
 *    - <@c float: duty cycle of joint <i>i</i>>
 * */
class WorldStateSerializerBehavior : public BehaviorBase {
public:
	WorldStateSerializerBehavior(); //!< constructor
	
	virtual void doStart(); //!< starts listening for sensor update events
	virtual void doStop(); //!< stops listening for events
	virtual void doEvent(); //!< core functionality - performs serialization, sends to sockets
	static std::string getClassDescription() {
		char tmp[80];
		sprintf(tmp,"Sends sensor information to port %d and current pid values to port %d",*config->main.wsjoints_port,*config->main.wspids_port);
		return tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }
	
	//! returns string corresponding to the Java GUI which should be launched
	virtual std::string getGUIType() const { return "org.tekkotsu.mon.WorldStateRecordGUI"; }
	//! returns port number the Java GUI should connect to
	virtual unsigned int getPort() const { return config->main.wsjoints_port; }
	
protected:
	//! writes @a value to @a dst and advances @a dst by sizeof(T)
	/*! doesn't do any byte swapping, so this is only used if LoadSave indicates no byte swapping is needed */
	template<class T>
	inline static void copy(char **dst, const T& value) {
		memcpy(*dst, &value, sizeof(T));
		(*dst) += sizeof(T);
	}

	//! writes @a num copies of T from @a src to @a dst and advances @a dst by @a num * sizeof(T)
	/*! doesn't do any byte swapping, so this is only used if LoadSave indicates no byte swapping is needed */
	template<class T>
	inline static void copy(char **dst, const T * src, size_t num) {
		memcpy(*dst, src, num*sizeof(T));
		(*dst) += num*sizeof(T);
	}
	
	//! writes @a num characters from @a src to @a dst and advances @a dst by @a num * sizeof(T)
	inline static void copy(char **dst, const std::string& src, size_t num) {
		memcpy(*dst, src.c_str(), num*sizeof(std::string::value_type));
		(*dst) += num*sizeof(std::string::value_type);
	}

	Socket *wsJoints; //!< socket for sending current joint data
	Socket *wsPIDs; //!< socket for sending current PID info
	unsigned int lastProcessedTime; //!< the time that the last event was processed
	
private:
	WorldStateSerializerBehavior(const WorldStateSerializerBehavior&); //!< don't call
	WorldStateSerializerBehavior& operator= (const WorldStateSerializerBehavior&); //!< don't call
};

/*! @file
 * @brief Describes WorldStateSerializerBehavior, which copies WorldState into a buffer for transmission over the network
 * @author alokl (Creator)
 */

#endif
