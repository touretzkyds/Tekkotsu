//-*-c++-*-
#ifndef INCLUDED_EchoBehavior_h_
#define INCLUDED_EchoBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"

//! Waits for a connection, echos any data received back to the sender
class EchoBehavior : public BehaviorBase {
public:
	static EchoBehavior * theOne; //!< the singleton object (only one of these objects can be active at a time or they would conflict over ports)
	static unsigned short port; //!< the port to listen on for incoming UDP and TCP connections
	static int server_callbackT(char *buf, int bytes); //!< called by wireless when there's new data
	static int client_callbackT(char *buf, int bytes); //!< called by wireless when there's new data
	static int server_callbackU(char *buf, int bytes); //!< called by wireless when there's new data
	static int client_callbackU(char *buf, int bytes); //!< called by wireless when there's new data
	
	//! constructor
	EchoBehavior()
		: BehaviorBase("EchoBehavior")
	{
			theOne=this;
			bzero((char*)route,sizeof(route));
			for(unsigned int i=0; i<NUM_ROUTE; i++) {
				sockets[i]=NULL;
				socks[i]=-1;
				route[i][i]=true;				
			}
	}
	//! destructor
	~EchoBehavior() { theOne=NULL; }

	virtual void doStart();
	virtual void doStop();	
	virtual void doEvent();
	
	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",port);
		return std::string("Waits for a connection, echos any data received back to the sender, using port ")+tmp;
	}
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	//! indicates one of the available data sinks: combinations of client/server and TCP/UDP
	enum routeIndex_t {
		STCP=0,  //!< server TCP
		SUDP, //!< server UDP
		CTCP, //!< client TCP
		CUDP, //!< client UDP
		NUM_ROUTE //!< total number of different connections available
	};
	static const char * const routeNames[NUM_ROUTE]; //!< a user-readable name for each incoming or outgoing route

	class Socket * sockets[NUM_ROUTE]; //!< an array of sockets, one for each incoming or outgoing route
	int socks[NUM_ROUTE]; //!< the system socket number for each of #sockets, used to detect when a socket has been closed
	bool route[NUM_ROUTE][NUM_ROUTE]; //!< a table of bools indicating how data should be echoed -- if route[from][to] is set, route it
	void setupNetwork(); //!< initialize server ports
	void teardownNetwork(); //!< close open connections
	void processCallback(routeIndex_t src, char * buf, int bytes); //!< called by one of the wireless callbacks to do processing
	void intersect(unsigned char& bits, std::string arg); //!< unsets bits of @a bits which aren't represented by @a arg
	
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	EchoBehavior(const EchoBehavior&); //!< don't call (copy constructor)
	EchoBehavior& operator=(const EchoBehavior&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines EchoBehavior, which waits for a connection, echos any data received back to the sender
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
