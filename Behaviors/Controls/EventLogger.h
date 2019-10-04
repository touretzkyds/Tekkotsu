//-*-c++-*-
#ifndef INCLUDED_EventLogger_h_
#define INCLUDED_EventLogger_h_

#include "ControlBase.h"
#include "Events/EventListener.h"
#include <fstream>
#include <set>
#include <queue>
#include <stack>

class FilterBankGenerator;
class BehaviorBase;
class StateNode;
/*! @cond INTERNAL */
// libxml2 forward declarations
extern "C" {
	struct _xmlNode;
	struct _xmlDoc;
	struct _xmlAttr;
	typedef _xmlNode xmlNode;
	typedef _xmlDoc xmlDoc;
	typedef _xmlAttr xmlAttr;
}
/*! @endcond */

//! allows logging of events to the console or a file, also provides some remote logging facilities over #logSocket, required by Storyboard tool
/*! Users' behaviors can call logMessage(), logImage(), and logWebcam() to insert the corresponding data into #logSocket via an XML 'event' node.
 *
 *  The protocol used with #logSocket is:
 *  - '<tt>list</tt>' - send list of all instantiated StateNodes
 *  - '<tt>spider </tt><i>name</i>' - spider the current structure of StateNode named <i>name</i>
 *  - '<tt>listen </tt><i>name</i>' - send updates regarding the activation status of <i>name</i> and its subnodes; you can specify a state which is not yet running
 *  - '<tt>ignore </tt><i>name</i>' - cancels a previous listen command
 *  - '<tt>clear</tt>' - cancels all previous listen commands; should be called at the beginning or end of each connection, preferably both
 *  
 *  Each of those commands should be terminated with a newline -
 *  i.e. one command per line
 *
 *  After a <tt>list</tt> command, the first line will be the number
 *  of StateNodes, followed by that number of lines, one StateNode
 *  name per line.
 *
 *  After a <tt>spider</tt> command, an XML description of the model
 *  will be sent.  If no matching StateNode is found, an warning will
 *  be displayed on #serr, and an empty model
 *  ("<model></model>") returned over the network
 *  connection.
 *
 *  All other commands give no direct response - listen can be
 *  executed before the specified StateNode is yet running, and ignore
 *  doesn't care whether or not the specified StateNode was actually
 *  being listened for.
 *
 *  The format of the model is:
 @verbatim
 <!DOCTYPE model [
 <!ELEMENT model (state*, transition*)>
 <!ELEMENT state (state*, transition*)>
 <!ELEMENT transition (source+, dest+)>
 <!ELEMENT source (#PCDATA)>
 <!ELEMENT dest (#PCDATA)>
 
 <!ATTLIST state id CDATA #REQUIRED>
 <!ATTLIST state class CDATA #REQUIRED>
 <!ATTLIST transition id CDATA #REQUIRED>
 <!ATTLIST transition class CDATA #REQUIRED>
 ]>@endverbatim
 *
 *  The format of status updates following a listen command is:
 @verbatim
 <!DOCTYPE event [
 <!ELEMENT event (fire*, statestart*, statestop*)>
 <!ELEMENT fire (EMPTY)>
 <!ELEMENT statestart (EMPTY)>
 <!ELEMENT statestop (EMPTY)>

 <!ATTLIST fire id CDATA #REQUIRED>
 <!ATTLIST fire time CDATA #REQUIRED>
 <!ATTLIST statestart id CDATA #REQUIRED>
 <!ATTLIST statestart time CDATA #REQUIRED>
 <!ATTLIST statestop id CDATA #REQUIRED>
 <!ATTLIST statestop time CDATA #REQUIRED>
 ]>@endverbatim
 *
 * The 'event' node is also used for the results of logImage(), logMessage(), and logWebcam().
*/
class EventLogger : public ControlBase, public EventListener {
public:
	//!constructor
	EventLogger();
	//!destructor
	virtual ~EventLogger();

	//!opens a custom (embedded) menu to toggle individual EGIDs
	virtual ControlBase* doSelect();
	
	virtual void refresh();

	//!sends all events received to stdout and/or logfile
	virtual void processEvent(const EventBase& event);

	//!returns #logSocket
	static class Socket* getLogSocket() { return logSocket; }
	
	//! returns #port
	static int getLogSocketPort() { return port; }
	
	//! sets #port
	static void setLogSocketPort(int p) { port=p; }

	//! send the current camera image over the log socket
	static void logImage(FilterBankGenerator& fbg, unsigned int layer, unsigned int channel, const BehaviorBase* source=NULL);

	//! send a string over the log socket
	static void logMessage(std::string msg, const BehaviorBase* source=NULL, const char* icon=NULL, unsigned int placement=0);
	
	//! request that the desktop side take a picture with the webcam (if available)
	static void logWebcam(const BehaviorBase* source=NULL);
	
	static int callback(char *buf, int bytes); //!< called by wireless when there's new data

protected:
	static EventLogger * theOne; //!< the instance which will handle network communication

	//! a separate processEvent to distinguish between events requested for logging and events requested by a remote monitor
	class StateMachineListener : public EventListener {
		//! forwards any events received to EventLogger::theOne's EventLogger::processStateMachineEvent()
		/*! EventLogger::runCommand() is responsible for maintaining which events this is listening to */
		virtual void processEvent(const EventBase& event) {
			EventLogger::theOne->processStateMachineEvent(event);
		}
	};
	static class StateMachineListener smProcess; //!< handles state machine transitions if the Storyboard GUI (or other remote monitor) is listening for state machine events

	virtual void clearSlots();

	//!sets the status char of slot @a i to @a c
	void setStatus(unsigned int i, char c);

	//!checks to see if logfilePath differs from the StringInputControl's value and switches it if it is
	void checkLogFile();
	
	//! dumps all of the transitions and subnodes of a given statenode
	/*! if parent is NULL, will dump the results over #logSocket, otherwise adds the xml tree as a child of @a parent */
	void spider(const StateNode* n, xmlNode* parent=NULL);

	//! returns true iff @a n or one of its parents is found in #listen
	bool isListening(const StateNode* n);

	//! parses commands sent from callback()
	void runCommand(const std::string& s);

	//!just to prettify the data sent out - probably should make this a null-op to save bandwidth after debugging is done
	void indent(unsigned int level);
	
	//!searches currently instantiated StateNodes to find the one named @a name
	const StateNode * find(const std::string& name);

	//!if there is a remote monitor listening for state machine transitions, this will send them over
	/*!this is called by the StateMachineListener, which is subscribed to only
	 * those machines which have been requested by the remote monitor */
	virtual void processStateMachineEvent(const EventBase& event);
	
	//! dumps elements of #queuedEvents over #logSocket, popping and freeing as it goes
	static void dumpQueuedEvents();
	
	//! writes an xmlNode out over #logSocket, freeing @a node when complete
	/*! uses @a doc if provided, otherwise makes a new temporary one which is then deleted again before the function returns */
	static void dumpNode(xmlNode* node, xmlDoc* doc=NULL);

	//!address of the logfile, if any (empty string is no logfile)
	std::string logfilePath;

	//!if a filename is given, events are logged to here
	std::ofstream logfile;
	
	//! events which are logged will be sent over this port in an xml format.  See eventlog.dtd in the docs directory
	static class Socket* logSocket;
	
	//! port number #logSocket will listen on
	static int port;
	
	//! reference count for #logSocket -- when this hits 0, close the socket
	static unsigned int logSocketRefCount;
	
	//!controls the level of verbosity - currently 0 through 2
	unsigned int verbosity;

	typedef std::set<BehaviorBase*> registry_t; //!< the type of the behavior registry (BehaviorBase::registry)

	typedef std::set<std::string> listen_t; //!< the type of #listen
	listen_t listen; //!< a set of state machine names which should have their subnodes monitored

	typedef std::queue<xmlNode*> queuedEvents_t; //!< the type of #queuedEvents
	static queuedEvents_t queuedEvents; //!< if logImage/logMessage/etc. are called during a transition, need to queue them until the transition event is complete

	typedef std::stack<xmlNode*> transStack_t; //!< the type of #transStack
	static transStack_t transStack; //!< if another transition occurs during the processing of another, have to recurse on processing the new transition first
};

/*! @file
 * @brief Describes EventLogger, which allows logging of events to the console or a file
 * @author ejt (Creator)
 */

#endif
