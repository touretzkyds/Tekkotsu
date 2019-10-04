#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Behaviors/Nodes/LedNode.h"
#else
#  include "Behaviors/StateNode.h"
typedef StateNode LedNode;
#endif

#include "Behaviors/Controls/EventLogger.h"
#include "Shared/ProjectInterface.h"
#include "Behaviors/Transitions/TextMsgTrans.h"
#include "Behaviors/Transitions/NullTrans.h"
#include "Sound/SoundManager.h"
#include "Vision/JPEGGenerator.h"

//! a class for testing the image logging facilities
class ImageLogTestNode : public LedNode {
public:
	//!constructor
	ImageLogTestNode(std::string name, int n)
		: LedNode(name)
	{
#ifdef TGT_HAS_LEDS
		getMC()->displayNumber(n,LedEngine::onedigit);
#else
		(void)n; // avoid warning
#endif
	}
	virtual void doStart() {
		LedNode::doStart();
		EventLogger::logImage(*ProjectInterface::defColorJPEGGenerator,ProjectInterface::fullLayer,0,this);
		sndman->playFile("camera.wav");
	}
};

//! a class for testing the text message logging facilities
class MessageLogTestNode : public LedNode {
public:
	//!constructor
	MessageLogTestNode(std::string name, int n)
		: LedNode(name)
	{
#ifdef TGT_HAS_LEDS
		getMC()->displayNumber(n,LedEngine::onedigit);
#else
		(void)n; // avoid warning
#endif
	}
	virtual void doStart() {
		LedNode::doStart();
		EventLogger::logMessage("Hello World!",this); //icon and placement arguments also available
	}
};

//! a class for testing the external camera request facilities
class WebcamLogTestNode : public LedNode {
public:
	//!constructor
	WebcamLogTestNode(std::string name, int n)
		: LedNode(name)
	{
#ifdef TGT_HAS_LEDS
		getMC()->displayNumber(n,LedEngine::onedigit);
#else
		(void)n; // avoid warning
#endif
	}
	virtual void doStart() {
		LedNode::doStart();
		EventLogger::logWebcam(this);
	}
};


//! tests different methods of the state machine viewer logging facilities
class LogTestMachine : public StateNode {

	// ****************************
	// ******* CONSTRUCTORS *******
	// ****************************
public:
	//! default constructor, use type name as instance name
	LogTestMachine()
		: StateNode(), start(NULL)
	{}

	//! constructor, take an instance name
	LogTestMachine(const std::string& nm)
		: StateNode(nm), start(NULL)
	{}

	
	// ****************************
	// ********* METHODS **********
	// ****************************
public:
	virtual void setup() {
		StateNode::setup(); 

		//setup sub-nodes
		start=addNode(new StateNode("Waiting"));
		ImageLogTestNode * img_node=new ImageLogTestNode("Image",1); addNode(img_node);
		MessageLogTestNode * msg_node=new MessageLogTestNode("Message",2); addNode(msg_node);
		WebcamLogTestNode * webcam_node=new WebcamLogTestNode("Webcam",3); addNode(webcam_node);

		//link with transitions
		start->addTransition(new TextMsgTrans(img_node,"image"));
		start->addTransition(new TextMsgTrans(msg_node,"message"));
		start->addTransition(new TextMsgTrans(webcam_node,"webcam"));
		img_node->addTransition(new NullTrans(start));
		msg_node->addTransition(new NullTrans(start));
		webcam_node->addTransition(new NullTrans(start));
	}

	virtual void doStart() {
		std::cout <<
		"This behavior tests the EventLogger features of the Storyboard Viewer.\n"
		"Send a text message: ('!msg foo' in the ControllerGUI input field or 'msg foo' from HAL prompt)\n"
		"  'image' to log the current camera image\n"
		"  'message' to log 'Hello World'\n"
		"  'webcam' to request that the desktop machine take a 3rd person snapshot\n" << std::flush;
		start->start();
	}

	static std::string getClassDescription() { return "Allows testing of various EventLogger facilities via text message events"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// ****************************
	// ********* MEMBERS **********
	// ****************************
protected:
	StateNode * start; //!< the subnode to begin within on doStart()


	// ****************************
	// ********** OTHER ***********
	// ****************************
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	LogTestMachine(const LogTestMachine&); //!< don't call (copy constructor)
	LogTestMachine& operator=(const LogTestMachine&); //!< don't call (assignment operator)
};

REGISTER_BEHAVIOR_MENU_OPT(LogTestMachine,"Background Behaviors/Debugging Tests",BEH_NONEXCLUSIVE);

/*! @file
 * @brief Defines LogTestMachine, which allows testing of various EventLogger facilities via text message events
 * @author ejt (Creator)
 */
