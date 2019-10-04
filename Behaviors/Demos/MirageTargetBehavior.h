//-*-c++-*-
#ifndef INCLUDED_MirageTargetBehavior_h_
#define INCLUDED_MirageTargetBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Wireless/netstream.h"
#include "Motion/KinematicJoint.h"
#include <cmath>

//! Connects to the Mirage simulation environment, places an "Easter Egg" at the end of an invisible two-link arm, and moves it around.
/*! You could also run this as a separate executable, outside of Tekkotsu.  Mirage supports multiple "robots"
 *  within the same envionment, so you could have "live" targets being fed from some other source... 
 *  See tools/test/mirage_target for an example of using the behavior from a separate executable.
 *
 *  This has also been over-engineered a bit to allow you to change parameters on the fly via text message events, e.g. 'Link1.Period=10' */
class MirageTargetBehavior : virtual public plist::Dictionary, public BehaviorBase, public plist::PrimitiveListener, public KinematicJoint::BranchListener {
public:
	
	//! A specialized version of KinematicJoint to handle rotational motion of a virtual link (at the end of which a visible target will be affixed)
	/*! We don't actually do any kinematic computations in this behavior, so we could just use a
	 *  plain plist::Dictionary with appropriate keys (r, Name, Model) instead, which might be a little more straightforward,
	 *  but would be susceptible to changes in the protocol (e.g. renamed or newly required fields expected by Mirage)
	 *
	 *  We could also use a single-link kinematic joint and "manually" adjust the x-y-z location directly, but that would be too easy.  */
	class MovingJoint : public KinematicJoint {
	public:
		//! constructor, set up motion parameters
		MovingJoint(const std::string& _name, const std::string& _model, float _len, float _min, float _max, float _period)
			: KinematicJoint(), outputOverride(_name), period(_period)
		{
			setEntry("Name",outputOverride); // quick hack to override the outputOffset setting with a "fake" joint
			addEntry("Period",period);
			r=_len;
			qmin=_min;
			qmax=_max;
			model=_model;
		}
		
		//! updates the joint position based on #period and superclass's qmin and qmax settings
		void computeQ(float t) {
			if(period==0)
				return;
			if(std::fmod(qmin,(float)M_PI) == std::fmod(qmax,(float)M_PI))
				setQ( qmin + (qmax-qmin) * std::fmod(t/period,1) );
			else
				setQ( (qmax+qmin)/2 + (qmax-qmin)/2 * std::sin(t*2*(float)M_PI/period) );
		}
		
		plist::Primitive<std::string> outputOverride; //!< will be set to override the outputOffset field of KinematicJoint
		plist::Primitive<float> period; //!< time to allow for each cycle
	};
	
	
	//! constructor
	MirageTargetBehavior()
		: BehaviorBase(), PrimitiveListener(), KinematicJoint::BranchListener(),
		name(getName()), x(250), y(0), z(0), mirage_host("localhost"), mirage_port(19785), fps(25), l1(NULL), l2(NULL), l3(NULL), 
		mirage(), starttime(0)
	{
		l1 = new MovingJoint(name+":Link1", "", 0, (float)M_PI*3/5, (float)M_PI*7/5, 19);
		l2 = new MovingJoint(name+":Link2", "", 150, 0, 2*(float)M_PI, 6);
		l3 = new MovingJoint(name+":Target", "EggTop", 40, 0, 0, 0);
		l1->addBranch(l2);
		l2->addBranch(l3);
		
		addEntry("Name",name);
		addEntry("X",x);
		addEntry("Y",y);
		addEntry("Z",z);
		addEntry("Host",mirage_host);
		addEntry("Port",mirage_port);
		addEntry("FPS",fps);
		addEntry("Link1",*l1); // note defereference -- KinematicJoint owns its branches, so we don't want
		addEntry("Link2",*l2); //   to add as pointer, or else plist::Dictionary will try to assume ownership
		addEntry("Link3",*l3); //   (leads to double-delete on destruction)
	}
	//! destructor
	~MirageTargetBehavior() {
		delete l1; // recursively deletes rest of chain
		l1=l2=l3=NULL;
	}

	virtual void doStart();
	virtual void doStop();

	static std::string getClassDescription() { return "Connects to the Mirage simulation environment, places an \"Easter Egg\" at the end of an invisible two-link arm, and moves it around."; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
	//! timers to update target position and text messages to change parameters on the fly
	virtual void doEvent();

	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	virtual void kinematicJointReconfigured(KinematicJoint& joint);

	//! keep #name in sync with BehaviorBase::instanceName;
	virtual void setName(const std::string& newname) { BehaviorBase::setName(name=newname); }
	
	//! process a 'key=value' assignment
	int apply(const char* arg);
	
	//! send a target update to mirage based on time from doStart() @a t (seconds)
	/*! If @a msg is non-NULL, the position list will be appended to that message,
	 *  otherwise a new message is created and sent from within the function call */
	void updateQ(float t, plist::Dictionary* msg=NULL);
	
	//! actually puts the message on the network
	void sendMessage(plist::Dictionary& msg);
	
	plist::Primitive<std::string> name; //!< identifier to make target unique within Mirage objects
	plist::Primitive<float> x; //!< the x position of the root of the target chain
	plist::Primitive<float> y; //!< the y position of the root of the target chain
	plist::Primitive<float> z; //!< the z position of the root of the target chain
	plist::Primitive<std::string> mirage_host; //!< hostname to connect to
	plist::Primitive<unsigned int> mirage_port; //!< port number to connect to
	plist::Primitive<float> fps; //!< "frames per second", rate to send updates to Mirage
	MovingJoint * l1; //!< controls rotation of first link
	MovingJoint * l2; //!< controls rotation of second link
	MovingJoint * l3; //!< controls rotation of the target (by default, easter egg is rotationally symmetric)
	
protected:
	//! display usage information, dump key/value list
	void usage(const std::string& key, const char* arg) {
		if(key.size()==0)
			std::cerr << "Bad assignment: " << arg << std::endl;
		else
			std::cerr << "Unknown key: " << key << std::endl;
		std::cerr << "Usage: [ key=val [..]]\nKeys are:\n" << *this;
	}
	
	onetstream mirage; //!< network communication stream with Mirage
	unsigned int starttime; //!< timestamp of doStart()
	
private:
	MirageTargetBehavior(const MirageTargetBehavior&); //!< don't call (copy constructor)
	MirageTargetBehavior& operator=(const MirageTargetBehavior&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines MirageTargetBehavior, which connects to the Mirage simulation environment, places an "Easter Egg" at the end of an invisible two-link arm, and moves it around.
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
