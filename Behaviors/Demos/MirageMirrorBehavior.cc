#ifndef PLATFORM_APERIOS

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Motion/Kinematics.h"
#include "Wireless/netstream.h"
#include "Shared/ProjectInterface.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/WorldState.h"
#include "Shared/string_util.h"

//! A simple example of how to put an "invisible" marker in the Mirage simulator, mirrors the location of a pink target as seen by the camera
class MirageMirrorBehavior : public virtual plist::Dictionary, public BehaviorBase, public plist::PrimitiveListener {
public:
	//! constructor
	MirageMirrorBehavior()
		: plist::Dictionary(), BehaviorBase(), host("localhost"), port(19785), mirage(), egg()
	{
		egg.model="EggTop";
		egg.material="Red Plastic";
		egg.visible=false;
		
		// this lets us reconfigure settings using text message events:
		addEntry("Model",egg.model);
		addEntry("Material",egg.material);
		addEntry("Visible",egg.visible);
		addEntry("Host",host);
		addEntry("Port",port);
		egg.model.addPrimitiveListener(this);
		egg.material.addPrimitiveListener(this);
		egg.visible.addPrimitiveListener(this);
		host.addPrimitiveListener(this);
		port.addPrimitiveListener(this);
	}

	virtual void doStart() {
		BehaviorBase::doStart();
		
		// request pink blob detection
		erouter->addListener(this,EventBase::visObjEGID,ProjectInterface::visPinkBallSID,EventBase::statusETID);
		// support runtime re-configuration
		erouter->addListener(this,EventBase::textmsgEGID);
		
		// connect to Mirage
		openConnection();
	}
	
	virtual void doStop() {
		closeConnection();
		BehaviorBase::doStop();
	}		

	virtual void doEvent() {
		if(event->getGeneratorID()==EventBase::visObjEGID) {
			// saw a pink object
			const VisionObjectEvent& vis = dynamic_cast<const VisionObjectEvent&>(*event);
			fmat::Column<4> tgt = kine->projectToGround(vis);
			if(std::abs(tgt[3])>=1)
				updateLocation(tgt[0],tgt[1]);
		} else if(event->getGeneratorID()==EventBase::textmsgEGID) {
			// pull out key=value arguments
			const char * arg = dynamic_cast<const TextMsgEvent&>(*event).getText().c_str();
			const char* eq = strchr(arg,'=');
			if(eq==NULL) {
				std::cerr << instanceName << " bad input: " << arg << std::endl;
				return;
			}
			std::string key = string_util::trim(std::string(arg,eq-arg));
			std::string val = string_util::trim(eq+1);
			plist::ObjectBase * entry = resolveEntry(key);
			if(entry==NULL) {
				std::cerr << instanceName << " unknown key: " << key << std::endl;
				return;
			}
			entry->set(plist::Primitive<std::string>(val));
		}
	}
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pl) {
		if(&pl==&host || &pl==&port) {
			closeConnection();
			openConnection();
		} else {
			// one of the egg properties, resend the whole model
			updateModel();
		}
	}

	static std::string getClassDescription() { return "A simple example of how to put an \"invisible\" marker in the Mirage simulator, mirrors the location of a pink target as seen by the camera"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
	plist::Primitive<std::string> host; //!< host where mirage is running
	plist::Primitive<unsigned int> port; //!< port where mirage is listening for connections

protected:
	//! open a connection to Mirage and calls updateModel()
	void openConnection() {
		mirage.clear(); // reset results of previous connection attempts
		std::cout << "Connecting..." << std::flush;
		if(!mirage.open(host,port)) {
			std::cout << "refused." << std::endl;
			stop();
			return;
		}
		std::cout << "connected." << std::endl;
		mirage << "<messages>\n";
		updateModel();
	}
	
	//! sends the description of the model to place (#egg)
	void updateModel() {
		// generic way to do it:
		/*std::set<KinematicJoint*> root;
		root.insert(&egg);
		KinematicJointSaver kinSaver(root);
		plist::Dictionary msg;
		msg.addEntry("Model",kinSaver);*/
		
		// cute way to do it since we're only using one item:
		plist::Array arr; // array of items
		arr.addEntry(egg); // we only have one though
		plist::Dictionary msg;
		msg.addEntry("Model",arr);
		sendMessage(msg);
	}
	
	//! updates the location where #egg is placed
	void updateLocation(float x, float y) {
		std::cout << "seen " << x << ' ' << y << std::endl;
		plist::Point location;
		location[0] = x;
		location[1] = y;
		plist::Dictionary msg;
		msg.addEntry("Location",location);
		sendMessage(msg);
	}
	
	//! tacks on the robot ID and actually puts it on the network
	void sendMessage(plist::Dictionary& msg) {
		plist::Primitive<std::string> name(instanceName);
		msg.addEntry("ID",name);
		msg.saveStream(mirage,true);
		mirage.flush();
		if(!mirage)
			stop();
	}
	
	//! closes the connection to Mirage
	void closeConnection() {
		mirage << "</messages>";
		mirage.flush();
		mirage.close();
	}
	
	onetstream mirage; //!< network communication stream with Mirage
	KinematicJoint egg; //!< the object we're placing in Mirage
	
private:
	MirageMirrorBehavior(const MirageMirrorBehavior&); //!< don't call (copy constructor)
	MirageMirrorBehavior& operator=(const MirageMirrorBehavior&); //!< don't call (assignment operator)
};

REGISTER_BEHAVIOR_MENU_OPT(MirageMirrorBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Defines MirageMirrorBehavior, which is a simple example of how to put an "invisible" marker in the Mirage simulator, mirrors the location of a pink target as seen by the camera
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
