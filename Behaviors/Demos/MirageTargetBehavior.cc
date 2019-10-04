#ifndef PLATFORM_APERIOS

#include "MirageTargetBehavior.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Shared/get_time.h"
#include "Shared/string_util.h"

REGISTER_BEHAVIOR_MENU_OPT(MirageTargetBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

using namespace std; 

void MirageTargetBehavior::doStart() {
	BehaviorBase::doStart();
	
	std::cout << "Connecting..." << std::flush;
	if(!mirage.open(mirage_host,mirage_port)) {
		std::cout << "refused." << std::endl;
		stop();
		return;
	}
	std::cout << "connected." << std::endl;
	mirage << "<messages>\n";
	
	plist::Dictionary msg;
	plist::Point location;
	location[0]=x;
	location[1]=y;
	location[2]=z;
	msg.addEntry("Location",location);
	
	KinematicJointSaver kinSaver(*l1);
	msg.addEntry("Model",kinSaver);
	
	updateQ(0, &msg);
	sendMessage(msg);
	
	starttime = get_time();
	erouter->addListener(this, EventBase::textmsgEGID);
	if(fps>0)
		erouter->addTimer(this, 0, static_cast<unsigned int>(1000/fps + 0.5f), true);
	
	x.addPrimitiveListener(this);
	y.addPrimitiveListener(this);
	z.addPrimitiveListener(this);
	fps.addPrimitiveListener(this);
	
	l1->model.addPrimitiveListener(this);
	l2->model.addPrimitiveListener(this);
	l3->model.addPrimitiveListener(this);
	l1->material.addPrimitiveListener(this);
	l2->material.addPrimitiveListener(this);
	l3->material.addPrimitiveListener(this);

	l1->addBranchListener(this);
	l2->addBranchListener(this);
	l3->addBranchListener(this);
}

void MirageTargetBehavior::doStop() {
	x.removePrimitiveListener(this);
	y.removePrimitiveListener(this);
	z.removePrimitiveListener(this);
	fps.removePrimitiveListener(this);
	
	l1->model.removePrimitiveListener(this);
	l2->model.removePrimitiveListener(this);
	l3->model.removePrimitiveListener(this);
	l1->material.removePrimitiveListener(this);
	l2->material.removePrimitiveListener(this);
	l3->material.removePrimitiveListener(this);
	
	l1->removeBranchListener(this);
	l2->removeBranchListener(this);
	l3->removeBranchListener(this);
	
	mirage << "</messages>";
	mirage.flush();
	mirage.close();
	BehaviorBase::doStop();
}

void MirageTargetBehavior::doEvent() {
	if(event->getGeneratorID()==EventBase::timerEGID) {
		float t = (event->getTimeStamp() - starttime)/1000.f;
		updateQ(t);
	} else if(event->getGeneratorID()==EventBase::textmsgEGID) {
		apply(dynamic_cast<const TextMsgEvent&>(*event).getText().c_str());
	}
}

void MirageTargetBehavior::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&x || &pl==&y || &pl==&z) {
		plist::Point location;
		location[0]=x;
		location[1]=y;
		location[2]=z;
		plist::Dictionary msg;
		msg.addEntry("Location",location);
		sendMessage(msg);
	} else if(&pl==&fps) {
		if(fps>0)
			erouter->addTimer(this, 0, static_cast<unsigned int>(1000/fps + 0.5f), true);
		else
			erouter->removeTimer(this, 0);
	} else if(&pl==&mirage_host || &pl==&mirage_port) {
		addReference();
		stop();
		start();
		removeReference();
	} else {
		kinematicJointReconfigured(*l1);
	}
}

void MirageTargetBehavior::kinematicJointReconfigured(KinematicJoint& joint) {
	KinematicJointSaver kinSaver(*l1);
	plist::Dictionary msg;
	msg.addEntry("Model",kinSaver);
	sendMessage(msg);
}

int MirageTargetBehavior::apply(const char* arg) {
	const char* eq = strchr(arg,'=');
	if(eq==NULL) {
		usage("",arg);
		return 2;
	}
	std::string key = string_util::trim(std::string(arg,eq-arg));
	std::string val = string_util::trim(eq+1);
	ObjectBase * entry = resolveEntry(key);
	if(entry==NULL) {
		usage(key,NULL);
		return 2;
	}
	entry->set(plist::Primitive<std::string>(val));
	return 0;
}

void MirageTargetBehavior::updateQ(float t, plist::Dictionary* msg /*=NULL*/) {
	l1->computeQ(t);
	l2->computeQ(t);
	l3->computeQ(t);
	
	plist::DictionaryOf<plist::Primitive<float> > positions;
	positions[l1->outputOverride] = l1->getQ();
	positions[l2->outputOverride] = l2->getQ();
	positions[l3->outputOverride] = l3->getQ();
	
	if(msg!=NULL) {
		msg->addEntry("Positions",positions.clone());
	} else {
		plist::Dictionary mmsg;
		mmsg.addEntry("Positions",positions);
		sendMessage(mmsg);
	}
	
	std::cout << (l3->getWorldPosition() + fmat::pack(x,y,z)) << std::endl;
}

void MirageTargetBehavior::sendMessage(plist::Dictionary& msg) {
	msg.addEntry("ID",name);
	msg.saveStream(mirage,true);
	mirage.flush();
	if(!mirage)
		stop();
}

#endif // not PLATFORM_APERIOS

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
