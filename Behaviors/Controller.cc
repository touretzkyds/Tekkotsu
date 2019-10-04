#include "Controller.h"
#include "Motion/EmergencyStopMC.h"
#include "Motion/MMAccessor.h"
#include "IPC/SharedObject.h"
#include "Shared/WorldState.h"
#include "Shared/get_time.h"
#include "Sound/SoundManager.h"
#include "Events/TextMsgEvent.h"
#include "Shared/RobotInfo.h"
#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS2xxInfo.h"
#include "Shared/ERS7Info.h"
#include "Shared/ChiaraInfo.h"
#include "Shared/string_util.h"
#include "Shared/ProjectInterface.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif

#include <sstream>

Controller * Controller::theOneController=NULL;

//these are given appropriate values in init once we know which model we're running on
EventBase Controller::nextItem;
EventBase Controller::prevItem;
EventBase Controller::nextItemFast;
EventBase Controller::prevItemFast;
EventBase Controller::selectItem;
EventBase Controller::cancel;

using namespace string_util;
using namespace std;

void Controller::doStart() {
	BehaviorBase::doStart();
	sndman->loadFile(config->controller.select_snd);
	sndman->loadFile(config->controller.next_snd);
	sndman->loadFile(config->controller.prev_snd);
	sndman->loadFile(config->controller.read_snd);
	sndman->loadFile(config->controller.cancel_snd);
	erouter->addListener(this,EventBase::estopEGID);
	// Turn on wireless
	gui_comm=wireless->socket(Socket::SOCK_STREAM, 2048, 32000);
	wireless->setReceiver(gui_comm->sock, gui_comm_callback);
	wireless->setDaemon(gui_comm,true);
	wireless->listen(gui_comm->sock, config->controller.gui_port);
	theOneController=this;
#ifdef TGT_HAS_LEDS
	SharedObject<LedMC> leds;
	leds->setWeights(~FaceLEDMask,0);
	leds->setWeights(FaceLEDMask,.75f);
	display=motman->addPersistentMotion(leds,isControlling?MotionManager::kEmergencyPriority:MotionManager::kIgnoredPriority);
#endif
	reset();
}

void Controller::doStop() {
	sndman->releaseFile(config->controller.select_snd);
	sndman->releaseFile(config->controller.next_snd);
	sndman->releaseFile(config->controller.prev_snd);
	sndman->releaseFile(config->controller.read_snd);
	sndman->releaseFile(config->controller.cancel_snd);
	erouter->removeListener(this);
	reset();
	motman->removeMotion(display);
	display=MotionManager::invalid_MC_ID;
	//these two lines help prevent residual display in case that was the only MotionCommand using LEDs
#ifdef TGT_HAS_LEDS
	for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
		motman->setOutput(NULL,i,0.f);
#endif
	gui_comm->printf("goodbye\n");
	wireless->setDaemon(gui_comm,false);
	wireless->close(gui_comm);
	theOneController=NULL;
	BehaviorBase::doStop();
}

void Controller::doEvent() {
	if(event->getTypeID()==EventBase::activateETID) { //estop just turned on
		if(!isControlling)
			activate();
	}	else { //estop just turned off
		if(isControlling)
			deactivate();
	}
}

bool Controller::trapEvent(const EventBase& e) {
	if(!chkCmdStack())
		return false;
	last_time=cur_time;
	cur_time=get_time();
	//this will prevent inadvertant controller commands when you pick up an ERS-7
	if(state->buttons[nextItem.getSourceID()] && state->buttons[prevItem.getSourceID()] && state->buttons[selectItem.getSourceID()])
		return true;
	
	if(nextItem.sameGenSource(e)) {
		nextEv_val=e.getMagnitude();
		nextEv_dur=e.getDuration();
		if(nextEv_val==0 && prevEv_val==0)
			alreadyGotBoth=false;
		if(nextEv_val>.75 && prevEv_val>.75 && nextEv_dur<666 && prevEv_dur<666) {
			if(alreadyGotBoth)
				return true;
			else {
				alreadyGotBoth=true;
				return setNext(cmdstack.top()->doReadStdIn());
			}
		}
		if(e.getTypeID()==nextItem.getTypeID() && e.getDuration()<666)
			return setNext(cmdstack.top()->doNextItem());
		if(e.getTypeID()==nextItemFast.getTypeID() && e.getDuration()>666 && calcPulse(cur_time,last_time,static_cast<unsigned int>(50/e.getMagnitude())))
			return setNext(cmdstack.top()->doNextItem());
	}
	if(prevItem.sameGenSource(e)) {
		prevEv_val=e.getMagnitude();
		prevEv_dur=e.getDuration();
		if(nextEv_val==0 && prevEv_val==0)
			alreadyGotBoth=false;
		if(nextEv_val>.75 && prevEv_val>.75 && nextEv_dur<666 && prevEv_dur<666) {
			if(alreadyGotBoth)
				return true;
			else {
				alreadyGotBoth=true;
				return setNext(cmdstack.top()->doReadStdIn());
			}
		}
		if(e.getTypeID()==prevItem.getTypeID() && e.getDuration()<666)
			return setNext(cmdstack.top()->doPrevItem());
		if(e.getTypeID()==prevItemFast.getTypeID() && e.getDuration()>666 && calcPulse(cur_time,last_time,static_cast<unsigned int>(50/e.getMagnitude())))
			return setNext(cmdstack.top()->doPrevItem());
	}
	if(e.getDuration()>250) {
		if(e==selectItem)
			return setNext(cmdstack.top()->doSelect());
		if(e==cancel)
			return setNext(cmdstack.top()->doCancel());
	}
	return true;
}

void Controller::reset() {
	while(cmdstack.size()>1)
		pop();
	if(!cmdstack.empty()) {
		cmdstack.top()->deactivate();
		cmdstack.pop();
	}
	refresh();
}

void Controller::refresh() {
	if(!chkCmdStack())
		return;
	cmdstack.top()->refresh();
}

void Controller::refreshSketchWorld() {
	theOneController->gui_comm->printf("refreshsketchworld\n");
}

void Controller::refreshSketchLocal() {
	theOneController->gui_comm->printf("refreshsketchlocal\n");
}

void Controller::refreshSketchCamera() {
	theOneController->gui_comm->printf("refreshsketchcamera\n");
}

void Controller::push(ControlBase* c) {
	if(!chkCmdStack())
		return;
	cmdstack.top()->pause();
	cmdstack.push(c);
	theOneController->gui_comm->printf("push\n");
	setNext(cmdstack.top()->activate(display,gui_comm));
}

void Controller::pop() {
	cmdstack.top()->deactivate();
	cmdstack.pop();
	theOneController->gui_comm->printf("pop\n");
	refresh();
}

Controller& Controller::setRoot(ControlBase* r) {
	reset();
	root=r;
	refresh();
	return *this;
}

Controller& Controller::setEStopID(MotionManager::MC_ID estopid) {
	estop_id=estopid;
	if(static_cast<EmergencyStopMC*>(motman->peekMotion(estopid))->getStopped()) {
		if(!isControlling)
			activate();
	} else {
		if(isControlling)
			deactivate();
	}		
	return *this;
}

void Controller::loadGUI(const std::string& type, const std::string& name, unsigned int port, const std::vector<std::string>& args) {
	if(theOneController==NULL)
		return;
	std::stringstream ss;
	ss << "load\n" << type << '\n' << name << '\n' << port << '\n';
	for(unsigned int i=0; i<args.size(); i++) {
		ss << '"';
		for(unsigned int j=0; j<args[i].size(); j++) {
			if(args[i][j]=='\\' || args[i][j]=='"' || args[i][j]=='\n')
				ss << '\\';
			ss << args[i][j];
		}
		ss << "\" ";
	}
	ss << '\n';
	theOneController->gui_comm->write((const byte*)ss.str().c_str(),ss.str().size());
}

void Controller::closeGUI(const std::string& name) {
	if(theOneController==NULL)
		return;
	ASSERTRET(theOneController->gui_comm!=NULL,"null gui_comm");

	theOneController->gui_comm->printf("close\n%s\n",name.c_str());
}

int Controller::gui_comm_callback(char *buf, int bytes) {
	std::string s(buf,bytes);
	//	cout << "Controller Received: " << s << endl;
	if(theOneController==NULL)
		return 0;

	static std::string incomplete;

	//pass a line at a time to the controller
	while(s.size()>0) {
		std::string::size_type endline=s.find('\n');
		if(endline==std::string::npos) {
			incomplete+=s;
			return 0;
		}
		
		//strip a \r\n or a \n
		if(endline>0 && s[endline-1]=='\r')
			incomplete+=s.substr(0,endline-1);
		else
			incomplete+=s.substr(0,endline);
		
		//is now complete
		theOneController->takeLine(incomplete); 
		incomplete.erase();
		s=s.substr(endline+1);
	}
	
	return 0;
}

int Controller::console_callback(char *buf, int bytes) {
	std::string s(buf,bytes);
	//	cout << "Console Received: " << s << endl;
	if(theOneController==NULL)
		return 0;

	static std::string incomplete;

	//pass a line at a time to the controller
	while(s.size()>0) {
		std::string::size_type endline=s.find('\n');
		if(endline==std::string::npos) {
			incomplete+=s;
			return 0;
		}

		//strip a \r\n or a \n
		if(endline>0 && s[endline-1]=='\r')
			incomplete+=s.substr(0,endline-1);
		else
			incomplete+=s.substr(0,endline);
		
		//is now complete
		switch(config->main.consoleMode) {
			case Config::main_config::CONTROLLER:
				theOneController->takeLine(incomplete); break;
			case Config::main_config::TEXTMSG:
				erouter->postEvent(TextMsgEvent(incomplete,0)); break;
			case Config::main_config::AUTO:
				if(wireless->isConnected(theOneController->gui_comm->sock)) 	 
					erouter->postEvent(TextMsgEvent(incomplete,0)); 	 
				else
					theOneController->takeLine(incomplete); 
				break;
		}
		incomplete.erase();
		s=s.substr(endline+1);
	}
	
	return 0;
}

/*! Select which model is running and call initButtons with the appropriate button offsets
 *  This could be somewhat simplified by using capabilities.getButtonOffset(), (wouldn't need
 *  the ERS2xx case with essentially duplicated ERS210 and ERS220 cases), but this
 *  style has the advantage that the symbols are checked by the compiler so there's no
 *  chance of a typo in a button name going unnoticed. */
void Controller::init() {
	usesButtons=false; // to be reset if initButtons is called
	
	if(TargetName == ERS2xxInfo::TargetName) {
		// compatability mode, see which of the targets is actually running
		// Note using ERS2xxInfo namespace to get appropriate offsets!
		// could remove duplication with "direct" 210/220 cases below by using something like:
		//   capabilities.getButtonOffset(ERS210Info::outputNames[ERS210Info::fooButOffset])
		if(RobotName == ERS210Info::TargetName) {
			initButtons(666,250,ERS2xxInfo::HeadFrButOffset,ERS2xxInfo::HeadBkButOffset,ERS2xxInfo::HeadFrButOffset,ERS2xxInfo::HeadBkButOffset,ERS2xxInfo::ChinButOffset,ERS2xxInfo::BackButOffset);
		} else if(RobotName == ERS220Info::TargetName) {
			//the 220 doesn't really support "fast" because it's using boolean buttons
			//i'm using a "hack" on the 210 because the pressure sensitivity causes status
			//events to continually be sent but since this is just on/off, it only gets the
			//activate/deactivate.  To fix this, override nextItemFast and prevItemFast with
			// timers and do timer management in processEvents()
			initButtons(666,50,ERS2xxInfo::TailLeftButOffset,ERS2xxInfo::TailRightButOffset,ERS2xxInfo::TailLeftButOffset,ERS2xxInfo::TailRightButOffset,ERS2xxInfo::TailCenterButOffset,ERS2xxInfo::BackButOffset);
		} else {
			cerr << "Controller: Unsupported 2xx model '" << RobotName << "'!  Appears to have buttons, but Controller doesn't know how to use them." << endl;
		}
	} else if(RobotName == ERS210Info::TargetName) {
		initButtons(666,250,ERS210Info::HeadFrButOffset,ERS210Info::HeadBkButOffset,ERS210Info::HeadFrButOffset,ERS210Info::HeadBkButOffset,ERS210Info::ChinButOffset,ERS210Info::BackButOffset);
	} else if(RobotName == ERS220Info::TargetName) {
		//the 220 doesn't really support "fast" because it's using boolean buttons
		//i'm using a "hack" on the 210 because the pressure sensitivity causes status
		//events to continually be sent but since this is just on/off, it only gets the
		//activate/deactivate.  To fix this, override nextItemFast and prevItemFast with
		// timers and do timer management in processEvents()
		initButtons(666,50,ERS220Info::TailLeftButOffset,ERS220Info::TailRightButOffset,ERS220Info::TailLeftButOffset,ERS220Info::TailRightButOffset,ERS220Info::TailCenterButOffset,ERS220Info::BackButOffset);
	} else if(RobotName == ERS7Info::TargetName) {
		initButtons(500,25,ERS7Info::FrontBackButOffset,ERS7Info::RearBackButOffset,ERS7Info::FrontBackButOffset,ERS7Info::RearBackButOffset,ERS7Info::MiddleBackButOffset,ERS7Info::HeadButOffset);
	} else if(RobotName == ChiaraInfo::TargetName) {
		// doesn't support button navigation
	} else {
#ifdef TGT_HAS_BUTTONS
		// not that big a deal, don't bother with the warning :-/
		//cerr << "Controller: Unsupported model '" << RobotName << "'!  Appears to have buttons, but Controller doesn't know how to use them." << endl;
#endif
	}
}

void Controller::initButtons(unsigned fastTime, unsigned downTime, unsigned nextB, unsigned prevB, unsigned nextFastB, unsigned prevFastB, unsigned selectB, unsigned cancelB) {
	nextItem=EventBase(EventBase::buttonEGID,nextB,EventBase::deactivateETID,0);
	prevItem=EventBase(EventBase::buttonEGID,prevB,EventBase::deactivateETID,0);
	nextItemFast=EventBase(EventBase::buttonEGID,nextFastB,EventBase::statusETID,fastTime);
	prevItemFast=EventBase(EventBase::buttonEGID,prevFastB,EventBase::statusETID,fastTime);
	selectItem=EventBase(EventBase::buttonEGID,selectB,EventBase::deactivateETID,downTime);
	cancel=EventBase(EventBase::buttonEGID,cancelB,EventBase::deactivateETID,downTime);
	usesButtons=true;
}


bool Controller::select(ControlBase* item, const std::string& name) {
  // Depth first
  const std::vector<ControlBase*>& slots = item->getSlots();
  for(unsigned int i=0; i<slots.size(); i++) {
    if (slots[i] != NULL) {
      if (slots[i]->getName() == name) { // sensitive to #Name
	char in[10];
	snprintf(in, 9, "%d", i); in[9]='\0';
	ControlBase * ret = item->takeInput(in);
	if(ret!=NULL) {
	  setNext(ret);
	  return true;
	}
      } else {
	if (select(slots[i], name)) 
	  return true;
      }
    }
  }
  return false;
}

void Controller::takeLine(const std::string& s) {
	//	cout << "RECEIVED: " << s << endl;
	if(s.size()==0)
		return;
	// break s into a vector of arguments
	std::vector<std::string> args;
	std::vector<unsigned int> offsets;
	if(!string_util::parseArgs(s,args,offsets)) {
		serr->printf("Controller::takeLine(\"%s\") was malformed.\n",s.c_str());
		return;
	}
	if(args.size()==0 || offsets.size()==0)
		return;
	// now look through for a ';' (separates multiple commands)
	unsigned int last=offsets[0];
	for(unsigned int i=0; i<args.size(); i++) {
		if(args[i]==";") { // if we found a ';', recurse with substring
			takeLine(s.substr(last,offsets[i]-last));
			if(i+1==args.size()) // last arg is a ';'
				return;
			last=offsets[i+1];
		}
		if(args[i]=="\\;") // if we found a '\;', replace it with base ';'
			args[i]=";";
	}
	if(!chkCmdStack())
		return;
	if(args[0][0]!='!') {
		setNext(cmdstack.top()->takeInput(s));
	} else {
		if(last!=offsets[0]) { // only changes if we found a ';' - in that case, need to do last segment
			takeLine(s.substr(last));
		} else if(args[0]=="!refresh") {
			refresh();
		} else if(args[0]=="!reset") {
			reset();
		} else if(args[0]=="!cancel") {
			setNext(cmdstack.top()->doCancel());
		} else if(args[0]=="!select") {
		  if (args.size() == 1)
				setNext(cmdstack.top()->doSelect());
		  else {
		    select(root, args[1].c_str());
		    refresh();
		  }
		} else if(args[0]=="!next") {
			setNext(cmdstack.top()->doNextItem());
		} else if(args[0]=="!prev") {
			setNext(cmdstack.top()->doPrevItem());
		} else if(args[0]=="!dump_stack") {
			dumpStack();
		} else if(args[0]=="!post") {
			if(args.size()<4) {
				serr->printf("Bad post command, need at least 3 arguments: generator source type [duration]\n");
				return;
			}
			//parse generator id -- could be a generator name or a numeric value
			int egid=0;
			for(;egid<EventBase::numEGIDs && args[1]!=EventBase::EventGeneratorNames[egid];egid++) {}
			if(egid==EventBase::numEGIDs) {
				egid=atoi(args[1].c_str());
				if(egid==0 && args[1]!="0") {
					serr->printf("Bad event generator '%s'\n",args[1].c_str());
					return;
				}
			}
			//parse source id -- numeric value, unless egid is buttonEGID, in which case we can look up a button name
			//(if you want to add support for other symbolic source types, this is where to do it)
			unsigned int source;
			if(egid==EventBase::buttonEGID) {
				source=capabilities.findButtonOffset(args[2].c_str());
				if(source==-1U) {
					source=atoi(args[2].c_str());
					if(source==0 && args[2]!="0") {
						serr->printf("Invalid button name or index '%s'\n",args[2].c_str());
						return;
					}
				}
			} else {
				source=atoi(args[2].c_str());
			}
			//parse type id -- numeric, name, or abbreviated name
			int etid=0;
			for(;etid<EventBase::numETIDs && args[3]!=EventBase::EventTypeNames[etid];etid++) {}
			if(etid==EventBase::numETIDs) {
				etid=0;
				for(;etid<EventBase::numETIDs && args[3]!=EventBase::EventTypeAbbr[etid];etid++) {}
				if(etid==EventBase::numETIDs) {
					etid=atoi(args[3].c_str());
					if(etid==0 && args[3]!="0") {
						serr->printf("Bad event type '%s'\n",args[3].c_str());
						return;
					}
				}
			}
			//duration field (optional, have to check args.size())
			int dur=0;
			if(args.size()>4)
				dur=atoi(args[4].c_str());
			//send event!
			if(egid==EventBase::buttonEGID && isControlling && usesButtons)
				erouter->removeTrapper(this);
			erouter->postEvent((EventBase::EventGeneratorID_t)egid,source,(EventBase::EventTypeID_t)etid,dur);
			if(egid==EventBase::buttonEGID && isControlling && usesButtons)
				erouter->addTrapper(this,EventBase::buttonEGID);
		} else if(args[0]=="!msg") {
			if(offsets.size()>1)
				erouter->postEvent(TextMsgEvent(s.substr(offsets[1]),0));
			else
				erouter->postEvent(TextMsgEvent("",0));
		} else if(args[0]=="!hello") {
			static unsigned int count=0;
			count++;
			theOneController->gui_comm->printf("hello\n%d\n",count);
		} else if(args[0]=="!root") {
			ControlBase * ret=root->takeInput(s.substr(offsets[1]));
			if(ret!=NULL)
				setNext(ret);
			// for some reason corrupts ControllerGUI's stack, so let's just always send it
			dumpStack();
		} else if(args[0]=="!hilight") {
			std::vector<unsigned int> hilights;
			for(unsigned int i=1; i<args.size(); i++)
				hilights.push_back(atoi(args[i].c_str()));
			cmdstack.top()->setHilights(hilights);
		} else if(args[0]=="!input") {
			const std::vector<unsigned int>& hilights=cmdstack.top()->getHilights();
			const std::vector<ControlBase*>& slots=cmdstack.top()->getSlots();
			std::string in = offsets.size() > 1 ? s.substr(offsets[1]) : "";
			for(unsigned int i=0; i<hilights.size(); i++)
				if(hilights[i]<slots.size() && slots[hilights[i]]!=NULL) {
					ControlBase * ret=slots[hilights[i]]->takeInput(in);
					if(ret!=NULL)
						setNext(ret);
				}
			refresh();
		} else if(args[0]=="!set") {
			setConfig(s.substr(offsets[1]).c_str());
		} else if(args[0]=="!sim") {
#ifdef PLATFORM_APERIOS
			serr->printf("!sim command invalid -- not running in simulator!\n");
#else
			ProjectInterface::sendCommand(s.substr(offsets[1]));
#endif
		} else
			setNext(cmdstack.top()->takeInput(s));
	}
}

void Controller::dumpStack() {
	theOneController->gui_comm->printf("stack_dump\n%lu\n",(unsigned long)cmdstack.size());
	//this is rather ugly - can't iterate a stack, have to unstack and restack it.  Oh well.
	std::stack< ControlBase* > tmpstack;
	while(!cmdstack.empty()) {
		tmpstack.push(cmdstack.top());
		cmdstack.pop();
	}
	while(!tmpstack.empty()) {
		theOneController->gui_comm->printf("%s\n",tmpstack.top()->getName().c_str());
		cmdstack.push(tmpstack.top());
		tmpstack.pop();
	}
}	

int Controller::setConfig(const std::string& str) {
	string::size_type eq=str.find('=');
	if(eq==string::npos)
		return -2;
	plist::ObjectBase* entry = config->resolveEntry(string_util::trim(str.substr(0,eq)));
	if(entry==NULL) {
		string::size_type p=str.find('.');
		string sec=string_util::trim(str.substr(0,p));
		string key=string_util::trim(str.substr(p+1,eq-p-1));
		string val=string_util::trim(str.substr(eq+1));
		if(config->setValue(sec,key,val)==NULL)
			return -2;
		return 0;
	}
	plist::PrimitiveBase* prim = dynamic_cast<plist::PrimitiveBase*>(entry);
	if(prim==NULL)
		return -2;
	prim->set(string_util::trim(str.substr(eq+1)));
	return 0;
}

bool Controller::setNext(ControlBase* next) {
	if(next==NULL)
		pop();
	else if(next!=cmdstack.top())
		push(next);
	return true;
}

void Controller::activate() {
#ifdef TGT_HAS_LEDS
	motman->setPriority(display,MotionManager::kEmergencyPriority);
#endif
	if(usesButtons)
		erouter->addTrapper(this,EventBase::buttonEGID);
	isControlling=true;
	if(!cmdstack.empty())
		cmdstack.top()->activate(display,gui_comm);
	else
		chkCmdStack();
}

void Controller::deactivate() {
	//these two lines help prevent residual display in case that was the only MotionCommand using LEDs
	isControlling=false;
#ifdef TGT_HAS_LEDS
	motman->setPriority(display,MotionManager::kIgnoredPriority);
	for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; i++)
		motman->setOutput(NULL,i,0.f);
#endif
	erouter->removeTrapper(this);
	cmdstack.top()->pause();
}

bool Controller::chkCmdStack() {
	if(cmdstack.empty()) {
		if(root==NULL)
			return false;
		cmdstack.push(root);
		ControlBase * next = cmdstack.top()->activate(display,gui_comm);
		if(next==NULL)
			cout << "*** WARNING Controller root returned NULL on activate!" << endl;
		else if(next!=root)
			push(next);
	}
	return true;
}


/*! @file
 * @brief Implements Controller class, a behavior that should be started whenever the emergency stop goes on to provide menus for robot control
 * @author ejt (Creator)
 */
