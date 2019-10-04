#include "EventLogger.h"
#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif
#include "ValueEditControl.h"
#include "StringInputControl.h"
#include "NullControl.h"
#include <sstream>
#include "Sound/SoundManager.h"
#include "Vision/FilterBankGenerator.h"
#include "Vision/JPEGGenerator.h"
#include "Shared/Base64.h"
#include "Behaviors/StateNode.h"
#include "Behaviors/Transition.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

Socket* EventLogger::logSocket=NULL;
unsigned int EventLogger::logSocketRefCount=0;
int EventLogger::port=10080;
EventLogger * EventLogger::theOne=NULL;
EventLogger::queuedEvents_t EventLogger::queuedEvents;
EventLogger::transStack_t EventLogger::transStack;

EventLogger::StateMachineListener EventLogger::smProcess;

EventLogger::EventLogger()
	: ControlBase("Event Logger","Allows you to see/log all of the un-trapped events as they are generated"),
		logfilePath(), logfile(), verbosity(0), listen() {
	for(unsigned int i=0; i<EventBase::numEGIDs; i++) {
		std::string tmp=EventBase::EventGeneratorNames[i];
		pushSlot(new NullControl(("[ ] "+tmp).c_str(),"Show/hide events from "+tmp));
	}
	pushSlot(NULL);
	pushSlot(new ValueEditControl<unsigned int>("Verbosity","Controls verbosity level: 0=(gen,source,type); 1=0+gen_id,source_id,type_id; 2=1+duration,timestamp; 3=2+magnitude; additional columns may be added for subclass info","Please enter a new verbosity level: 0=(gen,source,type); 1=0+gen_id,source_id,type_id; 2=1+duration,timestamp; 3=2+magnitude; additional columns may be added for subclass info",&verbosity));
	pushSlot(new ControlBase("[X] Console Output","If selected, outputs events to the console"));
	pushSlot(new StringInputControl("[ ] File Output","Please enter the filename to log to (in /ms/...)"));
	if(logSocket==NULL) {
		theOne=this;
		ASSERT(logSocketRefCount==0,"logSocket is NULL, ref count is non-zero");
		logSocket=wireless->socket(Socket::SOCK_STREAM,1024,1<<15);
		wireless->setDaemon(logSocket);
		wireless->setReceiver(logSocket, callback);
		wireless->listen(logSocket,port);
	}
	logSocketRefCount++;
}

EventLogger::~EventLogger() {
	while(!queuedEvents.empty())
		queuedEvents.pop();
	clearSlots();
	if(--logSocketRefCount==0) {
		wireless->setDaemon(logSocket,false);
		wireless->close(logSocket);
		logSocket=NULL;
	}
	if(theOne==this)
		theOne=NULL;
}

ControlBase* EventLogger::doSelect() {
	ControlBase* ans=this;
	for(unsigned int i=0; i<hilights.size(); i++) {
		unsigned int cur=hilights[i];
		if(cur<EventBase::numEGIDs) {
			if(options[cur]->getName()[1]!=' ') {
				erouter->removeListener(this,(EventBase::EventGeneratorID_t)(cur));
				setStatus(cur,' ');
			} else {
				erouter->addListener(this,(EventBase::EventGeneratorID_t)(cur));
				setStatus(cur,'X');
			}
		} else if(cur==EventBase::numEGIDs+1) {
			ans=options[cur];
		} else if(cur==EventBase::numEGIDs+2) {
			if(options[cur]->getName()[1]!=' ') {
				setStatus(cur,' ');
			} else {
				setStatus(cur,'X');
			}
		} else if(cur==EventBase::numEGIDs+3) {
			if(options[cur]->getName()[1]!=' ') {
				logfile.close();
				options[cur]->setName("[ ] File Output");
			} else {
				ans=options[cur];
			}
		}
		sndman->playFile(config->controller.select_snd);
	}
	if(ans==this)
		refresh();
	return ans;
}

void EventLogger::refresh() {
	checkLogFile();
	ControlBase::refresh();
}

//!sends all events received to stdout and/or logfile
void EventLogger::processEvent(const EventBase& event) {
	std::string logdata = event.getDescription(true,verbosity);
	if(options[EventBase::numEGIDs+2]->getName()[1]=='X')
		sout->printf("EVENT: %s\n",logdata.c_str());
	if(logSocket!=NULL && wireless->isConnected(logSocket->sock)) {
		xmlNode * cur = xmlNewNode(NULL,(const xmlChar*)"");
		xmlSetProp(cur,(const xmlChar*)"type",(const xmlChar*)"log");
		xmlNode * desc = xmlNewNode(NULL,(const xmlChar*)"param");
		event.saveXML(cur);
		xmlAddChild(cur,desc);
		xmlSetProp(desc,(const xmlChar*)"name",(const xmlChar*)"description");
		xmlSetProp(desc,(const xmlChar*)"value",(const xmlChar*)event.getDescription(true,3).c_str());
		xmlBuffer* buf=xmlBufferCreate();
		xmlDoc * doc = xmlNewDoc((const xmlChar*)"1.0");
		int n=xmlNodeDump(buf,doc,cur,0,1);
		xmlFreeNode(cur);
		xmlFreeDoc(doc);
		byte * nbuf = logSocket->getWriteBuffer(n+1);
		if(nbuf!=NULL) {
			memcpy(nbuf,xmlBufferContent(buf),n);
			nbuf[n]='\n';
			logSocket->write(n+1);
		}
		xmlBufferFree(buf);
	}
	checkLogFile();
	if(logfile)
		logfile << logdata << std::endl;
}

void EventLogger::logImage(FilterBankGenerator& fbg, unsigned int layer, unsigned int channel, const BehaviorBase* source/*=NULL*/) {
	if(logSocket==NULL || !wireless->isConnected(logSocket->sock))
		return;

	char * binbuf;
	unsigned int len;
	if(JPEGGenerator* jpeg=dynamic_cast<JPEGGenerator*>(&fbg)) {
		binbuf=(char*)jpeg->getImage(layer,channel);
		len=jpeg->getImageSize(layer,channel);
	} else {
		fbg.selectSaveImage(layer,channel);
		len=fbg.getBinSize();
		binbuf=new char[len];
		fbg.saveBuffer(binbuf,len);
	}
	std::string b64buf=base64::encode(binbuf,len);
	if(binbuf!=(char*)fbg.getImage(layer,channel)) //cached, should be a simple return
		delete [] binbuf;
	
	xmlNode * cur = xmlNewNode(NULL,(const xmlChar*)"event");
	xmlSetProp(cur,(const xmlChar*)"type",(const xmlChar*)"image");
	if(source!=NULL)
		xmlSetProp(cur,(const xmlChar*)"sid",(const xmlChar*)source->getName().c_str());
	char timebuf[20];
	snprintf(timebuf,20,"%d",get_time());
	xmlSetProp(cur,(const xmlChar*)"time",(const xmlChar*)timebuf);
	xmlNewChild(cur,NULL,(const xmlChar*)"image",(const xmlChar*)b64buf.c_str());
	queuedEvents.push(cur);
	if(transStack.empty())
		dumpQueuedEvents();
}

void EventLogger::logMessage(std::string msg, const BehaviorBase* source/*=NULL*/, const char* icon/*=NULL*/, unsigned int placement/*=0*/) {
	if(logSocket==NULL || !wireless->isConnected(logSocket->sock))
		return;

	xmlNode * cur = xmlNewNode(NULL,(const xmlChar*)"event");
	xmlSetProp(cur,(const xmlChar*)"type",(const xmlChar*)"userlog");
	if(source!=NULL)
		xmlSetProp(cur,(const xmlChar*)"sid",(const xmlChar*)source->getName().c_str());
	if(icon!=NULL)
		xmlSetProp(cur,(const xmlChar*)"icon",(const xmlChar*)icon);
	const unsigned int len=20;
	char sbuf[len];
	snprintf(sbuf,len,"%d",placement);
	xmlSetProp(cur,(const xmlChar*)"voff",(const xmlChar*)sbuf);
	snprintf(sbuf,len,"%d",get_time());
	xmlSetProp(cur,(const xmlChar*)"time",(const xmlChar*)sbuf);
	xmlNodeSetContent(cur,(const xmlChar*)msg.c_str());
	queuedEvents.push(cur);
	if(transStack.empty())
		dumpQueuedEvents();
}

void EventLogger::logWebcam(const BehaviorBase* source/*=NULL*/) {
	if(logSocket==NULL || !wireless->isConnected(logSocket->sock))
		return;

	xmlNode * cur = xmlNewNode(NULL,(const xmlChar*)"event");
	xmlSetProp(cur,(const xmlChar*)"type",(const xmlChar*)"webcam");
	if(source!=NULL)
		xmlSetProp(cur,(const xmlChar*)"sid",(const xmlChar*)source->getName().c_str());
	const unsigned int len=20;
	char sbuf[len];
	snprintf(sbuf,len,"%d",get_time());
	xmlSetProp(cur,(const xmlChar*)"time",(const xmlChar*)sbuf);
	xmlNodeSetContent(cur,(const xmlChar*)" ");
	queuedEvents.push(cur);
	if(transStack.empty())
		dumpQueuedEvents();
}

void EventLogger::clearSlots() {
	erouter->removeListener(this);
	ControlBase::clearSlots();
}

void EventLogger::setStatus(unsigned int i, char c) {
	std::string tmp=options[i]->getName();
	tmp[1]=c;
	options[i]->setName(tmp);
}

void EventLogger::checkLogFile() {
	unsigned int cur=EventBase::numEGIDs+3;
	StringInputControl * strin=dynamic_cast<StringInputControl*>(options[cur]);
	ASSERTRET(strin!=NULL,"The StringInputControl is misplaced");
	if(strin->getLastInput()!=logfilePath) {
		logfile.close();
		logfilePath=strin->getLastInput();
		logfile.clear();
		if(logfilePath.size()!=0) {
			sout->printf("Opening `%s'\n",(config->portPath(logfilePath)).c_str());
			logfile.open((config->portPath(logfilePath)).c_str());
			if(!logfile.fail()) {
				setStatus(cur,'X');
				strin->setName(strin->getName()+": "+logfilePath);
			} else {
				serr->printf("Opening `%s' failed\n",(config->portPath(logfilePath)).c_str());
			}
		}
	}
}


void EventLogger::spider(const StateNode* n, xmlNode* parent/*=NULL*/) {
	if(n==NULL)
		return;

	xmlNode * cur = xmlNewNode(NULL,(const xmlChar*)"state");
	ASSERTRET(cur!=NULL,"EventLogger::spider() could not allocate new xml state node");
	xmlSetProp(cur,(const xmlChar*)"class",(const xmlChar*)n->getClassName().c_str());
	xmlSetProp(cur,(const xmlChar*)"id",(const xmlChar*)n->getName().c_str());
	
	const std::vector<StateNode*>& subnodes=n->getNodes();
	if(subnodes.size()>0) {
		// it's not a leaf node, has subnodes and transitions between them

		std::set<const Transition*> transitions;
		// now recurse on sub-nodes, extracting all of the subnodes transitions
		for(unsigned int i=0; i<subnodes.size(); i++) {
			spider(subnodes[i],cur);
			const std::vector<Transition*>& curt=subnodes[i]->getTransitions();
			transitions.insert(curt.begin(),curt.end());
		}

		// now output transitions between subnodes we collected in previous step
		for(std::set<const Transition*>::const_iterator it=transitions.begin(); it!=transitions.end(); it++) {
			xmlNode * t = xmlAddChild(cur,xmlNewNode(NULL,(const xmlChar*)"transition"));
			ASSERTRET(t!=NULL,"EventLogger::spider() could not allocate new xml transition node");
			xmlSetProp(t,(const xmlChar*)"class",(const xmlChar*)(*it)->getClassName().c_str());
			xmlSetProp(t,(const xmlChar*)"id",(const xmlChar*)(*it)->getName().c_str());
			
			typedef std::vector<StateNode*> statevec_t;
			const statevec_t& incoming=(*it)->getSources();
			for(statevec_t::const_iterator nit=incoming.begin(); nit!=incoming.end(); ++nit) {
				xmlNode * sn = xmlAddChild(t,xmlNewNode(NULL,(const xmlChar*)"source"));
				ASSERTRET(sn!=NULL,"EventLogger::spider() could not allocate new xml transition source node");
				xmlNodeSetContent(sn,(const xmlChar*)(*nit)->getName().c_str());
			}
			const statevec_t& outgoing=(*it)->getDestinations();
			for(statevec_t::const_iterator nit=outgoing.begin(); nit!=outgoing.end(); ++nit) {
				xmlNode * sn = xmlAddChild(t,xmlNewNode(NULL,(const xmlChar*)"destination"));
				ASSERTRET(sn!=NULL,"EventLogger::spider() could not allocate new xml transition source node");
				xmlNodeSetContent(sn,(const xmlChar*)(*nit)->getName().c_str());
			}
		}
	}
	if(parent==NULL)
		dumpNode(cur);
	else
		xmlAddChild(parent,cur);
}
	
bool EventLogger::isListening(const StateNode* n) {
	while(n!=NULL) {
		if(listen.find(n->getName())!=listen.end())
			return true;
		n=n->getParent();
	}
	return false;
}

void EventLogger::indent(unsigned int level) {
	for(unsigned int i=0; i<level; i++)
		logSocket->printf("  ");
}

const StateNode * EventLogger::find(const std::string& sname) {
	const registry_t& registry=BehaviorBase::getRegistry();
	for(registry_t::const_iterator it=registry.begin(); it!=registry.end(); it++) {
		const StateNode * cur=dynamic_cast<const StateNode*>(*it);
		if(cur!=NULL && cur->getName()==sname)
			return cur;
	}
	//serr->printf("WARNING: EventLogger Could not find StateNode named `%s'\n",sname.c_str());
	return NULL;
}

void EventLogger::runCommand(const std::string& s) {
	if(s==std::string("list")) {
		const registry_t& registry=BehaviorBase::getRegistry();
		unsigned int numstate=0;
		for(registry_t::const_iterator it=registry.begin(); it!=registry.end(); it++) {
			const StateNode * cur=dynamic_cast<const StateNode*>(*it);
			if(cur!=NULL)
				numstate++;
		}
		logSocket->printf("%d\n",numstate);
		for(registry_t::const_iterator it=registry.begin(); it!=registry.end(); it++) {
			const StateNode * cur=dynamic_cast<const StateNode*>(*it);
			if(cur!=NULL)
				logSocket->printf("%s\n",cur->getName().c_str());
		}

	} else if(s.find("spider ")==0) {
		const StateNode * n=find(s.substr(7));
		if(n==NULL) {
			serr->printf("WARNING: EventLogger could not find \"%s\" for spidering\n",s.substr(7).c_str());
			logSocket->printf("<model></model>\n");
		} else {
			logSocket->printf("<model>\n");
			spider(n);
			logSocket->printf("</model>\n");
		}

	} else if(s.find("listen ")==0) {
		if(listen.size()==0) {
			erouter->addListener(&smProcess,EventBase::stateMachineEGID);
			erouter->addListener(&smProcess,EventBase::stateTransitionEGID);
		}
		listen.insert(s.substr(7));

	} else if(s.find("ignore ")==0) {
		listen.erase(s.substr(7));
		if(listen.size()==0)
			erouter->removeListener(&smProcess);

	} else if(s=="clear") {
		listen.clear();
		erouter->removeListener(&smProcess);

	} else {
		serr->printf("EventLogger::runCommand() - bad message: '%s'\n",s.c_str());
	}
}

// The command packet reassembly mechanism
int EventLogger::callback(char *buf, int bytes) {
	if(EventLogger::theOne==NULL)
		return 0;
	static std::string cmd;
	for(int i=0; i<bytes; i++) {
		if(buf[i]=='\n') {
			EventLogger::theOne->runCommand(cmd);
			cmd.clear();
		} else if(buf[i]!='\r')
			cmd+=buf[i];
	}
  return 0;
}

void EventLogger::processStateMachineEvent(const EventBase& e) {
	if(!wireless->isConnected(logSocket->sock) || listen.size()==0)
		return;
	
	if(e.getGeneratorID()==EventBase::stateTransitionEGID) {
		bool care=false;
		const Transition * trans = reinterpret_cast<Transition*>(e.getSourceID());
		const std::vector<StateNode*>& incoming=trans->getSources();
		const std::vector<StateNode*>& outgoing=trans->getDestinations();
		for(std::vector<StateNode*>::const_iterator it=incoming.begin(); it!=incoming.end() && !care; it++)
			care=isListening(*it);
		for(std::vector<StateNode*>::const_iterator it=outgoing.begin(); it!=outgoing.end() && !care; it++)
			care=isListening(*it);
		if(!care)
			return;
		
		if(e.getTypeID()==EventBase::activateETID) {
			xmlNode * root = xmlNewNode(NULL,(const xmlChar*)"event");
			xmlNode * fire = xmlAddChild(root,xmlNewNode(NULL,(const xmlChar*)"fire"));
			xmlSetProp(fire,(const xmlChar*)"id",(const xmlChar*)trans->getName().c_str());
			const unsigned int len=20;
			char sbuf[len];
			snprintf(sbuf,len,"%d",e.getTimeStamp());
			xmlSetProp(fire,(const xmlChar*)"time",(const xmlChar*)sbuf);
			transStack.push(root);
			queuedEvents.push(root);
		} else {
			ASSERTRET(!transStack.empty(),"got a transition deactivate that I should care about, but I didn't see the activate");
			transStack.pop();
			if(transStack.empty())
				dumpQueuedEvents();
		}
		
	} else if(e.getGeneratorID()==EventBase::stateMachineEGID) {
		if(e.getTypeID()==EventBase::statusETID)
			return;
		const StateNode * beh=reinterpret_cast<StateNode*>(e.getSourceID());
		if(!isListening(beh))
			return;
		if(e.getTypeID()!=EventBase::activateETID && e.getTypeID()!=EventBase::deactivateETID) {
			serr->printf("WARNING: Unrecognized TypeID %d\n",e.getTypeID());
			return;
		}
		
		xmlNode * root = transStack.empty() ? xmlNewNode(NULL,(const xmlChar*)"event") : transStack.top();
		const char* sttypestr = (e.getTypeID()==EventBase::activateETID) ? "statestart" : "statestop";
		xmlNode * st = xmlAddChild(root,xmlNewNode(NULL,(const xmlChar*)sttypestr));
		xmlSetProp(st,(const xmlChar*)"id",(const xmlChar*)beh->getName().c_str());
		const unsigned int len=20;
		char sbuf[len];
		snprintf(sbuf,len,"%d",e.getTimeStamp());
		xmlSetProp(st,(const xmlChar*)"time",(const xmlChar*)sbuf);
		
		if(transStack.empty()) {
			queuedEvents.push(root);
			dumpQueuedEvents();
		}
		
	} else {
		serr->printf("WARNING: Unknown event %s (%s)\n",e.getName().c_str(),e.getDescription().c_str());
	}
}

void EventLogger::dumpQueuedEvents() {
	xmlDoc * doc = xmlNewDoc((const xmlChar*)"1.0");
	while(!queuedEvents.empty()) {
		dumpNode(queuedEvents.front(),doc);
		queuedEvents.pop();
	}
	xmlFreeDoc(doc);
}

void EventLogger::dumpNode(xmlNode* node, xmlDoc* doc/*=NULL*/) {
	xmlBuffer* buf=xmlBufferCreate();
	int n=xmlNodeDump(buf,doc,node,0,1);
	byte * nbuf = logSocket->getWriteBuffer(n+1);
	if(nbuf!=NULL) {
		memcpy(nbuf,xmlBufferContent(buf),n);
		nbuf[n]='\n';
		logSocket->write(n+1);
	}
	xmlBufferFree(buf);
	xmlFreeNode(node);
}




/*! @file
 * @brief Describes EventLogger, which allows logging of events to the console or a file
 * @author ejt (Creator)
 */
