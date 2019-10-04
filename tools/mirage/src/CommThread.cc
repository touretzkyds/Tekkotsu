#include "CommThread.h"

#include "Shared/MarkScope.h"
#include "Shared/debuget.h"

#include <algorithm>

Thread::Lock CommThread::messageLock;
std::queue<std::pair<xmlDoc*,std::string> > CommThread::messages;
std::set<CommThread*> CommThread::closings;
std::map<std::string,std::set<CommThread*> > CommThread::owners;

Thread::Lock CommThread::instancesLock;
std::set<CommThread*> CommThread::instances;

const char* CommThread::Subscription::encodingNames[] = { "YUV", "PNG", "JPG", NULL };
INSTANTIATE_NAMEDENUMERATION_STATICS(CommThread::Encoding);

ServerThread * serverThread = NULL;

void* ServerThread::run() {
	while(true) { // will be cancelled in streamserver::accept via CommThread
		try {
			while(!streamserver.isServing() && !streamserver.serve(PORT)) {
				std::cerr << "Could not bind to server port " << PORT << " -- is another Mirage still running?  Will retry..." << std::endl;
				sleep(5);
				testCancel();
			}
			new CommThread(streamserver); // blocks until accept or throws an exception on error (or until thread cancellation)
		} catch(const std::exception& ex) {
			std::cerr << "Mirage server comm error: " << ex.what() << std::endl;
		}
	}
	return NULL;
}

void CommThread::shutdown() {
	while(instances.size()>0) {
		CommThread* th;
		{
			MarkScope autolock(instancesLock);
			if(instances.size()==0)
				break;
			th = *instances.begin();
			MarkScope ml(th->messageLock);
			th->addReference();
			th->stop(); // should remove itself from the set
		}
		th->join(); // make sure it's done
		th->removeReference();
	}
}

void CommThread::processMessages() {
	MarkScope autolock(messageLock);
	for(; !messages.empty(); messages.pop()) {
		xmlDoc * msg = messages.front().first;
		std::string clientID = messages.front().second;
		
		Client& client = Client::getIncomingClient(clientID);
		client.setParseTree(msg);
		client.readParseTree();
		client.activate();
	}
	
	for(std::set<CommThread*>::const_iterator cit=closings.begin(); cit!=closings.end(); ++cit) {
		for(std::map<std::string,std::set<CommThread*> >::iterator it=owners.begin(); it!=owners.end(); ++it)
			it->second.erase(*cit);
	}
	closings.clear();
	
	std::set<std::string> empty; // copy out those to be deleted so we don't invalide iterator
	for(std::map<std::string,std::set<CommThread*> >::iterator it=owners.begin(); it!=owners.end(); ++it) {
		if(it->second.empty()) {
			//std::cout << it->first << " is inactive " << std::endl;
			if(!Client::getClient(it->first).persist)
				empty.insert(it->first);
		}
	}
	for(std::set<std::string>::const_iterator it=empty.begin(); it!=empty.end(); ++it) {
		Client::getClient(*it).removeReference();
		size_t removed = owners.erase(*it); // probably already done by destructor, but just in case
		if(removed>0)
			std::cout << "Warning references remain for empty client " << *it << std::endl;
	}
}

CommThread::~CommThread() {
	if(isStarted())
		std::cerr << "CommThread running in destructor?!?!" << std::endl;
	if(subscription.isStarted())
		subscription.stop().join(); // do before insert to closing to ensure client isn't deleted first (race condition)
	MarkScope autolock(messageLock);
	subscription.robotID.clear(); // triggers unsubscription, should be done before client might be deleted
	closings.insert(this);
}


bool CommThread::launched() {
	MarkScope autolock(instancesLock);
	instances.insert(this);
	addReference();
	return true;
}

void * CommThread::run() {
	//std::cout << this << " Got connection from " << c.getPeerAddress().get_name() << std::endl;
	
	const size_t BUFSIZE=4*1024;
	char buf[BUFSIZE];
	xmlSAXHandler handler;
	memset(&handler,0,sizeof(handler));
	handler.startElement = startElementCallback;
	handler.endElement = endElementCallback;
	handler.characters = charactersCallback;
	xmlParserCtxt * parser = xmlCreatePushParserCtxt(&handler,this,NULL,0,NULL);
	if(parser==NULL) {
		std::cerr << "Comm thread could not create parser" << std::endl;
		return NULL;
	}
	parser->dictNames = 0; // For some reason this seems to completely break if node names are interned.
	
	while(c.read(buf,1)) { // block for more data
		c.readsome(buf+1,BUFSIZE-1); // read rest of the packet (doesn't necessarily fill buffer, just what's available...)
		testCancel();
		//std::cout << this << " buffer is '" << std::string(buf,std::min(100,c.gcount()+1)) << "'" << std::endl;
		int err = xmlParseChunk(parser,buf,c.gcount()+1,0);
		if(err!=0)
			std::cerr << "Comm thread encountered XML error " << err << std::endl;
	}
	testCancel();
	// if there was anything left on that last read, process it (and either way, tell libxml that's the end of the stream)
	//std::cout << this << " buffer is '" << std::string(buf,c.gcount()) << "'" << std::endl;
	int err = xmlParseChunk(parser,buf,c.gcount(),1);
	if(err!=0)
		std::cerr << "Comm thread encountered closing XML error " << err << std::endl;
	xmlFreeDoc(parser->myDoc);
	xmlFreeParserCtxt(parser);
	//std::cout << "Closed connection to " << c.getPeerAddress().get_name() << std::endl;
	return NULL;
}

Thread& CommThread::stop() {
	MarkScope l(stopLock);
	Thread::stop();
	// closing the socket ensures stoppage
	// (deadlock if Comm::run::testCancel(), stop(), Comm::run::read(), join()... the stop doesn't interrupt the read if it came too early)
	if(c.is_open())
		c.close();
	return *this;
}

void CommThread::dereference() {
	MarkScope autolock(instancesLock);
	instances.erase(this);
	removeReference();
}

void CommThread::startElement(const std::string& name, const xmlChar ** attrs) {
	/*if(state==SUBSCRIPTION)
		std::cout << this << " start " << name << std::endl;*/
	if(name=="messages") {
		state=MESSAGE;
		return;
	}
	if(name=="subscription") {
		state=SUBSCRIPTION;
		return;
	}
	if(state==NONE) {
		std::cerr << "Unknown wrapper " << name << std::endl;
		return;
	}
	if(name=="plist") {
		doc=xmlNewDoc((xmlChar*)"1.0");
		if (doc == NULL) 
			throw std::bad_alloc();
		node=xmlNewNode(NULL,(const xmlChar*)name.c_str());
		xmlFreeNode(xmlDocSetRootElement(doc,node));
	} else
		node=xmlNewChild(node,NULL,(const xmlChar*)name.c_str(),NULL);
	if(node==NULL)
		throw std::bad_alloc();
	if(attrs!=NULL) {
		for(; attrs[0]!=NULL && attrs[1]!=NULL; attrs+=2)
			xmlNewProp(node,attrs[0],attrs[1]);
	}
	content.clear();
}

void CommThread::endElement(const std::string& name) {
	/*if(state==SUBSCRIPTION)
		std::cout << this << " end " << name << " holding '" << content << "'" << std::endl;*/
	if(name=="messages" || name=="subscription") {
		state=NONE;
		return;
	}
	if(state==NONE)
		return;
	if(content.size()>0)
		xmlNodeSetContent(node,(const xmlChar*)content.c_str());
	content.clear();
	if(name=="plist") {
		switch(state) {
			case NONE: {
			} break;
			case MESSAGE: {
				plist::Dictionary dict;
				plist::Primitive<std::string> clientID;
				dict.addEntry("ID",clientID);
				dict.setLoadSavePolicy(plist::Collection::FIXED,plist::Collection::SYNC);
				dict.setUnusedWarning(false);
				dict.setParseTree(doc); // dict now owns the xmlDoc reference and will free it in destructor
				dict.readParseTree();
				
				if(clientID.size()==0) {
					std::cerr << "Message missing client ID:" << std::endl;
					dict.setLoadSavePolicy(plist::Collection::SYNC,plist::Collection::SYNC);
					dict.readParseTree();
					std::cerr << dict << std::endl;
				} else {
					/*dict.setLoadSavePolicy(plist::Collection::SYNC,plist::Collection::SYNC);
					 dict.readParseTree();
					 std::cerr << dict << std::endl;*/
					MarkScope autolock(messageLock);
					owners[clientID].insert(this);
					dict.stealParseTree(); // reclaim parse tree so it isn't deleted
					messages.push(std::make_pair(doc,clientID));
				}
			} break;
			case SUBSCRIPTION: {
				MarkScope autolock(subscription.lock);
				subscription.setParseTree(doc);
				subscription.readParseTree();
			} break;
		}
		doc=NULL;
		node=NULL;
	} else
		node=node->parent;
}

void CommThread::characters(const std::string& str) {
	if(node!=NULL && node->children==NULL)
		content+=str;
}


CommThread::Subscription::~Subscription() {
	ASSERT(robotID.size()==0,"Subscription destruction with robotID " << robotID);
	ASSERT(!isStarted(),"Subscription destruction while still running!");
	if(current) {
		current->removeReference();
		current=NULL;
	}
	Client::removeSensorListenerEverywhere(this);
}

void CommThread::Subscription::plistValueChanged(const plist::PrimitiveBase& pl) {
	// lock should have been obtained by caller
	if(&pl==&robotID) {
		MarkScope autolock(messageLock);
		if(robotID.getPreviousValue().size()>0)
			owners[robotID.getPreviousValue()].erase(&parent);
		unsubscribe(robotID.getPreviousValue());
		subscribe(robotID);
		if(robotID.size()>0)
			owners[robotID].insert(&parent);
	} else if(&pl==&sendSensors) {
		if(robotID.size()>0 && !isStarted()) { // will pick up setting in run() if running
			// so if not running, have to subscribe to sensors
			if(sendSensors)
				Client::getClient(robotID).addSensorListener(this);
			else
				Client::getClient(robotID).removeSensorListener(this);
		}
	} else if(&pl==&cameraIdx) {
		unsubscribe(robotID);
		subscribe(robotID);
	} else if(&pl==&singleFrame) {
		if(singleFrame)
			unsubscribe(robotID);
		else
			subscribe(robotID);
	}
}

void CommThread::Subscription::plistValueTouched(const plist::PrimitiveBase& pl) {
	if(&pl==&singleFrame)
		subscribe(robotID);
}

void CommThread::Subscription::plistCollectionEntryAdded(Collection& /*col*/, ObjectBase& /*primitive*/) {
	if(!isStarted()) {
		plist::ArrayOf<CameraInfo>& cameras = Client::getClient(robotID).cameras;
		if(cameraIdx<cameras.size()) {
			cameras[cameraIdx].addImageListener(this);
			start();
		}
	}
}
void CommThread::Subscription::plistCollectionEntryRemoved(Collection& /*col*/, ObjectBase& /*primitive*/) {
	if(isStarted() && cameraIdx>=Client::getClient(robotID).cameras.size())
		stop();
}
void CommThread::Subscription::plistCollectionEntriesChanged(Collection& /*col*/) {
	plist::ArrayOf<CameraInfo>& cameras = Client::getClient(robotID).cameras;
	if(isStarted()) {
		if(cameraIdx>=cameras.size())
			stop();
	} else if(cameraIdx<cameras.size()) {
		cameras[cameraIdx].addImageListener(this);
		start();
	}
}

void CommThread::Subscription::subscribe(const std::string& rid) {
	if(rid.size()>0) {
		plist::ArrayOf<CameraInfo>& cameras = Client::getIncomingClient(rid).cameras;
		cameras.addCollectionListener(this);
		if(cameraIdx<cameras.size()) {
			cameras[cameraIdx].addImageListener(this);
			if(!isStarted())
				start();
		} else if(isStarted()) {
			stop();
		} else if(sendSensors) {
			Client::getClient(rid).addSensorListener(this);
		}
	} else if(isStarted()) {
		stop();
	}
}

void CommThread::Subscription::unsubscribe(const std::string& rid) {
	if(rid.size()>0) {
		Client::getClient(rid).removeSensorListener(this);
		plist::ArrayOf<CameraInfo>& cameras = Client::getClient(rid).cameras;
		if(cameraIdx<cameras.size())
			cameras[cameraIdx].removeImageListener(this);
		cameras.removeCollectionListener(this);
	}
}

void CommThread::Subscription::renderedImage(const CameraInfo& cam, ImageBuffer * img) {
	{
		MarkScope autolock(lock);
		if(next!=NULL)
			next->removeReference();
		next = img;
		if(next!=NULL)
			next->addReference();
	}
	newImage.signal();
	if(singleFrame)
		unsubscribe(robotID);
}

void CommThread::Subscription::sensorsUpdated(const DynamicRobotState& values) {
	ASSERTRET(!isStarted(),"sensorsUpdated while thread is running");
	ASSERTRET(sendSensors,"sensorsUpdated but sendSensors is false!");
	values.saveStream(c,true);
	c.flush();
	if(singleFrame)
		Client::getClient(robotID).removeSensorListener(this);
}

void* CommThread::Subscription::run() {
	std::cout << "Subscription " << robotID << ":" << cameraIdx << " started" << std::endl;
	// we'll be sending from this thread, don't interfere from sensors updated callback in main thread
	Client::getClient(robotID).removeSensorListener(this);
	plist::DictionaryBase * sensorValues = &Client::getClient(robotID).sensorValues;
	while(true) {
		testCancel();
		if(current!=NULL) {
			bool valid=false;
			const char * buf;
			size_t len;
			switch(*encoding) {
				case ENCODE_YUV: valid = current->getYUV(buf,len); break;
				case ENCODE_PNG: valid = current->getPNG(buf,len,pngCompressionLevel); break;
				case ENCODE_JPG: valid = current->getJPG(buf,len,jpgQualityLevel); break;
			}
			if(valid) {
				c.write(buf,len);
				if(sendSensors)
					sensorValues->saveStream(c,true);
				if(!c.flush())
					return NULL;
				testCancel();
			}
		}
		MarkScope autolock(lock);
		//std::cout << "done writing " << current << std::endl;
		if(current!=NULL)
			current->removeReference();
		current=next;
		next=NULL;
		if(current==NULL) {
			newImage.wait(lock);
			current=next;
			next=NULL;
		}
		//std::cout << "writing " << current << std::endl;
	}
}

void CommThread::Subscription::cancelled() {
	if(robotID.size()>0) {
		std::cout << "Subscription " << robotID << ":" << cameraIdx << " ended" << std::endl;
		if(sendSensors) // no longer sending from thread, restart independent listener
			Client::getClient(robotID).addSensorListener(this);
	}
	//robotID.clear();
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
