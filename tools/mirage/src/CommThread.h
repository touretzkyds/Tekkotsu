//-*-c++-*-
#ifndef INCLUDED_CommThread_h_
#define INCLUDED_CommThread_h_

#include "Client.h"
#include "ImageBuffer.h"

#include "IPC/Thread.h"
#include "Shared/ReferenceCounter.h"
#include "Wireless/netstream.h"

#include <set>
#include <queue>
#include <libxml/parser.h>

//! Opens a series of CommThreads as connections come in
class ServerThread : public Thread {
public:
	static const unsigned short PORT=19785; //!< port to listen on
	ServerThread() : Thread(), streamserver(PORT) {} //!< constructor, waits for a call to start()
protected:
	netstream_server streamserver; //!< accepts connections for duraction of ServerThread scope
	void* run(); //!< creates a netstream_server and pulls connections into CommThread instances
};
extern ServerThread * serverThread;


//! Maintains a network connection, parsing the XML messages, queuing them up until until polled by the graphics thread
/*! Ogre3D apparently doesn't handle multi-thread access to its data structures, so all graphics operations
 *  need to be done from the same thread.  Hence #messages queues incoming data until the graphics
 *  thread calls processMessages() to distribute them to appropriate Client instances.
 *
 *  Each CommThread will spawn a new child thread once a connection is made, so you only need to make one
 *  CommThread instance to kick things off.  CommThreads will delete themselves when their connection closes. */
class CommThread : virtual public ReferenceCounter, public Thread {
public:
	//! constructor
	CommThread(netstream_server& s) : ReferenceCounter(), Thread(), c(), state(NONE), doc(NULL), node(NULL), content(), subscription(*this,c)
	{
		if(!s.accept(c)) {
			testCancel();
			throw std::ios_base::failure("server shut down");
		}
		start(); 
	}
	
	//! closes all CommThreads
	static void shutdown();
	
	//! call from graphics thread to cause queued #messages to be sent to Clients
	static void processMessages();
	
	static size_t numConnections() { return instances.size(); }
	
	static void dropClient(const std::string& cl) {
		MarkScope autolock(messageLock);
		owners.erase(cl);
	}
	
protected:
	~CommThread();
	
	ionetstream c;
	enum State { NONE, MESSAGE, SUBSCRIPTION } state;
	xmlDoc* doc;
	xmlNode* node;
	std::string content;
	
	enum Encoding { ENCODE_YUV, ENCODE_PNG, ENCODE_JPG };
	friend class plist::NamedEnumeration<Encoding>;
	
	class Subscription : virtual public plist::Dictionary, public Thread, protected plist::PrimitiveListener, 
		protected plist::CollectionListener, protected CameraInfo::ImageListener, protected Client::SensorListener
	{
	public:
		Subscription(CommThread& p, ionetstream& comm) : plist::Dictionary(), Thread(), plist::PrimitiveListener(), plist::CollectionListener(), CameraInfo::ImageListener(),
			lock(), robotID(), sendSensors(false), cameraIdx(-1U), singleFrame(false), pngCompressionLevel(2), jpgQualityLevel(85), encoding(ENCODE_JPG,encodingNames),
			newImage(), current(NULL), next(NULL), c(comm), parent(p)
		{
			addEntry("ID",robotID);
			addEntry("SendSensors",sendSensors);
			addEntry("CameraIndex",cameraIdx);
			addEntry("SingleFrame",singleFrame);
			addEntry("Encoding",encoding);
			addEntry("PNGLevel",pngCompressionLevel);
			addEntry("JPGQuality",jpgQualityLevel);
			robotID.addPrimitiveListener(this);
			sendSensors.addPrimitiveListener(this);
			cameraIdx.addPrimitiveListener(this);
			singleFrame.addPrimitiveListener(this);
			setLoadSavePolicy(FIXED,SYNC);
		}
		~Subscription();
		
		Thread::Lock lock;
		
		plist::Primitive<std::string> robotID;
		plist::Primitive<bool> sendSensors;
		plist::Primitive<unsigned int> cameraIdx;
		plist::Primitive<bool> singleFrame;
		plist::Primitive<unsigned int> pngCompressionLevel;
		plist::Primitive<unsigned int> jpgQualityLevel;
		
		static const char* encodingNames[];
		plist::NamedEnumeration<Encoding> encoding;
		
	protected:
		virtual void plistValueChanged(const plist::PrimitiveBase& pl);
		virtual void plistValueTouched(const plist::PrimitiveBase& pl);
		virtual void plistCollectionEntryAdded(Collection& /*col*/, ObjectBase& /*primitive*/);
		virtual void plistCollectionEntryRemoved(Collection& /*col*/, ObjectBase& /*primitive*/);
		virtual void plistCollectionEntriesChanged(Collection& /*col*/);
		virtual void subscribe(const std::string& rid);
		virtual void unsubscribe(const std::string& rid);
		virtual void renderedImage(const CameraInfo& cam, ImageBuffer * img);
		virtual void sensorsUpdated(const DynamicRobotState& values);
		virtual void* run();
		virtual void cancelled();
		
		Thread::Condition newImage;
		ImageBuffer * current;
		ImageBuffer * next;
		ionetstream& c;
		CommThread& parent;
	} subscription;
	
	static Thread::Lock messageLock;
	static std::queue<std::pair<xmlDoc*,std::string> > messages; //!< incoming messages waiting to be processed
	static std::set<CommThread*> closings; //!< network connections which have been closed during current buffer accumulation
	
	//! The network connections which have contributed data for each client
	/*! This allows multiple connections to share an object... the object is considered 'active' as long
	  *  as it has at least one previous contributor still connected */
	static std::map<std::string,std::set<CommThread*> > owners;
	
	virtual bool launched();
	virtual void * run();
	virtual Thread& stop();
	virtual void dereference();
	
	static void startElementCallback(void * ctx, const xmlChar * name, const xmlChar ** attrs) {
		static_cast<CommThread*>(ctx)->startElement((char*)name,attrs);
	}
	void startElement(const std::string& name, const xmlChar ** attrs);	
	
	static void endElementCallback(void * ctx, const xmlChar * name) {
		static_cast<CommThread*>(ctx)->endElement((char*)name);
	}
	void endElement(const std::string& name);
	
	static void charactersCallback(void * ctx, const xmlChar * ch, int len) {
		static_cast<CommThread*>(ctx)->characters(std::string((char*)ch,len));
	}
	void characters(const std::string& str);
	
	
	static Thread::Lock instancesLock;
	static std::set<CommThread*> instances;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
