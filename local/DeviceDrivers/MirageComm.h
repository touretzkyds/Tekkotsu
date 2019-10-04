#include "Motion/KinematicJoint.h"
#include "Wireless/netstream.h"
#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Shared/debuget.h"

//! Handles communication with Mirage to place and control an object in the environment
/*! The MirageComm constructor will open a connection to Mirage (if not already open).
 *  Then call setName() to register a name for your object.  Finally, assign a KinematicJoint via
 *  setKinematics() to provide information to configure and display your object.
 *
 *  You can reuse the MirageComm instance for multiple objects in Mirage, simply call setName()
 *  to change the name of the object being controlled.  Alternatively, you can use multiple
 *  MirageComms with a single shared network connection.  Each MirageComm tracks the
 *  name of the object is controlling and sends this state in each update to Mirage.
 *
 *  Parameters assigned by MirageComm are buffered until MirageComm
 *  hits its destructor, setName(), or flush() is called.
 *
 *  Objects are kept in the Mirage simulation until all network connections associated
 *  with that object are closed.  However, if setPersist(true) is called, the object will be kept
 *  active even if all associated network connections are closed.
 *
 *  @code
 *  #include "local/DeviceDrivers/MirageComm.h"
 *  #include "Wireless/netstream.h"
 *  ionetstream net;
 *  MirageComm mirage(net); // without arguments, makes connection to "localhost"
 *  mirage.setName("target");
 *  mirage.setPersist(true); // so we can close net without removing object
 *  KinematicJoint * k = new KinematicJoint; // this will hold object description
 *  k->model="Sphere";
 *  k->material="Pink";
 *  k->modelScale=plist::Point(15,15,15);
 *  mirage.setKinematics(k); // mirage will delete k after the flush
 *  mirage.setPosition(500,0,0); // place object at (500,0,0) mm
 *  // mirage destructor will automatically flush assigned attributes
 *  //   at end of scope, or you can call mirage.flush()
 *  @endcode
 *
 *  When you want to close the connection (e.g. remove the object "target" from previous example)
 *  just close the ionetstream connection.  If you want to be "nice" about this, let MirageComm do it:
 *  @code
 *  ionetstream net; // or continue 'net' scope from above to avoid connection overhead
 *  MirageComm mirage(net);
 *  mirage.setName("target");
 *  mirage.setPersist(false); // disable persistance, object will be removed when 'net' is closed
 *  mirage.close(); // sends XML closing tags to make Mirage's parser feel warm and fuzzy
 *  @endcode
 *
 *  Note that if you can match the network connection scope to the desired lifetime of the
 *  object in Mirage (e.g. make 'net' a class member of your object's controller), then you don't need to call setPersist(),
 *  and can simply rely on objects being "garbage collected" when you eventually close the connection.
 */
class MirageComm {
public:
	//! Constructor, if @a io is not already connected, it will reconnect to @a host and @a port
	/*! If the connection fails, no status is returned, check io.is_open() or the MirageComm bool cast */
	MirageComm(ionetstream& io, const std::string& host="localhost", unsigned short port=19785) : comm(io), name(), msg() {
		if(!comm.is_open())
			connect(host,port);
	}
	
	//! destructor
	~MirageComm() { if(msg.size()>0) flush(); }
	
	//! opens connection to Mirage
	bool connect(const std::string& host, unsigned short port=19785) {
		if(!comm.open(IPaddr(host,port)))
			return false;
		comm.clear();
		comm << "<messages>" << std::endl;
		return true;
	}
	
	//! Closes connection to Mirage.
	/*! Any objects which have not had setPersist(true) and have not been "touched"
	 *  by other connections will be removed from the environment. */
	void close() {
		if(comm.is_open()) {
			flush();
			comm << "</messages>" << std::endl;
			comm.close();
		}
	}
	
	//! Indicates which object is being controlled.
	/*! If an object with this name already exists, subsequent 'set' calls will control that object.
	 *  If no such object exists, a new one will be created.
	 *  This implies a flush if any parameters have been set under the old name. */
	MirageComm& setName(const std::string& n) {
		if(name.size()>0) //! If no name has been set yet, inherit parameters (i.e. doesn't flush parameters without a name)
			flush();
		name = n;
		return *this;
	}
	
	//! If passed 'true', this will prevent the current object from being removed when the connection is closed.
	MirageComm& setPersist(bool persist) {
		msg.addValue("Persist",persist);
		return *this;
	}
	
	//! Assigns kinematic information to define the display and movement of the object, will delete @a kj after flush!
	/*! This will delete @a kj after the next flush, so pass the result of kj->clone() if you don't want to lose your instance */
	MirageComm& setKinematics(KinematicJoint* kj) {
		KinematicJointSaver * kinSaver = new KinematicJointSaver(*kj,true);
		msg.addEntry("Model",kinSaver);
		return *this;
	}
	
	//! Assigns a value to a specific field of the root kinematic object
	/*! Many times no joints are in use, and you only want to modify a subset of fields, 
	 *  without respecifying the kinematic description—this will let you do it. */
	template<class T>
	MirageComm& setKinematicProperty(const std::string& paramName, const T& val) {
		plist::Dictionary::const_iterator it = msg.findEntry("Model");
		if(it!=msg.end()) {
			// TODO there should be an array wrapping this I think?
			plist::Dictionary& kd = dynamic_cast<plist::Dictionary&>(*it->second);
			plist::Dictionary::const_iterator it2 = kd.findEntry(paramName);
			if(it2!=kd.end()) {
				std::stringstream ss;
				ss << val;
				plist::PrimitiveBase& prim = dynamic_cast<plist::PrimitiveBase&>(*it2->second);
				prim.set(ss.str());
			} else {
				kd.addValue(paramName,val);
			}
		} else {
			// TODO should this dict be wrapped in an array?!?
			plist::Dictionary * kd = new plist::Dictionary;
			kd->addValue(paramName,val);
			msg.addEntry("Model",kd);
		}
		return *this;
	}
	
	//! Controls position of the object
	MirageComm& setPosition(float x, float y, float z) { 
		msg.addEntry("Location",new plist::Point(x,y,z));
		return *this;
	}
	//! Controls position of the object
	MirageComm& setPosition(const fmat::Column<3>& x) {
		msg.addEntry("Location",new plist::Point(x[0],x[1],x[2]));
		return *this;
	}
	//! Controls position of the object
	MirageComm& setPosition(const fmat::SubVector<3>& x) {
		msg.addEntry("Location",new plist::Point(x[0],x[1],x[2]));
		return *this;
	}
	//! Controls position of the object
	MirageComm& setPosition(const fmat::SubVector<3,const fmat::fmatReal>& x) {
		msg.addEntry("Location",new plist::Point(x[0],x[1],x[2]));
		return *this;
	}
	//! Controls position of the object
	MirageComm& setPosition(const plist::Point& x) {
		msg.addEntry("Location",x.clone());
		return *this;
	}
	
	//! Controls orientation of the object
	MirageComm& setOrientation(const fmat::Quaternion& q) {
		plist::Point* pq = new plist::Point;
		q.exportTo((*pq)[0],(*pq)[1],(*pq)[2]);
		msg.addEntry("Orientation",pq);
		return *this;
	}
	
	//! Will cause previous 'set' calls to be transmitted to Mirage
	bool flush() {
		if(msg.size()==0)
			return true;
		ASSERTRETVAL(name.size()>0,"flushing without name",false);
		msg.addEntry("ID",name);
		msg.saveStream(comm,true);
		msg.clear();
		return comm.flush().good();
	}
	
	//! returns true if the communication socket is still good
	operator bool() const { return comm.good(); }
	
protected:
	ionetstream& comm; //!< connection to Mirage
	plist::Primitive<std::string> name; //!< name ("ID") of the current object in Mirage
	plist::Dictionary msg; //!< buffers settings to Mirage until flush();
};
