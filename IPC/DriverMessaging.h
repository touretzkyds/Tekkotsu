//-*-c++-*-
#ifndef INCLUDED_DriverMessaging_h_
#define INCLUDED_DriverMessaging_h_

#include "Shared/plistCollections.h"
#include "Shared/plistSpecialty.h"
#include <string>
#include <set>
#include <map>

//! Functions and data structures for sending messages from "user land" (behavior in Main, or motion commands in Motion) to drivers in the hardware abstraction layer
/*! TODO: not implemented for non-threaded IPC, assuming shared memory space... the 'Multiprocess' mode will probably be deprecated before this matters 
 */
namespace DriverMessaging {
	
	class Message;
	
	//! Interface for drivers to receive messages, see addListener() and removeListener()
	/*! We're not using the normal EventRouter system because this will be using interprocess communication, and
	 *  don't want to deal with interprocess subscriptions for a subset of events */
	class Listener {
	public:
		//! destructor
		virtual ~Listener() {}
		
		//! callback function, will be executed in thread of original postMessage() call
		virtual void processDriverMessage(const Message& d)=0;
	};
	
	//! allows drivers to subscribe to messages
	void addListener(Listener* l, const char* const t);
	
	//! allows drivers to unsubscribe from messages
	void removeListener(Listener* l, const char* const t);
	
	//! allows drivers to unsubscribe from all messages
	void removeListener(Listener* l);
	
	//! returns true if any drivers are subscribed to the specified message class
	bool hasListener(const char* const t);
	
	//! posts a message to all subscribing drivers
	/*! TODO not implemented for non-threaded IPC, assuming shared memory space... the 'Multiprocess' mode will probably be deprecated before this matters */
	void postMessage(const Message& d);
	
	
	
	//! Base class for all driver messages, this subclasses XMLLoadSave in case we want to serialize for messages from non-threaded (full multi-process) IPC, or offboard computation
	/*! TODO not implemented for non-threaded IPC, assuming shared memory space... the 'Multiprocess' mode will probably be deprecated before this matters */
	class Message : public virtual LoadSave {
	protected:
		//! Constructor to be used by subclasses (there is no public constructor, this is an abstract class)
		/*! The className argument is used to initialize #CLASS_NAME, the pointer address should be static, used for fast class comparisons */
		explicit Message(const char* const className) : LoadSave(), CLASS_NAME(className) {}
		
	public:
		//! Each class should supply a name so they can be identified more easily, this is assigned via the protected constructor
		/*! Do we need this?  Not sure why I added this vs. using RTTI and typeinfo/typeid stuff */
		const char* const CLASS_NAME;
		
		//! Copy constructor
		Message(const Message& m) : LoadSave(), CLASS_NAME(m.CLASS_NAME) {}
		
		//! Assignment operator
		Message& operator=(const Message& m) { return *this; }
		
		//! Destructor
		virtual ~Message()=0;
	};
	
	
	//! Encodes a set of load predictions for joints so driver can offset target positions accordingly
	class LoadPrediction : public Message, public virtual plist::Dictionary {
	public:
		//! constructor, assign load predictions via direct access to #loads
		LoadPrediction() : Message(NAME), loads() {
			addEntry("Loads",loads);
			setLoadSavePolicy(FIXED,SYNC);
		}
		
		//! holds load predictions, mapping outputs with predictions to those predictions (don't need to supply predictions for all outputs)
		plist::DictionaryOf<plist::Primitive<float> > loads;
		static const char* const NAME; //!< class identifier
	};
	
	
	//! Indicates priority of polling outputs/buttons/sensors, for those drivers which must actively poll for sensor data
	class SensorPriority : public Message, public virtual plist::Dictionary {
	public:
		//! constructor
		SensorPriority() : Message(NAME), outputs(), buttons(), sensors() {
			addEntry("Outputs",outputs);
			addEntry("Buttons",buttons);
			addEntry("Sensors",sensors);
			setLoadSavePolicy(FIXED,SYNC);
		}
		plist::DictionaryOf<plist::Primitive<float> > outputs; //!< priority of output feedbacks
		plist::DictionaryOf<plist::Primitive<float> > buttons; //!< priority of button status
		plist::DictionaryOf<plist::Primitive<float> > sensors; //!< priority of other sensors
		static const char* const NAME; //!< class identifier
	};
	
	class ContactPoint : public virtual plist::Dictionary {
	public:
		ContactPoint() : plist::Dictionary(), frame(), point() { init(); }
		template<typename T> ContactPoint(unsigned int offset, const T& p) : plist::Dictionary(), frame(offset), point(p) { init(); }
		ContactPoint(unsigned int offset, float x, float y, float z) : plist::Dictionary(), frame(offset), point(x,y,z) { init(); }
		plist::OutputSelector frame;
		plist::Point point;
	protected:
		void init() {
			addEntry("Frame",frame);
			addEntry("Point",point);
			frame.setRange(0,-1U); // accept anything (i.e. reference frames are fine)
			setLoadSavePolicy(FIXED,SYNC);
		}			
	};
	
	//! For simulation purposes, notifies visualizer to hold the specified points fixed relative to the world, and move the base frame instead
	class FixedPoints : public Message, public virtual plist::ArrayOf<ContactPoint> {
	public:
		//! constructor, use addEntry()/setEntry() to manage points
		FixedPoints() : Message(NAME), flushOnMotionUpdate(true) { setLoadSavePolicy(SYNC,SYNC); }
		
		//! set the fixed point for a reference frame (convenience function to convert to a DriverMessaging::Point)
		template<typename T> void addEntry(unsigned int frame, const T& p) {
			addEntry(new ContactPoint(frame, p));
		}
		//! set the fixed point for a reference frame (convenience function to convert to a DriverMessaging::Point)
		void addEntry(unsigned int frame, float x, float y, float z) {
			addEntry(new ContactPoint(frame, x,y,z));
		}
		using plist::ArrayOf<ContactPoint>::addEntry;
		
		bool flushOnMotionUpdate; //!< if true (the default), the points are synced to the @e next motion update, instead of being applied immediately
		static const char* const NAME; //!< class identifier
	};
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
