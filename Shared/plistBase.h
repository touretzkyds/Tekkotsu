//-*-c++-*-
#ifndef INCLUDED_plistBase_h_
#define INCLUDED_plistBase_h_

#include "XMLLoadSave.h"
#include "Cloneable.h"
#include <exception>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <cctype>

/*
 From: <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
 
 <!ENTITY % plistObject "(array | data | date | dict | real | integer | string | true | false )" >
 <!ELEMENT plist %plistObject;>
 <!ATTLIST plist version CDATA "1.0" >
 
 <!-- Collections -->
 <!ELEMENT array (%plistObject;)*>
 <!ELEMENT dict (key, %plistObject;)*>
 <!ELEMENT key (#PCDATA)>
 
 <!--- Primitive types -->
 <!ELEMENT string (#PCDATA)>
 <!ELEMENT data (#PCDATA)> <!-- Contents interpreted as Base-64 encoded -->
 <!ELEMENT date (#PCDATA)> <!-- Contents should conform to a subset of ISO 8601 (in particular, YYYY '-' MM '-' DD 'T' HH ':' MM ':' SS 'Z'.  Smaller units may be omitted with a loss of precision) -->
 
 <!-- Numerical primitives -->
 <!ELEMENT true EMPTY>  <!-- Boolean constant true -->
 <!ELEMENT false EMPTY> <!-- Boolean constant false -->
 <!ELEMENT real (#PCDATA)> <!-- Contents should represent a floating point number matching ("+" | "-")? d+ ("."d*)? ("E" ("+" | "-") d+)? where d is a digit 0-9.  -->
 <!ELEMENT integer (#PCDATA)> <!-- Contents should represent a (possibly signed) integer number in base 10 -->
 */

extern "C" {
	struct _xmlNode;
	struct _xmlDoc;
	struct _xmlAttr;
	struct _xmlNs;
	typedef _xmlNode xmlNode; //!< forward declaration of xmlNode to avoid including actual header here
	typedef _xmlDoc xmlDoc; //!< forward declaration of xmlDoc to avoid including actual header here
	typedef _xmlAttr xmlAttr; //!< forward declaration of xmlAttr to avoid including actual header here
	typedef _xmlNs xmlNs; //!< forward declaration of xmlNs to avoid including actual header here
	typedef unsigned char xmlChar; //!< forward declaration of xmlChar to avoid including actual header here
}	

//! A collection of classes to implement the Propery List data storage format, a XML standard used by Apple and others
namespace plist {
	
#ifdef PLIST_CLONE_ABS
#  error PLIST_CLONE_ABS already defined!
#else
#  if !defined(__GNUC__) || __GNUC__ > 3 || (__GNUC__ == 3 && (__GNUC_MINOR__ > 3))
//! defines abstract clone() (=0) using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise
#    define PLIST_CLONE_ABS(TYPE)		virtual TYPE* clone() const __attribute__ ((warn_unused_result)) =0;
#  else
//! defines abstract clone() (=0) using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise
#    define PLIST_CLONE_ABS(TYPE)		virtual Cloneable* clone() const =0;
#  endif
#endif
	
#ifdef PLIST_CLONE_DEF
#  error PLIST_CLONE_DEF already defined!
#else
#  if !defined(__GNUC__) || __GNUC__ > 3 || (__GNUC__ == 3 && (__GNUC_MINOR__ > 3))
//! forward-declares clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise
#    define PLIST_CLONE_FWD(TYPE)		virtual TYPE* clone() const __attribute__ ((warn_unused_result))=0;
//! declares clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_DEF(TYPE,RETVAL)		virtual TYPE* clone() const __attribute__ ((warn_unused_result));
//! implements clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMP(TYPE,RETVAL)		TYPE* TYPE::clone() const { return RETVAL; }
//! implements clone() using templated polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMPT(TEMPL,TYPE,RETVAL)		template<typename TEMPL> TYPE<TEMPL>* TYPE<TEMPL>::clone() const { return RETVAL; }
//! implements clone() using templated polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMPT2(TEMPL1,TEMPL2,TYPE,RETVAL)		template<typename TEMPL1,typename TEMPL2> TYPE<TEMPL1,TEMPL2>* TYPE<TEMPL1,TEMPL2>::clone() const { return RETVAL; }
#  else
//! forward-declares clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise
#    define PLIST_CLONE_FWD(TYPE)		virtual Cloneable* clone() const =0;
//! declares clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_DEF(TYPE,RETVAL)		virtual Cloneable* clone() const { return RETVAL; }
//! implements clone() using polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMP(TYPE,RETVAL)
//! implements clone() using templated polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMPT(TEMPL,TYPE,RETVAL)
//! implements clone() using templated polymorphic return type @a TYPE if supported by current version of gcc, Cloneable otherwise; returns @a RETVAL
#    define PLIST_CLONE_IMPT2(TEMPL1,TEMPL2,TYPE,RETVAL)
#  endif
#endif
	
	//! Base class for the plist listener callbacks
	class Listener {
	public:
		//! destructor
		virtual ~Listener() {}
	};
	
	class PrimitiveBase;
	//! If you wish to be notified any time a particular plist primitive's value has been changed, inherit from this and implement the callback, then register it with the plist object through Primitive::addPrimitiveListener()
	class PrimitiveListener : public Listener {
	public:
		//! This will be called whenever a plist you have registered with is changed
		/*! @a pl is const to help you avoid infinite recursion from an
		 *  accidental modification of @a pl's value -- use a const cast
		 *  if you're sure you know what you're doing */
		virtual void plistValueChanged(const PrimitiveBase& pl)=0;
		
		//! This will be called whenever a plist you have registered with is reassigned is current value (usually something you'll want to ignore...)
		/*! Argument is const to help you avoid infinite recursion from an
		 *  accidental modification of its value -- use a const cast
		 *  if you're sure you know what you're doing */
		virtual void plistValueTouched(const PrimitiveBase& /*pl*/) {}
	};
	
	class ObjectBase;
	class Collection;
	//! If you wish to be notified any time an entry is added, removed, or replaced from a Dictionary, Array, or Vector, inherit from this and implement one or both of the functions, then register it with the collection's addCollectionListener()
	class CollectionListener : public Listener {
	public:
		//! This will be called whenever an entry is added to the collection
		virtual void plistCollectionEntryAdded(Collection& /*col*/, ObjectBase& /*primitive*/) {}
		//! This will be called whenever an entry is added to the collection
		virtual void plistCollectionEntryRemoved(Collection& /*col*/, ObjectBase& /*primitive*/) {}
		//! This will be called whenever an entry is replaced, or multiple entries are added/removed at once, such as when an assignment occurs
		virtual void plistCollectionEntriesChanged(Collection& /*col*/) {}
	};
	
	//! This base class provides the root functionality for all plist entities -- Dictionary and the various templated subclasses of PrimitiveBase
	/*! The subclasses may throw XMLLoadSave::bad_format if the document is poorly structured or bad values are found. */
	class ObjectBase : public virtual XMLLoadSave, public virtual Cloneable {
		friend ObjectBase* loadXML(xmlNode* node);
	public:
		//! specifies that collections (e.g. plist::Array or plist::Dictionary) of these abstract base classes (ObjectBase, PrimitiveBase) can convert any primitive type to a plist::Primitive wrapper
		template<typename U, typename V> struct conversion_policy { typedef typename U::template WrapValueConversion<V> value_conversion; };
		
		ObjectBase(); //!< constructor
		virtual ~ObjectBase()=0; //!< destructor
		
		//! polymorphic assignment (throws std::bad_cast if the assignment is between invalid types, i.e. a primitive and a collection, or different collection types)
		virtual void set(const ObjectBase&)=0;
		
		//! casting operator: return current value as specified type (throws std::runtime_error exception if bad cast, e.g. dictionary or array to value type)
		/*! The implementation for this function is defined by a series of specializations.
		 *  This allows you to add casts for additional user-defined types, as well as get
		 *  compile time error if you attempt to cast to an unsupported type.
		 *  (I really wish we had virtual templated functions...) */
		template<typename T> T castTo() const;
		
		//! return current value as a string
		virtual std::string toString() const=0;
		//! return current value as a boolean (throws std::runtime_error exception if incompatable, e.g. dictionary or array to value type)
		/*! Does something a little smarter than assuming numeric conversion if called on a string... */
		virtual bool toBool() const { return toLong(); }
		//! return current value as a character (throws std::runtime_error exception if incompatable, e.g. dictionary or array to value type)
		/*! Does something a little smarter than assuming numeric conversion if called on a string... */
		virtual char toChar() const { return toLong(); }
		//! return current value as an (long) integer (throws std::runtime_error exception if incompatable, e.g. dictionary or array to value type)
		virtual long toLong() const=0;
		//! return current value as a double (throws std::runtime_error exception if incompatable, e.g. dictionary or array to value type)
		virtual double toDouble() const=0;
		
		//! subclasses are expected to provide a working implementation
		virtual void loadXML(xmlNode* node)=0;
		//! subclasses are expected to provide a working implementation
		virtual void saveXML(xmlNode* node) const=0;
		
		//! allows a copy to be made of an event, supporting polymorphism
		PLIST_CLONE_ABS(ObjectBase);
			
		virtual void setParseTree(xmlDoc * doc) const;
		
	protected:
		//! polymorphic assignment operator, see assign()
		/*! This is protected for two reasons: one, so you don't accidentally use it via simple '=' statement,
		 *  and two, to avoid 'operator= was hidden' warnings in every base class (because they keep
		 *  reintroducing their default operator=(), hiding/shadowing this one (if it were virtual, as it would
		 *  need to be to take on the role filled by assign(). */
		ObjectBase& operator=(const ObjectBase&) { return *this; }
		
		virtual xmlNode* FindRootXMLElement(xmlDoc* doc) const;
		
		//!Provides accessor functions to struct fields without having to include libxml.h everywhere
		//!@name libxml Forwards
		static bool xNodeHasName(xmlNode* node, const char* name); //!< returns true if the name of @a node matches @a name
		static const xmlChar* xNodeGetName(xmlNode* node); //!< returns name of @a node (not a libxml function)
		static xmlChar* xGetNodePath(xmlNode* node); //!< returns path from document root to @a node (forwards to xmlGetNodePath, returns new allocation, so call xmlFree)
		static const xmlChar* xNodeGetURL(xmlNode* node); //!< returns URL/file of @a node (not a libxml function)
		static xmlNode* xNodeGetChildren(xmlNode* node); //!< returns children of @a node (not a libxml function)
		static xmlNode* xNodeGetLastChild(xmlNode* node); //!< returns last child of @a node (not a libxml function)
		static xmlNode* xNodeGetNextNode(xmlNode* node); //!< returns next node (sibling) after @a node (not a libxml function)
		static xmlNode* xNodeGetPrevNode(xmlNode* node); //!< returns previous node (sibling) before @a node (not a libxml function)
		static xmlNode* xNodeGetParent(xmlNode* node); //!< returns parent node of @a node (not a libxml function)
		static xmlDoc* xNodeGetDoc(xmlNode* node); //!< returns document node of @a node (not a libxml function)
		static bool xNodeIsText(xmlNode* node); //!< returns true if node is an XML_TEXT_NODE (not a libxml function)
		static bool xNodeIsElement(xmlNode* node); //!< returns true if node is an XML_ELEMENT_NODE (not a libxml function)
		static bool xNodeIsComment(xmlNode* node); //!< returns true if node is an XML_COMMENT_NODE (not a libxml function)
		 //@}
		
		//! returns true if @a str is some form of affirmative (e.g. "true" or "yes")
		static bool matchTrue(const std::string& str) { return str=="true" || str=="yes"; }
		//! returns true if @a str is some form of negative (e.g. "false" or "no")
		static bool matchFalse(const std::string& str) { return str=="false" || str=="no"; }
	};
	//! output of an ObjectBase to a stream
	inline std::ostream& operator<<(std::ostream& os, const ObjectBase& pb) {
		return os << pb.toString();
	}
	
	// specializations to funnel cast requests through the appropriate conversion
	/// @cond INTERNAL
	template<> inline bool ObjectBase::castTo<bool>() const { return toBool(); }
	template<> inline char ObjectBase::castTo<char>() const { return toLong(); }
	template<> inline unsigned char ObjectBase::castTo<unsigned char>() const { return toLong(); }
	template<> inline short ObjectBase::castTo<short>() const { return toLong(); }
	template<> inline unsigned short ObjectBase::castTo<unsigned short>() const { return toLong(); }
	template<> inline int ObjectBase::castTo<int>() const { return toLong(); }
	template<> inline unsigned int ObjectBase::castTo<unsigned int>() const { return toLong(); }
	template<> inline long ObjectBase::castTo<long>() const { return toLong(); }
	template<> inline unsigned long ObjectBase::castTo<unsigned long>() const { return toLong(); }
	template<> inline long long ObjectBase::castTo<long long>() const { return toLong(); }
	template<> inline unsigned long long ObjectBase::castTo<unsigned long long>() const { return toLong(); }
	template<> inline float ObjectBase::castTo<float>() const { return static_cast<float>(toDouble()); }
	template<> inline double ObjectBase::castTo<double>() const { return toDouble(); }
	template<> inline const char* ObjectBase::castTo<const char*>() const { return toString().c_str(); }
	template<> inline std::string ObjectBase::castTo<std::string>() const { return toString(); }
	/// @endcond
	
	template<typename T> class Primitive; // forward declaration so we can solve string/Primitive<string> ambiguity in operator= below
	
	//! Provides common functionality to all primitive value classes (implemented in a templated subclass Primitive)
	/*! This class supports callbacks upon modification through the use of the
	 *  PrimitiveListener interface.  Note that we only store a pointer to the
	 *  listener list, which is typically unallocated when no listeners are
	 *  active.  This should ensure minimal memory usage per object, as well as
	 *  support safe storage of plist objects in inter-process shared memory
	 *  regions.
	 *
	 *  If you are using these in a shared memory region, just be sure that only
	 *  the process with listeners does any and all modifications, and that it
	 *  unsubscribes before detaching from the region (or else destroys the region
	 *  itself) */
	class PrimitiveBase : public ObjectBase {
	public:
		//! constructor
		PrimitiveBase() : ObjectBase(), primitiveListeners() {}
		//! copy constructor (don't copy listeners)
		PrimitiveBase(const PrimitiveBase& pb) : ObjectBase(pb), primitiveListeners() {}
		//! assignment (don't assign listeners); doesn't trigger fireValueChanged, subclass should do that from its own operator=() following assignment
		virtual PrimitiveBase& operator=(const PrimitiveBase& pb) { ObjectBase::operator=(pb); return *this; }
		//! assignment from Primitive<string>, solely to resolve ambiguity with this type between operator=(PrimitiveBase) and operator=(std::string)
		PrimitiveBase& operator=(const Primitive<std::string>& v);
		//! assignment from std::string, wraps it in a plist::Primitive and passes on to operator=(PrimitiveBase)
		PrimitiveBase& operator=(const std::string& v);
		//! assignment from long value, wraps it in a plist::Primitive and passes on to operator=(PrimitiveBase)
		PrimitiveBase& operator=(long v);
		//! assignment from unsigned long value, wraps it in a plist::Primitive and passes on to operator=(PrimitiveBase)
		PrimitiveBase& operator=(unsigned long v);
		//! assignment from double value, wraps it in a plist::Primitive and passes on to operator=(PrimitiveBase)
		PrimitiveBase& operator=(double v);
		//! destructor
		~PrimitiveBase();
		
		//! assign a new value
		virtual void set(const std::string& str)=0;
		virtual void set(const ObjectBase& ob) { const PrimitiveBase& pb = dynamic_cast<const PrimitiveBase&>(ob); *this=pb; }
		//! return current value as a string
		virtual std::string get() const=0;
		
		virtual bool operator==(const PrimitiveBase& other)=0;
		
		virtual std::string toString() const { return get(); }
		
		//! get notified of changes; be sure to call removeValueListener before destructing @a vl!
		virtual void addPrimitiveListener(PrimitiveListener* vl) const;
		//! no longer take notification of changes to this object's value
		virtual void removePrimitiveListener(PrimitiveListener* vl) const;
		//! test if @a l is currently registered as a listener
		virtual bool isPrimitiveListener(PrimitiveListener * vl) const;

	protected:
		//! run through #primitiveListeners, calling PrimitiveListener::plistValueChanged(*this) or PrimitiveListener::plistValueTouched(*this)
		virtual void fireValueChanged(bool touch) const;
		//! stores a list of the current listeners
		mutable std::set<PrimitiveListener*>* primitiveListeners;
	};
	//! output stringified value (from PrimitiveBase::get()) to stream
	inline std::ostream& operator<<(std::ostream& os, const PrimitiveBase& pb) {
		return os << pb.get();
	}
	//! input value from next word in @a is, via PrimitiveBase::set()
	inline std::istream& operator>>(std::istream& is, PrimitiveBase& pb) {
		std::string s;
		is >> s;
		pb.set(s);
		return is;
	}

} //namespace plist


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
