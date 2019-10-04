//-*-c++-*-
#ifndef INCLUDED_plistPrimitives_h_
#define INCLUDED_plistPrimitives_h_

#include "plistBase.h"
#include "string_util.h"
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <map>
#include <libxml/xmlmemory.h>

extern "C" {
	//!@name libxml2 forward declarations
	//!forward declaration of the libxml2 struct of the same name
	xmlNode* xmlAddPrevSibling(xmlNode* node, xmlNode* sibling);
	xmlNode* xmlNewText(const xmlChar* s);
	xmlNode* xmlNewComment(const xmlChar* s);
	xmlNode* xmlAddChild(xmlNode * parent, xmlNode* child);
	xmlNode* xmlNewChild(xmlNode* parent, xmlNs* ns, const xmlChar * name, const xmlChar * content);
	int xmlStrEqual(const xmlChar* a, const xmlChar* b);
	xmlChar* xmlNodeGetContent(xmlNode* node);
	void xmlNodeSetContent(xmlNode* node, const xmlChar* content);
	xmlAttr* xmlHasProperty(xmlNode* node, const xmlChar* name);
	xmlChar* xmlGetProperty(xmlNode* node, const xmlChar* name);
	long xmlGetLineNo(xmlNode* node);
	xmlChar* xmlGetNodePath(xmlNode* node);
	void xmlNodeSetName(xmlNode* node, const xmlChar* name);
	void xmlFreeNode(xmlNode* node);
	void xmlUnlinkNode(xmlNode* node);
	//@}
}

namespace plist {
	//! returns a string indicating the plist entry type to use for the specified type
	/*! some primitives (bool, char) aren't handled because they require a specialization
	 *  of Primitive and won't use this function.  If you want to use a plist Primitive of
	 *  some custom type, you might be able to just define a new specialization of 
	 *  this function and provide iostream <</>> operators for your type... */
	template<typename T> const char* getTypeName();
	/// @cond INTERNAL
	template<> inline const char* getTypeName<short>() { return "integer"; }
	template<> inline const char* getTypeName<unsigned short>() { return "integer"; }
	template<> inline const char* getTypeName<int>() { return "integer"; }
	template<> inline const char* getTypeName<unsigned int>() { return "integer"; }
	template<> inline const char* getTypeName<long>() { return "integer"; }
	template<> inline const char* getTypeName<unsigned long>() { return "integer"; }
	template<> inline const char* getTypeName<long long>() { return "integer"; }
	template<> inline const char* getTypeName<unsigned long long>() { return "integer"; }
	template<> inline const char* getTypeName<float>() { return "real"; }
	template<> inline const char* getTypeName<double>() { return "real"; }
	/// @endcond
	
	//! Implements type-specific functionality through template specialization, mainly involving value conversion and stringification formatting
	/*! Provides smart-pointer style functionality for transparent
	 *  access to the value storage, as well as automatic casting
	 *  to and from the value type so you can almost always treat
	 *  the Primitive as if it was the value itself. */
	template<typename T>
	class Primitive : public PrimitiveBase {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		
		//! constructor
		Primitive() : PrimitiveBase(), val(), prevVal() {}
		//! copy constructor, automatic conversion from value type
		Primitive(const T& v) : PrimitiveBase(), val(v), prevVal() {} 
		//! assignment from value type (template specializations add in-place modiciation (e.g. +=, *=))
		Primitive& operator=(const T& v) { if(&v==&prevVal) std::swap(val,prevVal); else { prevVal=val; val=v; } fireValueChanged(prevVal==val); return *this; }
		virtual Primitive& operator=(const PrimitiveBase& pb) { if(dynamic_cast<const Primitive*>(&pb)!=this) operator=(pb.castTo<T>()); return *this; }
		//! assignment from primitive of the same type (just assign value)
		Primitive& operator=(const Primitive& p) { operator=(p.val); return *this; }

		Primitive& operator+=(const T& v) { prevVal=val; val+=v; fireValueChanged(prevVal==val); return *this; } //!< add in-place
		Primitive& operator-=(const T& v) { prevVal=val; val-=v; fireValueChanged(prevVal==val); return *this; } //!< subtract in-place
		Primitive& operator*=(const T& v) { prevVal=val; val*=v; fireValueChanged(prevVal==val); return *this; } //!< multiply in-place
		Primitive& operator/=(const T& v) { prevVal=val; val/=v; fireValueChanged(prevVal==val); return *this; } //!< divide in-place
		
		//! smart pointer, dereference to access primitive storage
		const T& operator*() const { return val; }
		//! smart pointer, dereference to access primitive storage
		const T* operator->() const { return &val; }
		
		//! cast operator to automatically convert to value type
		operator T() const { return val; }

		//! interprets \a node as holding the specialization type
		void loadXML(xmlNode* node);
		//! saves #val into \a node
		void saveXML(xmlNode* node) const;
		void set(const std::string& str);
		using PrimitiveBase::set;
		std::string get() const {
			std::stringstream sstr;
			sstr <<std::setprecision(std::numeric_limits<T>::digits10)<< val;
			return sstr.str();
		}
		
		//bool operator==(const T& v) { return v==val; }
		//bool operator==(const Primitive& p) { return p.val==val; }
		virtual bool operator==(const PrimitiveBase& other) { 
			if(const Primitive* p = dynamic_cast<const Primitive*>(&other))
				return p->val == val;
			else
				return p->get() == get();
		}
		
		virtual long toLong() const { return static_cast<long>(val); }
		virtual double toDouble() const { return static_cast<double>(val); }
		
		//! clone definition for Primitive<T>
		PLIST_CLONE_DEF(Primitive<T>,new Primitive<T>(val));
		
		const T& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value

	protected:
		T val; //!< storage of primitive value
		T prevVal; //!< following each assignment, this is the "old" value -- very handy for PrimitiveListeners
	};
	
	template<typename T>
	void Primitive<T>::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		bool bt=xNodeHasName(node,"true");
		bool bf=xNodeHasName(node,"false");
		if(!bt && !bf && !xNodeHasName(node,"integer") && !xNodeHasName(node,"real") && !xNodeHasName(node,"string")) {
			std::stringstream errstr;
			errstr << "Error: plist::Primitive<" << typeid(T).name() << "> expects " << getTypeName<T>() << ", got unknown type " << (const char*)xNodeGetName(node);
			throw bad_format(node,errstr.str());
		}
		if(!xNodeHasName(node,getTypeName<T>()))
			std::cerr << "Warning: plist expected " << getTypeName<T>() << " got " << (const char*)xNodeGetName(node) << ", trying to convert. (line " << xmlGetLineNo(node) << ")" << std::endl;
		if(bt) {
			prevVal=val;
			val = true;
			fireValueChanged(prevVal==val);
		} else if(bf) {
			prevVal=val;
			val = false;
			fireValueChanged(prevVal==val);
		} else {
			xmlChar * cont=xmlNodeGetContent(node);
			const std::string str((const char*)cont);
			xmlFree(cont);
			set(str);
		}
	}
	template<typename T>
	void Primitive<T>::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		xmlNodeSetName(node,(const xmlChar*)getTypeName<T>());
		std::stringstream str;
		str <<std::setprecision(std::numeric_limits<T>::digits10)<< val;
		xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
	}
	template<typename T>
	void Primitive<T>::set(const std::string& str) {
		std::string trimmed = string_util::trim(str);
		if(trimmed.size()==0) {
			std::cerr << "Warning: plist expected " << getTypeName<T>() << ", got empty string" << std::endl;
			return;
		}
		bool neg=false;
		if(trimmed[0]=='-') {
			neg=true;
			trimmed=trimmed.substr(1);
		}
		prevVal=val;
		if((trimmed=="∞" || string_util::makeLower(trimmed)=="inf") && std::numeric_limits<T>::has_infinity) {
			val = neg ? -std::numeric_limits<T>::infinity() : std::numeric_limits<T>::infinity();
		} else {
			std::stringstream sstr(str);
			if(!(sstr >> val)) {
				std::string err="Expected "; err+=getTypeName<T>(); err+=" value, got '"+str+"'";
				throw bad_format(NULL,err);
			}
			while(sstr.good() && isspace(sstr.peek()))
				sstr.get();
			if(!sstr.eof()) {
				std::string err="Expected "; err+=getTypeName<T>(); err+=" value, got '"+sstr.str()+"'";
				throw bad_format(NULL,err);
			}
		}
		fireValueChanged(prevVal==val);
	}
	
	//! implements the clone function for Primitive<T>
	PLIST_CLONE_IMPT(T,Primitive,new Primitive<T>(val));

	
	//! Auto subscribe/unsubscribe from a Primitive
	struct AutoPrimitiveListener : public plist::PrimitiveListener {
		//! constructor, specify value
		AutoPrimitiveListener(const plist::PrimitiveBase& source) : plist::PrimitiveListener(), src(source) { src.addPrimitiveListener(this); }
		//! assignment, no-op
		AutoPrimitiveListener& operator=(const AutoPrimitiveListener&) { return *this; }
		//! destructor, automatically unsubscribes
		~AutoPrimitiveListener() { src.removePrimitiveListener(this); }
		const plist::PrimitiveBase& src; //!< the value being monitored
	};
	//! Produces a callback on a member function when the value changes
	template<typename O>
	struct PrimitiveCallbackMember : public plist::AutoPrimitiveListener {
		//! constructor
		PrimitiveCallbackMember(const plist::PrimitiveBase& source, O& component, void(O::*func)(), bool callNow=true)
		: plist::AutoPrimitiveListener(source), comp(component), callback(func) { if(callNow) (comp.*callback)(); }
		//! assignment, no-op
		PrimitiveCallbackMember& operator=(const PrimitiveCallbackMember&) { return *this; }
		virtual void plistValueChanged(const plist::PrimitiveBase& /*pl*/) { (comp.*callback)(); }
		O& comp; //!< instance to call on
		void(O::*callback)(); //!< function to call
	};
	
	
	
/// @cond SHOW_PLIST_OBJECT_SPECIALIZATION
	
	//! provides a @c bool specialization of Primitive<T>
	/*! A @c bool can be treated as either a string or an integer\n
	 *  When saving, "true" or "false" will be used, but users could
	 *  also specify "yes"/"no" or "on"/"off".  If an integer is used,
	 *  it will be interpreted as usual: 0==false, otherwise true. */
	template<>
	class Primitive<bool> : public PrimitiveBase {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		Primitive() : PrimitiveBase(), val(), prevVal() {} //!< constructor
		Primitive(const bool& v) : PrimitiveBase(), val(v), prevVal() {} //!< casting constructor
		Primitive& operator=(const bool& v) { if(&v==&prevVal) std::swap(val,prevVal); else { prevVal=val; val=v; } fireValueChanged(prevVal==val); return *this; } //!< assignment constructor
		virtual Primitive& operator=(const PrimitiveBase& pb) { if(dynamic_cast<const Primitive*>(&pb)!=this) operator=(pb.castTo<bool>()); return *this; }
		Primitive& operator=(const Primitive& p) { operator=(p.val); return *this; }
		//bool& operator*() { return val; }
		const bool& operator*() const { return val; } //!< dereference will return data storage
		//bool* operator->() { return &val; }
		const bool* operator->() const { return &val; } //!< can use -> to access members of data storage
		operator bool() const { return val; } //!< casting operator

		void loadXML(xmlNode* node); //!< interprets @a node as a bool
		void saveXML(xmlNode* node) const;  //!< saves #val into @a node
		void set(const std::string& str);
		using PrimitiveBase::set;
		std::string get() const {
			return val?"true":"false";
		}
		
		//bool operator==(const bool& v) { return v==val; }
		//bool operator==(const Primitive& p) { return p.val==val; }
		virtual bool operator==(const PrimitiveBase& other) { 
			if(const Primitive* p = dynamic_cast<const Primitive*>(&other))
				return p->val == val;
			else
				return p->get() == get();
		}
				
		virtual long toLong() const { return static_cast<long>(val); }
		virtual double toDouble() const { return static_cast<double>(val); }
		
		//! clone definition for Primitive<bool>
		PLIST_CLONE_DEF(Primitive<bool>,new Primitive<bool>(val));
		
		const bool& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value
		
	protected:
		bool val; //!< the actual data storage
		bool prevVal; //!< following each assignment, this is the "old" value -- very handy for PrimitiveListeners
	};
	
	//! provides a @c char specialization of plist::Primitive<T>, adds a unique #numeric property to the usual template implementation
	/*! A @c char can be treated as either a string or an integer, you can use
	 *  the setNumeric(bool) function to indicate which style to use when saving */
	template<>
	class Primitive<char> : public PrimitiveBase {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		Primitive() : PrimitiveBase(), val(), prevVal(), numeric(false) {} //!< constructor
		Primitive(const char& v, bool isNum=false) : PrimitiveBase(), val(v), prevVal(), numeric(isNum) {} //!< casting constructor
		Primitive& operator=(char v) { if(&v==&prevVal) std::swap(val,prevVal); else { prevVal=val; val=v; } fireValueChanged(prevVal==val); return *this; } //!< assignment
		virtual Primitive& operator=(const PrimitiveBase& pb) { if(dynamic_cast<const Primitive*>(&pb)!=this) operator=(pb.castTo<char>()); return *this; }
		//! assignment from primitive of the same type (just assign value)
		Primitive& operator=(const Primitive& p) { operator=(p.val); return *this; }
		Primitive& operator+=(char v) { prevVal=val; val+=v; fireValueChanged(prevVal==val); return *this; } //!< add-assign
		Primitive& operator-=(char v) { prevVal=val; val-=v; fireValueChanged(prevVal==val); return *this; } //!< subtract-assign
		Primitive& operator*=(char v) { prevVal=val; val*=v; fireValueChanged(prevVal==val); return *this; } //!< multiply-assign
		Primitive& operator/=(char v) { prevVal=val; val/=v; fireValueChanged(prevVal==val); return *this; } //!< divide-assign
		//char& operator*() { return val; }
		const char& operator*() const { return val; } //!< dereference will return data storage
		//char* operator->() { return &val; }
		const char* operator->() const { return &val; } //!< can use -> to access members of data storage
		operator char() const { return val; } //!< casting operator
		
		void setNumeric(bool isNum) { numeric=isNum; } //!< sets #numeric
		bool getNumeric() const { return numeric; } //!< returns #numeric
		
		void loadXML(xmlNode* node); //!< interprets @a node as a char
		void saveXML(xmlNode* node) const; //! saves #val into @a node
		void set(const std::string& str);
		using PrimitiveBase::set;
		std::string get() const {
			if(numeric) {
				std::stringstream sstr;
				sstr << (int)val;
				return sstr.str();
			} else
				return std::string(1,val);
		}

		//bool operator==(const char& v) { return v==val; }
		//bool operator==(const Primitive& p) { return p.val==val; }
		virtual bool operator==(const PrimitiveBase& other) { 
			if(const Primitive* p = dynamic_cast<const Primitive*>(&other))
				return p->val == val;
			else
				return p->get() == get();
		}
		
		virtual long toLong() const { return static_cast<long>(val); }
		virtual double toDouble() const { return static_cast<double>(val); }
		
		//! clone definition for Primitive<char>
		PLIST_CLONE_DEF(Primitive<char>,new Primitive<char>(val));
		
		const char& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value
		
	protected:
		char val; //!< data storage
		char prevVal; //!< following each assignment, this is the "old" value -- very handy for PrimitiveListeners
		bool numeric; //!< if true, requests that saves store the numeric value instead of corresponding character
	};
	
	//! provides an @c unsigned @c char specialization of plist::Primitive<T>, adds a unique #numeric property to the usual template implementation
	/*! A @c char can be treated as either a string or an integer, you can use
	 *  the setNumeric(bool) function to indicate which style to use when saving */
	template<>
	class Primitive<unsigned char> : public PrimitiveBase {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		Primitive() : PrimitiveBase(), val(), prevVal(), numeric(false) {} //!< constructor
		Primitive(const unsigned char& v, bool isNum=false) : PrimitiveBase(), val(v), prevVal(), numeric(isNum) {} //!< casting constructor
		Primitive& operator=(unsigned char v) { prevVal=val; val=v; fireValueChanged(prevVal==val); return *this; } //!< assignment
		virtual Primitive& operator=(const PrimitiveBase& pb) { if(dynamic_cast<const Primitive*>(&pb)!=this) operator=(pb.castTo<unsigned char>()); return *this; }
		//! assignment from primitive of the same type (just assign value)
		Primitive& operator=(const Primitive& p) { operator=(p.val); return *this; }
		Primitive& operator+=(unsigned char v) { prevVal=val; val+=v; fireValueChanged(prevVal==val); return *this; } //!< add-assign
		Primitive& operator-=(unsigned char v) { prevVal=val; val-=v; fireValueChanged(prevVal==val); return *this; } //!< subtract-assign
		Primitive& operator*=(unsigned char v) { prevVal=val; val*=v; fireValueChanged(prevVal==val); return *this; } //!< multiple-assign
		Primitive& operator/=(unsigned char v) { prevVal=val; val/=v; fireValueChanged(prevVal==val); return *this; } //!< divide-assign
		//unsigned char& operator*() { return val; }
		const unsigned char& operator*() const { return val; } //!< dereference will return data storage
		//unsigned char* operator->() { return &val; }
		const unsigned char* operator->() const { return &val; } //!< can use -> to access members of data storage
		operator unsigned char() const { return val; } //!< casting operator
		
		void setNumeric(bool isNum) { numeric=isNum; } //!< sets #numeric
		bool getNumeric() const { return numeric; } //!< returns #numeric
		
		void loadXML(xmlNode* node); //!< interprets @a node as a unsigned char
		void saveXML(xmlNode* node) const; //! saves #val into @a node
		void set(const std::string& str);
		using PrimitiveBase::set;
		std::string get() const {
			if(numeric) {
				std::stringstream sstr;
				sstr << (int)val;
				return sstr.str();
			} else
				return std::string(1,val);
		}
		
		//bool operator==(const unsigned char& v) { return v==val; }
		//bool operator==(const Primitive& p) { return p.val==val; }
		virtual bool operator==(const PrimitiveBase& other) { 
			if(const Primitive* p = dynamic_cast<const Primitive*>(&other))
				return p->val == val;
			else
				return p->get() == get();
		}
		
		virtual long toLong() const { return static_cast<long>(val); }
		virtual double toDouble() const { return static_cast<double>(val); }
		
		//! clone definition for Primitive<unsigned char>
		PLIST_CLONE_DEF(Primitive<unsigned char>,new Primitive<unsigned char>(val));
		
		const unsigned char& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value
		
	protected:
		unsigned char val; //!< data storage
		unsigned char prevVal; //!< following each assignment, this is the "old" value -- very handy for PrimitiveListeners
		bool numeric; //!< if true, requests that saves store the numeric value instead of corresponding character
	};
	
	
	//! Provides a @c std::string specialization of Primitive<T>
	/*! Doesn't need to provide a operator cast because we subclass std::string itself! */
	template<>
	class Primitive<std::string> : public PrimitiveBase, public std::string {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		Primitive() : PrimitiveBase(), std::string(), prevVal() {} //!< constructor
		Primitive(const std::string& v) : PrimitiveBase(), std::string(v), prevVal() {} //!< casting constructor
		Primitive(const std::string& v, size_type off, size_type count=npos) : PrimitiveBase(), std::string(v,off,count), prevVal() {} //!< casting constructor
		Primitive(const char* v, size_type count) : PrimitiveBase(), std::string(v,count), prevVal() {} //!< casting constructor
		Primitive(const char* v) : PrimitiveBase(), std::string(v), prevVal() {} //!< casting constructor
		Primitive(size_type count, char v) : PrimitiveBase(), std::string(count,v), prevVal() {} //!< casting constructor
		Primitive& operator=(const std::string& v) { if(&v==&prevVal) std::string::swap(prevVal); else { prevVal=*this; std::string::operator=(v); } fireValueChanged(prevVal==*this); return *this; } //!< assignment
		Primitive& operator=(const char* v) { prevVal=*this; std::string::operator=(v); fireValueChanged(prevVal==*this); return *this; } //!< assignment
		Primitive& operator=(char v) { prevVal=*this; std::string::operator=(v); fireValueChanged(prevVal==*this); return *this; } //!< assignment
		virtual Primitive& operator=(const PrimitiveBase& pb) { if(dynamic_cast<const Primitive*>(&pb)!=this) operator=(pb.toString()); return *this; }
		//! assignment from primitive of the same type (just assign value)
		Primitive& operator=(const Primitive& p) { operator=(static_cast<const std::string&>(p)); return *this; }
		//std::string& operator*() { return *this; }
		const std::string& operator*() const { return *this; } //!< dereference will return data storage as a string (for uniformity with the other Primitives, although unnecessary with this instantiation)
		//std::string* operator->() { return this; }
		const std::string* operator->() const { return this; } //!< returns a pointer to this (for uniformity with the other Primitives, although unnecessary with this instantiation)
		//no casting operator because we subclass string
		
		void loadXML(xmlNode* node); //!< interprets @a node as a string
		void saveXML(xmlNode* node) const; //!< saves the content of the string into @a node
		void set(const std::string& str) { operator=(str); } // operator= will fireValueChanged
		using PrimitiveBase::set;
		std::string get() const { return *this; }
		
		//bool operator==(const std::string& v) { return v==*this; }
		//bool operator==(const Primitive& p) { return static_cast<std::string&>(*this)==static_cast<const std::string&>(p); }
		virtual bool operator==(const PrimitiveBase& other) { 
			return other.get() == *this;
		}
		
		virtual bool toBool() const;
		virtual char toChar() const;
		virtual long toLong() const;
		virtual double toDouble() const;
		
		void clear() { prevVal=*this; std::string::clear(); fireValueChanged(prevVal==*this); }
		void resize(size_type n, char c) { prevVal=*this; std::string::resize(n,c); fireValueChanged(prevVal==*this); }
		// todo: should add forwarding calls for other std::string members...
		
		//! clone definition for Primitive<std::string>
		PLIST_CLONE_DEF(Primitive<std::string>,new Primitive<std::string>(get()));
		
		const std::string& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value
		
	protected:
		std::string prevVal; //!< stores the previously assigned value for reference/reset by a value listener
	};
	
	extern template class Primitive<short>;
	extern template class Primitive<unsigned short>;
	extern template class Primitive<int>;
	extern template class Primitive<unsigned int>;
	extern template class Primitive<long>;
	extern template class Primitive<unsigned long>;
	extern template class Primitive<float>;
	extern template class Primitive<double>;
	
	/// @endcond
	
	
	//! provides some accessors common across NamedEnumeration types
	class NamedEnumerationBase : public PrimitiveBase {
	public:
		NamedEnumerationBase() : PrimitiveBase(), strictValue(true) {} //!< constructor
		NamedEnumerationBase(const NamedEnumerationBase& ne) : PrimitiveBase(ne), strictValue(ne.strictValue) {} //!< copy constructor
		NamedEnumerationBase& operator=(const std::string& v) { set(v); return *this; } //!< assignment from string
		NamedEnumerationBase& operator=(const NamedEnumerationBase& ne) { PrimitiveBase::operator=(ne); return *this; } //!< assignment (doesn't copy #strictValue)
		virtual NamedEnumerationBase& operator=(const PrimitiveBase& pb)=0;
		
		virtual void set(long x)=0; //!< converts from a numeric value, may throw bad_format if #strictValue is set
		using PrimitiveBase::set;
		
		//! fills @a names with the names and values for the enum -- needed for generic access to the names (e.g. UI generation)
		virtual void getPreferredNames(std::map<int,std::string>& names) const=0;
		//! fills @a names with the names and values for the enum -- needed for generic access to the names (e.g. UI generation)
		virtual void getAllNames(std::map<std::string,int>& names) const=0;
		
		std::string getDescription(bool preferredOnly=true); //!< returns a string listing the available symbolic names
		void setStrict(bool strict) { strictValue=strict; } //!< sets #strictValue (whether or not to allow assignment from numeric values which don't have a symbolic name)
		bool getStrict() const { return strictValue; } //!< returns #strictValue (whether or not to allow assignment from numeric values which don't have a symbolic name)
		
	protected:
		bool strictValue; //!< if true, don't allow conversion from numeric string which doesn't correspond to a named value
	};
	
	//! Provides an interface for the use of enumerations in a plist -- you can specify values by either the string name or the corresponding integer value
	/*! Where an array of names is specified, you must order the array such that
	 *  the enumeration value can be used as an index into the array.  The array must be
	 *  terminated with NULL so NamedEnumeration can tell how many
	 *  elements there are.
	 *
	 *  <b>Binary size and symbol definition</b>: this class contains two static STL maps
	 *  for storing the string names of the enumeration values.  The problem is that
	 *  due to the way static members of templates are handled, you will wind up
	 *  with extensive symbol declarations in each translation unit which references
	 *  this header, which can lead to significant binary bloat (particularly with
	 *  debugging symbols).  The solution is to limit the instantiation of these statics.
	 *  
	 *  - Easy way out: define USE_GLOBAL_PLIST_STATICS, which will then declare the statics
	 *    here in the header, and allow the duplication to occur (which is fine for small
	 *    projects or if you don't reference this header widely)
	 *    
	 *  - Otherwise, you can then declare:
	 *    @code
	 *    template<typename T> std::map<std::string,T> plist::NamedEnumeration<T>::namesToVals;
	 *    template<typename T> std::map<T,std::string> plist::NamedEnumeration<T>::valsToNames;
	 *    @endcode
	 *    in the translation units where you introduce new types to the template parameter.
	 *    This will greatly limit symbol replication, although there will still be some minor
	 *    duplication if more than just the "new" types are found in the current unit.
	 *    You may prefer to call the macro INSTANTIATE_ALL_NAMEDENUMERATION_STATICS() to ensure
	 *    future compatability in the unlikely event more statics are added in the future.
	 *    
	 *  - For the ultimate minimal binary size, explicitly declare a template
	 *    specialization for each type you use: (note 'YOUR_T' is meant to be replaced!)
	 *    @code
	 *    // replace YOUR_T with the type you are using:
	 *    template<> std::map<std::string,YOUR_T> plist::NamedEnumeration<YOUR_T>::namesToVals = std::map<std::string,YOUR_T>();
	 *    template<> std::map<YOUR_T,std::string> plist::NamedEnumeration<YOUR_T>::valsToNames = std::map<YOUR_T,std::string>();
	 *    @endcode
	 *    You can do this only once, in a single translation unit, yielding zero replication of symbols.
	 *    For convenience, we provide a macro INSTANTIATE_NAMEDENUMERATION_STATICS(T) which will do this for you. */
	template<typename T> 
	class NamedEnumeration : public NamedEnumerationBase {
	public:
		template<typename U, typename V> struct conversion_policy { typedef typename U::template ConversionTo<V> value_conversion; };
		NamedEnumeration() : NamedEnumerationBase(), val(), prevVal() {} //!< constructor
		NamedEnumeration(const NamedEnumeration& ne) : NamedEnumerationBase(ne), val(ne.val), prevVal() {} //!< copy constructor
		NamedEnumeration(const T& v, const char * const* enumnames) : NamedEnumerationBase(), val(v), prevVal() { setNames(enumnames); } //!< constructor, pass initial value, array of strings (the names); assumes enumeration is sequential starting at 0, and runs until the names entry is NULL (i.e. @a names must be terminated with a NULL entry)
		NamedEnumeration(const T& v) : NamedEnumerationBase(), val(v), prevVal() {} //!< automatic casting from the enumeration
		NamedEnumeration& operator=(const T& v) { if(&v==&prevVal) std::swap(val,prevVal); else { prevVal=val; val=v; } fireValueChanged(prevVal==val); return *this; } //!< assignment from enumeration value (numeric)
		NamedEnumeration& operator=(const std::string& v) { set(v); return *this; } //!< assignment from string
		NamedEnumeration& operator=(const NamedEnumeration& ne) { operator=(ne.val); return *this; } //!< assignment
		//T& operator*() { return val; }
		const T& operator*() const { return val; } //!< value access
		operator T() const { return val; } //!< automatic casting to the enumeration value
		static void setNames(const char *  const* enumnames); //!< calls clearNames() and then resets the array of names, @a enumnames must be terminated with NULL, and the enumeration must be sequential starting at 0; these names become the "preferred" name for each value
		static const std::map<T,std::string>& getPreferredNames() { return valsToNames; } //!< returns mapping from numeric value to "preferred" name (one-to-one)
		static const std::map<std::string,T>& getAllNames() { return namesToVals; } //!< returns mapping from names to values (many-to-one allowed)
		static void clearNames(); //!< removes all names, thus causing only numeric values to be accepted
		static void addNameForVal(const std::string& enumname, const T& v); //!< adds an alternative name mapping to the specified numeric value; if the value doesn't already have a name, this is also the "preferred" name
		static void setPreferredNameForVal(const std::string& enumname, const T& v); //!< replaces any previous "preferred" name for a specific value

		//! polymorphic assignment, throws bad_format if #strictValue is requested and the value is invalid integer
		virtual NamedEnumeration& operator=(const PrimitiveBase& pb) {
			if(dynamic_cast<const NamedEnumeration*>(&pb)==this)
				return *this;
			if(const std::string* str = dynamic_cast<const std::string*>(&pb))
				set(*str);
			else {
				T tv=static_cast<T>(pb.toLong());
				if(strictValue && valsToNames.find(tv)==valsToNames.end())
					throw bad_format(NULL, "NamedEnumeration unable to assign arbitrary integer value because strict checking is requested");
				val=tv;
			}
			return *this;
		}
		
		//! interprets @a node as either a string holding the name, or a number corresponding to its index (name is preferred)
		void loadXML(xmlNode* node) {
			if(node==NULL)
				return;
			if(xNodeHasName(node,"true") || xNodeHasName(node,"false")) {
				std::string name=(const char*)xNodeGetName(node);
				std::cerr << "Warning: plist NamedEnumeration should use <string>" << name << "</string>, not <" << name << "/>" << std::endl;
				try {
					set(name);
				} catch(const bad_format& err) {
					throw bad_format(node,err.what()); //add current node to exception and rethrow
				}
			} else if(xNodeHasName(node,"integer") || xNodeHasName(node,"real") || xNodeHasName(node,"string")) {
				xmlChar * cont=xmlNodeGetContent(node);
				try {
					set((const char*)cont);
				} catch(const bad_format& err) {
					xmlFree(cont);
					throw bad_format(node,err.what()); //add current node to exception and rethrow
				} catch(...) {
					xmlFree(cont);
					throw;
				}
				xmlFree(cont);
			} else
				throw bad_format(node,"Error: plist NamedEnumeration must be numeric or valid string");
		}
		//! saves name of current value (if available, index used otherwise) into @a node
		void saveXML(xmlNode* node) const {
			if(node==NULL)
				return;
			std::string name;
			if(getNameForVal(val,name)) {
				xmlNodeSetName(node,(const xmlChar*)"string");
			} else {
				xmlNodeSetName(node,(const xmlChar*)"integer");
			}
			xmlNodeSetContent(node,(const xmlChar*)name.c_str());
		}
		void set(const std::string& str) {
			prevVal=val;
			if(!getValForName(str,val))
				throw bad_format(NULL,"Error: plist::NamedEnumeration must be numeric or valid string (cannot be '"+str+"')");
			fireValueChanged(prevVal==val); 
		}
		void set(long x) {
			T tv;
			if(!convert(x,tv)) {
				std::stringstream ss;
				ss << "Error: plist::NamedEnumeration must correspond to valid string (cannot be '" << x << "')";
				throw bad_format(NULL,ss.str());
			}
			prevVal=val;
			val=tv;
			fireValueChanged(prevVal==val); 
		}
		using NamedEnumerationBase::set;
		std::string get() const {
			std::string name;
			getNameForVal(val,name);
			return name;
		}
		
		//bool operator==(const T& v) { return v==val; }
		//bool operator==(const NamedEnumeration& p) { return p.val==val; }
		virtual bool operator==(const PrimitiveBase& other) { 
			if(const NamedEnumeration* p = dynamic_cast<const NamedEnumeration*>(&other))
				return p->val == val;
			else
				return p->get() == get();
		}
		
		virtual long toLong() const { return static_cast<long>(val); }
		virtual double toDouble() const { return static_cast<double>(val); }
		
		//! implements the clone function for NamedEnumeration<T>
		PLIST_CLONE_DEF(NamedEnumeration<T>,new NamedEnumeration<T>(*this));

		const T& getPreviousValue() const { return prevVal; } //!< returns the previously assigned value
		
	protected:
		//! provides the generic access to values and names from NamedEnumerationBase; protected because if you know the full type, better to use the static version of the function
		virtual void getPreferredNames(std::map<int,std::string>& names) const;
		//! provides the generic access to values and names from NamedEnumerationBase; protected because if you know the full type, better to use the static version of the function
		virtual void getAllNames(std::map<std::string,int>& names) const;
		
		//! attempts to convert x to T and store in v, but failing and returning false if strictValue is set and x does not correspond to a name
		bool convert(long x, T& v) const {
			T tv=static_cast<T>(x);
			if(strictValue && valsToNames.find(tv)==valsToNames.end())
				return false;
			v=tv;
			return true;
		}
		//! sets @a v to the enumeration value named @a name; returns false if no such name is found
		bool getValForName(std::string name, T& v) const {
			std::transform(name.begin(), name.end(), name.begin(), (int(*)(int)) std::toupper);
			typename std::map<std::string,T>::const_iterator vit=namesToVals.find(name);
			if(vit!=namesToVals.end())
				v=vit->second;
			else {
				long iv;
				if(sscanf(name.c_str(),"%ld",&iv)==0)
					return false;
				if(!convert(iv,v))
					 return false;
			}
			return true;
		}
		//! retrieves the "preferred" name for the enumeration value @a v; returns false if no name is registered
		bool getNameForVal(const T& v, std::string& name) const {
			typename std::map<T,std::string>::const_iterator nit=valsToNames.find(v);
			if(nit!=valsToNames.end()) {
				name=nit->second;
				return true;
			}
			std::stringstream str;
			str << v;
			name=str.str();
			return false;
		}
		T val; //!< storage of enum value
		T prevVal; //!< storage of enum value
		
		//! look up table of string names to enum values (can have multiple string names mapping to same enum -- deprecated values for example)
		/*! See class notes regarding instantiation options for static values like this */
		static std::map<std::string,T> namesToVals;
		//! look up table of enum values to preferred display name (by default, first name given)
		/*! See class notes regarding instantiation options for static values like this */
		static std::map<T,std::string> valsToNames;
	};
	//! implements the clone function for NamedEnumeration<T>
	PLIST_CLONE_IMPT(T,NamedEnumeration,new NamedEnumeration<T>(*this));

#ifdef USE_GLOBAL_PLIST_STATICS
	template<typename T> std::map<std::string,T> plist::NamedEnumeration<T>::namesToVals;
	template<typename T> std::map<T,std::string> plist::NamedEnumeration<T>::valsToNames;
#endif
	
	//! Unless you enable GLOBAL_PLIST_STATICS, call this macro in each translation unit which introduces new template types
	/*! @see NamedEnumeration for further discussion */
#define INSTANTIATE_ALL_NAMEDENUMERATION_STATICS() \
	namespace plist { \
		template<typename T> std::map<std::string,T> NamedEnumeration<T>::namesToVals; \
		template<typename T> std::map<T,std::string> NamedEnumeration<T>::valsToNames; \
	}
	
	//! Unless you enable GLOBAL_PLIST_STATICS, call this macro in one of your source files to provide a definition of the statics for a specific type
	/*! @see NamedEnumeration for further discussion */
#define INSTANTIATE_NAMEDENUMERATION_STATICS(T) \
	namespace plist { \
		template<> std::map<std::string,T> NamedEnumeration<T>::namesToVals = std::map<std::string,T>(); \
		template<> std::map<T,std::string> NamedEnumeration<T>::valsToNames = std::map<T,std::string>(); \
	}
	
	template<typename T> void NamedEnumeration<T>::setNames(const char *  const* enumnames) {
		// first test if names are equivalent, this improves thread safety (but still is not thread safe during first initialization!)
		size_t incnt=0;
		while(enumnames[incnt]!=NULL)
			++incnt;
		bool differ=false;
		if(incnt!=valsToNames.size() || incnt!=namesToVals.size())
			differ=true;
		else {
			for(size_t i=0; enumnames[i]!=NULL; ++i) {
				std::string name=enumnames[i];
				typename std::map<T,std::string>::const_iterator v2nit=valsToNames.find(static_cast<T>(i));
				if(v2nit==valsToNames.end() || v2nit->second!=name) {
					differ=true;
					break;
				}
				std::transform(name.begin(), name.end(), name.begin(), (int(*)(int)) std::toupper);
				typename std::map<std::string,T>::const_iterator n2vit=namesToVals.find(name);
				if(n2vit==namesToVals.end() || n2vit->second!=static_cast<T>(i)) {
					differ=true;
					break;
				}
			}
		}
		if(differ) { // we need to reset name/value maps
			// to further improve thread safety, load into a set of local maps, and then do a 'swap' which should be fast and leave less time for invalid state.
			std::map<std::string,T> newNamesToVals;
			std::map<T,std::string> newValsToNames;
			for(size_t i=0; enumnames[i]!=NULL; ++i) {
				std::string name=enumnames[i];
				newValsToNames[static_cast<T>(i)]=name;
				std::transform(name.begin(), name.end(), name.begin(), (int(*)(int)) std::toupper);
				newNamesToVals[name]=static_cast<T>(i);
			}
			namesToVals.swap(newNamesToVals);
			valsToNames.swap(newValsToNames);
		}
	}
	template<typename T> void NamedEnumeration<T>::clearNames() {
		namesToVals.clear();
		valsToNames.clear();
	}
	template<typename T> void NamedEnumeration<T>::addNameForVal(const std::string& enumname, const T& v) {
		if(valsToNames.find(v)==valsToNames.end())
			valsToNames[v]=enumname;
		std::string name=enumname;
		std::transform(name.begin(), name.end(), name.begin(), (int(*)(int)) std::toupper);
		namesToVals[name]=v;
	}
	template<typename T> void NamedEnumeration<T>::setPreferredNameForVal(const std::string& enumname, const T& v) {
		valsToNames[v]=enumname;
		addNameForVal(enumname,v);
	}
		
	template<typename T> void NamedEnumeration<T>::getPreferredNames(std::map<int,std::string>& names) const {
		names.clear();
		for(typename std::map<T,std::string>::const_iterator it=valsToNames.begin(); it!=valsToNames.end(); ++it)
			names.insert(std::pair<int,std::string>(it->first,it->second));
	}
	template<typename T> void NamedEnumeration<T>::getAllNames(std::map<std::string,int>& names) const {
		names.clear();
		for(typename std::map<std::string,T>::const_iterator it=namesToVals.begin(); it!=namesToVals.end(); ++it)
			names.insert(std::pair<std::string,int>(it->first,it->second));
	}
	
} //namespace plist


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
