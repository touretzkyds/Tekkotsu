//-*-c++-*-
#ifndef INCLUDED_plistSpecialty_h_
#define INCLUDED_plistSpecialty_h_

#include "plist.h"
#include "Shared/RobotInfo.h"
#include "Shared/string_util.h"
#include <limits>

namespace plist {
#ifdef FMAT_DEFAULT_DOUBLE
	typedef double PLISTREAL;
#else
	typedef float PLISTREAL;
#endif

	//! Handles mapping <string> elements to the appropriate numeric offset using Capabilities database
	/*! Values can be prefixed with the name of a model and '/' -- this is recommended for
	 *  numeric values so they can be mapped portably to corresponding outputs on other models.
	 *  This is not necessary when specifying output names, as the name is semantic and allows
	 *  mapping through Capabilities.
	 *
	 *  By default, this class will accept any output offset, but through setRange(), can be configured
	 *  to accept unactuated reference frames as well (e.g. CameraFrame).  Invalid offsets can either
	 *  trigger an exception or simply reset to UNUSED depending on #throwOnInvalid.
	 *  
	 *  This class stores the numeric output offset so it can be indexed into arrays. */
	class OutputSelector : public Primitive<unsigned int> {
	public:
		enum { UNUSED=-1U };
		class bad_value : public std::exception {
		public:
			//! constructor for invalid/unknown output names, or known ones which are out-of-range if @a range is true
			explicit bad_value(const std::string& val, bool range=false) throw() : std::exception(), strValue(val), intValue(UNUSED), rangeError(range), message() { initMessage(); }
			//! constructor for invalid numeric output offsets, or known ones which are out-of-range if @a range is true
			explicit bad_value(unsigned int val, bool range=false) throw() : std::exception(), strValue(), intValue(val), rangeError(range), message() { initMessage(); }
			//! constructor for valid offsets which are out-of-range for this selector
			bad_value(const std::string& strVal, unsigned int intVal) throw() : std::exception(), strValue(strVal), intValue(intVal), rangeError(true), message() { initMessage(); }
			//! destructor
			virtual ~bad_value() throw() {}
			//! returns an error message describing the error
			virtual const char* what() const throw() { return message.c_str(); }
			void initMessage(); //!< initializes #message based on member fields
			std::string strValue; //!< the string value which caused the exception (may be empty if a numeric value was used, or an empty value was passed, see intValue to differentiate)
			unsigned int intValue; //!< the numeric value which caused the exception, -1U if strValue was invalid or empty
			bool rangeError; //!< true if the output was recognized as a known output, but out-of-range for this selector
			std::string message; //!< the string which be returned in what()
		};
		
		OutputSelector() : Primitive<unsigned int>(UNUSED), useNumeric(false), defModel(), saveModel(capabilities.getRobotName()), rangeBegin(0), rangeEnd(capabilities.getNumOutputs()), throwInvalid(false), throwUnused(false) {} //!< constructor
		OutputSelector(unsigned int v, bool isNum=false) : Primitive<unsigned int>(v), useNumeric(isNum), defModel(), saveModel(capabilities.getRobotName()), rangeBegin(0), rangeEnd(capabilities.getNumOutputs()), throwInvalid(false), throwUnused(false) {} //!< casting constructor
		OutputSelector& operator=(const std::string& v) { set(v); return *this; }
		OutputSelector& operator=(const unsigned int& v);
		OutputSelector& operator=(const OutputSelector& v) { operator=(v.val); return *this; }
		using Primitive<unsigned int>::operator=;
		
		void loadXML(xmlNode* node);
		void saveXML(xmlNode* node) const;
		void set(const std::string& str);
		using Primitive<unsigned int>::set;
		std::string get() const;
		
		void setNumeric(bool isNum) { useNumeric=isNum; } //!< sets #useNumeric
		bool getNumeric() const { return useNumeric; } //!< returns #useNumeric
		
		void setDefaultModel(const std::string& name) { defModel = name; } //!< sets #defModel
		const std::string& getDefaultModel() const { return defModel; } //!< returns #defModel
		
		void setSaveModel(const std::string& name) { saveModel = name; } //!< sets #saveModel
		const std::string& getSaveModel() const { return saveModel; } //!< returns #saveModel
		
		//! Sets range of acceptable output offsets between @a begin (inclusive) and @a end (exclusive).
		/*! You could use this to restrict the type of outputs which can be assigned (e.g. LEDOffset to LEDOffset+NumLEDs),
		 *  or to expand the range to include unactuated reference frames (0 to NumReferenceFrames) */
		void setRange(unsigned int begin, unsigned int end);
		unsigned int getRangeBegin() const { return rangeBegin; } //!< returns #rangeBegin
		unsigned int getRangeEnd() const { return rangeEnd; } //!< returns #rangeEnd
		
		void throwOnInvalid(bool b) { throwInvalid=b; } //!< sets #throwInvalid
		bool throwOnInvalid() const { return throwInvalid; } //!< returns #throwInvalid
		
		//! clone definition for OutputSelector
		PLIST_CLONE_DEF(OutputSelector,new OutputSelector(val));
	protected:
		//! If true, requests that saves store the numeric value instead of corresponding output name
		bool useNumeric;
		//! Stores the model to assume when loading a numeric output offset without a model specifier *from a string/file*, blank means current global #capabilities
		std::string defModel;
		//! If non-empty, indicates the model specifier to use when saving as a numeric (#useNumeric)
		/*! This is set whenever a value with a specifier is loaded from a file. */
		std::string saveModel;
		//! the first acceptable output offset, lower offsets will be set as #UNUSED
		unsigned int rangeBegin;
		//! The one-past-last acceptable output offset, higher offsets will be set as #UNUSED.
		unsigned int rangeEnd;
		//! If true, out-of-range or unknown values will throw an OutputSelector::bad_value exception instead of being marked UNUSED
		bool throwInvalid;
		//! If true, will throw an OutputSelector::bad_value exception for attempts to explicitly mark the output UNUSED (e.g. empty string, -1U numeric...)
		bool throwUnused;
	};

	//! Handles angle measurements by adding a 'unit' attribute to numeric values, or a character suffix in a string value. */
	/*! Values are always stored in radians, but may be converted from/to degrees when loading/saving.
	 *
	 *  Valid values for a 'unit' attribute are: rad, radian, radians, °, deg, degree, degrees, π, pi, %, percent.
	 *
	 *  Valid values for a suffix are: °, π, π/, %.  The 'π/' should be followed by another number to form a fraction, e.g. '3π/2' or 'π/3'.
	 *
	 *  To conform completely to Apple's plist specification, the 'unit' attribute should be avoided,
	 *  and unit suffixes should only be used in 'string' elements.  This code can handle both, but other
	 *  editors might not.  Use of the setPedantic() function can give a warning when such values are
	 *  loaded, and rely on solely on strings for non-radian values.
	 */
	class Angle : public Primitive<PLISTREAL> {
	public:
		enum Format {
			FORMAT_NONE, //!< value will be in radians, but no unit will be specified
			FORMAT_RADIAN, //!< value will be in radians
			FORMAT_DEGREE, //!< value will be in degrees
			FORMAT_PI, //!< value will be multiple of pi
			FORMAT_PERCENT, //!< value will be a percentage of a circle
			FORMAT_SAME, //!< use same units that was loaded, or 'auto' if not loaded
			FORMAT_AUTO //!< uses degrees if value works out to be a integer, or raw radian otherwise
		};
		
		enum Pedantic {
			PEDANTIC_FULL, //!< uses 'string' elements for plist compliance, though less intuitive since these are really numeric values
			PEDANTIC_VALID, //!< uses a 'unit' attribute on the value element to specify the units... may be stripped by editor, but validates
			PEDANTIC_CUTE //!< uses a 'real' element, regardless of presence of suffix... editor may refuse to open due to non-numeric characters
		};
		
		Angle() : Primitive<PLISTREAL>(0), saveFormat(FORMAT_SAME), loadedFormat(FORMAT_AUTO) {} //!< constructor
		Angle(PLISTREAL v) : Primitive<PLISTREAL>(v), saveFormat(FORMAT_SAME), loadedFormat(FORMAT_AUTO) {} //!< casting constructor
		Angle& operator=(const std::string& v) { set(v); return *this; }
		using Primitive<PLISTREAL>::operator=;
		
		void loadXML(xmlNode* node);
		void saveXML(xmlNode* node) const;
		void set(const std::string& str);
		using Primitive<PLISTREAL>::set;
		void setDegree(PLISTREAL v) { operator=(v/180*static_cast<PLISTREAL>(M_PI)); }
		PLISTREAL getDegree() const { return val/static_cast<PLISTREAL>(M_PI)*180; }
		std::string get() const;
		
		void setFormat(Format fmt) { saveFormat=fmt; } //!< sets #saveFormat
		Format getFormat() const { return saveFormat; } //!< returns #saveFormat
		Format getLoadedFormat() const { return loadedFormat; } // !< returns #loadedFormat
		
		static void setDefaultFormat(Format fmt) { defaultFormat = fmt; } //!< sets #defaultFormat
		static Format getDefaultFormat() { return defaultFormat; } //!< returns #defaultFormat
		
		static void setPedantic(Pedantic p) { pedantic=p; } //!< sets #pedantic
		static Pedantic getPedantic() { return pedantic; } //!< returns #pedantic
		
		//! clone definition for Angle
		PLIST_CLONE_DEF(Angle,new Angle(val));
	protected:
		void parseRadian(const std::string& str);
		
		//! Specifies what format we should save in
		Format saveFormat;
		//! Specifies the format last loaded (to be reused if saveFormat is FORMAT_SAME)
		Format loadedFormat;
		//! The format to use if loadedFormat is FORMAT_AUTO;
		static Format defaultFormat;
		//! If true, saved elements will use 'string' instead of 'real' with an element, and warnings will be issued
		static Pedantic pedantic;
	};
	
	//! A simple class for storing 3D points, will be serialized as an array and provides operator[], but you can also use the #x, #y, #z fields directly if preferred
	class Point : public virtual plist::ArrayOf<plist::Primitive<PLISTREAL> > {
	public:
		typedef PLISTREAL storage_t; //!< storage type for point (could make this a template, seems unnnecessary)
		
		//! constructor
		Point() : x(0), y(0), z(0) { init(); }
		//! constructor with data point
		Point(storage_t x_, storage_t y_, storage_t z_) : x(x_), y(y_), z(z_) { init(); }
		//! copy constructor
		/*! This version is needed in addition to the templated 'copy constructor', gcc doesn't recognize,
		 *  template instance as copy constructor, winds up creating another default copy constructor
		 *  which doesn't init properly */
		Point(const Point& p) : x(p[0]), y(p[1]), z(p[2]) { init(); }
		//! conversion from other point class (e.g. C array, fmat::Column, std::vector, etc.)
		template<class T> Point(const T& p) : x(p[0]), y(p[1]), z(p[2]) { init(); }
		
		//! copies data from @a a via T::operator[] into (x, y, z), returns *this for convenience
		template<typename T> Point& importFrom(const T& a) { x=a[0]; y=a[1]; z=a[2]; return *this; }
		//! copies data from (x, y, z) into a T instance via T(x,y,z) constructor call
		template<typename T> T construct() const { return T(x,y,z); }
		//! copies data from (x, y, z) into a T instance via T::operator[]
		template<typename T> T exportTo() const { T a; a[0]=x; a[1]=y; a[2]=z; return a; }
		//! copies data from (x, y, z) into @a a via T::operator[], returns reference to a for convenience
		template<typename T> T& exportTo(T& a) const { a[0]=x; a[1]=y; a[2]=z; return a; }
		
		//! sets x,y,z values
		void set(storage_t x_, storage_t y_, storage_t z_) { x=x_; y=y_; z=z_; }
		using plist::ArrayOf<plist::Primitive<PLISTREAL> >::set;
		
		void saveXML(xmlNode* node) const {
			if(node==NULL)
				return;
			if(x==y && y==z) {
				x.saveXML(node);
			} else {
				plist::ArrayOf<plist::Primitive<storage_t> >::saveXML(node);
			}
		}
		void loadXML(xmlNode* node) {
			if(node==NULL)
				return;
			if(xNodeHasName(node,"real")) {
				x.loadXML(node);
				z = y = x;
			} else {
				plist::ArrayOf<plist::Primitive<storage_t> >::loadXML(node);
			}
		}
		
		plist::Primitive<storage_t> x; //!< x coordinate
		plist::Primitive<storage_t> y; //!< y coordinate
		plist::Primitive<storage_t> z; //!< z coordinate
		
	protected:
		//! add entries for data fields
		void init() {
			addEntry(x);
			addEntry(y);
			addEntry(z);
			setLoadSavePolicy(FIXED,SYNC);
			setSaveInlineStyle(true);
		}			
	};
		
	
	template<class T>
	class RGBColor : virtual public DictionaryOf<plist::Primitive<T> >, virtual public ArrayOf<plist::Primitive<T> > {
	public:
		RGBColor() : DictionaryOf<plist::Primitive<T> >(), ArrayOf<plist::Primitive<T> >(), red(0), green(0), blue(0), alpha(255) { init(); }
		RGBColor(T r, T g, T b, T a=getMax()) : DictionaryOf<plist::Primitive<T> >(), ArrayOf<plist::Primitive<T> >(), red(r), green(g), blue(b), alpha(a) { init(); }
		
		plist::Primitive<T> red;
		plist::Primitive<T> green;
		plist::Primitive<T> blue;
		plist::Primitive<T> alpha;
		
		virtual void loadXML(xmlNode * node);
		virtual void saveXML(xmlNode * node) const;
		using DictionaryOf<plist::Primitive<T> >::saveXML;
		virtual void set(const RGBColor<T>& x);
		virtual void set(const plist::ObjectBase& x);
		virtual void set(const std::string& str);
		using DictionaryOf<plist::Primitive<T> >::set;
		using ArrayOf<plist::Primitive<T> >::set;
		using DictionaryOf<plist::Primitive<T> >::operator[];
		using ArrayOf<plist::Primitive<T> >::operator[];
		// to avoid 'ambiguous' usage of rgb[0] (size_t vs. string constructor)
		plist::Primitive<T>& operator[](int i) { return ArrayOf<plist::Primitive<T> >::operator[](i); }
		const plist::Primitive<T>& operator[](int i) const { return ArrayOf<plist::Primitive<T> >::operator[](i); }
		
		virtual std::string toString() const;
		
		virtual size_t size() const { return ArrayOf<plist::Primitive<T> >::size(); }

		virtual void clear() {
			DictionaryOf<plist::Primitive<T> >::clear();
			ArrayOf<plist::Primitive<T> >::clear();
			init();
		}
		
		ObjectBase* resolveEntry(const std::string& s) const {
			ObjectBase* obj = DictionaryOf<plist::Primitive<T> >::resolveEntry(s);
			return (obj!=NULL) ? obj : ArrayOf<plist::Primitive<T> >::resolveEntry(s);
		}
		unsigned int getLongestKeyLen(const regex_t* reg, unsigned int depth) const { return DictionaryOf<plist::Primitive<T> >::getLongestKeyLen(reg,depth); }
		virtual bool canContain(const ObjectBase& obj) { return DictionaryOf<plist::Primitive<T> >::canContain(obj); }
		
		PLIST_CLONE_DEF(RGBColor,new RGBColor<T>(red,green,blue,alpha));

	protected:
		virtual void fireEntryRemoved(ObjectBase& val);
		
		static float makeFloat(const T& v) { return v/(float)getMax(); }
		static int makeInt(const T& v) { return static_cast<int>(makeFloat(v)*255 + 0.5); }
		static T fromByte(unsigned char x) { return static_cast<T>(x)<<(8*(sizeof(T)-sizeof(unsigned char))); }
		static T fromReal(double x) { return static_cast<T>(x*getMax()+.5); }
		static T getMax() { return std::numeric_limits<T>::max(); }
		static void setNumeric(plist::Primitive<T>&) {}
		
		void init() {
			DictionaryOf<plist::Primitive<T> >::addEntry("Red",red);
			DictionaryOf<plist::Primitive<T> >::addEntry("Green",green);
			DictionaryOf<plist::Primitive<T> >::addEntry("Blue",blue);
			DictionaryOf<plist::Primitive<T> >::addEntry("Alpha",alpha);
			ArrayOf<plist::Primitive<T> >::addEntry(red);
			ArrayOf<plist::Primitive<T> >::addEntry(green);
			ArrayOf<plist::Primitive<T> >::addEntry(blue);
			ArrayOf<plist::Primitive<T> >::addEntry(alpha);
			setNumeric(red);
			setNumeric(green);
			setNumeric(blue);
			setNumeric(alpha);
			Collection::setLoadSavePolicy(Collection::FIXED,Collection::SYNC);
			ArrayOf<plist::Primitive<T> >::setSaveInlineStyle(true);
		}
	};
	
	template<> inline float RGBColor<float>::getMax() { return 1; }
	template<> inline double RGBColor<double>::getMax() { return 1; }
	
	template<> inline unsigned char RGBColor<unsigned char>::fromByte(unsigned char x) { return x; }
	template<> inline char RGBColor<char>::fromByte(unsigned char x) { return static_cast<int>(x)-128; }
	template<> inline float RGBColor<float>::fromByte(unsigned char x) { return x/255.f; }
	template<> inline double RGBColor<double>::fromByte(unsigned char x) { return x/255.0; }
	
	template<> inline float RGBColor<float>::fromReal(double x) { return static_cast<float>(x); }
	template<> inline double RGBColor<double>::fromReal(double x) { return x; }
	
	template<> inline void RGBColor<char>::setNumeric(plist::Primitive<char>& n) { n.setNumeric(true); }
	template<> inline void RGBColor<unsigned char>::setNumeric(plist::Primitive<unsigned char>& n) { n.setNumeric(true); }
	
	template<class T> void RGBColor<T>::loadXML(xmlNode * node) {
		if(ObjectBase::xNodeHasName(node,"array")) {
			ArrayOf<plist::Primitive<T> >::loadXML(node);
		} else if(ObjectBase::xNodeHasName(node,"dict")) {
			DictionaryOf<plist::Primitive<T> >::loadXML(node);
		} else if(ObjectBase::xNodeHasName(node,"string")) {
			xmlChar * cont=xmlNodeGetContent(node);
			try {
				set(std::string((const char*)cont));
			} catch(const XMLLoadSave::bad_format& err) {
				xmlFree(cont);
				throw XMLLoadSave::bad_format(node,err.what()); //add current node to exception and rethrow
			} catch(...) {
				xmlFree(cont);
				throw;
			}
			xmlFree(cont);
		} else {
			std::stringstream ss;
			ss << "plist::RGBColor can't accept value of type "<< ObjectBase::xNodeGetName(node);
			ss << ", (try CSS-style string or array of RGB values in range 0-" << getMax() << ")";
			throw XMLLoadSave::bad_format(node,ss.str());
		}
	}
	
	template<class T> void RGBColor<T>::saveXML(xmlNode * node) const {
		if(ObjectBase::xNodeHasName(node,"array")) {
			ArrayOf<plist::Primitive<T> >::saveXML(node);
		} else if(ObjectBase::xNodeHasName(node,"dict")) {
			DictionaryOf<plist::Primitive<T> >::saveXML(node);
		} else {
			xmlNodeSetName(node,(const xmlChar*)"string");
			xmlNodeSetContent(node,(const xmlChar*)toString().c_str());
		}
	}
	
	template<class T> void RGBColor<T>::set(const RGBColor<T>& v) {
		red=v.red;
		green=v.green;
		blue=v.blue;
		alpha=v.alpha;
	}
	template<class T> void RGBColor<T>::set(const plist::ObjectBase& x) {
		if(const RGBColor * v = dynamic_cast<const RGBColor*>(&x)) {
			set(*v);
		} else if(const DictionaryBase * d = dynamic_cast<const DictionaryBase*>(&x)) {
			DictionaryOf<plist::Primitive<T> >::set(*d);
		} else if(const ArrayBase * a = dynamic_cast<const ArrayBase*>(&x)) {
			ArrayOf<plist::Primitive<T> >::set(*a);
		}
	}
	
	template<class T> void RGBColor<T>::set(const std::string& str) {
		std::string s = string_util::makeLower(string_util::trim(str));
		if(s[0]=='#') {
			if(s.size()==4) {
				for(unsigned int i=0; i<3; ++i) {
					unsigned char x=0;
					if('0'<=s[i+1] && s[i+1]<='9')
						x = s[i+1]-'0';
					else if('a'<=s[i+1] && s[i+1]<='f')
						x = s[i+1]-'a'+10;
					else
						throw XMLLoadSave::bad_format(NULL,"plist::RGBColor bad character in hex specification: "+str);
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromByte(x*16+x);
				}
				alpha=getMax();
			} else if(s.size()==7) {
				for(unsigned int i=0; i<3; ++i) {
					char ca=s[i*2+1], cb=s[i*2+2];
					unsigned char xa=0, xb=0;
					if('0'<=ca && ca<='9')
						xa = ca-'0';
					else if('a'<=ca && ca<='f')
						xa = ca-'a'+10;
					else
						throw XMLLoadSave::bad_format(NULL,"plist::RGBColor bad character in hex specification: "+str);
					if('0'<=cb && cb<='9')
						xb = cb-'0';
					else if('a'<=cb && cb<='f')
						xb = cb-'a'+10;
					else
						throw XMLLoadSave::bad_format(NULL,"plist::RGBColor bad character in hex specification: "+str);
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromByte(xa*16+xb);
				}
				alpha=getMax();
			} else {
				throw XMLLoadSave::bad_format(NULL,"plist::RGBColor wrong number of characters (3 or 6) in hex specification: "+str);
			}
		} else if(s.substr(0,4)=="rgb(") {
			std::vector<std::string> tok = string_util::tokenize(s.substr(4),",");
			if(tok.size()!=3)
				throw XMLLoadSave::bad_format(NULL,"plist::RGBColor expects three arguments for rgb() specification");
			for(unsigned int i=0; i<tok.size(); ++i) {
				std::string t = string_util::trim(tok[i]);
				if(t[t.size()-1]=='%') {
					float x;
					std::stringstream(t.substr(0,t.size()-1)) >> x;
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromReal(x/100.0);
				} else {
					unsigned int x;
					std::stringstream(t) >> x;
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromByte(x);
				}
			}
			alpha=getMax();
		} else if(s.substr(0,5)=="rgba(") {
			std::vector<std::string> tok = string_util::tokenize(s.substr(5),",");
			if(tok.size()!=4)
				throw XMLLoadSave::bad_format(NULL,"plist::RGBColor expects four arguments for rgba() specification");
			for(unsigned int i=0; i<3; ++i) {
				std::string t = string_util::trim(tok[i]);
				if(t[t.size()-1]=='%') {
					double x;
					std::stringstream(t.substr(0,t.size()-1)) >> x;
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromReal(x/100.0);
				} else {
					unsigned int x;
					std::stringstream(t) >> x;
					ArrayOf<plist::Primitive<T> >::operator[](i)=fromByte(x);
				}
			}
			std::string t = string_util::trim(tok[3]);
			double x;
			std::stringstream(t) >> x;
			alpha=fromReal(x);
		} else {
			if(s=="black") {
				red=green=blue=0;
			} else if(s=="silver") {
				red=green=blue=fromByte(0xC0);
			} else if(s=="gray") {
				red=green=blue=fromByte(0x80);
			} else if(s=="white") {
				red=green=blue=fromByte(0xFF);
			} else if(s=="maroon") {
				red=fromByte(0x80); green=blue=0;
			} else if(s=="red") {
				red=fromByte(0xFF); green=blue=0;
			} else if(s=="purple") {
				red=blue=fromByte(0x80); green=0;
			} else if(s=="fuchsia") {
				red=blue=fromByte(0xFF); green=0;
			} else if(s=="green") {
				green=fromByte(0x80); red=blue=0;
			} else if(s=="lime") {
				green=fromByte(0xFF); red=blue=0;
			} else if(s=="olive") {
				red=green=fromByte(0x80); blue=0;
			} else if(s=="yellow") {
				red=green=fromByte(0xFF); blue=0;
			} else if(s=="navy") {
				blue=fromByte(0x80); red=green=0;
			} else if(s=="blue") {
				blue=fromByte(0xFF); red=green=0;
			} else if(s=="teal") {
				green=blue=fromByte(0x80); red=0;
			} else if(s=="aqua") {
				green=blue=fromByte(0xFF); red=0;
			} else {
				throw XMLLoadSave::bad_format(NULL,"plist::RGBColor unknown specification: "+str);
			}
			alpha=getMax();
		}
	}
	
	template<class T> std::string RGBColor<T>::toString() const {
		std::stringstream ss;
		ss << (alpha!=getMax() ? "rgba(" : "rgb(");
		ss << makeInt(red) << ',' << makeInt(green) << ',' << makeInt(blue);
		if(alpha!=getMax())
			ss << ',' << makeFloat(alpha);
		ss <<')';
		return ss.str();
	}
	
	PLIST_CLONE_IMPT(T,RGBColor,new RGBColor<T>(red,green,blue,alpha));

	template<class T> void RGBColor<T>::fireEntryRemoved(ObjectBase& val) {
		Collection::fireEntryRemoved(val);
		std::set<ObjectBase*>::iterator it=DictionaryOf<plist::Primitive<T> >::myRef.find(&val);
		bool deleted=false;
		if(it!=DictionaryOf<plist::Primitive<T> >::myRef.end()) {
			DictionaryOf<plist::Primitive<T> >::myRef.erase(it);
			if(!deleted) {
				delete &val;
				deleted=true;
			}
		}
		it=ArrayOf<plist::Primitive<T> >::myRef.find(&val);
		if(it!=ArrayOf<plist::Primitive<T> >::myRef.end()) {
			ArrayOf<plist::Primitive<T> >::myRef.erase(it);
			if(!deleted) {
				delete &val;
				deleted=true;
			}
		}
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
