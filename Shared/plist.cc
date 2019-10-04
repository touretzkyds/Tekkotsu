#include "plist.h"
#include <stdexcept>

using namespace std;

namespace plist {

	ObjectBase* loadXML(xmlNode* node) {
		ObjectBase* obj=NULL;
		if(ObjectBase::xNodeHasName(node,"array")) {
			Array * a=new Array;
			a->setLoadSavePolicy(Collection::SYNC,Collection::SYNC); // just in case defaults change...
			obj=a;
		} else if(ObjectBase::xNodeHasName(node,"dict")) {
			Dictionary * d=new Dictionary;
			d->setLoadSavePolicy(Collection::SYNC,Collection::SYNC); // just in case defaults change...
			obj=d;
		} else if(ObjectBase::xNodeHasName(node,"real"))
			obj=new Primitive<double>;
		else if(ObjectBase::xNodeHasName(node,"integer"))
			obj=new Primitive<long>;
		else if(ObjectBase::xNodeHasName(node,"string"))
			obj=new Primitive<std::string>;
		else if(ObjectBase::xNodeHasName(node,"true"))
			obj=new Primitive<bool>;
		else if(ObjectBase::xNodeHasName(node,"false"))
			obj=new Primitive<bool>;
		else
			return NULL;
		obj->loadXML(node);
		return obj;
	}
	
	//! allows us to use the LoadSave suite for loading and parsing general XML functions, but forwards loadXML to plist::loadXML() and stores the result as a member
	class PolymorphicLoader : public ObjectBase {
	public:
		PolymorphicLoader() : ObjectBase(), obj(NULL) {} //!< constructor
		virtual ~PolymorphicLoader() {} //!< destructor
		PolymorphicLoader(const PolymorphicLoader& p) : ObjectBase(p), obj(NULL) {} //!< copy constructor -- doesn't copy #obj, sets the local instance to NULL
		PolymorphicLoader& operator=(const PolymorphicLoader& p) { ObjectBase::operator=(p); return *this; } //!< copy constructor -- doesn't copy #obj, keeps current value of #obj
		virtual void loadXML(xmlNode* node) { obj=plist::loadXML(node); } //!< forward call to the static plist implementation
		virtual void saveXML(xmlNode * /*node*/) const { throw std::logic_error("PolymorphicLoader::saveXML called, should not happen (saveXML on the object itself...)"); } //!< shouldn't ever be called -- call saveXML() on #obj directly
		
		virtual std::string toString() const { return ""; } //!< shouldn't ever be called
		virtual long toLong() const { return 0; } //!< shouldn't ever be called
		virtual double toDouble() const { return 0; } //!< shouldn't ever be called
		virtual void set(const ObjectBase&) {} //!< shouldn't ever be called
		PLIST_CLONE_DEF(PolymorphicLoader,new PolymorphicLoader); //!< shouldn't ever be called
		
		ObjectBase * obj; //!< storage of results from loadXML
	};
	PLIST_CLONE_IMP(PolymorphicLoader,new PolymorphicLoader);
	
	ObjectBase* loadFile(const std::string& file) {
		PolymorphicLoader loader;
		if(!loader.loadFile(file.c_str()))
			return NULL;
		return loader.obj;
	}
	
	ObjectBase* loadBuffer(const char* buf, unsigned int len) {
		PolymorphicLoader loader;
		if(!loader.loadBuffer(buf,len))
			return NULL;
		return loader.obj;
	}
	
} //namespace plist

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
