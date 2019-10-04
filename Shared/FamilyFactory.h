//-*-c++-*-
#ifndef INCLUDED_FamilyFactory_h_
#define INCLUDED_FamilyFactory_h_

#include "Factories.h"
#include <string>
#include <map>
#include <set>
#include <iostream>

//! A class which can track a set of subclass types (with shared base class FamilyT), generating new instances on demand based on some form of identifier (the NameT template parameter)
/*! Register your class via registerType() or registerFactory(), and then instantiate it later
 *  with a call to create().  Note that you have a variety of options to control how the product
 *  is constructed via your choice of the FactoryBaseT and FactoryT template parameters.
 *
 *  The FactoryBaseT defines the interface, exposing available factory functor calls which return
 *  the shared base class.  The FactoryT is a template template parameter which indicates
 *  the factory to be created for each call to registerType().  This should take the type to be
 *  produced as its template parameter, and implement the interface expected by FactoryBaseT.
 *
 *  By default, FamilyFactory looks for a type named 'Factory' within the FactoryBaseT namespace.
 *  This parameter would be ignored if you only ever call registerFactory, and pass your own factory
 *  instances.
 *
 *  A cute way to automatically trigger type registration is to define a static member of your class,
 *  and then call registerType() or registerFactory() for its initialization.
 *
 *  See Factories.h for some basic factory types. */
template<class FamilyT, typename NameT=std::string, class FactoryBaseT=Factory0Arg<FamilyT>, template<class U> class FactoryT=FactoryBaseT::template Factory>
class FamilyFactory {
public:
	//! allows indirect access to the product family type
	typedef FamilyT FamilyType;
	//! allows indirect access to the product class name type
	typedef NameT NameType;
	//! allows indirect access to the factory base class
	typedef FactoryBaseT FactoryBaseType;
	//! allows indirect access to create factory classes
	template<class T> struct FactoryType : FactoryT<T> {};
	
	//! default constructor
	FamilyFactory() : factories() {}
	
	//! destructor
	virtual ~FamilyFactory() {
		for(typename factories_t::iterator it=factories.begin(); it!=factories.end(); ++it)
			delete it->second;
		factories.clear();
	}

	//! returns a singleton factory for the current template arguments... be careful that clients use the same templates as those used to register
	static FamilyFactory& getRegistry() { static FamilyFactory registry; return registry; }
	
	//! clears @a types and fills in the currently registered type identifiers
	template<typename N> void getTypeNames(std::set<N>& types) const;
	
	//! returns the number of currently registered type identifiers
	unsigned int getNumTypes() const { return factories.size(); }
	
	//! creates a factory for the specified type from FactoryT and registers it as @a type
	template<typename T> const NameT& registerType(const NameT& type) { return registerFactory(type,new FactoryT<T>); }
	
	//! registers the specified factory for producing objects known by @a type; FamilyFactory assumes responsibilty for deallocation of @a f
	const NameT& registerFactory(const NameT& type, FactoryBaseT* f);
	
	//! requests a new instance of the specified @a type be created, without passing any arguments to the factory
	/*! Note that the factory interface chosen by FactoryBaseT may not actually provide such a call! */
	FamilyT* create(const NameT& type) const { FactoryBaseT * f=lookupFactory(type); return f ? (*f)() : NULL; }
	
	//! requests a new instance of the specified @a type be created, passing a single argument to the factory
	/*! Note that the factory interface chosen by FactoryBaseT may not actually provide such a call! */
	template<typename A1>
	FamilyT* create(const NameT& type, const A1& a1) const { FactoryBaseT * f=lookupFactory(type); return f ? (*f)(a1) : NULL; }
	
	//! requests a new instance of the specified @a type be created, passing two arguments to the factory
	/*! Note that the factory interface chosen by FactoryBaseT may not actually provide such a call! */
	template<typename A1, typename A2>
	FamilyT* create(const NameT& type, const A1& a1, const A2& a2) const { FactoryBaseT * f=lookupFactory(type); return f ? (*f)(a1,a2) : NULL; }
	
	//! requests a new instance of the specified @a type be created, passing three arguments to the factory
	/*! Note that the factory interface chosen by FactoryBaseT may not actually provide such a call! */
	template<typename A1, typename A2, typename A3>
	FamilyT* create(const NameT& type, const A1& a1, const A2& a2, const A3& a3) const { FactoryBaseT * f=lookupFactory(type); return f ? (*f)(a1,a2,a3) : NULL; }
	
protected:
	//! utility function to see if @a type has been registered and return it, or NULL if not found
	FactoryBaseT* lookupFactory(const NameT& type) const;
		
	//! type of #factories
	typedef std::map<NameT,FactoryBaseT*> factories_t;
	factories_t factories; //!< storage for type identifier to factory mapping
};

template<class FamilyT, typename NameT, class FactoryBaseT, template<class U> class FactoryT>
template<typename N>
void FamilyFactory<FamilyT,NameT,FactoryBaseT,FactoryT>::getTypeNames(std::set<N>& types) const {
	types.clear();
	for(typename factories_t::const_iterator it=factories.begin(); it!=factories.end(); ++it)
		types.insert(it->first);
}

template<class FamilyT, typename NameT, class FactoryBaseT, template<class U> class FactoryT>
const NameT& FamilyFactory<FamilyT,NameT,FactoryBaseT,FactoryT>::registerFactory(const NameT& type, FactoryBaseT* f) {
		typename factories_t::const_iterator it=factories.find(type);
		if(it!=factories.end()) {
			std::cerr << "WARNING: Type " << type << " was already registered!  Overwriting previous..."  << std::endl;
			delete it->second;
		}
		factories[type]=f;
		return type;
}

template<class FamilyT, typename NameT, class FactoryBaseT, template<class U> class FactoryT>
FactoryBaseT* FamilyFactory<FamilyT,NameT,FactoryBaseT,FactoryT>::lookupFactory(const NameT& type) const {
	typename factories_t::const_iterator it=factories.find(type);
	if(it==factories.end())
		return NULL;
	return it->second;
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
