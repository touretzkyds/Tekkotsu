//-*-c++-*-
#ifndef INCLUDED_plistCollections_h_
#define INCLUDED_plistCollections_h_

#include "plistPrimitives.h"
#include <map>
#include <vector>
#include <set>
#include <regex.h>

namespace plist {
	ObjectBase* loadXML(xmlNode* node); // defined in plist.h, we need the prototype
	
	//! allocates a new T instance, unless T is a known abstract type (ObjectBase, PrimitiveBase, Collection), in which case will throw a bad_cast via template specialization
	template<class T> T* allocate() { return new T; }
	//! assigns one T to another using operator=, unless T is ObjectBase, in which case set() is used via template specialization
	template<class T> void assign(T& a, const T& b) { a=b; }
	
	//! @cond INTERNAL
	template<> inline ObjectBase* allocate() { throw std::runtime_error("cannot plist::allocate generic instances (ObjectBase)"); }
	template<> inline PrimitiveBase* allocate() { throw std::runtime_error("cannot plist::allocate generic instances (PrimitiveBase)"); }
	template<> inline Collection* allocate() { throw std::runtime_error("cannot plist::allocate generic instances (Collection)"); }
	template<> inline void assign(ObjectBase& a, const ObjectBase& b) { a.set(b); }
	//! @endcond
	
	//! attempts to load a new T instance from the specified xmlNode
	template<class T> T* loadXML(xmlNode* node) {
		T* cobj = plist::allocate<T>();
		try {
			cobj->loadXML(node);
		} catch(...) {
			delete cobj;
			throw;
		}
		return cobj;
	}

	//! @cond INTERNAL
	template<> ObjectBase* loadXML(xmlNode* node);
	template<> PrimitiveBase* loadXML(xmlNode* node);
	template<> Collection* loadXML(xmlNode* node);
	//! @endcond
	
	//! Provides a common base class for the collection-oriented primitives, Dictionary and Array
	/*! 
	 *  When a collection, you can call addEntry() or setEntry() you can either:
	 *    - pass a pointer to an ObjectBase or directly pass a primitive value (int, float, char, etc.),
	 *      in which case the Array will assume management of the corresponding ObjectBase
	 *      instance (freeing the memory region when removed)
	 *    - pass a reference to an ObjectBase, in which case you retain control over the object's
	 *      allocation
	 *
	 *  This class supports callbacks upon modification through the use of the
	 *  CollectionListener interface.  Note that we only store a pointer to the
	 *  listener list, which is typically unallocated when no listeners are
	 *  active.  This should ensure minimal memory usage per object, as well as
	 *  support safe storage of plist objects in inter-process shared memory
	 *  regions.
	 *
	 *  If you are using these in a shared memory region, just be sure that only
	 *  the process with listeners does any and all modifications, and that it
	 *  unsubscribes before detaching from the region (or else destroys the region
	 *  itself)
	 *
	 *  There isn't a callback if entries themselves are modified, only when new
	 *  entries are added, or old ones removed.  If you want to know any time any
	 *  aspect of any entry is modified, listen for the add and remove callbacks,
	 *  and then add yourself as a listener to each entry individually.  */
	class Collection : public ObjectBase {
	public:
		//! Specifies that a collection of collections cannot contain any primitive values
		template<typename U, typename V> struct conversion_policy { typedef typename U::DeniedValueConversions value_conversion; };
		
		//! used to define values for LoadSavePolicy values so we can test a bit out of the policy value
		enum LoadSaveActionBitMask {
			ADDITIONS=1, //!< if this bit is set in #loadPolicy's value, entries not found in the collection will be added, or for #savePolicy, entries will be added to the file
			REMOVALS=2 //!< if this bit is set in #loadPolicy's value, entries not found in the file will be removed from the collection, or for #savePolicy, entries will be removed from the file
		};
		
		//! Arguments for setLoadPolicy() and setSavePolicy(), specifies how to handle orphaned entries when loading or saving
		/*! <table>
		 *  <tr><th>@b Loading</th><th>SYNC</th><th>INTERSECT</th><th>UNION</th><th>FIXED</th></tr>
		 *  <tr><td>entry not in file <br> (missing from source)</td><td>remove from<br>collection</td><td>remove from<br>collection</td><td>ignore</td><td>ignore</td></tr>
		 *  <tr><td>entry not in collection <br> (missing from destination)</td><td>add to<br>collection</td><td>ignore</td><td>add to<br>collection</td><td>ignore</td></tr>
		 *  </table> <br>
		 *  
		 *  <table>
		 *  <tr><th>@b Saving</th><th>SYNC</th><th>INTERSECT</th><th>UNION</th><th>FIXED</th></tr>
		 *  <tr><td>entry not in file <br> (missing from destination)</td><td>add to<br>file</td><td>ignore</td><td>add to<br>file</td><td>ignore</td></tr>
		 *  <tr><td>entry not in collection <br> (missing from source)</td><td>remove<br>from file</td><td>remove<br>from file</td><td>ignore</td><td>ignore</td></tr>
		 *  </table>
		 *  
		 *  Commonly, you'll want to use SYNC (the default) to support dynamically-resized storage, and FIXED load/SYNC save
		 *  for configuration settings (or perhaps FIXED load and UNION save to keep 'extra' values in the file...)*/
		enum LoadSavePolicy {
			FIXED = 0, //!< destination will have the same entries as before, ignores orphans and only updates entries with matching keys
			UNION = ADDITIONS, //!< destination will have its current entries, as well as new ones from the source
			INTERSECT = REMOVALS, //!< destination will only hold entries which are in both locations, removes entries not found in source, ignores new entries
			SYNC = ADDITIONS|REMOVALS //!< destination will mirror source, new entries are added, missing entries are removed
		};

		//! assignment (don't assign listeners); subclass should call fireEntriesChanged after calling this (and updating its own storage)
		Collection& operator=(const Collection& d) { if(&d==this) return *this; ObjectBase::operator=(d); return *this; }
		
		//! destructor
		~Collection();
		
		//! recursively resolves @a path interpreted as a series of collection entry names separated by '.', returns NULL if it doesn't exist
		virtual ObjectBase* resolveEntry(const std::string& path) const=0;
		//! Processes a command of the form 'collection.key = value' or 'collection.array += entry'
		/*! Returns whether the assignment was successful, but does not provide error message (see other version if you want a string to give user) */
		bool resolveAssignment(const std::string& arg);
		//! Processes a command of the form 'collection.key = value' or 'collection.array += entry'
		/*! Returns whether the assignment was successful, and if an error occurrs, places the error message in errstream */
		bool resolveAssignment(const std::string& arg, std::ostream& errstream);
		//! remove all entries in one fell swoop
		virtual void clear()=0;
		//! return the size of the collection
		virtual size_t size() const=0;
		
		//! get notified of changes; be sure to call removeCollectionListener() before destructing @a l!
		virtual void addCollectionListener(CollectionListener* l) const;
		//! no longer take notification of changes to this object's value
		virtual void removeCollectionListener(CollectionListener* l) const;
		//! test if @a l is currently registered as a listener
		virtual bool isCollectionListener(CollectionListener* l) const;
		
		void setSaveCondensed(bool b) { saveCondensed=b; } //!< set #saveCondensed (not automatically recursive)
		bool getSaveCondensed() const { return saveCondensed; } //!< returns #saveCondensed
		
		void setUnusedWarning(bool b) { warnUnused=b; } //!< set #warnUnused
		bool getUnusedWarning() const { return warnUnused; } //!< returns #warnUnused

		virtual LoadSavePolicy getLoadPolicy() const { return loadPolicy; } //!< returns #loadPolicy
		virtual LoadSavePolicy getSavePolicy() const { return savePolicy; } //!< returns #savePolicy
		virtual void setLoadPolicy(LoadSavePolicy lp) { loadPolicy=lp; } //!< assigns #loadPolicy
		virtual void setSavePolicy(LoadSavePolicy sp) { savePolicy=sp; } //!< assigns #savePolicy
		virtual void setLoadSavePolicy(LoadSavePolicy lp, LoadSavePolicy sp) { setLoadPolicy(lp); setSavePolicy(sp); } //!< assigns #loadPolicy and #savePolicy respectively
		
		//! defines separator between sub-collections
		/*!  (defined as a function instead of just a constant member so there's no issues with initialization order) */
		static const std::string& subCollectionSep() {
			static std::string sep(1,'.');
			return sep;
		}
		
		//! returns longest key length which matches the regular expression
		virtual unsigned int getLongestKeyLen(const regex_t* reg=NULL, unsigned int depth=-1) const=0;
		
		//! returns true if the Collection subclass allows storage of the argument
		virtual bool canContain(const ObjectBase& obj)=0;
		
		virtual long toLong() const;
		virtual double toDouble() const;
		
	protected:
		//! constructor
		Collection() : ObjectBase(), collectionListeners(), saveCondensed(false), warnUnused(true), loadPolicy(SYNC), savePolicy(SYNC) {autoFormat=false;}
		//! constructor
		Collection(LoadSavePolicy lp, LoadSavePolicy sp) : ObjectBase(), collectionListeners(), saveCondensed(false), warnUnused(true), loadPolicy(lp), savePolicy(sp) {autoFormat=false;}
		//! copy constructor (don't assign listeners)
		Collection(const Collection& d) : ObjectBase(d), collectionListeners(), saveCondensed(d.saveCondensed), warnUnused(d.warnUnused), loadPolicy(d.loadPolicy), savePolicy(d.savePolicy) {autoFormat=false;}
		
		//! run through #collectionListeners, calling CollectionListener::plistCollectionEntryAdded(*this,val)
		virtual void fireEntryAdded(ObjectBase& val);
		//! run through #collectionListeners, calling CollectionListener::plistCollectionEntryRemoved(*this,val)
		virtual void fireEntryRemoved(ObjectBase& val);
		//! run through #collectionListeners, calling CollectionListener::plistCollectionEntriesChanged(*this)
		virtual void fireEntriesChanged();
		//! stores a list of the current listeners
		mutable std::set<CollectionListener*>* collectionListeners;
		
		//! returns index corresponding to @a name, which should encode an integer value less than or equal to the current size
		static size_t getIndex(const std::string& name);
		
		//! returns a prefix for items within the collection
		static std::string getIndentationPrefix(xmlNode* node);
		
		//! when an empty string is needed for not found items
		/*!  (defined as a function instead of just a constant member so there's no issues with initialization order) */
		static const std::string& emptyStr() {
			static std::string mt;
			return mt;
		}
		//! how much to indent each sub-collection
		/*!  (defined as a function instead of just a constant member so there's no issues with initialization order) */
		static const std::string& perIndent() {
			static std::string pi(1,'\t');
			return pi;
		}
		//! when saving comments to file, the key name will automatically be prepended to the comment, unless the key is found within this many characters from the beginning of the comment
		static const unsigned int KEY_IN_COMMENT_MAX_POS=10;
		
		//! if true, will skip putting "headline" comments before sub-collections
		bool saveCondensed;
		//! if true (the default) loadXML will give warnings using FIXED policy and there are ignored entries in the source while loading or ignored entries in the destination while saving
		bool warnUnused;		
		//! specifies how to handle "orphaned" entries while loading
		LoadSavePolicy loadPolicy;
		//! specifies how to handle "orphaned" entries while saving
		LoadSavePolicy savePolicy;
	};
	
	
	//! Monitors a collection to keep tabs on all its entries, so PrimitiveListener::plistValueChanged will be called if any entry is changed
	struct AutoCollectionListener : public plist::PrimitiveListener, public plist::CollectionListener {
		//! constructor, specify collection to monitor and whether you want plistValueChanged to be called when new entries are added
		AutoCollectionListener(const plist::Collection& source, bool updateOnNewEntry)
		: plist::PrimitiveListener(), src(source), updateOnNew(updateOnNewEntry)
		{
			activate();
		}
		//! destructor, unsubscribe from everything
		~AutoCollectionListener() {
			deactivate();
		}
		//! assignment (no-op)
		AutoCollectionListener& operator=(const AutoCollectionListener&) { return *this; }
		
		virtual void plistCollectionEntryAdded(plist::Collection& col, ObjectBase& primitive);
		virtual void plistCollectionEntryRemoved(plist::Collection& col, ObjectBase& primitive);
		virtual void plistCollectionEntriesChanged(plist::Collection& col);
		virtual void plistSubCollectionAdded(plist::Collection& /*col*/) {} //!< called if any sub collections are added for potential recursive handling
		virtual void plistSubCollectionRemoved(plist::Collection& /*col*/) {} //!< called if any sub collections are removed for potential recursive handling
		
		//! add listeners
		virtual void activate(bool callNow=true) {
			src.addCollectionListener(this);
			if(callNow)
				plistCollectionEntriesChanged(const_cast<plist::Collection&>(src));
		}
		//! remove listeners
		virtual void deactivate();
		
		const plist::Collection& src; //!< the collection being monitored
		bool updateOnNew; //!< whether to call plistValueChanged whenever new primitives are added to the collection
	private:
		//! copy is a bad idea: as a member of a collection, don't want to listen to original collection's values
		AutoCollectionListener(const AutoCollectionListener&);
	};
	//! Produces a callback on a member function when a value in the collection changes
	template<typename O>
	struct CollectionCallbackMember : public plist::AutoCollectionListener {
		//! constructor
		CollectionCallbackMember(const plist::Collection& source, O& component, void(O::*func)(), bool callOnAdd=true)
		: plist::AutoCollectionListener(source,callOnAdd), comp(component), callback(func) { if(updateOnNew) (comp.*callback)(); }
		virtual void plistValueChanged(const plist::PrimitiveBase& /*pl*/) { (comp.*callback)(); }
		virtual void activate(bool callNow=true) { if(updateOnNew && callNow) (comp.*callback)(); }
		O& comp; //!< instance to call on
		void(O::*callback)(); //!< function to call
		
		CollectionCallbackMember& operator=(const CollectionCallbackMember&) { return *this; } //!< assignment is a no-op
	private:
		CollectionCallbackMember(CollectionCallbackMember&);
	};
	
	
	
	
	//! Maintains a set of (key,value) pairs, see DictionaryOf, and the Dictionary typedef
	/*! You can add or set entries by a quite a few variations on addEntry(), setEntry(), or addValue().
	 *  Basically these boil down to either:
	 *    - pass a pointer to an ObjectBase or directly pass a primitive value (int, float, char, etc.),
	 *      in which case the dictionary will assume management of the corresponding ObjectBase
	 *      instance (freeing the memory region when removed)
	 *    - pass a reference to an ObjectBase, in which case you retain control over the object's
	 *      allocation
	 */
	class DictionaryBase : virtual public Collection {
		friend std::ostream& operator<<(std::ostream& os, const DictionaryBase& d);
	public:
		//! shorthand for the type of the storage
		typedef std::map<std::string,ObjectBase*> storage_t;
		//! shorthand for iterators to be returned
		typedef storage_t::iterator iterator;
		//! shorthand for iterators to be returned
		typedef storage_t::const_iterator const_iterator;
		
		//! Indicates that no value conversions are allowed
		/*! The actual storage type is still allowed, so technically we could use EntryConstraint<PO>
		 *  instead as the conversion policy, but that doesn't gain anything because you would need
		 *  to know the PO to test for it.  At least with this you can test for DeniedValueConversions as a base
		 *  class and then fall back to testing individual PO's if you want. */
		struct DeniedValueConversions {
			virtual ~DeniedValueConversions() {} //!< no-op destructor
		};
		
		//! This base conversion policy doesn't actually specify any conversions at all -- this serves as a base class to provide the ability to directly add entries of the specified type; the subclasses will add value conversions
		/*! This class is useless to end users: to use it they would have to know the template type being used,
		 *  which if they knew, they could just dynamic_cast to the DictionaryOf type directly.  The point of this
		 *  class is to provide the abstract functions to the its subclasses, which use them to implement their
		 *  various conversions. */
		template<typename PO>
		struct EntryConstraint {
			virtual ~EntryConstraint() {} //!< no-op destructor
			//! insert a new entry to the dictionary, with key @a name and value @a val (replaces any previous entry by same name, but can give a warning)
			virtual void setEntry(const std::string& name, PO& val, bool warnExists=false)=0;
			//! insert a new entry to the dictionary, with key @a name and value @a val (replaces any previous entry by same name with a warning)
			virtual void addEntry(const std::string& name, PO& val, const std::string& comment="", bool warnExists=true)=0;
			//! insert a new entry to the dictionary, with key @a name and value @a val (replaces any previous entry by same name, but can give a warning); control of (de)allocation will be assumed by the dictionary
			virtual void setEntry(const std::string& name, PO* val, bool warnExists=false)=0;
			//! insert a new entry to the dictionary, with key @a name and value @a val (replaces any previous entry by same name with a warning); control of (de)allocation will be assumed by the dictionary
			virtual void addEntry(const std::string& name, PO* val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		//! Abstract base class to test whether the collection will accept strings (possibly converting to a value type, or storing directly as string depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::StringConversion>, and if it is successful, you
		 *  can pass the string without having to know the actual value type of the dictionary. */
		struct StringConversion {
			virtual ~StringConversion() {} //!< no-op destructor
			//! generic addition of value, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, const std::string& val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		//! Abstract base class to test whether the collection will accept integers (possibly converting to another value type, or storing directly as a [unsigned] long depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::IntegerConversion>, and if it is successful, you
		 *  can pass the data without having to know the actual value type of the dictionary. */
		struct IntegerConversion {
			virtual ~IntegerConversion() {} //!< no-op destructor
			//! generic addition of value, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, long val, const std::string& comment="", bool warnExists=true)=0;
			//! generic addition of value, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, unsigned long val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		//! Abstract base class to test whether the collection will accept floating point numbers (possibly converting to another value type, or storing directly as a double depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::RealConversion>, and if it is successful, you
		 *  can pass the data without having to know the actual value type of the dictionary. */
		struct RealConversion {
			virtual ~RealConversion() {} //!< no-op destructor
			//! generic addition of value, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, double val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		//! This conversion policy accepts entries of the specified template type, and will try to create new instances of that type constructed from any values which are passed.
		/*! Use of this conversion policy requires that the template parameter is not abstract,
		 *  as the policy will be trying to create new instances directly from the specified type. */
		template<typename PO>
		struct ConversionTo : public StringConversion, public IntegerConversion, public EntryConstraint<PO> {
			//! insert a new entry to the map, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., or one of the variable-sized Collection's, like Array, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			template<typename T>
			void addValue(const std::string& name, const T& val, const std::string& comment="", bool warnExists=true) { addEntry(name,new PO(val),comment,warnExists); }
			virtual void addValue(const std::string& name, const std::string& val, const std::string& comment="", bool warnExists=true) { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(name,po,comment,warnExists); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, char val[], const std::string& comment="", bool warnExists=true) { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(name,po,comment,warnExists); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, const char val[], const std::string& comment="", bool warnExists=true) { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(name,po,comment,warnExists); }
			virtual void addValue(const std::string& name, long val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new PO(val),comment,warnExists); }
			virtual void addValue(const std::string& name, unsigned long val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new PO(val),comment,warnExists); }
		};
		
		//! This conversion policy accepts any entries of the specified template type, values will be directly wrapped as Primitives so no conversion at all is actually performed
		/*! Use addEntry() to add ObjectBase subclasses -- addValue is for POD types */
		template<typename PO>
		struct WrapValueConversion : public StringConversion, public IntegerConversion, public RealConversion, public EntryConstraint<PO> {
			//! insert a new entry to the map, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			template<typename T>
			void addValue(const std::string& name, const T& val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<T>(val),comment,warnExists); }
			virtual void addValue(const std::string& name, const std::string& val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<std::string>(val),comment,warnExists); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, char val[], const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<std::string>(val),comment,warnExists); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
			virtual void addValue(const std::string& name, const char val[], const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<std::string>(val),comment,warnExists); }
			virtual void addValue(const std::string& name, long val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<long>(val),comment,warnExists); }
			virtual void addValue(const std::string& name, unsigned long val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<unsigned long>(val),comment,warnExists); }
			virtual void addValue(const std::string& name, double val, const std::string& comment="", bool warnExists=true) { this->addEntry(name,new Primitive<double>(val),comment,warnExists); }
		};
		
		//! remove the entry with key @a name, returns true if something was actually removed (if false, wasn't there to begin with)
		virtual bool removeEntry(const std::string& name);
		
		//! change the key for an entry from @a oldname to @a newname, returns false if @a oldname didn't exist (thus no change was made)
		/*! If the collection owns the reference to the object, you have to use this function instead
		 *  of a pair of calls to removeEntry/addEntry, otherwise you will wind up with an invalid pointer! */
		virtual bool renameEntry(const std::string& oldname, const std::string& newname);
		//! exchange the values for a pair of keys -- if either key doesn't exist, forwards call to renameEntry()
		/*! returns true if the swap was successful, only returns false if both keys are invalid */
		virtual bool swapEntry(const std::string& a, const std::string& b);
		
		//! returns a reference to the entry with the specified name, creating it if it doesn't exist
		virtual ObjectBase& getEntry(const std::string& name)=0;
		//! returns a reference to the entry with the specified name, creating it if it doesn't exist
		virtual ObjectBase& operator[](const std::string& name)=0;
		//! returns a pointer to entry with the specified 'path', which may recurse through sub-collections
		virtual ObjectBase* resolveEntry(const std::string& path) const;
				
		//! returns an iterator to an entry in the current dictionary
		const_iterator findEntry(const std::string& name) const { return dict.find(name); }
		
		virtual void clear();
		
		//! return an STL const_iterator to the first entry
		const_iterator begin() const { return dict.begin(); }
		//! return the one-past-end const_iterator
		const_iterator end() const { return dict.end(); }
		//! return the size of the dictionary
		virtual size_t size() const { return dict.size(); }
		
		//! replaces previous comment for @a name, or adds it if it doesn't already exist (can preceed actual entry!)
		void setComment(const std::string& name, const std::string& comment);
		//! returns comment retrieved from loaded file, or any subsequent call to setComment
		const std::string& getComment(const std::string& name) const;
		
		virtual void loadXML(xmlNode* node);
		virtual void saveXML(xmlNode* node) const { std::set<std::string> seen; saveXML(node,!(savePolicy&ADDITIONS),seen); }
		//! saves the dictionary into the specified node
		/*! @param[in] node the xml node which should be saved into
		 *  @param[in] onlyOverwrite if is true, only saves entries for keys already found in the node (this overrides the current savePolicy value)
		 *  @param[in] seen used to keep track of which nodes have been seen in @a node -- may be of particular interest with @a onlyOverride set
		 *
		 *  @a seen is not cleared before being used.*/
		virtual void saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const;
		
		virtual std::string toString() const;
		
		//! returns the length of the longest key, subject to an optional regular expression and max depth
		virtual unsigned int getLongestKeyLen(const regex_t* reg=NULL, unsigned int depth=-1) const;
		
		//! returns true if the specified object will be deallocated when removed from the dictionary
		bool ownsReference(ObjectBase * val) const { return myRef.find(val)==myRef.end(); }
		
	protected:
		//! constructor
		explicit DictionaryBase(bool growable) : Collection(growable?UNION:FIXED,SYNC), dict(), myRef(), comments() { setLoadSavePolicy(growable?UNION:FIXED,SYNC); }
		//! copy constructor (don't assign listeners)
		DictionaryBase(const DictionaryBase& d) : Collection(d), dict(d.dict), myRef(d.myRef), comments(d.comments) { cloneMyRef(); setLoadSavePolicy(d.getLoadPolicy(),d.getSavePolicy()); }
		//! assignment (don't assign listeners)
		DictionaryBase& operator=(const DictionaryBase& d) { Collection::operator=(d); return *this; }
		
		//! destructor
		~DictionaryBase() { clear(); }
		
		//! indicates that the storage implementation should mark this as an externally supplied heap reference, which needs to be deleted on removal/destruction
		virtual void takeObject(const std::string& name, ObjectBase* obj);
		
		virtual void fireEntryRemoved(ObjectBase& val);
		
		//! returns an entry matching just the prefix
		/*! @param[in] name the name being looked up
		 *  @param[out] seppos the position of the separator (sub-collections are separated by '.')
		 *  @return iterator of the sub entry*/
		iterator getSubEntry(const std::string& name, std::string::size_type& seppos);
		//! returns an entry matching just the prefix
		/*! @param[in] name the name being looked up
		 *  @param[out] seppos the position of the separator (sub-collections are separated by '.')
		 *  @return iterator of the sub entry*/
		const_iterator getSubEntry(const std::string& name, std::string::size_type& seppos) const;
		
		//! called after an assignment or copy to clone the objects in #myRef to perform a deep copy
		virtual void cloneMyRef();
		
		//! called with each node being loaded so subclass can handle appropriately
		virtual bool loadXMLNode(const std::string& key, xmlNode* val, const std::string& comment)=0;
		
		//! called with each node being saved so subclass can handle appropriately, return true if successful and reset key if changed
		virtual bool saveOverXMLNode(xmlNode* k, xmlNode* val, const std::string& key, std::string comment, const std::string& indentStr, std::set<std::string>& seen) const;
		
		//! called with each node being saved so subclass can handle appropriately, return true if successful and set key
		virtual void saveXMLNode(xmlNode* node, const std::string& key, const ObjectBase* val, const std::string& indentStr, size_t longestKeyLen) const;
		
		//! storage of entries -- mapping from keys to value pointers
		storage_t dict;
		
		//! objects which have been handed over to the collection for eventual de-allocation
		std::set<ObjectBase*> myRef;

		//! shorthand for the type of #comments
		typedef std::map<std::string,std::string> comments_t;
		//! storage of entry comments -- mapping from keys to help text comments for manual editing or user feedback
		/*! not every key necessarily has a comment! */
		comments_t comments;
	};

	//! This is a specialization for float conversions to add RealConversions
	template<>
	struct DictionaryBase::ConversionTo<plist::Primitive<float> > : public DictionaryBase::StringConversion, public IntegerConversion, public DictionaryBase::RealConversion, public DictionaryBase::EntryConstraint<plist::Primitive<float> > {
		//! insert a new entry to the map, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., or one of the variable-sized Collection's, like Array, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		template<typename T>
		void addValue(const std::string& name, const T& val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<float>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, const std::string& val, const std::string& comment="", bool warnExists=true) { plist::Primitive<float>* po=new plist::Primitive<float>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		virtual void addValue(const std::string& name, char val[], const std::string& comment="", bool warnExists=true) { plist::Primitive<float> * po=new plist::Primitive<float>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		virtual void addValue(const std::string& name, const char val[], const std::string& comment="", bool warnExists=true) { plist::Primitive<float> * po=new plist::Primitive<float>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		virtual void addValue(const std::string& name, long val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<float>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, unsigned long val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<float>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, double val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<float>((float)val),comment,warnExists); }
	};

	//! This is a specialization for double conversions to add RealConversions
	template<>
	struct DictionaryBase::ConversionTo<plist::Primitive<double> > : public DictionaryBase::StringConversion, public IntegerConversion, public DictionaryBase::RealConversion, public DictionaryBase::EntryConstraint<plist::Primitive<double> > {
		//! insert a new entry to the map, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., or one of the variable-sized Collection's, like Array, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		template<typename T>
		void addValue(const std::string& name, const T& val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<double>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, const std::string& val, const std::string& comment="", bool warnExists=true) { plist::Primitive<double>* po=new plist::Primitive<double>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		virtual void addValue(const std::string& name, char val[], const std::string& comment="", bool warnExists=true) { plist::Primitive<double> * po=new plist::Primitive<double>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the dictionary
		virtual void addValue(const std::string& name, const char val[], const std::string& comment="", bool warnExists=true) { plist::Primitive<double> * po=new plist::Primitive<double>; try { po->set(val); } catch(...) { delete po; throw; } addEntry(name,po,comment,warnExists); }
		virtual void addValue(const std::string& name, long val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<double>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, unsigned long val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<double>(val),comment,warnExists); }
		virtual void addValue(const std::string& name, double val, const std::string& comment="", bool warnExists=true) { addEntry(name,new plist::Primitive<double>(val),comment,warnExists); }
	};

	
	//! A dictionary which requires all elements to be subtypes of the PO template argument
	/*! You can add or set entries by a quite a few variations on addEntry(), setEntry(), and addValue (via the Alloc conversion policy)
	 *  Basically these boil down to either:
	 *    - pass a pointer to an ObjectBase or directly pass a primitive value (int, float, char, etc.),
	 *      in which case the dictionary will assume management of the corresponding ObjectBase
	 *      instance (freeing the memory region when removed)
	 *    - pass a reference to an ObjectBase, in which case you retain control over the object's
	 *      allocation
	 *  
	 *  You have probably noticed this is a templated class -- you can provide any of the ObjectBase
	 *  subclasses to restrict the storage to that particular type, which will make life easier when
	 *  retrieving objects since their type will be known.
	 *
	 *  However, if you @e want an dictionary of mixed types, you can pass ObjectBase itself for the
	 *  template parameter, and you can then insert any combination of the plist types into the
	 *  same dictionary.  For convenience, a plist::Dictionary typedef is provided which does exactly this.
	 *  
	 *  So plist::Dictionary can handle any mixture of types, whereas plist::DictionaryOf<PO> will @e only
	 *  accept the plist objects of type PO (or their subclasses).  The Alloc template argument
	 *  allows you to define how new string values will be handled from DictionaryBase.
	 *
	 *  The optional conversion policy template specifies a base class for the dictionary which
	 *  can control how the dictionary will handle conversions from non-PO-based types.
	 */
	template<typename PO, typename Alloc=typename PO::template conversion_policy<DictionaryBase,PO>::value_conversion >
	class DictionaryOf : public DictionaryBase, public Alloc {
		/// @cond INTERNAL
		typedef typename storage_t::const_iterator::iterator_category const_iterator_category;
		typedef typename std::pair<storage_t::key_type,PO*> const_iterator_value;
		typedef typename storage_t::const_iterator::difference_type const_iterator_difference;
		/// @endcond
	public:
		//! shorthand for the type of the storage
		typedef typename DictionaryBase::storage_t storage_t;
		
		/// @cond INTERNAL
		
		//! iterator implementation which wraps storage_t::const_iterator to transparently dynamic_cast to the PO for client
		class const_iterator : public std::iterator<const_iterator_category, const_iterator_value, const_iterator_difference> {
		public:
			const_iterator(const storage_t::const_iterator& sit) : it(sit), tmp() {}
			const const_iterator_value& operator*() const { tmp.first=it->first; tmp.second=dynamic_cast<PO*>(it->second); return tmp; }
			const const_iterator_value* operator->() const { tmp.first=it->first; tmp.second=dynamic_cast<PO*>(it->second); return &tmp; }
			const_iterator& operator++() { ++it; return *this; }
			const_iterator operator++(int) { return const_iterator(it++); }
			const_iterator& operator--() { --it; return *this; }
			const_iterator operator--(int) { return const_iterator(it--); }
			bool operator==(const const_iterator& rhs) const { return it==rhs.it; }
			bool operator!=(const const_iterator& rhs) const { return it!=rhs.it; }
			
		protected:
			storage_t::const_iterator it;
			mutable const_iterator_value tmp;
		};
		/// @endcond
		
		//! constructor
		DictionaryOf() : DictionaryBase(true), Alloc() {}
		//! constructor
		explicit DictionaryOf(bool growable) : DictionaryBase(growable), Alloc() {}
		//! copy constructor (don't assign listeners)
		DictionaryOf(const DictionaryOf& d) : DictionaryBase(d), Alloc(d) {}
		//! assignment (don't assign listeners); subclass should call fireEntriesChanged after calling this (and updating its own storage)
		DictionaryOf& operator=(const DictionaryOf& a);

		virtual void set(const ObjectBase& ob) { const DictionaryBase& d=dynamic_cast<const DictionaryBase&>(ob); set(d); }
		//! handles actual setting of one dictionary to another, similar to operator=(DictionaryOf), but allows for polymorphic conversion to the template type
		virtual void set(const DictionaryBase& a);
		
		//! destructor
		~DictionaryOf() { }
		
		//! Replaces the entry with the specified key, optionally warns as it does so.  If you simply want to set the *value* of the specified entry, try getEntry() and assignment...
		/*! By passing a reference to the entry, you indicate you will retain responsibility for deallocation */
		virtual void setEntry(const std::string& name, PO& val, bool warnExists=false);
		//! Convenience method for adding new entries, with optional comment and warning if replacing a previous entry
		/*! By passing a reference to the entry, you indicate you will retain responsibility for deallocation */
		virtual void addEntry(const std::string& name, PO& val, const std::string& comment="", bool warnExists=true);
		//! Replaces the entry with the specified key, optionally warns as it does so.  If you simply want to set the *value* of the specified entry, try getEntry() and assignment...
		/*! By passing a pointer to the entry, you indicate you wish the dictionary to assume responsibility for deallocation */
		virtual void setEntry(const std::string& name, PO* val, bool warnExists=false);
		//! Convenience method for adding new entries, with optional comment and warning if replacing a previous entry
		/*! By passing a pointer to the entry, you indicate you wish the dictionary to assume responsibility for deallocation */
		virtual void addEntry(const std::string& name, PO* val, const std::string& comment="", bool warnExists=true);
		
		// stupid older compilers don't support covariant returns
#if !defined(__GNUC__) || __GNUC__ > 3 || (__GNUC__ == 3 && (__GNUC_MINOR__ > 3))
		typedef PO entryRet_t; //!< older compilers don't support covariant returns, so this is the templated storage type on modern compilers, generic ObjectBase otherwise, can check defined(PLIST_NO_COVRET)
#else
		typedef ObjectBase entryRet_t; //!< older compilers don't support covariant returns, so this is the templated storage type on modern compilers, generic ObjectBase otherwise, can check defined(PLIST_NO_COVRET)
#  define PLIST_NO_COVRET
#endif
		
		//! returns a reference to the entry with the specified name, creating it if it doesn't exist
		entryRet_t& getEntry(const std::string& name) {
			DictionaryBase::const_iterator it=dict.find(name);
			if(it!=dict.end())
				return dynamic_cast<PO&>(*it->second);
			else {
				PO* p = plist::allocate<PO>(); // do before dictionary access (next line) in case of exception (don't leak NULL dictionary entries!)
				dict[name]=p;
				fireEntryAdded(*p);
				return *p;
			}
		}
		//! returns a reference to the entry with the specified name, creating it if it doesn't exist
		entryRet_t& operator[](const std::string& name) { return getEntry(name); }
		//! returns an iterator the \<key, value\> pair with the specified key (returns end() if not found)
		const_iterator findEntry(const std::string& name) const { return dict.find(name); }
				
		//! return an STL const_iterator to the first entry (note implicit conversion to specialized const_iterator)
		const_iterator begin() const { return dict.begin(); }
		//! return the one-past-end const_iterator (note implicit conversion to specialized const_iterator)
		const_iterator end() const { return dict.end(); }
		
		virtual bool canContain(const ObjectBase& obj) { return (dynamic_cast<const PO*>(&obj)!=NULL); }
		
		//! clone implementation for Dictionary
		PLIST_CLONE_DEF(DictionaryOf,(new DictionaryOf<PO,Alloc>(*this)));
			
	protected:
		//! called with each node being loaded so subclass can handle appropriately
		virtual bool loadXMLNode(const std::string& name, xmlNode* val, const std::string& comment);
	};
	
	/*! plist::Dictionary can handle any mixture of types, whereas plist::DictionaryOf<PO> will @e only
	 *  accept the plist objects of type PO (or their subclasses).  This typedef is simply for
	 *  convenience and passes ObjectBase for the template parameter. */
	typedef DictionaryOf<ObjectBase> Dictionary;
	extern template class DictionaryOf<ObjectBase>;
	
	
	template<typename PO, typename Alloc>
	void DictionaryOf<PO,Alloc>::setEntry(const std::string& name, PO& val, bool warnExists/*=false*/) {
		DictionaryBase::iterator it=dict.find(name);
		if(it!=dict.end()) {
			//found exact name match
			if(&val==it->second) {
				if(warnExists)
					std::cerr << "Warning: entry ("<<name<<","<<val<<") was already added, ignoring duplication..." << std::endl;
				myRef.erase(&val); // reset reference
				return; // same val reference already registered
			}
			if(warnExists) {
				std::cerr << "Warning: new entry ("<<name<<","<<val<<") conflicted with previous entry ("<<name<<","<<(*it->second)<<")" << std::endl;
				std::cerr << "         (use setEntry(...,false) if you expect you might need to overwrite)" << std::endl;
			}
			removeEntry(name);
			//fall through to add new val
		}
		dict[name]=&val;
		fireEntryAdded(val);
	}
	template<typename PO, typename Alloc>
	void DictionaryOf<PO,Alloc>::addEntry(const std::string& name, PO& val, const std::string& comment, bool warnExists) {
		DictionaryBase::storage_t::iterator it=dict.find(name);
		if(it!=dict.end()) {
			//found exact name match
			if(&val==it->second) {
				if(warnExists)
					std::cerr << "Warning: entry ("<<name<<","<<val<<") was already added, ignoring duplication..." << std::endl;
				myRef.erase(&val);
				return; // same val reference already registered
			}
			if(warnExists) {
				std::cerr << "Warning: new entry ("<<name<<","<<val<<") conflicted with previous entry ("<<name<<","<<(*it->second)<<")" << std::endl;
				std::cerr << "         (use setEntry() if you expect you might need to overwrite)" << std::endl;
			}
			removeEntry(name);
			//fall through to add new val
		}
		if(comment.size()>0)
			comments[name]=comment;
		dict[name]=&val;
		fireEntryAdded(val);
	}
	template<typename PO, typename Alloc>
	void DictionaryOf<PO,Alloc>::setEntry(const std::string& name, PO* val, bool warnExists/*=false*/) {
		DictionaryBase::iterator it=dict.find(name);
		if(it!=dict.end()) {
			//found exact name match
			if(val==it->second) {
				if(warnExists)
					std::cerr << "Warning: entry ("<<name<<","<<(*val)<<") was already added, ignoring duplication..." << std::endl;
				myRef.insert(val);
				return; // same val reference already registered
			}
			if(warnExists) {
				std::cerr << "Warning: new entry ("<<name<<","<<(*val)<<") conflicted with previous entry ("<<name<<","<<(*it->second)<<")" << std::endl;
				std::cerr << "         (use setEntry(...,false) if you expect you might need to overwrite)" << std::endl;
			}
			removeEntry(name);
			//fall through to add new val
		}
		dict[name]=val;
		takeObject(name,val);
		fireEntryAdded(*val);
	}
	template<typename PO, typename Alloc>
	void DictionaryOf<PO,Alloc>::addEntry(const std::string& name, PO* val, const std::string& comment, bool warnExists) {
		DictionaryBase::iterator it=dict.find(name);
		if(it!=dict.end()) {
			//found exact name match
			if(val==it->second) {
				if(warnExists)
					std::cerr << "Warning: entry ("<<name<<","<<(*val)<<") was already added, ignoring duplication..." << std::endl;
				myRef.insert(val);
				return; // same val reference already registered
			}
			if(warnExists) {
				std::cerr << "Warning: new entry ("<<name<<","<<(*val)<<") conflicted with previous entry ("<<name<<","<<(*it->second)<<")" << std::endl;
				std::cerr << "         (use setEntry() if you expect you might need to overwrite)" << std::endl;
			}
			removeEntry(name);
			//fall through to add new val
		}
		dict[name]=val;
		if(comment.size()>0)
			comments[name]=comment;
		takeObject(name,val);
		fireEntryAdded(*val);
	}
	
	//! implements the clone function for dictionary
	PLIST_CLONE_IMPT2(PO,Alloc,DictionaryOf,(new DictionaryOf<PO,Alloc>(*this)));
	
	template<typename PO, typename Alloc>
	bool DictionaryOf<PO,Alloc>::loadXMLNode(const std::string& key, xmlNode* val, const std::string& comment) {
		DictionaryBase::const_iterator it=dict.find(key);
		if(it!=dict.end()) {
			//found pre-existing entry
			try {
				//it's reasonable to assume that in common usage, the same type will be used each time
				//so let's try to load into the current entry as is
				it->second->loadXML(val);
				if(dynamic_cast<Collection*>(it->second)!=NULL) {
					const std::string headline=("======== "+key+" ========");
					if(comment.compare(0,headline.size(),headline)!=0)
						setComment(key,comment);
					else if(comment.size()>headline.size())
						setComment(key,comment.substr(headline.size()));
				} else if(comment.size()>0)
					setComment(key,comment);
				return true;
			} catch(...) {
				// apparently that didn't work, we'll need to make a new reference or clone
				if(loadPolicy!=SYNC) // can we do that?
					throw; // nope -- give up
				// still here? let's try a fresh load using the polymorphic loader (fall through below)
				removeEntry(key);
			}
		} else if(!(loadPolicy&ADDITIONS)) {
			if(warnUnused)
				std::cerr << "Warning: reading plist dictionary, key '" << key << "' does not match a registered variable.  Ignoring..." << std::endl;
			return false;
		}
		PO * cobj = plist::loadXML<PO>(val);
		if(dynamic_cast<Collection*>(cobj)!=NULL) {
			const std::string headline=("======== "+key+" ========");
			if(comment.compare(0,headline.size(),headline)!=0)
				addEntry(key,cobj,comment);
			else
				addEntry(key,cobj,comment.substr(headline.size()));
		} else 
			addEntry(key,cobj,comment);
		return true;
	}
	
	template<typename PO, typename Alloc>
	void DictionaryOf<PO,Alloc>::set(const DictionaryBase& d) {
		if(&d == this)
			return;
		// if we're doing a large dictionary, might be worth checking if we're actually the same type
		if(const DictionaryOf* od = dynamic_cast<const DictionaryOf*>(&d)) {
			operator=(*od); // same template type, use faster version!
			return;
		}
		DictionaryBase::operator=(d);
		
		std::set<std::string> seen;
		for(DictionaryBase::const_iterator dit=d.begin(); dit!=d.end(); ++dit) {
			const std::string key=dit->first;
			ObjectBase* val=dit->second;
			const std::string comment=d.getComment(key);
			seen.insert(key);
			DictionaryBase::storage_t::const_iterator it=dict.find(key);
			if(it!=dict.end()) {
				//found pre-existing entry
				try {
					//it's reasonable to assume that in common usage, the same type will be used each time
					//so let's try to load into the current entry as is
					it->second->set(*val);
					if(comment.size()>0)
						setComment(key,comment);
					continue;
				} catch(...) {
					// apparently that didn't work, we'll need to make a new reference or clone
					if(loadPolicy!=SYNC) // can we do that?
						throw; // nope -- give up
					// still here? let's try a fresh load using the polymorphic loader (fall through below)
					removeEntry(key);
				}
			} else if(!(loadPolicy&ADDITIONS)) {
				if(warnUnused)
					std::cerr << "Warning: reading plist dictionary, key '" << key << "' does not match a registered variable.  Ignoring..." << std::endl;
				continue;
			}
			PO* obj=dynamic_cast<PO*>(val);
			if(obj==NULL) {
				obj = plist::allocate<PO>();
				try {
					obj->set(*val);
				} catch(...) {
					delete obj;
					throw;
				}
				myRef.insert(obj);
			} else {
				obj = dynamic_cast<PO*>(obj->clone());
				if(obj==NULL) {
					// this shouldn't be possible: obj is a PO, so obj->clone should be as well
					throw std::runtime_error("plist::Dictionary could not assign clone of object due to impossible type conflict (bad clone?)");
				}
				myRef.insert(obj);
			}
			if(dynamic_cast<Collection*>(obj)!=NULL) {
				const std::string headline=("======== "+key+" ========");
				if(comment.compare(0,headline.size(),headline)!=0)
					addEntry(key,*obj,comment);
				else
					addEntry(key,*obj,comment.substr(headline.size()));
			} else 
				addEntry(key,*obj,comment);
		}
		if((loadPolicy&REMOVALS) && seen.size()!=size()) {
			std::set<std::string> rem;
			for(const_iterator it=begin(); it!=end(); ++it) {
				if(seen.find(it->first)==seen.end())
					rem.insert(it->first);
			}
			for(std::set<std::string>::const_iterator it=rem.begin(); it!=rem.end(); ++it)
				removeEntry(*it);
		}
	}
	
	template<typename PO, typename Alloc>
	DictionaryOf<PO,Alloc>& DictionaryOf<PO,Alloc>::operator=(const DictionaryOf& d) {
		if(&d==this)
			return *this;
		DictionaryBase::operator=(d);
		
		std::set<std::string> seen;
		for(const_iterator dit=d.begin(); dit!=d.end(); ++dit) {
			const std::string key=dit->first;
			PO* val=dit->second;
			const std::string comment=d.getComment(key);
			seen.insert(key);
			DictionaryBase::const_iterator it=dict.find(key);
			if(it!=dict.end()) {
				//found pre-existing entry
				try {
					//it's reasonable to assume that in common usage, the same type will be used each time
					//so let's try to load into the current entry as is
					plist::assign(*dynamic_cast<PO*>(it->second),*val);
					if(comment.size()>0)
						setComment(key,comment);
					continue;
				} catch(...) {
					// apparently that didn't work, we'll need to make a new reference or clone
					if(loadPolicy!=SYNC) // can we do that?
						throw; // nope -- give up
					// still here? let's try a fresh load using the polymorphic loader (fall through below)
					removeEntry(key);
				}
			} else if(!(loadPolicy&ADDITIONS)) {
				if(warnUnused)
					std::cerr << "Warning: reading plist dictionary, key '" << key << "' does not match a registered variable.  Ignoring..." << std::endl;
				continue;
			}
			PO* obj = dynamic_cast<PO*>(val->clone());
			if(obj==NULL) {
				// this shouldn't be possible: val is a PO, so obj->clone should be as well
				throw std::runtime_error("plist::Dictionary could not assign clone of object due to impossible type conflict (bad clone?)");
			}
			myRef.insert(obj);
			if(dynamic_cast<Collection*>(obj)!=NULL) {
				const std::string headline=("======== "+key+" ========");
				if(comment.compare(0,headline.size(),headline)!=0)
					addEntry(key,*obj,comment);
				else
					addEntry(key,*obj,comment.substr(headline.size()));
			} else 
				addEntry(key,*obj,comment);
		}
		if((loadPolicy&REMOVALS) && seen.size()!=size()) {
			std::set<std::string> rem;
			for(const_iterator it=begin(); it!=end(); ++it) {
				if(seen.find(it->first)==seen.end())
					rem.insert(it->first);
			}
			for(std::set<std::string>::const_iterator it=rem.begin(); it!=rem.end(); ++it)
				removeEntry(*it);
		}
		return *this;
	}
	
	
	//! Maintains an array of value, see ArrayOf, and the Array typedef
	/*! You can add or set entries by a quite a few variations on addEntry(), setEntry(), or addValue().
	 *  Basically these boil down to either:
	 *    - pass a pointer to an ObjectBase or directly pass a primitive value (int, float, char, etc.),
	 *      in which case the Array will assume management of the corresponding ObjectBase
	 *      instance (freeing the memory region when removed)
	 *    - pass a reference to an ObjectBase, in which case you retain control over the object's
	 *      allocation
	 */
	class ArrayBase : virtual public Collection {
		friend std::ostream& operator<<(std::ostream& os, const ArrayBase& d);
	public:
		//! shorthand for the type of the storage
		typedef std::vector<ObjectBase*> storage_t;
		//! shorthand for iterators to be returned
		typedef storage_t::iterator iterator;
		//! shorthand for iterators to be returned
		typedef storage_t::const_iterator const_iterator;
		
		//! Indicates that no value conversions are allowed
		/*! The actual storage type is still allowed, so technically we could use EntryConstraint<PO>
		 *  instead as the conversion policy, but that doesn't gain anything because you would need
		 *  to know the PO to test for it.  At least with this you can test for DeniedValueConversions as a base
		 *  class and then fall back to testing individual PO's if you want. */
		struct DeniedValueConversions {
			virtual ~DeniedValueConversions() {} //!< no-op destructor
		};
		
		template<typename PO>
		struct EntryConstraint {
			virtual ~EntryConstraint() {} //!< no-op destructor
			//! replaces previous entry at the specified @a index, which must represent an integer value less than or equal to the current array size
			virtual void addEntry(PO& val, const std::string& comment="")=0;
			//! replaces previous entry at the specified @a index, which must represent an integer value less than or equal to the current array size
			virtual void addEntry(PO* val, const std::string& comment="")=0;
			
			//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size
			virtual void setEntry(size_t index, PO& val, bool warnExists=false)=0;
			//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size
			virtual void addEntry(size_t index, PO& val, const std::string& comment="")=0;
			//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
			virtual void setEntry(size_t index, PO* val, bool warnExists=false)=0;
			//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
			virtual void addEntry(size_t index, PO* val, const std::string& comment="")=0;
		};
		
		//! Abstract base class to test whether the collection will accept strings (possibly converting to a value type, or storing directly as string depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::StringConversion>, and if it is successful, you
		 *  can pass the string without having to know the actual value type of the dictionary. */
		struct StringConversion {
			virtual ~StringConversion() {} //!< no-op destructor
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(const std::string& val, const std::string& comment="")=0;
			//! generic addition of value at a specified position
			virtual void addValue(size_t index, const std::string&, const std::string& comment="")=0;
		};
		
		//! Abstract base class to test whether the collection will accept integers (possibly converting to another value type, or storing directly as a [unsigned] long depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::IntegerConversion>, and if it is successful, you
		 *  can pass the data without having to know the actual value type of the dictionary. */
		struct IntegerConversion {
			virtual ~IntegerConversion() {} //!< no-op destructor
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(long val, const std::string& comment="", bool warnExists=true)=0;
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(unsigned long val, const std::string& comment="", bool warnExists=true)=0;
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, long val, const std::string& comment="", bool warnExists=true)=0;
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, unsigned long val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		//! Abstract base class to test whether the collection will accept floating point numbers (possibly converting to another value type, or storing directly as a double depending on concrete type)
		/*! The point of this class is to handle the situation where you have a DictionaryBase and user input to append, and
		 *  you don't want to have to test every combination of template parameters to DictionaryOf to find out if you can
		 *  add the data.  Instead, test dynamic_cast<plist::DictionaryBase::RealConversion>, and if it is successful, you
		 *  can pass the data without having to know the actual value type of the dictionary. */
		struct RealConversion {
			virtual ~RealConversion() {} //!< no-op destructor
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(double val, const std::string& comment="", bool warnExists=true)=0;
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, double val, const std::string& comment="", bool warnExists=true)=0;
		};
		
		
		//! This conversion policy accepts entries of the specified template type, and will try to create new instances of that type constructed from any values which are passed.
		/*! Use of this conversion policy requires that the template parameter is not abstract,
		 *  as the policy will be trying to create new instances directly from the specified type. */
		template<typename PO>
		struct ConversionTo : public StringConversion, public EntryConstraint<PO> {
			//! insert a new entry to the end of the vector, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., or one of the variable-sized Collection's, like Array, control of (de)allocation will be assumed by the Array
			template<typename T>
			void addValue(const T& val, const std::string& comment="") { addEntry(new PO(val),comment); }
			virtual void addValue(const std::string& val, const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(po,comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings
			virtual void addValue(char val[], const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(po,comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings
			virtual void addValue(const char val[], const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(po,comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(long val, const std::string& comment="") { this->addEntry(new PO(val),comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(unsigned long val, const std::string& comment="") { this->addEntry(new PO(val),comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(double val, const std::string& comment="") { this->addEntry(new PO(val),comment); }
			
			//! inserts new entry at the specified @a index, which must be less than or equal to the current array size, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			template<typename T>
			void addValue(size_t index, const T& val, const std::string& comment="") { addEntry(index,new PO(val),comment); }
			virtual void addValue(size_t index, const std::string& val, const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(index,po,comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, char val[], const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(index,po,comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, const char val[], const std::string& comment="") { PO * po=new PO; try { po->set(val); } catch(...) { delete po; throw; } this->addEntry(index,po,comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, long val, const std::string& comment="") { this->addEntry(index,new PO(val),comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, unsigned long val, const std::string& comment="") { this->addEntry(index,new PO(val),comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, double val, const std::string& comment="") { this->addEntry(index,new PO(val),comment); }
		};
		
		//! This conversion policy accepts any entries of the specified template type, values will be directly wrapped as Primitives so no conversion at all is actually performed
		/*! Use addEntry() to add ObjectBase subclasses -- addValue is for POD types */
		template<typename PO>
		struct WrapValueConversion : public StringConversion, public EntryConstraint<PO> {
			//! insert a new entry to the end of the vector, and corresponding comment; expects @a val to be either a primitive type, like int, float, etc., control of (de)allocation will be assumed by the Array
			template<typename T>
			void addValue(const T& val, const std::string& comment="") { this->addEntry(new Primitive<T>(val),comment); }
			virtual void addValue(const std::string& val, const std::string& comment="") { this->addEntry(new Primitive<std::string>(val),comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings
			virtual void addValue(char val[], const std::string& comment="") { this->addEntry(new Primitive<std::string>(val),comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings
			virtual void addValue(const char val[], const std::string& comment="") { this->addEntry(new Primitive<std::string>(val),comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(long val, const std::string& comment="") { this->addEntry(new Primitive<long>(val),comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(unsigned long val, const std::string& comment="") { this->addEntry(new Primitive<unsigned long>(val),comment); }
			//! generic addition of value at end of collection, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(double val, const std::string& comment="") { this->addEntry(new Primitive<double>(val),comment); }
			
			//! inserts new entry at the specified @a index, which must be less than or equal to the current array size, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			template<typename T>
			void addValue(size_t index, const T& val, const std::string& comment="") { this->addEntry(index,new Primitive<T>(val),comment); }
			virtual void addValue(size_t index, const std::string& val, const std::string& comment="") { this->addEntry(index,new Primitive<std::string>(val),comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, char val[], const std::string& comment="") { this->addEntry(index,new Primitive<std::string>(val),comment); }
			//! "specialization" (actually just another override) for handling character arrays as strings, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			virtual void addValue(size_t index, const char val[], const std::string& comment="") { this->addEntry(index,new Primitive<std::string>(val),comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, long val, const std::string& comment="") { this->addEntry(index,new Primitive<long>(val),comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, unsigned long val, const std::string& comment="") { this->addEntry(index,new Primitive<unsigned long>(val),comment); }
			//! generic addition of value at specified position, control of (de)allocation of corresponding Primitive instance will be assumed by the Array
			void addValue(size_t index, double val, const std::string& comment="") { this->addEntry(index,new Primitive<double>(val),comment); }
		};
		
		//! adds a new entry to the end of the array, (de)allocation retained by caller
		virtual void forceEntry(ObjectBase& val, const std::string& comment="")=0;
		//! adds a new entry to the end of the array, (de)allocation responsibility of array
		virtual void forceEntry(ObjectBase* val, const std::string& comment="")=0;
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size
		virtual void forceEntry(size_t index, ObjectBase& val, const std::string& comment="")=0;
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
		virtual void forceEntry(size_t index, ObjectBase* val, const std::string& comment="")=0;
		
		//! remove the entry at position @a index, returns true if something was actually removed (if false, wasn't there to begin with)
		virtual bool removeEntry(size_t index);
		//! return the value at position @a index, which must exist (no range checking)
		ObjectBase& getEntry(size_t index) const { return *arr[index]; }
		//! return the value at position @a index, which must exist (no range checking, equivalent to getEntry(index))
		ObjectBase& operator[](size_t index) const { return *arr[index]; }
		virtual ObjectBase* resolveEntry(const std::string& path) const;
		
		virtual void clear();
		
		//! return an STL const_iterator to the first entry
		const_iterator begin() const { return arr.begin(); }
		//! return the one-past-end const_iterator
		const_iterator end() const { return arr.end(); }
		//! return the size of the array
		virtual size_t size() const { return arr.size(); }
		//! return first element
		ObjectBase& front() const { return *arr.front(); }
		//! return last element
		ObjectBase& back() const { return *arr.back(); }
		
		//! replaces previous comment for @a name, or adds it if it doesn't already exist (can preceed actual entry!)
		virtual void setComment(size_t index, const std::string& comment);
		//! returns comment retrieved from loaded file, or any subsequent call to setComment
		virtual const std::string& getComment(size_t index) const;
		
		virtual void loadXML(xmlNode* node);
		virtual void saveXML(xmlNode* node) const;
		
		virtual std::string toString() const;
		
		virtual unsigned int getLongestKeyLen(const regex_t* reg=NULL, unsigned int depth=-1) const;
		
		//! returns true if the specified object will be deallocated when removed from the array
		bool ownsReference(ObjectBase * val) const { return myRef.find(val)==myRef.end(); }
		
		virtual void setSaveInlineStyle(bool b) { saveInlineStyle=b; } //!< sets #saveInlineStyle
		virtual bool getSaveInlineStyle() const { return saveInlineStyle; } //!< returns #saveInlineStyle
		
protected:
		//! constructor
		ArrayBase(bool growable) : Collection(growable?SYNC:FIXED,SYNC), arr(), myRef(), comments(), saveInlineStyle(false) { setLoadSavePolicy(growable?SYNC:FIXED,SYNC); }
		//! copy constructor
		ArrayBase(const ArrayBase& d) : Collection(d), arr(d.arr), myRef(d.myRef), comments(d.comments), saveInlineStyle(false) { cloneMyRef(); setLoadSavePolicy(d.getLoadPolicy(),d.getSavePolicy()); }
		//! assignment
		ArrayBase& operator=(const ArrayBase& d) { Collection::operator=(d); return *this; }
		
		//! destructor
		~ArrayBase() { clear(); }
		
		//! indicates that the storage implementation should mark this as an externally supplied heap reference, which needs to be deleted on removal/destruction
		virtual void takeObject(size_t index, ObjectBase* obj);
		
		virtual void fireEntryRemoved(ObjectBase& val);
		
		//! returns an entry matching just the prefix
		/*! @param[in] name the name being looked up
		 *  @param[out] seppos the position of the separator (sub-collections are separated by '.')
		 *  @return iterator of the sub entry*/
		iterator getSubEntry(const std::string& name, std::string::size_type& seppos);
		//! returns an entry matching just the prefix
		/*! @param[in] name the name being looked up
		 *  @param[out] seppos the position of the separator (sub-collections are separated by '.')
		 *  @return iterator of the sub entry*/
		const_iterator getSubEntry(const std::string& name, std::string::size_type& seppos) const;
		
		//! called after an assignment or copy to clone the objects in #myRef to perform a deep copy
		virtual void cloneMyRef();
		
		//! called by loadXML(), loads a single xmlNode into a specified position, replacing previous entry if it can't accept the new value (subject to the load/save policy...)
		virtual bool loadXMLNode(size_t index, xmlNode* val, const std::string& comment)=0;
		
		//! storage of entries
		storage_t arr;
		
		//! objects which have been handed over to the collection for eventual de-allocation
		std::set<ObjectBase*> myRef;
		
		//! shorthand for the type of #comments
		typedef std::map<size_t,std::string> comments_t;
		//! storage of entry comments -- mapping from keys to help text comments for manual editing or user feedback
		/*! not every key necessarily has a comment! */
		comments_t comments;
		
		//! If true, will avoid line breaks between entries in serialized output, also skips comment headers
		/*! This might make short lists more readable. */
		bool saveInlineStyle;
	};
	
	
	//! A collection of plist objects, similar to a Dictionary, but no keys -- order matters!, see plist::Array
	/*! You can add or set entries by a quite a few variations on addEntry(), setEntry(), and addValue (via the Alloc conversion policy)
	 *  Basically these boil down to either:
	 *    - pass a pointer to an ObjectBase or directly pass a primitive value (int, float, char, etc.),
	 *      in which case the Array will assume management of the corresponding ObjectBase
	 *      instance (freeing the memory region when removed)
	 *    - pass a reference to an ObjectBase, in which case you retain control over the object's
	 *      allocation
	 *  
	 *  You have probably noticed this is a templated class -- you can provide any of the ObjectBase
	 *  subclasses to restrict the storage to that particular type, which will make life easier when
	 *  retrieving objects since their type will be known.
	 *
	 *  However, if you @e want an Array of mixed types, you can pass ObjectBase itself for the
	 *  template parameter, and you can then insert any combination of the plist types into the
	 *  same array.  For convenience, a plist::Array typedef is provided which does exactly this.
	 *  
	 *  So plist::Array can handle any mixture of types, whereas plist::ArrayOf<PO> will @e only
	 *  accept the plist objects of type PO (or their subclasses).  The Alloc template argument
	 *  allows you to define how new string values will be handled from ArrayBase.
	 *
	 *  The optional conversion policy template specifies a base class for the dictionary which
	 *  can control how the dictionary will handle conversions from non-PO-based types.
	 */
	template<typename PO, typename Alloc=typename PO::template conversion_policy<ArrayBase,PO>::value_conversion >
	class ArrayOf : public ArrayBase, public Alloc {
		/// @cond INTERNAL
		typedef typename storage_t::const_iterator::iterator_category const_iterator_category;
		typedef typename storage_t::const_iterator::difference_type const_iterator_difference;
		/// @endcond
	public:
		//! shorthand for the type of the storage
		typedef typename ArrayBase::storage_t storage_t;
		
		/// @cond INTERNAL
		
		//! iterator implementation which wraps storage_t::const_iterator to transparently dynamic_cast to the PO for client
		class const_iterator : public std::iterator<const_iterator_category, const PO*, const_iterator_difference> {
		public:
			typedef PO* value_type;
			typedef const_iterator_difference difference_type;
			
			const_iterator(const storage_t::const_iterator& sit) : it(sit), tmp(NULL) {}
			const_iterator(const const_iterator& oit) : it(oit.it), tmp(NULL) {}
			const_iterator& operator=(const const_iterator& oit) { it=oit.it; return *this; }
			
			value_type operator*() const { return dynamic_cast<PO*>(*it); }
			value_type const* operator->() const { tmp=dynamic_cast<PO*>(*it); return &tmp; }
			const_iterator& operator++() { ++it; return *this; }
			const_iterator operator++(int) { return const_iterator(it++); }
			const_iterator& operator--() { --it; return *this; }
			const_iterator operator--(int) { return const_iterator(it--); }

			bool operator==(const const_iterator& rhs) const { return it==rhs.it; }
			bool operator!=(const const_iterator& rhs) const { return it!=rhs.it; }
			
			// Random access iterator requirements
			value_type operator[](const difference_type& __n) const { return dynamic_cast<PO*>(it[__n]); }
			const_iterator& operator+=(const difference_type& __n) { it += __n; return *this; }
			const_iterator operator+(const difference_type& __n) const { return const_iterator(it + __n); }
			const_iterator& operator-=(const difference_type& __n) { it -= __n; return *this; }
			const_iterator operator-(const difference_type& __n) const { return const_iterator(it - __n); }
			bool operator<(const const_iterator& __rhs) const { return it < __rhs.it; }
			bool operator>(const const_iterator& __rhs) const { return it > __rhs.it; }
			bool operator<=(const const_iterator& __rhs) const { return it <= __rhs.it; }
			bool operator>=(const const_iterator& __rhs) const { return it >= __rhs.it; }
			difference_type operator-(const const_iterator& __rhs) const { return it - __rhs.it; }
			
		protected:
			storage_t::const_iterator it;
			mutable PO* tmp; //!< used by operator->() so it doesn't return pointer-to-temporary
		};
		/// @endcond
		
		//! constructor
		ArrayOf() : ArrayBase(true), Alloc() {}
		//! constructor, create @a n entries using default constructor, defaults to non-growable (load does not change size)
		explicit ArrayOf(typename storage_t::size_type n) : ArrayBase(false), Alloc() {
			arr.resize(n);
			for(ArrayBase::iterator it=arr.begin(); it!=arr.end(); ++it)
				*it=new PO;
			myRef.insert(arr.begin(),arr.end());
		}
		//! constructor, create @a n copies of @a t, defaults to non-growable (load does not change size)
		ArrayOf(typename storage_t::size_type n, const PO& t, bool growable=false) : ArrayBase(growable), Alloc() {
			arr.resize(n);
#ifdef PLATFORM_APERIOS
			for(ArrayBase::iterator it=arr.begin(); it!=arr.end(); ++it) {
				*it=dynamic_cast<PO*>(t.clone()); // older compiler on Aperios, doesn't support polymorphic return, just assume
#  ifdef DEBUG
				if(*it==NULL)
					std::cerr << "plist::Array construction from replicated value given NULL via clone" << std::endl;
#  endif
			}
#else
			for(ArrayBase::iterator it=arr.begin(); it!=arr.end(); ++it)
				*it=t.clone();
#endif
			myRef.insert(arr.begin(),arr.end());
		}
		//! copy constructor (don't assign listeners)
		ArrayOf(const ArrayOf& d) : ArrayBase(d), Alloc(d) {}
		//! assignment (don't assign listeners); subclass should call fireEntriesChanged after calling this (and updating its own storage)
		ArrayOf& operator=(const ArrayOf& a);

		virtual void set(const ObjectBase& ob) { const ArrayBase& a=dynamic_cast<const ArrayBase&>(ob); set(a); }
		virtual void set(const ArrayBase& a); //!< handles polymorphic assignment of ArrayBase subclasses, similar to operator=(ArrayOf), but allows conversion of entries
		
		//! destructor
		~ArrayOf() { }
		
		//! adds a new entry to the end of the array, (de)allocation retained by caller
		virtual void addEntry(PO& val, const std::string& comment="") { if(comment.size()>0) setComment(size(),comment); arr.push_back(&val); fireEntryAdded(*arr.back()); }
		//! adds a new entry to the end of the array, (de)allocation responsibility of array
		virtual void addEntry(PO* val, const std::string& comment="") { if(comment.size()>0) setComment(size(),comment); arr.push_back(val); takeObject(size()-1,val); fireEntryAdded(*arr.back()); }
		
		//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size
		virtual void setEntry(size_t index, PO& val, bool warnExists=false);
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size
		virtual void addEntry(size_t index, PO& val, const std::string& comment="");
		//! replaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
		virtual void setEntry(size_t index, PO* val, bool warnExists=false);
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
		virtual void addEntry(size_t index, PO* val, const std::string& comment="");
		
		//! adds a new entry to the end of the array, (de)allocation retained by caller
		virtual void forceEntry(ObjectBase& val, const std::string& comment="") { if(PO* po=dynamic_cast<PO*>(&val)) addEntry(*po,comment); else throw bad_format(NULL,"plist::ArrayBase::forceEntry() of wrong type"); }
		//! adds a new entry to the end of the array, (de)allocation responsibility of array
		virtual void forceEntry(ObjectBase* val, const std::string& comment="") { if(PO* po=dynamic_cast<PO*>(val)) addEntry(po,comment); else throw bad_format(NULL,"plist::ArrayBase::forceEntry() of wrong type"); }
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size
		virtual void forceEntry(size_t index, ObjectBase& val, const std::string& comment="") { if(PO* po=dynamic_cast<PO*>(&val)) addEntry(index,*po,comment); else throw bad_format(NULL,"plist::ArrayBase::forceEntry() of wrong type"); }
		//! displaces previous entry at the specified @a index, which must be less than or equal to the current array size; control of (de)allocation will be assumed by the Array
		virtual void forceEntry(size_t index, ObjectBase* val, const std::string& comment="") { if(PO* po=dynamic_cast<PO*>(val)) addEntry(index,po,comment); else throw bad_format(NULL,"plist::ArrayBase::forceEntry() of wrong type"); }
		
		//! return the value at position @a index, which must exist (no range checking)
		PO& getEntry(size_t index) const { return dynamic_cast<PO&>(*arr[index]); }
		//! return the value at position @a index, which must exist (no range checking, equivalent to getEntry(index))
		PO& operator[](size_t index) const { return dynamic_cast<PO&>(*arr[index]); }
		
		//! return an STL const_iterator to the first entry
		const_iterator begin() const { return arr.begin(); }
		//! return the one-past-end const_iterator
		const_iterator end() const { return arr.end(); }
		//! return first element
		PO& front() const { return dynamic_cast<PO&>(*arr.front()); }
		//! return last element
		PO& back() const { return dynamic_cast<PO&>(*arr.back()); }
		
		//! returns true if the array can store the specified object
		virtual bool canContain(const ObjectBase& obj) { return (dynamic_cast<const PO*>(&obj)!=NULL); }
		
		//! clone implementation for Array
		PLIST_CLONE_DEF(ArrayOf,(new ArrayOf<PO,Alloc>(*this)));
			
	protected:
		//! called with each node being loaded so subclass can handle appropriately
		virtual bool loadXMLNode(size_t index, xmlNode* val, const std::string& comment);
	};
	
	// This specialization allows use of the generic Array(n) constructor, but everything will be NULL: user must be sure to initialize all of the elements!
	template<> ArrayOf<ObjectBase, ObjectBase::conversion_policy<ArrayBase,ObjectBase>::value_conversion>::ArrayOf(storage_t::size_type n) ATTR_deprecated;
	
	/*! plist::Array can handle any mixture of types, whereas plist::ArrayOf<PO> will @e only
	 *  accept the plist objects of type PO (or their subclasses).  This typedef is simply for
	 *  convenience and passes ObjectBase for the template parameter. */
	typedef ArrayOf<ObjectBase> Array;
	extern template class ArrayOf<ObjectBase>;
	extern template class ArrayOf<Dictionary>;
	extern template class ArrayOf<Primitive<float> >;
	extern template class ArrayOf<Primitive<double> >;
	

	template<typename PO, typename Alloc>
	void ArrayOf<PO,Alloc>::setEntry(size_t index, PO& val, bool warnExists/*=false*/) {
		if(index==size()) {
			arr.push_back(&val);
			fireEntryAdded(val);
		} else {
			if(arr[index]==&val)
				return;
			if(warnExists) {
				std::cerr << "Warning: new entry "<<index<<" ("<<val<<") conflicted with previous entry "<<index<<" ("<<(*arr[index])<<")"<<std::endl;
				std::cerr << "         (use setEntry(...,false) if you expect you might need to overwrite)" << std::endl;
			}
			arr[index]=&val;
			fireEntriesChanged();
		}
	}
	template<typename PO, typename Alloc>
	void ArrayOf<PO,Alloc>::addEntry(size_t index, PO& val, const std::string& comment/*=""*/) {
		if(index==size()) {
			arr.push_back(&val);
		} else {
			ArrayBase::storage_t::iterator it=arr.begin();
			advance(it,index);
			arr.insert(it,&val);
		}
		if(comment.size()>0)
			setComment(index,comment);
		fireEntryAdded(val);
	}
	template<typename PO, typename Alloc>
	void ArrayOf<PO,Alloc>::setEntry(size_t index, PO* val, bool warnExists/*=false*/) {
		if(index>size())
			throw bad_format(NULL,"Error: attempted to setEntry() at index beyond one-past-end of plist::Array");
		else if(index==size()) {
			arr.push_back(val);
			fireEntryAdded(*val);
		} else {
			if(arr[index]==val)
				return;
			if(warnExists) {
				std::cerr << "Warning: new entry "<<index<<" ("<<val<<") conflicted with previous entry "<<index<<" ("<<(*arr[index])<<")"<<std::endl;
				std::cerr << "         (use setEntry(...,false) if you expect you might need to overwrite)" << std::endl;
			}
			std::set<ObjectBase*>::iterator it=myRef.find(arr[index]);
			if(it!=myRef.end()) {
				myRef.erase(*it);
				delete arr[index];
			}
			arr[index]=val;
			takeObject(index,val);
			fireEntriesChanged();
		}
	}
	template<typename PO, typename Alloc>
	void ArrayOf<PO,Alloc>::addEntry(size_t index, PO* val, const std::string& comment/*=""*/) {
		if(index>size())
			throw bad_format(NULL,"Error: attempted to addEntry() at index beyond one-past-end of plist::Array");
		else if(index==size()) {
			arr.push_back(val);
		} else {
			ArrayBase::storage_t::iterator it=arr.begin();
			advance(it,index);
			arr.insert(it,val);
		}
		takeObject(index,val);
		if(comment.size()>0)
			setComment(index,comment);
		fireEntryAdded(*val);
	}
	
	//! implements the clone function for Array
	PLIST_CLONE_IMPT2(PO,Alloc,ArrayOf,(new ArrayOf<PO,Alloc>(*this)));
	
	template<typename PO, typename Alloc>
	bool ArrayOf<PO,Alloc>::loadXMLNode(size_t index, xmlNode* val, const std::string& comment) {
		if(index<size()) {
			//have pre-existing entry
			try {
				//it's reasonable to assume that in common usage, the same type will be used each time
				//so let's try to load into the current entry as is
				arr[index]->loadXML(val);
				if(comment.size()>0)
					setComment(index,comment);
				return true;
			} catch(...) {
				// apparently that didn't work, we'll need to make a new reference or clone
				if(loadPolicy!=SYNC) // can we do that?
					throw; // nope -- give up
				// still here? let's try a fresh load using the polymorphic loader (fall through below)
			}
		} else if(!(savePolicy&ADDITIONS)) {
			if(warnUnused && savePolicy==FIXED)
				std::cerr << "Warning: plist::Array ran out of registered items (" << size() << ") during load.  Ignoring extraneous items from source..." << std::endl;
			return false;
		}
		PO * cobj = plist::loadXML<PO>(val);
		if(index<size()) {
			setEntry(index,cobj,false);
			setComment(index,comment);
		} else {
			addEntry(index,cobj,comment);
		}
		return true;
	}
	
	template<typename PO, typename Alloc>
	void ArrayOf<PO,Alloc>::set(const ArrayBase& a) {
		if(&a == this)
			return;
		// if we're doing a large list, might be worth checking if we're actually the same type
		if(const ArrayOf* ao = dynamic_cast<const ArrayOf*>(&a)) {
			operator=(*ao); // same template type, use faster version!
			return;
		}
		ArrayBase::operator=(a);
		// otherwise we'll have to check the conversion on every single entry...
		for(unsigned int index=0; index<a.size(); ++index) {
			ObjectBase* val = &a[index];
			const std::string comment = a.getComment(index);
			if(index<size()) {
				//have pre-existing entry
				try {
					//it's reasonable to assume that in common usage, the same type will be used each time
					//so let's try to load into the current entry as is
					arr[index]->set(*val);
					if(comment.size()>0)
						setComment(index,comment);
					continue;
				} catch(const bad_format& ex) {
					// apparently that didn't work, we'll need to make a new reference or clone
					if(loadPolicy!=SYNC) // can we do that?
						throw; // nope -- give up
					// still here? let's try a fresh load using the polymorphic loader (fall through below)
				}
			} else if(!(savePolicy&ADDITIONS)) {
				if(warnUnused && savePolicy==FIXED)
					std::cerr << "Warning: plist::Array ran out of registered items (" << size() << ") during load.  Ignoring extraneous items from source..." << std::endl;
				break;
			}
			PO* obj=dynamic_cast<PO*>(val);
			if(obj==NULL) {
				obj = plist::allocate<PO>();
				try {
					obj->set(*val);
				} catch(...) {
					delete obj;
					throw;
				}
				myRef.insert(obj);
			} else {
				obj = dynamic_cast<PO*>(obj->clone());
				if(obj==NULL) {
					// this shouldn't be possible: obj is a PO, so obj->clone should be as well
					throw std::runtime_error("plist::Array could not assign clone of object due to impossible type conflict (bad clone?)");
				}
				myRef.insert(obj);
			}
			if(index<size()) {
				setEntry(index,obj,false);
				setComment(index,comment);
			} else {
				addEntry(index,obj,comment);
			}
		}
	}

	template<typename PO, typename Alloc>
	ArrayOf<PO,Alloc>& ArrayOf<PO,Alloc>::operator=(const ArrayOf& a) {
		if(&a==this)
			return *this;
		ArrayBase::operator=(a);
		for(unsigned int index=0; index<a.size(); ++index) {
			PO* val = &a[index];
			const std::string comment = a.getComment(index);
			if(index<size()) {
				//have pre-existing entry
				try {
					//it's reasonable to assume that in common usage, the same type will be used each time
					//so let's try to load into the current entry as is
					plist::assign(*dynamic_cast<PO*>(arr[index]),*val);
					if(comment.size()>0)
						setComment(index,comment);
					continue;
				} catch(const bad_format& ex) {
					// apparently that didn't work, we'll need to make a new reference or clone
					if(loadPolicy!=SYNC) // can we do that?
						throw; // nope -- give up
					// still here? let's try a fresh load using the polymorphic loader (fall through below)
				}
			} else if(!(savePolicy&ADDITIONS)) {
				if(warnUnused && savePolicy==FIXED)
					std::cerr << "Warning: plist::Array ran out of registered items (" << size() << ") during load.  Ignoring extraneous items from source..." << std::endl;
				break;
			}
			PO* obj = dynamic_cast<PO*>(val->clone());
			if(obj==NULL) {
				// this shouldn't be possible: val is a PO, so obj->clone should be as well
				throw std::runtime_error("plist::Array could not assign clone of object due to impossible type conflict (bad clone?)");
			}
			myRef.insert(obj);
			if(index<size()) {
				setEntry(index,obj,false);
				setComment(index,comment);
			} else {
				addEntry(index,obj,comment);
			}
		}
		return *this;
	}
		
	//! take a regex and maximum depth for display (displays entries whos names match the filter @a sel
	/*! @param os the ostream to write into
	 *  @param c the collection to dump
	 *  @param sel the regular expression in string form
	 *  @param selType how to interpret @a sel, this is the flags argument to regcomp(), e.g. REG_EXTENDED or REG_BASIC
	 *  @param depth maximum recursion depth for sub-collections */
	std::ostream& filteredDisplay(std::ostream& os, const ObjectBase& c, const std::string& sel, int selType, unsigned int depth);
	
	//! take a compiled regex and maximum depth for display
	/*! @param os the ostream to write into
	 *  @param c the collection to dump
	 *  @param reg a pre-compiled regular expression, or NULL to match everything
	 *  @param depth maximum recursion depth for sub-collections */
	std::ostream& filteredDisplay(std::ostream& os, const ObjectBase& c, const regex_t* reg, unsigned int depth);
	
	//! provides textual output
	inline std::ostream& operator<<(std::ostream& os, const DictionaryBase& d) { return filteredDisplay(os,d,NULL,-1U); }
	
	//! provides textual output
	inline std::ostream& operator<<(std::ostream& os, const ArrayBase& d) { return filteredDisplay(os,d,NULL,-1U); }
	
} //namespace plist

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
