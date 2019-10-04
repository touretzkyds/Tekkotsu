#include "plistCollections.h"
#include "string_util.h"
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <iomanip>

//better to put this here instead of the header
using namespace std; 

namespace plist {
	template class DictionaryOf<ObjectBase>;
	template class ArrayOf<ObjectBase>;
	template class ArrayOf<Dictionary>;
	template class ArrayOf<Primitive<float> >;
	template class ArrayOf<Primitive<double> >;

	// This specialization allows use of the generic Array(n) constructor, but everything will be NULL: user must be sure to initialize all of the elements!
	template<> ArrayOf<ObjectBase, ObjectBase::conversion_policy<ArrayBase,ObjectBase>::value_conversion>::ArrayOf(storage_t::size_type n)
		: ArrayBase(false), ObjectBase::conversion_policy<ArrayBase,ObjectBase>::value_conversion()
	{
		arr.resize(n);
	}

	template<> ObjectBase* loadXML(xmlNode* node) {
		ObjectBase * obj=plist::loadXML(node);
		if(obj==NULL)
			throw XMLLoadSave::bad_format(node,"plist::loadXML encountered an unknown value type");
		return obj;
	}
	
	template<> PrimitiveBase* loadXML(xmlNode* node) {
		ObjectBase * obj=plist::loadXML(node);
		if(obj==NULL)
			throw XMLLoadSave::bad_format(node,"plist::loadXML encountered an unknown value type");
		PrimitiveBase * cobj = dynamic_cast<PrimitiveBase*>(obj);
		if(cobj==NULL) {
			delete obj;
			throw XMLLoadSave::bad_format(node,"plist::loadXML<PrimitiveBase*> attempted to load non-primitive (e.g. collection?)");
		}
		return cobj;
	}
	template<> Collection* loadXML(xmlNode* node) {
		ObjectBase * obj=plist::loadXML(node);
		if(obj==NULL)
			throw XMLLoadSave::bad_format(node,"plist::loadXML encountered an unknown value type");
		Collection * cobj = dynamic_cast<Collection*>(obj);
		if(cobj==NULL) {
			delete obj;
			throw XMLLoadSave::bad_format(node,"plist::loadXML<Collection*> attempted to load non-collection (e.g. primitive?)");
		}
		return cobj;
	}
	
	Collection::~Collection() {
		delete collectionListeners;
		collectionListeners=NULL;
	}
	
	bool Collection::resolveAssignment(const std::string& arg) {
		std::stringstream errstream;
		return resolveAssignment(arg,errstream);
	}
	
	bool Collection::resolveAssignment(const std::string& arg, std::ostream& errstream) {
		string::size_type eqpos = arg.find("=");
		string value=string_util::trim(arg.substr(eqpos+1));
		bool isAppend = (arg[eqpos-1]=='+');
		string key=string_util::trim(arg.substr(0,isAppend?eqpos-1:eqpos));
		plist::ObjectBase* ob=resolveEntry(key);
		if(ob==NULL) {
			if(isAppend) {
				errstream << "Cannot append to unknown array '" << key << '\'' << endl;
			} else {
				string::size_type dotpos = key.rfind(".");
				string parent = key.substr(0,dotpos);
				string entry = key.substr(dotpos+1);
				ob=resolveEntry(parent);
				if(ob==NULL) {
					errstream << "'" << key << "' is unknown" << endl;
				} else if(plist::Collection* col=dynamic_cast<plist::Collection*>(ob)) {
					if((col->getLoadPolicy() & plist::Collection::ADDITIONS) != plist::Collection::ADDITIONS) {
						errstream << "Collection '" << parent << "' is not dynamically resizeable, cannot add entry for '" << entry << "'" << endl;
					} else if(plist::ArrayBase::StringConversion* arr=dynamic_cast<plist::ArrayBase::StringConversion*>(ob)) {
						try {
							size_t i=atoi(entry.c_str());
							arr->addValue(i,value);
							return true;
						} catch(const XMLLoadSave::bad_format& bf) {
							errstream << "'" << value << "' is a bad value for '" << key << "'" << endl;
							errstream << bf.what() << endl;
						} catch(const std::exception& e) {
							errstream << "An exception occured: " << e.what() << endl;
						}
					} else if(plist::DictionaryBase::StringConversion* dict=dynamic_cast<plist::DictionaryBase::StringConversion*>(ob)) {
						try {
							dict->addValue(entry,value);
							return true;
						} catch(const XMLLoadSave::bad_format& bf) {
							errstream << "'" << value << "' is a bad value for '" << key << "'" << endl;
							errstream << bf.what() << endl;
						} catch(const std::exception& e) {
							errstream << "An exception occured: " << e.what() << endl;
						}
					} else {
						errstream << "Unknown collection type or conversion policy for adding new entry '" << entry <<"' to '" << parent << "' from string value '" << value << '\'' << endl;
					}
				} else {
					errstream << "Cannot add subentries to non-collection '" << parent << "'" << endl;
				}
			}
		} else if(isAppend) {
			if(dynamic_cast<plist::Collection*>(ob)) {
				if(plist::ArrayBase* arr=dynamic_cast<plist::ArrayBase*>(ob)) {
					if((arr->getLoadPolicy() & plist::Collection::ADDITIONS) != plist::Collection::ADDITIONS) {
						errstream << "Array '" << key << "' is not dynamically resizeable, cannot append entry for '" << value << "'" << endl;
					} else if(plist::ArrayBase::StringConversion* strarr=dynamic_cast<plist::ArrayBase::StringConversion*>(ob)) {
						try {
							strarr->addValue(value);
							return true;
						} catch(const XMLLoadSave::bad_format& bf) {
							errstream << "'" << value << "' is a bad value for '" << key << "'" << endl;
							errstream << bf.what() << endl;
						} catch(const std::exception& e) {
							errstream << "An exception occured: " << e.what() << endl;
						}
					} else {
						errstream << "Internal error: don't know how to convert string value '" << value << "' into new entry for '" << key << "'" << endl;
					}
				} else { // probably a dictionary
					errstream << "Cannot append unnamed new entries to non-array, specify new entry name and use '='" << endl;
				}
			} else { // probably a primitive
				errstream << "Cannot add subentries to non-array '" << key << "'" << endl;
			}
		} else {
			if(plist::PrimitiveBase* pbp=dynamic_cast<plist::PrimitiveBase*>(ob)) {
				try {
					pbp->set(value);
					return true;
				} catch(const XMLLoadSave::bad_format& bf) {
					errstream << "'" << value << "' is a bad value for '" << key << "'" << endl;
					errstream << bf.what() << endl;
				} catch(const std::exception& e) {
					errstream << "An exception occured: " << e.what() << endl;
				}
			} else {
				errstream << "Cannot assign to a collection" << endl;
			}
		}
		return false;
	}
	
	void Collection::addCollectionListener(CollectionListener* l) const {
		if(l!=NULL) {
			if(collectionListeners==NULL)
				collectionListeners=new std::set<CollectionListener*>;
			collectionListeners->insert(l);
		}
	}
		
	void Collection::removeCollectionListener(CollectionListener* l) const {
		if(collectionListeners==NULL)
			return;
		std::set<CollectionListener*>::iterator it=collectionListeners->find(l);
		if(it!=collectionListeners->end()) {
			collectionListeners->erase(it);
			if(collectionListeners->empty()) {
				delete collectionListeners;
				collectionListeners=NULL;
			}
		}
	}
	
	bool Collection::isCollectionListener(CollectionListener* l) const {
		if(l==NULL)
			return false;
		if(collectionListeners==NULL)
			return false;
		std::set<CollectionListener*>::iterator it=collectionListeners->find(l);
		return it!=collectionListeners->end();
	}
		
	long Collection::toLong() const { throw std::runtime_error("Unable to cast collection to integer value"); }
	double Collection::toDouble() const { throw std::runtime_error("Unable to cast collection to floating point value"); }

	void Collection::fireEntryAdded(ObjectBase& val) {
		if(collectionListeners==NULL)
			return;
		// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
		std::set<CollectionListener*> pls=*collectionListeners;
		for(std::set<CollectionListener*>::const_iterator it=pls.begin(); collectionListeners!=NULL && it!=pls.end(); ++it) {
			// make sure current listener hasn't been removed
			if(collectionListeners->count(*it)>0)
				(*it)->plistCollectionEntryAdded(*this,val);
		}
	}
	
	void Collection::fireEntryRemoved(ObjectBase& val) {
		if(collectionListeners==NULL)
			return;
		// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
		std::set<CollectionListener*> pls=*collectionListeners;
		for(std::set<CollectionListener*>::const_iterator it=pls.begin(); collectionListeners!=NULL && it!=pls.end(); ++it) {
			// make sure current listener hasn't been removed
			if(collectionListeners->count(*it)>0)
				(*it)->plistCollectionEntryRemoved(*this,val);
		}
	}
	
	void Collection::fireEntriesChanged() {
		if(collectionListeners==NULL)
			return;
		// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
		std::set<CollectionListener*> pls=*collectionListeners;
		for(std::set<CollectionListener*>::const_iterator it=pls.begin(); collectionListeners!=NULL && it!=pls.end(); ++it) {
			// make sure current listener hasn't been removed
			if(collectionListeners->count(*it)>0)
				(*it)->plistCollectionEntriesChanged(*this);
		}
	}
	
	std::string Collection::getIndentationPrefix(xmlNode* node) {
		std::string indentStr;
		for(xmlNode* cur=node->parent; cur!=NULL; cur=cur->parent) {
			if((void*)cur==(void*)node->doc) { //if we hit the document node, discount it and we're done
				if(indentStr.size()>0)
					indentStr=indentStr.substr(0,indentStr.size()-perIndent().size());
				break;
			}
			indentStr+=perIndent();
		}
		return indentStr;
	}
	
	size_t Collection::getIndex(const std::string& name) {
		char * endp=0;
		long index=strtol(name.c_str(),&endp,0);
		if(index<0)
			return (size_t)-1;
		//throw bad_format(NULL,"Collection::getIndex passed negative index encoded in string: "+name);
		if(*endp!='\0')
			return (size_t)-1;
		//throw bad_format(NULL,"Collection::getIndex was called with a non-numeric value");
		return index;
	}
	
	
	void AutoCollectionListener::deactivate() {
		src.removeCollectionListener(this);
		if(const ArrayBase * arr=dynamic_cast<const ArrayBase*>(&src)) {
			for(plist::ArrayBase::const_iterator it=arr->begin(); it!=arr->end(); ++it)
				if(PrimitiveBase * pb = dynamic_cast<PrimitiveBase*>(*it))
					pb->removePrimitiveListener(this);
		} else if(const DictionaryBase * dict=dynamic_cast<const DictionaryBase*>(&src)) {
			for(plist::DictionaryBase::const_iterator it=dict->begin(); it!=dict->end(); ++it)
				if(PrimitiveBase * pb = dynamic_cast<PrimitiveBase*>(it->second))
					pb->removePrimitiveListener(this);
		} else {
			std::cerr << "plist::AutoCollectionListener could not unsubscribe from source entries because source is not a known Collection type" << std::endl;
		}
	}
	void AutoCollectionListener::plistCollectionEntryAdded(plist::Collection& /*col*/, ObjectBase& primitive) {
		if(plist::PrimitiveBase * pb = dynamic_cast<plist::PrimitiveBase*>(&primitive)) {
			pb->addPrimitiveListener(this);
			if(updateOnNew)
				plistValueChanged(*pb);
		} else
			plistSubCollectionAdded(dynamic_cast<plist::Collection&>(primitive));
	}
	void AutoCollectionListener::plistCollectionEntryRemoved(plist::Collection& /*col*/, ObjectBase& primitive) {
		if(plist::PrimitiveBase * pb = dynamic_cast<plist::PrimitiveBase*>(&primitive))
			pb->removePrimitiveListener(this); 
		else
			plistSubCollectionRemoved(dynamic_cast<plist::Collection&>(primitive));
	}
	void AutoCollectionListener::plistCollectionEntriesChanged(plist::Collection& /*col*/) {
		if(const ArrayBase * arr=dynamic_cast<const ArrayBase*>(&src)) {
			for(plist::ArrayBase::const_iterator it=arr->begin(); it!=arr->end(); ++it)
				if(PrimitiveBase * pb = dynamic_cast<PrimitiveBase*>(*it))
					pb->addPrimitiveListener(this);
		} else if(const DictionaryBase * dict=dynamic_cast<const DictionaryBase*>(&src)) {
			for(plist::DictionaryBase::const_iterator it=dict->begin(); it!=dict->end(); ++it)
				if(PrimitiveBase * pb = dynamic_cast<PrimitiveBase*>(it->second))
					pb->addPrimitiveListener(this);
		} else {
			std::cerr << "plist::AutoCollectionListener could not unsubscribe from source entries because source is not a known Collection type" << std::endl;
		}
	}
	
	
	bool DictionaryBase::removeEntry(const std::string& name) {
		storage_t::iterator it=dict.find(name);
		if(it==dict.end())
			return false;
		//still here, then we found exact name match
		ObjectBase* obj=it->second;
		dict.erase(it);
		comments.erase(name);
		fireEntryRemoved(*obj);
		return true;
	}
	
	bool DictionaryBase::renameEntry(const std::string& oldname, const std::string& newname) {
		storage_t::iterator oit=dict.find(oldname);
		if(oit==dict.end())
			return false;
		
		// check for previous inhabitant of the new name
		storage_t::iterator nit=dict.find(newname);
		if(nit!=dict.end()) {
			// we found exact name match on the new name -- remove previous entry
			ObjectBase* obj=nit->second;
			dict.erase(nit);
			comments.erase(newname);
			fireEntryRemoved(*obj);
		}
		
		ObjectBase* val=oit->second;
		dict.erase(oit);
		dict[newname]=val;
		
		// now move comment along too
		comments_t::iterator cit=comments.find(oldname);
		if(cit==comments.end()) { // no comment for item being moved...
			// any comments by a previous resident of the new name?
			cit = comments.find(newname);
			if(cit!=comments.end()) // if so, remove them
				comments.erase(cit);
		} else {
			// item being moved has a comment, bring it along...
			string com = cit->second;
			comments.erase(cit);
			comments[newname]=com;
		}
		fireEntriesChanged();
		return true;
	}
	
	bool DictionaryBase::swapEntry(const std::string& a, const std::string& b) {
		storage_t::iterator ait = dict.find(a);
		storage_t::iterator bit = dict.find(b);
		if(ait==dict.end() && bit==dict.end())
			return false;
		else if(ait==dict.end())
			return renameEntry(b,a);
		else if(bit==dict.end())
			return renameEntry(a,b);
		
		swap(ait->second,bit->second);
		
		// swap comments too
		comments_t::iterator acit = comments.find(a);
		comments_t::iterator bcit = comments.find(b);
		if(acit != comments.end()) {
			if(bcit != comments.end()) {
				// have comments for both
				swap(acit->second,bcit->second);
			} else {
				// only have a comment for a
				string com = acit->second;
				comments.erase(acit);
				comments[b]=com;
			}
		} else if(bcit != comments.end()) {
			// only have a comment for b
			string com = bcit->second;
			comments.erase(bcit);
			comments[a]=com;
		}
		fireEntriesChanged();
		return true;
	}

	ObjectBase* DictionaryBase::resolveEntry(const std::string& path) const {
		//do we have a key with this name?
		const_iterator it=dict.find(path);
		if(it!=dict.end())
			return it->second; //yes, return it
		
		//perhaps there's a sub-dictionary
		string::size_type p;
		it=getSubEntry(path,p);
		if(it==dict.end()) {
			// got noth'n
			return NULL;
		}
		
		//found a matching sub-collection, have it find the rest recursively
		const Collection* d=dynamic_cast<const Collection*>(it->second);
		return d->resolveEntry(path.substr(p+1));
	}
	
	void DictionaryBase::setComment(const std::string& name, const std::string& comment) {
		if(comment.size()==0)
			comments.erase(name);
		else if(comment.find("--")!=std::string::npos)
			throw std::runtime_error("per XML spec, comment string ('"+comment+"' cannot contain '--')");
		else
			comments[name]=comment;
	}

	const std::string& DictionaryBase::getComment(const std::string& name) const {
		storage_t::const_iterator it=dict.find(name);
		if(it==dict.end())
			return emptyStr();
		//found exact name match
		comments_t::const_iterator cit=comments.find(name);
		return (cit!=comments.end()) ? cit->second : emptyStr();
	}
	
	void DictionaryBase::loadXML(xmlNode* node) {
		//check if our node has been set to NULL (invalid or not found)
		if(node==NULL)
			return;
		if(!xNodeHasName(node,"dict"))
			throw bad_format(node,"Dictionary::loadXML expected <dict> value, got "+std::string((const char*)xNodeGetName(node)));
		
		LoadSavePolicy origLoadPolicy=loadPolicy;
		xmlChar * att = xmlGetProp(node,(const xmlChar*)"load");
		if(att!=NULL) {
			if(xmlStrcasecmp(att, (const xmlChar*)"fixed")==0) {
				setLoadPolicy(Collection::FIXED);
			} else if(xmlStrcasecmp(att, (const xmlChar*)"union")==0) {
				setLoadPolicy(Collection::UNION);
			} else if(xmlStrcasecmp(att, (const xmlChar*)"intersect")==0) {
				setLoadPolicy(Collection::INTERSECT);
			} else if(xmlStrcasecmp(att, (const xmlChar*)"sync")==0) {
				setLoadPolicy(Collection::SYNC);
			} else {
				std::cerr << "WARNING: unknown plist::Dictionary load mode '" << (char*)att << "', ignoring..." << std::endl;
				std::string file;
				xmlChar* uri = xmlNodeGetBase(node->doc,node);
				if(uri!=NULL && uri[0]!='\0')
					file = std::string(" of ") + (char*)uri + std::string(":");
				else
					file = " at line ";
				xmlFree(uri);
				std::cerr << "  (from" << file << xmlGetLineNo(node) << ")" << std::endl;
			}
		}
		xmlFree(att); att=NULL;
		
		try {
			std::string comment;
			std::set<std::string> seen;
			//process children nodes
			for(xmlNode* cur = skipToElement(node->children,comment); cur!=NULL; cur = skipToElement(cur->next,comment)) {
							
				//find the next key node
				xmlNode * k=cur;
				if(xmlStrcmp(k->name, (const xmlChar *)"key"))
					throw bad_format(k,"Dictionary format error: expect data in pairs of key and value (two values found in a row)");
				cur=skipToElement(cur->next);
				
				//find the following value (non-key) node
				xmlNode * v=cur;
				if(v==NULL)
					throw bad_format(cur,"Dictionary format error: expect data in pairs of key and value (dictionary ended with hanging key)");
				if(!xmlStrcmp(v->name, (const xmlChar *)"key"))
					throw bad_format(v,"Dictionary format error: expect data in pairs of key and value (two keys found in a row)");
				
				//find corresponding entry
				xmlChar* cont=xmlNodeGetContent(k);
				string key=(const char*)cont;
				xmlFree(cont);
				seen.insert(key);
				loadXMLNode(key,v,comment);
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
		} catch(...) {
			setLoadPolicy(origLoadPolicy);
			throw;
		}
		setLoadPolicy(origLoadPolicy);
	}
	
	void DictionaryBase::saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const {
		//check if our node has been set to NULL (invalid or not found)
		if(node==NULL)
			return;
		
		//set the type of the current node
		xmlNodeSetName(node,(const xmlChar*)"dict");
		
		//find the depth of the target node in the xml tree to maintain proper indentation
		std::string indentStr=getIndentationPrefix(node);
		
		//This will hold any comments found between elements -- if no comment is found, a new one may be added
		std::string comment;

		//process children nodes
		xmlNode* prev=node->children;
		for(xmlNode* cur = skipToElement(node->children,comment); cur!=NULL; cur = skipToElement(cur,comment)) {
			
			//find the next key node
			xmlNode * k=cur;
			if(xmlStrcmp(k->name, (const xmlChar *)"key")) {
				cur = k->next;
				xmlUnlinkNode(k);
				xmlFreeNode(k);
				continue;
			}
			cur=skipToElement(cur->next);
			
			//find the following value (non-key) node
			xmlNode * v=cur;
			if(v==NULL) {
				xmlUnlinkNode(k);
				xmlFreeNode(k);
				break;
			}
			if(!xmlStrcmp(v->name, (const xmlChar *)"key"))
				throw bad_format(v,"Dictionary format error: expect data in pairs of key and value (two keys found in a row)");
			
			xmlChar* cont=xmlNodeGetContent(k);
			std::string key=(const char*)cont;
			xmlFree(cont);
			if(!saveOverXMLNode(k,v,key,comment,indentStr,seen)) {
				cur=xNodeGetNextNode(cur);
				if(savePolicy&REMOVALS) {
					while(prev!=cur) {
						xmlNode* n=prev;
						prev=xNodeGetNextNode(prev);
						xmlUnlinkNode(n);
						xmlFreeNode(n);
					}
				} else {
					if(warnUnused && savePolicy==FIXED)
						cerr << "Warning: saving over existing plist dictionary, key '" << key << "' does not match a registered variable.  Ignoring..." << endl;
				}
				prev=cur;
			}
			prev=cur=xNodeGetNextNode(cur);
		}

		if(!onlyOverwrite && seen.size()!=dict.size()) {
			// clear text nodes from end of dictionary back to last entry
			for(xmlNode* cur=node->last; cur!=NULL && cur->type==XML_TEXT_NODE; cur=node->last) {
				xmlUnlinkNode(cur);
				xmlFreeNode(cur);
			}
			size_t longestKeyLen = getLongestKeyLen(NULL,1);
			// the main dictionary has entries that weren't seen... find which ones
			// if needed, this could be made faster (O(n) vs. current O(n lg n)) by assuming the maps
			// are sorted and moving two iterators through together instead of repeated find()'s
			for(storage_t::const_iterator it=dict.begin(); it!=dict.end(); ++it) {
				if(seen.find(it->first)==seen.end()) {
					//we didn't see this node in the existing xml tree, have to add a new node pair for it
					saveXMLNode(node,it->first,it->second,indentStr,longestKeyLen);
				}
			}
			std::string parentIndent;
			if(indentStr.size()>=perIndent().size())
				parentIndent=indentStr.substr(perIndent().size());
			xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+parentIndent).c_str()));
		}
	}

	std::string DictionaryBase::toString() const {
		stringstream s;
		s << *this;
		return s.str();
	}
	
	unsigned int DictionaryBase::getLongestKeyLen(const regex_t* reg/*=NULL*/, unsigned int depth/*=-1*/) const {
		if(depth==0)
			return 0;
		size_t longest=0;
		size_t seplen=subCollectionSep().size();
		for(DictionaryBase::const_iterator it=begin(); it!=end(); ++it) {
			if(reg!=NULL && regexec(reg,it->first.c_str(),0,NULL,0)!=0)
				continue;
			size_t cur=it->first.size();
			if(Collection* dp=dynamic_cast<Collection*>(it->second))
				cur+=dp->getLongestKeyLen(reg,depth-1)+seplen;
			longest=std::max(longest,cur);
		}
		return longest;
	}
	
	DictionaryBase::iterator DictionaryBase::getSubEntry(const std::string& name, std::string::size_type& seppos) {
		seppos=name.find(subCollectionSep());
		if(seppos==string::npos)
			return dict.end(); //no '.'s found -- go away
		iterator it=dict.find(name.substr(0,seppos));
		if(it==dict.end())
			return dict.end(); //no entry matching prefix -- go away
		const Collection* d=dynamic_cast<const Collection*>(it->second);
		if(d==NULL)
			return dict.end(); //matching prefix is not a collection -- go away
		return it;
	}
	DictionaryBase::const_iterator DictionaryBase::getSubEntry(const std::string& name, std::string::size_type& seppos) const {
		seppos=name.find(subCollectionSep());
		if(seppos==string::npos)
			return dict.end(); //no '.'s found -- go away
		const_iterator it=dict.find(name.substr(0,seppos));
		if(it==dict.end())
			return dict.end(); //no entry matching prefix -- go away
		const Collection* d=dynamic_cast<const Collection*>(it->second);
		if(d==NULL)
			return dict.end(); //matching prefix is not a collection -- go away
		return it;
	}
		
	void DictionaryBase::clear() {
		storage_t::size_type s=dict.size();
		// this bit of trickiness is to handle element destructors doing things to the list while it's being cleared
		std::set<ObjectBase*> refs=myRef;
		dict.clear();
		myRef.clear();
		comments.clear();
		if(s>0) //only fire if we had entries to begin with
			fireEntriesChanged();
		for(std::set<ObjectBase*>::iterator it=refs.begin(); it!=refs.end(); ++it)
			delete *it;
	}
	
	void DictionaryBase::takeObject(const std::string& /*name*/, ObjectBase* obj) {
		myRef.insert(obj);
	}

	void DictionaryBase::fireEntryRemoved(ObjectBase& val) {
		Collection::fireEntryRemoved(val);
		std::set<ObjectBase*>::iterator it=myRef.find(&val);
		if(it!=myRef.end()) {
			myRef.erase(it);
			delete &val;
		}
	}
	
	void DictionaryBase::cloneMyRef() {
		for(iterator dit=dict.begin(); dit!=dict.end(); ++dit) {
			std::set<ObjectBase*>::iterator rit=myRef.find(dit->second);
			if(rit!=myRef.end()) {
				myRef.erase(rit);
				myRef.insert(dit->second=dynamic_cast<ObjectBase*>((dit->second)->clone()));
			}
		}
		
		//slower implementation, but can handle multiple pointers to the same instance (which we don't support elsewhere, so no point in doing it)
		/*
		 std::set<ObjectBase*> ns;
		for(std::set<ObjectBase*>::iterator it=myRef.begin(); it!=myRef.end(); ++it) {
			ObjectBase* n=dynamic_cast<ObjectBase*>((*it)->clone());
			bool used=false;
			for(iterator dit=dict.begin(); dit!=dict.end(); ++dit) {
				if(*it==dit->second) {
					dit->second=n;
					used=true;
				}
			}
			if(!used) {
				cerr << "Warning: dictionary claims control over pointer not found in dictionary" << endl;
				delete n;
			} else
				ns.insert(n);
		}
		myRef=ns;
		*/
	}
	
	bool DictionaryBase::saveOverXMLNode(xmlNode* k, xmlNode* val, const std::string& key, std::string comment, const std::string& indentStr, std::set<std::string>& seen) const {
		//find corresponding entry
		storage_t::const_iterator it=findEntry(key);
		if(it==dict.end())
			return false;
		if(comment.size()==0) {
			bool isSub=dynamic_cast<const Collection*>(it->second);
			if(isSub)
				if(const ArrayBase* arr=dynamic_cast<const ArrayBase*>(it->second))
					isSub = !arr->getSaveInlineStyle();
			bool isFirst=true;
			const std::string indentedNewline="\n"+indentStr;
			comments_t::const_iterator cit=comments.find(key);
			if(cit!=comments.end()) {
				while(k->prev!=NULL && xNodeIsText(k->prev)) {
					xmlNode* n=k->prev;
					xmlUnlinkNode(n);
					xmlFreeNode(n);
				}
			}
			if(!saveCondensed) {
				const std::string headline=("======== "+it->first+" ========");
				if(isSub && static_cast<const Collection*>(it->second)->size()>0) {
					isFirst=(skipToElement(k->parent->children)==k);
					xmlAddPrevSibling(k,xmlNewText((const xmlChar*)(isFirst ? indentedNewline : indentedNewline+indentedNewline).c_str()));
					xmlAddPrevSibling(k,xmlNewComment((const xmlChar*)headline.c_str()));
				}
			}
			if(cit!=comments.end()) {
				if(isSub || cit->second.find(key)<KEY_IN_COMMENT_MAX_POS)
					comment=cit->second;
				else //if not a sub-dictionary, and comment doesn't already start with entry name, prepend entry name
					comment=key+": "+cit->second;
				string::size_type pos=comment.rfind('\n');
				while(pos!=string::npos) {
					if(comment.compare(pos+1,indentStr.size(),indentStr)!=0)
						comment.insert(pos+1,indentStr);
					if(pos==0)
						break;
					pos = comment.rfind('\n',pos-1);
				}
				if(!isSub)
					isFirst=(skipToElement(k->parent->children)==k);
				xmlAddPrevSibling(k,xmlNewText((const xmlChar*)(isFirst ? indentedNewline : indentedNewline+indentedNewline).c_str()));
				xmlAddPrevSibling(k,xmlNewComment((const xmlChar*)comment.c_str()));
				xmlAddPrevSibling(k,xmlNewText((const xmlChar*)indentedNewline.c_str()));
			}
		}
		it->second->saveXML(val);
		if(seen.find(key)!=seen.end()) {
			std::cerr << "WARNING: plist::Dictionary found duplicate key " << key << " during save" << std::endl;
		} else {
			seen.insert(key);
		}
		return true;
	}
	
	void DictionaryBase::saveXMLNode(xmlNode* node, const std::string& key, const ObjectBase* val, const std::string& indentStr, size_t longestKeyLen) const {
		bool isSub=dynamic_cast<const Collection*>(val);
		if(isSub)
			if(const ArrayBase* arr=dynamic_cast<const ArrayBase*>(val))
				isSub = !arr->getSaveInlineStyle();
		bool isFirst=(node->children==NULL);
		const std::string indentedNewline="\n"+indentStr;
		if(!saveCondensed) {
			const std::string headline=("======== "+key+" ========");
			if(isSub && static_cast<const Collection*>(val)->size()>0) {
				xmlAddChild(node,xmlNewText((const xmlChar*)(isFirst ? indentedNewline : indentedNewline+indentedNewline).c_str()));
				xmlAddChild(node,xmlNewComment((const xmlChar*)headline.c_str()));
			}
		}
		std::string comment;
		comments_t::const_iterator cit=comments.find(key);
		if(cit!=comments.end()) {
			if(isSub || cit->second.find(key)<KEY_IN_COMMENT_MAX_POS)
				comment=cit->second;
			else
				comment=key+": "+cit->second;
			string::size_type pos=comment.rfind('\n');
			while(pos!=string::npos) {
				if(comment.compare(pos+1,indentStr.size(),indentStr)!=0)
					comment.insert(pos+1,indentStr);
				if(pos==0)
					break;
				pos = comment.rfind('\n',pos-1);
			}
			xmlAddChild(node,xmlNewText((const xmlChar*)(isSub || isFirst ? indentedNewline : indentedNewline+indentedNewline).c_str()));
			xmlAddChild(node,xmlNewComment((const xmlChar*)comment.c_str()));
		}
		xmlAddChild(node,xmlNewText((const xmlChar*)indentedNewline.c_str()));
		xmlNode* k=xmlNewChild(node,NULL,(const xmlChar*)"key",(const xmlChar*)key.c_str());
		if(k==NULL)
			throw bad_format(node,"Error: plist Dictionary xml error on saving key");
		string space(longestKeyLen-string_util::utf8len(key)+1,' ');
		xmlAddChild(node,xmlNewText((const xmlChar*)space.c_str()));
		xmlNode* v=xmlNewChild(node,NULL,(const xmlChar*)"",NULL);
		if(v==NULL)
			throw bad_format(node,"Error: plist Dictionary xml error on saving value");
		val->saveXML(v);
	}
	
	
	bool ArrayBase::removeEntry(size_t index) {
		if(index>=arr.size())
			 return false;
		storage_t::iterator it=arr.begin();
		advance(it,index);
		ObjectBase* obj=*it;
		arr.erase(it);
		comments.erase(index);
		fireEntryRemoved(*obj);
		return true;
	}

	ObjectBase* ArrayBase::resolveEntry(const std::string& path) const {
		size_t index=getIndex(path);
		if(index<size())
			return &getEntry(index);
		std::string::size_type p;
		const_iterator it=getSubEntry(path,p);
		if(it==arr.end())
			return NULL;
		const Collection * d=dynamic_cast<const Collection*>(*it);
		return d->resolveEntry(path.substr(p+1));
	}
		
	void ArrayBase::clear() {
		storage_t::size_type s=arr.size();
		// this bit of trickiness is to handle element destructors doing things to the list while it's being cleared
		std::set<ObjectBase*> refs=myRef;
		arr.clear();
		comments.clear();
		myRef.clear();
		if(s>0) //only fire if we had entries to begin with
			fireEntriesChanged();
		for(std::set<ObjectBase*>::iterator it=refs.begin(); it!=refs.end(); ++it)
			delete *it;
	}
	
	void ArrayBase::setComment(size_t index, const std::string& comment) {
		if(comment.size()==0)
			comments.erase(index);
		else if(comment.find("--")!=std::string::npos)
			throw std::runtime_error("per XML spec, comment string ('"+comment+"' cannot contain '--')");
		else
			comments[index]=comment;
	}
	
	const std::string& ArrayBase::getComment(size_t index) const {
		comments_t::const_iterator it=comments.find(index);
		if(it==comments.end())
			return emptyStr();
		else
			return it->second;
	}
	
	void ArrayBase::loadXML(xmlNode* node) {
		//check if our node has been set to NULL (invalid or not found)
		if(node==NULL)
			return;
		if(!xNodeHasName(node,"array"))
			throw bad_format(node,"Array::loadXML expected <array> value, got "+std::string((const char*)xNodeGetName(node)));
		
		std::string comment;
		unsigned int i=0;
		for(xmlNode* cur = skipToElement(xNodeGetChildren(node),comment); cur!=NULL; cur = skipToElement(xNodeGetNextNode(cur),comment)) {
			if(!loadXMLNode(i++, cur, comment))
				 break;
		}
		if(loadPolicy&REMOVALS) {
			while(i<size())
				removeEntry(size()-1);
		} 
	}
	
	void ArrayBase::saveXML(xmlNode* node) const {
		//check if our node has been set to NULL (invalid or not found)
		if(node==NULL)
			return;
		
		//set the type of the current node
		xmlNodeSetName(node,(const xmlChar*)"array");
		
		//find the depth of the target node in the xml tree to maintain proper indentation
		std::string indentStr=getIndentationPrefix(node);
		std::string parentIndent;
		if(indentStr.size()>=perIndent().size())
			parentIndent=indentStr.substr(perIndent().size());
		
		//This will hold any comments found between elements -- if no comment is found, a new one may be added
		std::string comment;
		
		//This will be the index of the item we're loading next
		unsigned int i=0;
		
		//process children nodes
		xmlNode * prev=xNodeGetChildren(node);
		for(xmlNode* cur = skipToElement(prev,comment); cur!=NULL; cur = skipToElement(cur,comment)) {
			
			if(i==arr.size()) {
				if(savePolicy&REMOVALS) {
					while(prev!=NULL) {
						xmlNode* n=prev;
						prev=xNodeGetNextNode(prev);
						xmlUnlinkNode(n);
						xmlFreeNode(n);
					}
				} else {
					if(warnUnused && savePolicy==FIXED)
						std::cerr << "Warning: plist::Array ignoring extraneous items in destination during save..." << std::endl;
				}
				break;
			}
			if(comment.size()==0) {
				comments_t::const_iterator cit=comments.find(i);
				if(cit!=comments.end()) {
					std::stringstream buf;
					buf << i;
					if(/*isSub ||*/ cit->second.compare(0,buf.str().size(),buf.str())==0)
						comment=cit->second;
					else { //if not a sub-dictionary, and comment doesn't already start with entry name, prepend entry name
						comment=buf.str();
						comment+=": "+cit->second;
					}
					xmlAddPrevSibling(cur,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
					std::string::size_type pos=comment.rfind('\n');
					while(pos!=std::string::npos) {
						if(comment.compare(pos+1,indentStr.size(),indentStr)!=0)
							comment.insert(pos+1,indentStr);
						if(pos==0)
							break;
						pos = comment.rfind('\n',pos-1);
					}
					xmlAddPrevSibling(cur,xmlNewComment((const xmlChar*)comment.c_str()));
					xmlAddPrevSibling(cur,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
				}
			}
			arr[i++]->saveXML(cur);
			prev=cur=xNodeGetNextNode(cur);
		}
		
		if(!(savePolicy&ADDITIONS))
			return;
		
		bool hadUnsaved = (i<arr.size());
		for(; i<arr.size(); ++i) {
			comments_t::const_iterator cit=comments.find(i);
			if(cit!=comments.end()) {
				std::stringstream buf;
				buf << i;
				if(/*isSub ||*/ cit->second.compare(0,buf.str().size(),buf.str())==0)
					comment=cit->second;
				else { //if not a sub-dictionary, and comment doesn't already start with entry name, prepend entry name
					comment=buf.str();
					comment+=": "+cit->second;
				}
				xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
				std::string::size_type pos=comment.rfind('\n');
				while(pos!=std::string::npos) {
					if(comment.compare(pos+1,indentStr.size(),indentStr)!=0)
						comment.insert(pos+1,indentStr);
					if(pos==0)
						break;
					pos = comment.rfind('\n',pos-1);
				}
				xmlAddChild(node,xmlNewComment((const xmlChar*)comment.c_str()));
			}
			if(saveInlineStyle)
				xmlAddChild(node,xmlNewText((const xmlChar*)" "));
			else
				xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
			xmlNode* v=xmlNewChild(node,NULL,(const xmlChar*)"",NULL);
			if(v==NULL)
				throw bad_format(node,"Error: plist Array xml error on saving value");
			arr[i]->saveXML(v);
		}
		if(hadUnsaved) {
			if(saveInlineStyle)
				xmlAddChild(node,xmlNewText((const xmlChar*)" "));
			else
				xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+parentIndent).c_str()));
		}
	}
	
	std::string ArrayBase::toString() const {
		std::stringstream s;
		s << *this;
		return s.str();
	}
	
	unsigned int ArrayBase::getLongestKeyLen(const regex_t* reg/*=NULL*/, unsigned int depth/*=-1*/) const {
		if(depth==0)
			return 0;
		size_t longest=0;
		size_t seplen=subCollectionSep().size();
		for(size_t i=0; i<size(); ++i) {
			std::stringstream s;
			s << i;
			if(reg!=NULL && regexec(reg,s.str().c_str(),0,NULL,0)!=0)
				continue;
			size_t cur=s.str().size();
			if(Collection* dp=dynamic_cast<Collection*>(arr[i]))
				cur+=dp->getLongestKeyLen(reg,depth-1)+seplen;
			longest=std::max(longest,cur);
		}
		return longest;
	}
	
	void ArrayBase::takeObject(size_t /*index*/, ObjectBase* obj) {
		myRef.insert(obj);
	}
	
	void ArrayBase::fireEntryRemoved(ObjectBase& val) {
		Collection::fireEntryRemoved(val);
		std::set<ObjectBase*>::iterator it=myRef.find(&val);
		if(it!=myRef.end()) {
			myRef.erase(it);
			delete &val;
		}
	}
	
	ArrayBase::iterator ArrayBase::getSubEntry(const std::string& name, std::string::size_type& seppos) {
		seppos=name.find(subCollectionSep());
		if(seppos==std::string::npos)
			return arr.end(); //no '.'s found -- go away
		size_t index=getIndex(name.substr(0,seppos));
		if(index>=size())
			return arr.end(); //no entry matching prefix -- go away
		iterator it=arr.begin();
		advance(it,index);
		const Collection* d=dynamic_cast<const Collection*>(*it);
		if(d==NULL)
			return arr.end(); //matching prefix is not a collection -- go away
		return it;
	}
	ArrayBase::const_iterator ArrayBase::getSubEntry(const std::string& name, std::string::size_type& seppos) const {
		seppos=name.find(subCollectionSep());
		if(seppos==std::string::npos)
			return arr.end(); //no '.'s found -- go away
		size_t index=getIndex(name.substr(0,seppos));
		if(index>=size())
			return arr.end(); //no entry matching prefix -- go away
		const_iterator it=arr.begin();
		advance(it,index);
		const Collection* d=dynamic_cast<const Collection*>(*it);
		if(d==NULL)
			return arr.end(); //matching prefix is not a collection -- go away
		return it;
	}
	
	void ArrayBase::cloneMyRef() {
		for(iterator dit=arr.begin(); dit!=arr.end(); ++dit) {
			std::set<ObjectBase*>::iterator rit=myRef.find(*dit);
			if(rit!=myRef.end()) {
				myRef.erase(rit);
				myRef.insert(*dit=dynamic_cast<ObjectBase*>((*dit)->clone()));
			}
		}
	}
	
	std::ostream& filteredDisplay(std::ostream& os, const ObjectBase& c, const std::string& sel, int selType, unsigned int depth) {
		if(sel.size()==0)
			return filteredDisplay(os,c,NULL,depth);
		regex_t r;
		if(regcomp(&r,sel.c_str(),selType|REG_NOSUB)==0)
			filteredDisplay(os,c,&r,depth);
		regfree(&r);
		return os;
	}
		
	std::ostream& filteredDisplay(std::ostream& os, const ObjectBase& c, const regex_t* reg, unsigned int depth) {
		unsigned int seplen=Collection::subCollectionSep().size();
		unsigned int out=0;
		
		if(const ArrayBase* a=dynamic_cast<const ArrayBase*>(&c)) {
			unsigned int longest=std::max(a->getLongestKeyLen(reg,depth),static_cast<unsigned int>(os.width()));
			for(unsigned long i=0; i<a->size(); ++i) {
				stringstream ns;
				ns << i;
				if(reg!=NULL && regexec(reg,ns.str().c_str(),0,NULL,0)!=0)
					continue;
				out++;
				if(depth==0)
					return os << right << setw(longest) << "" << " = [...]" << endl;
				if(Collection* dp=dynamic_cast<Collection*>(&(*a)[i])) {
					stringstream ss;
					ss << left << std::setw(longest-snprintf(NULL,0,"%lu",i)-seplen);
					filteredDisplay(ss,*dp,reg,depth-1);
					std::string line;
					for(getline(ss,line); ss; std::getline(ss,line))
						os << (ns.str() + Collection::subCollectionSep() + line) << std::endl;
				} else {
					os << std::left << std::setw(longest) << ns.str() << " = " << (*a)[i] << std::endl;
				}
			}
			if(out==0)
				return os << right << setw(longest) << "" << " = (empty array)" << endl;
			
		} else if(const DictionaryBase* d=dynamic_cast<const DictionaryBase*>(&c)) {
			unsigned int longest=std::max(d->getLongestKeyLen(reg,depth),static_cast<unsigned int>(os.width()));
			for(DictionaryBase::storage_t::const_iterator it=d->begin(); it!=d->end(); ++it) {
				if(reg!=NULL && regexec(reg,it->first.c_str(),0,NULL,0)!=0)
					continue;
				out++;
				if(depth==0)
					return os << right << setw(longest) << "" << " = [...]" << endl;
				if(Collection* dp=dynamic_cast<Collection*>(it->second)) {
					stringstream ss;
					ss << left << setw(longest-it->first.size()-seplen);
					filteredDisplay(ss,*dp,reg,depth-1);
					string line;
					for(getline(ss,line); ss; getline(ss,line))
						os << (it->first + Collection::subCollectionSep() + line) << endl;
				} else {
					os << left << setw(longest) << it->first << " = " << *it->second << endl;
				}
			}
			if(out==0)
				return os << right << setw(longest) << "" << " = (empty dictionary)" << endl;
			
		} else {
			os << c.toString();
		}
		return os;
	}
	
} //namespace plist


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
