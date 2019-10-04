#include "plistBase.h"
#include "plistPrimitives.h"
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

//better to put this here instead of the header
using namespace std; 

namespace plist {
	
	ObjectBase::ObjectBase()
	: XMLLoadSave()
	{}
	
	ObjectBase::~ObjectBase() {}
	
	void ObjectBase::setParseTree(xmlDoc * doc) const {
		XMLLoadSave::setParseTree(doc);
		if(xmldocument==NULL)
			return;
		xmlNode* root=XMLLoadSave::FindRootXMLElement(doc);
		if(root != NULL && xmlStrcmp(root->name, (const xmlChar *)"plist")==0)
			return;
		xmlNodePtr cur = xmlNewNode(NULL,(const xmlChar*)"plist");
		xmlNewProp(cur,(const xmlChar*)"version",(const xmlChar*)"1.0");
		xmlFree(xmlDocSetRootElement(xmldocument,cur));
		xmlCreateIntSubset(xmldocument,(const xmlChar*)"plist",(const xmlChar*)"-//Apple//DTD PLIST 1.0//EN",(const xmlChar*)"http://www.apple.com/DTDs/PropertyList-1.0.dtd");
	}
	
	xmlNode* ObjectBase::FindRootXMLElement(xmlDoc* doc) const {
		if(doc==NULL)
			return NULL;
		xmlNode* root=XMLLoadSave::FindRootXMLElement(doc);
		if(root == NULL)
			throw bad_format(root,"Error: plist read empty document");
		if(xmlStrcmp(root->name, (const xmlChar *)"plist"))
			throw bad_format(root,"Error: plist read document of the wrong type, root node != plist");
		string filename;
		if(doc->name!=NULL && doc->name[0]!='\0') {
			filename="document '";
			filename+=doc->name;
			filename+="' ";
		}
		if(!xmlHasProp(root,(const xmlChar*)"version"))
			cerr << "Warning: plist " << filename << "does not carry version number, assuming 1.0" << endl;
		else {
			xmlChar* strv=xmlGetProp(root,(const xmlChar*)"version");
			double version=strtod((const char*)strv,NULL);
			if(version>1.0)
				cerr << "WARNING: plist " << filename << "is version " << strv << ", this software only knows 1.0.  Trying anyway..." << endl;
			if(version==0)
				cerr << "WARNING: plist " << filename << "seems to have invalid version '" << strv << "', this software only knows 1.0.  Trying anyway..." << endl;
			xmlFree(strv);
		}
		
		// find first element node within the plist
		xmlNode* cur=root->children;
		while(cur!=NULL && cur->type!=XML_ELEMENT_NODE)
			cur=cur->next;
		if(cur==NULL) //empty plist
			cur = xmlNewChild(root,NULL,(const xmlChar*)"",NULL);
		return cur;
	}
	
	bool ObjectBase::xNodeHasName(xmlNode* node, const char* name) {
		return xmlStrcasecmp(node->name,(const xmlChar*)name)==0;
	}
	const xmlChar* ObjectBase::xNodeGetName(xmlNode* node) {
		return node->name;
	}
	xmlChar* ObjectBase::xGetNodePath(xmlNode* node) {
		return xmlGetNodePath(node);
	}
	const xmlChar* ObjectBase::xNodeGetURL(xmlNode* node) {
		return node->doc->URL;
	}
	xmlNode* ObjectBase::xNodeGetChildren(xmlNode* node) {
		return node->children;
	}
	xmlNode* ObjectBase::xNodeGetLastChild(xmlNode* node) {
		return node->last;
	}
	xmlNode* ObjectBase::xNodeGetNextNode(xmlNode* node) {
		return node->next;
	}
	xmlNode* ObjectBase::xNodeGetPrevNode(xmlNode* node) {
		return node->prev;
	}
	xmlNode* ObjectBase::xNodeGetParent(xmlNode* node) {
		return node->parent;
	}
	xmlDoc* ObjectBase::xNodeGetDoc(xmlNode* node) {
		return node->doc;
	}
	bool ObjectBase::xNodeIsText(xmlNode* node) {
		return node->type==XML_TEXT_NODE;
	}
	bool ObjectBase::xNodeIsElement(xmlNode* node) {
		return node->type==XML_ELEMENT_NODE;
	}
	bool ObjectBase::xNodeIsComment(xmlNode* node) {
		return node->type==XML_COMMENT_NODE;
	}
		

	PrimitiveBase& PrimitiveBase::operator=(const Primitive<std::string>& v) { *this = static_cast<const PrimitiveBase&>(v); return *this; }
	PrimitiveBase& PrimitiveBase::operator=(const std::string& v) { *this=static_cast<const PrimitiveBase&>(Primitive<std::string>(v)); return *this; }
	PrimitiveBase& PrimitiveBase::operator=(long v) { *this=Primitive<long>(v); return *this; }
	PrimitiveBase& PrimitiveBase::operator=(unsigned long v) { *this=Primitive<unsigned long>(v); return *this; }
	PrimitiveBase& PrimitiveBase::operator=(double v) { *this=Primitive<double>(v); return *this; }
	
	PrimitiveBase::~PrimitiveBase() {
		delete primitiveListeners;
		primitiveListeners=NULL;
	}
	
	void PrimitiveBase::addPrimitiveListener(PrimitiveListener* vl) const {
		if(vl!=NULL) {
			if(primitiveListeners==NULL)
				primitiveListeners=new std::set<PrimitiveListener*>;
			primitiveListeners->insert(vl);
		}
	}
	void PrimitiveBase::removePrimitiveListener(PrimitiveListener* vl) const {
		if(primitiveListeners==NULL)
			return;
		std::set<PrimitiveListener*>::iterator it=primitiveListeners->find(vl);
		if(it!=primitiveListeners->end()) {
			primitiveListeners->erase(it);
			if(primitiveListeners->empty()) {
				delete primitiveListeners;
				primitiveListeners=NULL;
			}
		}
	}
	bool PrimitiveBase::isPrimitiveListener(PrimitiveListener* vl) const {
		if(vl==NULL)
			return false;
		if(primitiveListeners==NULL)
			return false;
		std::set<PrimitiveListener*>::iterator it=primitiveListeners->find(vl);
		return it!=primitiveListeners->end();
	}
	void PrimitiveBase::fireValueChanged(bool touchOnly) const {
		if(primitiveListeners==NULL)
			return;
		// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
		std::set<PrimitiveListener*> pls=*primitiveListeners;
		if(touchOnly) {
			for(std::set<PrimitiveListener*>::const_iterator it=pls.begin(); primitiveListeners!=NULL && it!=pls.end(); ++it) {
				// make sure current listener hasn't been removed
				if(primitiveListeners->count(*it)>0)
					(*it)->plistValueTouched(*this);
			}
		} else {
			for(std::set<PrimitiveListener*>::const_iterator it=pls.begin(); primitiveListeners!=NULL && it!=pls.end(); ++it) {
				// make sure current listener hasn't been removed
				if(primitiveListeners->count(*it)>0)
					(*it)->plistValueChanged(*this);
			}
		}
	}
		
} //namespace plist


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
