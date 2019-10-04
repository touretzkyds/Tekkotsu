#include "plistPrimitives.h"

namespace plist {
	template class Primitive<short>;
	template class Primitive<unsigned short>;
	template class Primitive<int>;
	template class Primitive<unsigned int>;
	template class Primitive<long>;
	template class Primitive<unsigned long>;
	template class Primitive<float>;
	template class Primitive<double>;
	
	void Primitive<bool>::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		if(xNodeHasName(node,"true")) {
			prevVal=val; 
			val=true;
			fireValueChanged((prevVal && val) || (!prevVal && !val)); 
		} else if(xNodeHasName(node,"false")) {
			prevVal=val; 
			val=false;
			fireValueChanged((prevVal && val) || (!prevVal && !val)); 
		} else if(xNodeHasName(node,"integer") || xNodeHasName(node,"real")) {
			prevVal=val; 
			xmlChar * cont=xmlNodeGetContent(node);
			std::stringstream str((char*)cont);
			str >> val;
			xmlFree(cont);
			fireValueChanged((prevVal && val) || (!prevVal && !val));
		} else if(xNodeHasName(node,"string")) {
			xmlChar * cont=xmlNodeGetContent(node);
			try {
				set((char*)cont);
				std::cerr << "Warning: plist boolean expects value of '<true/>', '<false/>', or numeric.  String value of '" << (char*)cont << "' is not recommended." << std::endl;
			} catch(const bad_format& err) {
				xmlFree(cont);
				throw bad_format(node,err.what());
			} catch(...) {
				xmlFree(cont);
				throw;
			}
			xmlFree(cont);
		} else
			throw bad_format(node,"Error: plist boolean must be 'true', 'false', or numeric type");
	}
	void Primitive<bool>::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		xmlNodeSetName(node,(const xmlChar*)(val?"true":"false"));
		xmlNodeSetContent(node,NULL);
	}
	void Primitive<bool>::set(const std::string& str) {
		prevVal=val;
		if(matchTrue(str))
			val=true;
		else if(matchFalse(str))
			val=false;
		else {
			float t;
			if(sscanf(str.c_str(),"%g",&t)==0)
				throw bad_format(NULL,"Error: plist boolean must be 'true', 'false', or numeric type");
			val=t;
		}
		fireValueChanged((prevVal && val) || (!prevVal && !val));
	}
	//! implements the clone function for Primitive<bool>
	PLIST_CLONE_IMP(Primitive<bool>,new Primitive<bool>(val));

	
	void Primitive<char>::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		xmlChar* cont=xmlNodeGetContent(node);
		try {
			if(xNodeHasName(node,"string")) {
				set((char*)cont);
			} else if(xNodeHasName(node,"integer")) {
				prevVal=val;
				val=strtol((const char*)cont,NULL,0);
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"real")) {
				prevVal=val;
				val=(char)strtod((const char*)cont,NULL);
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"true")) {
				prevVal=val;
				val=true;
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"false")) {
				prevVal=val;
				val=false;
				fireValueChanged(prevVal==val); 
			} else {
				throw bad_format(node,"Error: plist char must be either a string or integer");
			} 
		} catch(const bad_format& err) {
			xmlFree(cont);
			throw bad_format(node,err.what());
		} catch(...) {
			xmlFree(cont);
			throw;
		}
		xmlFree(cont);
	}
	void Primitive<char>::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		if(numeric) {
			xmlNodeSetName(node,(const xmlChar*)"integer");
			std::stringstream str;
			str << (int)val;
			xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
		} else {
			xmlNodeSetName(node,(const xmlChar*)"string");
			char str[2] = {val,'\0'};
			xmlNodeSetContent(node,(const xmlChar*)str);
		}
	}
	void Primitive<char>::set(const std::string& str) {
		prevVal=val;
		if(str.size()==0)
			throw bad_format(NULL,"Error: plist char must have non-empty content");
		val=str[0];
		if(str.size()>1) {
			std::cerr << "Warning: plist expected single char, found multi-char string '" << str << "'";
			if(matchTrue(str))
				val=true;
			else if(matchFalse(str))
				val=false;
			else {
				std::cerr << " (using first character '" << val << "')" << std::endl;
				return;
			}
			std::cerr << " (interpreted as boolean " << (bool)val << ")" << std::endl;
		}
		fireValueChanged(prevVal==val);
	}
	//! implements the clone function for Primitive<char>
	PLIST_CLONE_IMP(Primitive<char>,new Primitive<char>(val));
	
	
	void Primitive<unsigned char>::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		xmlChar* cont=xmlNodeGetContent(node);
		try {
			if(xNodeHasName(node,"string")) {
				set((char*)cont);
			} else if(xNodeHasName(node,"integer")) {
				prevVal=val;
				val=strtol((const char*)cont,NULL,0);
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"real")) {
				prevVal=val;
				val=(char)strtod((const char*)cont,NULL);
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"true")) {
				prevVal=val;
				val=true;
				fireValueChanged(prevVal==val); 
			} else if(xNodeHasName(node,"false")) {
				prevVal=val;
				val=false;
				fireValueChanged(prevVal==val); 
			} else {
				throw bad_format(node,"Error: plist unsigned char must be either a string or integer");
			} 
		} catch(const bad_format& err) {
			xmlFree(cont);
			throw bad_format(node,err.what());
		} catch(...) {
			xmlFree(cont);
			throw;
		}
		xmlFree(cont);
	}
	void Primitive<unsigned char>::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		if(numeric) {
			xmlNodeSetName(node,(const xmlChar*)"integer");
			std::stringstream str;
			str << (int)val;
			xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
		} else {
			xmlNodeSetName(node,(const xmlChar*)"string");
			xmlChar str[2] = {val,'\0'};
			xmlNodeSetContent(node,(const xmlChar*)str);
		}
	}
	void Primitive<unsigned char>::set(const std::string& str) {
		prevVal=val;
		if(str.size()==0)
			throw bad_format(NULL,"Error: plist char must have non-empty content");
		val=str[0];
		if(str.size()>1) {
			std::cerr << "Warning: plist expected single char, found multi-char string '" << str << "'";
			if(matchTrue(str))
				val=true;
			else if(matchFalse(str))
				val=false;
			else {
				std::cerr << " (using first character '" << val << "')" << std::endl;
				return;
			}
			std::cerr << " (interpreted as boolean " << (bool)val << ")" << std::endl;
		}
		fireValueChanged(prevVal==val); 
	}
	//! implements the clone function for Primitive<unsigned char>
	PLIST_CLONE_IMP(Primitive<unsigned char>,new Primitive<unsigned char>(val));
	
	
	void Primitive<std::string>::loadXML(xmlNode* node) {
		// operator= will call fireValueChanged, so no direct calls to fire here...
		if(node==NULL)
			return;
		if(xNodeHasName(node,"string")) {
			xmlChar * cont=xmlNodeGetContent(node);
			*this=(char*)cont;
			xmlFree(cont);
		} else {
			if(xNodeHasName(node,"integer") || xNodeHasName(node,"real")) {
				xmlChar * cont=xmlNodeGetContent(node);
				*this=(char*)cont;
				xmlFree(cont);
			} else if(xNodeHasName(node,"true"))
				*this="true";
			else if(xNodeHasName(node,"false"))
				*this="false";
			else
				throw bad_format(node,"Error: plist string must be 'true', 'false', or numeric type");
			std::cerr << "Warning: plist string expected, found " << (const char*)xNodeGetName(node) << " on line " << xmlGetLineNo(node) << std::endl;
		}
	}
	void Primitive<std::string>::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		xmlNodeSetName(node,(const xmlChar*)"string");
		xmlNodeSetContent(node,(const xmlChar*)c_str());
	}
	bool Primitive<std::string>::toBool() const { if(matchTrue(*this)) return true; else if(matchFalse(*this)) return false; else return toLong(); }
	char Primitive<std::string>::toChar() const { return std::isdigit((*this)[0]) ? toLong() : (*this)[0]; }
	long Primitive<std::string>::toLong() const { std::stringstream s(*this); long v; s >> v; return v; }
	double Primitive<std::string>::toDouble() const { std::stringstream s(*this); double v; s >> v; return v; }
	//! implements the clone function for Primitive<std::string>
	PLIST_CLONE_IMP(Primitive<std::string>,new Primitive<std::string>(get()));
	

	std::string NamedEnumerationBase::getDescription(bool preferredOnly/*=true*/) {
		if(preferredOnly) {
			std::map<int,std::string> valsToNames;
			getPreferredNames(valsToNames);
			if(valsToNames.size()==0) return "";
			std::string ans="Value is one of: { ";
			std::map<int,std::string>::const_iterator it=valsToNames.begin();
			ans+=it->second;
			for(++it; it!=valsToNames.end(); ++it) {
				ans+=" | ";
				ans+=it->second;
			}
			if(!strictValue)
				ans+=" | <integer_value>";
			ans+=" } ";
			return ans;
		} else {
			std::map<std::string,int> namesToVals;
			getAllNames(namesToVals);
			if(namesToVals.size()==0) return "";
			std::string ans="Value is one of: { ";
			std::map<std::string,int>::const_iterator it=namesToVals.begin();
			ans+=it->first;
			for(++it; it!=namesToVals.end(); ++it) {
				ans+=" | ";
				ans+=it->first;
			}
			if(!strictValue)
				ans+=" | <integer_value>";
			ans+=" } ";
			return ans;
		}
	}
} //namespace plist

/*! @file
* @brief 
* @author Ethan Tira-Thompson (ejt) (Creator)
*/

