#include "plistSpecialty.h"
#include "string_util.h"
#include <libxml/tree.h>
#include <cmath>
#include <cstdlib>

#ifdef PLATFORM_APERIOS
// has strtod, but not strtof :(
#  define strtof strtod
#endif

//better to put this here instead of the header
using namespace std;

namespace plist {
	
	void OutputSelector::bad_value::initMessage() {
		if(strValue.size()==0 && intValue==UNUSED) {
			message="plist specialty type OutputSelector was passed UNUSED, configured as invalid for this value";
		} else if(rangeError) {
			if(strValue.size()>0 && intValue!=UNUSED) {
				std::stringstream ss;
				ss << "plist specialty type OutputSelector was passed an out-of-range offset named " << strValue << " (" << intValue <<")";
				message=ss.str();
			} else if(strValue.size()>0) {
				message="plist specialty type OutputSelector was passed an out-of-range offset named '"+strValue+"'";
			} else {
				std::stringstream ss;
				ss << "plist specialty type OutputSelector was passed an out-of-range offset '" << intValue <<"'";
				message=ss.str();
			}
		} else {
			if(strValue.size()>0) {
				message="plist specialty type OutputSelector was passed '"+strValue+"', invalid for this robot model";
			} else {
				std::stringstream ss;
				ss << "plist specialty type OutputSelector was passed '" << intValue <<"', invalid for this robot model";
				message=ss.str();
			}
		}
	}
	
	OutputSelector& OutputSelector::operator=(const unsigned int& v) {
		if(&prevVal==&val) std::swap(val,prevVal); else { prevVal=val; val=v; }
		if(val!=UNUSED && (val<rangeBegin || val>=rangeEnd || val>=capabilities.getNumFrames()) ) {
			val=UNUSED;
			if(throwInvalid) {
				fireValueChanged(prevVal==val);
				if(capabilities.getFrameName(v)!=NULL)
					throw bad_value(capabilities.getFrameName(v),v);
				else
					throw bad_value(v,true);
			}
			cerr << "Error: plist specialty type OutputSelector assigned an out-of-range ("<<rangeBegin<<','<<std::min(rangeEnd,capabilities.getNumFrames())<<"] offset "<<v<<" for model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
		}
		if(throwUnused && val==UNUSED) {
			fireValueChanged(prevVal==val);
			throw bad_value(UNUSED);
		} else {
			fireValueChanged(prevVal==val); 
		}
		return *this;
	}
	
	void OutputSelector::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		xmlChar* cont=xmlNodeGetContent(node);
		try {
			if(xNodeHasName(node,"string")) {
				useNumeric=false;
				saveModel="";
				set((char*)cont);
			} else if(xNodeHasName(node,"integer") || xNodeHasName(node,"real")) {
				if(xNodeHasName(node,"real"))
					std::cerr << "Warning: plist specialty type OutputSelector should be either a string or integer, interpreting 'real' as 'integer'" << std::endl;
				useNumeric=true;
				saveModel="";
				prevVal=val;
				val=UNUSED;
				string strval = string_util::trim((const char*)cont);
				if(strval.size()>0) {
					char * endp=NULL;
					val = strtol(strval.c_str(),&endp,0);
					if(*endp!='\0') { // nope, found non-digit characters or unknown name
						throw bad_format(node,"Error: plist specialty type OutputSelector encountered malformed integer");
					} else if(defModel.size()!=0 && defModel!=capabilities.getRobotName()) {
						// need to map from this other model
						const Capabilities * cap = getCapabilities(defModel);
						if(cap==NULL) {
							unsigned int badval=val;
							val=UNUSED;
							if(throwInvalid) {
								fireValueChanged(prevVal==val);
								std::stringstream ss;
								ss << defModel << '/' << badval;
								throw bad_value(ss.str());
							}
							cerr << "Error: plist specialty type OutputSelector has an invalid default model specifier "<<defModel<<", marking 'UNUSED'" << endl;
						} else {
							const char * name = cap->getFrameName(val);
							if(name==NULL) { // unknown name for specified value
								unsigned int badval=val;
								val=UNUSED;
								if(throwInvalid) {
									fireValueChanged(prevVal==val);
									throw bad_value(badval);
								} else {
									cerr << "Error: plist specialty type OutputSelector attempted to set an invalid offset "<<badval<<" for model " << cap->getRobotName() << ", marking 'UNUSED'" << endl;
								}
							}
							// val currently in the other model's ordering, convert to current model
							if(val!=UNUSED) {
								val=capabilities.findFrameOffset(name);
								if(val==-1U) {
									unsigned int badval=val;
									val=UNUSED;
									if(throwInvalid) {
										fireValueChanged(prevVal==val);
										throw bad_value(name);
									} else {
										cerr << "Error: plist specialty type OutputSelector could not map "<<name<<" ("<<defModel<<'/'<<badval<<") to model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
									}
								}
							}
						}
					}
					if(val!=UNUSED && (val<rangeBegin || val>=rangeEnd || val>=capabilities.getNumFrames()) ) {
						unsigned int badval=val;
						val=UNUSED;
						if(throwInvalid) {
							fireValueChanged(prevVal==val);
							throw bad_value(strval,badval);
						}
						if(defModel.size()!=0 && defModel!=capabilities.getRobotName())
							cerr << "Error: plist specialty type OutputSelector loading an out-of-range ("<<rangeBegin<<','<<std::min(rangeEnd,capabilities.getNumFrames())<<"] offset "<<defModel<<'/'<<strval<<" ("<<badval<<") for model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
						else
							cerr << "Error: plist specialty type OutputSelector loading an out-of-range ("<<rangeBegin<<','<<std::min(rangeEnd,capabilities.getNumFrames())<<"] offset "<<strval<<" for model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
					}
				}
				if(throwUnused && val==UNUSED) {
					fireValueChanged(prevVal==val);
					throw bad_value(UNUSED);
				} else {
					fireValueChanged(prevVal==val); 
				}
			} else {
				throw bad_format(node,"Error: plist specialty type OutputSelector must be either a string or integer");
			} 
		} catch(const bad_format& err) {
			xmlFree(cont);
			throw err;
		} catch(...) {
			xmlFree(cont);
			throw;
		}
		xmlFree(cont);
	}
	
	void OutputSelector::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		if(val!=UNUSED && useNumeric && saveModel.size()==0) {
			xmlNodeSetName(node,(const xmlChar*)"integer");
			std::stringstream str;
			str << (int)val;
			xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
		} else {
			xmlNodeSetName(node,(const xmlChar*)"string");
			std::string str = get();
			if(str=="UNUSED")
				str.clear();
			xmlNodeSetContent(node,(const xmlChar*)str.c_str());
		}
	}
	
	void OutputSelector::set(const std::string& str) {
		prevVal=val;
		val=UNUSED;
		string strval = string_util::trim(str);
		string model = defModel;
		string::size_type modelpos = str.find('/');
		if(modelpos!=string::npos) {
			model = saveModel = str.substr(0,modelpos);
			strval = str.substr(modelpos+1);
		}
		if(strval.size()==0 || strval=="UNUSED") {
			if(throwUnused) {
				fireValueChanged(prevVal==val);
				throw bad_value(UNUSED);
			}
		} else {
			if(model.size()==0 || model==capabilities.getRobotName()) {
				// try interpreting as a number first...
				char * endp=NULL;
				val = strtol(strval.c_str(),&endp,0);
				if(*endp!='\0') { // nope, found non-digit characters, try interpreting as a name
					val = capabilities.findFrameOffset(strval.c_str());
					if(val==-1U) { // nope, invalid
						if(throwInvalid) {
							val=UNUSED;
							fireValueChanged(prevVal==val);
							throw bad_value(str);
						} else {
							val=UNUSED;
							cerr << "Error: plist specialty type OutputSelector attempted to set an invalid offset "<<strval<<" for model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
						}
					}
				}
			} else {
				const Capabilities * cap = getCapabilities(model);
				if(cap==NULL) {
					if(throwInvalid) {
						fireValueChanged(prevVal==val);
						throw bad_value(str);
					}
					cerr << "Error: plist specialty type OutputSelector attempted to set a bad model specifier "<<model<<", marking 'UNUSED'" << endl;
				} else {
					// try interpreting as a number first...
					char * endp=NULL;
					val = strtol(strval.c_str(),&endp,0);
					if(*endp!='\0') { // nope, found non-digit characters, try a name
						val = cap->findFrameOffset(strval.c_str());
						if(val!=-1U) {
							// recognized as a name, map it to current model
							val=capabilities.findFrameOffset(strval.c_str());
							if(val==-1U) {
								if(throwInvalid) {
									val=UNUSED;
									fireValueChanged(prevVal==val);
									throw bad_value(str);
								} else {
									val=UNUSED;
									cerr << "Error: plist specialty type OutputSelector could not map "<<str<<" to model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
								}
							}
						} else {
							// not recognized as a name on the other model, try the name on the current model (might've been a bad export, but we can recover)
							val=capabilities.findFrameOffset(strval.c_str());
							if(val!=-1U) {
								cerr << "Warning: plist specialty type OutputSelector couldn't find "<<str<<" on model " << cap->getRobotName() << ", but it was found on current host model " << capabilities.getRobotName() << ", using that.." << endl;
							} else {
								if(throwInvalid) {
									val=UNUSED;
									fireValueChanged(prevVal==val);
									throw bad_value(str);
								} else {
									val=UNUSED;
									cerr << "Error: plist specialty type OutputSelector attempted to set an invalid offset "<<str<<" for model " << cap->getRobotName() << ", marking 'UNUSED'" << endl;
								}
							}
						}
					} else if(val!=UNUSED) { // got a value, what's it's name, and does it map to current model?
						const char * name = cap->getFrameName(val);
						if(name==NULL) { // unknown name for specified value
							val=UNUSED;
							if(throwInvalid) {
								fireValueChanged(prevVal==val);
								throw bad_value(str);
							} else {
								cerr << "Error: plist specialty type OutputSelector attempted to set an invalid offset "<<str<<" for model " << cap->getRobotName() << ", marking 'UNUSED'" << endl;
							}
						}
						// val currently in the other model's ordering, convert to current model
						if(val!=UNUSED) {
							val=capabilities.findFrameOffset(name);
							if(val==-1U) {
								unsigned int badval=val;
								val=UNUSED;
								if(throwInvalid) {
									fireValueChanged(prevVal==val);
									throw bad_value(name);
								} else {
									cerr << "Error: plist specialty type OutputSelector could not map "<<name<<" ("<<model<<'/'<<badval<<") to model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
								}
							}
						}
					}
				}
			}
		}
		if(val!=UNUSED && (val<rangeBegin || val>=rangeEnd || val>=capabilities.getNumFrames())) {
			unsigned int badval=val;
			val=UNUSED;
			if(throwInvalid) {
				fireValueChanged(prevVal==val);
				throw bad_value(strval,badval);
			}
			cerr << "Error: plist specialty type OutputSelector loading an out-of-range ("<<rangeBegin<<','<<std::min(rangeEnd,capabilities.getNumFrames())<<"] offset "<<str<<" ("<<badval<<") for model " << capabilities.getRobotName() << ", marking 'UNUSED'" << endl;
		}
		if(throwUnused && val==UNUSED) {
			fireValueChanged(prevVal==val);
			throw bad_value(UNUSED);
		} else {
			fireValueChanged(prevVal==val);
		}
	}
	
	std::string OutputSelector::get() const {
		if(val==UNUSED)
			return "UNUSED";
		if(useNumeric) {
			unsigned int mappedValue=val;
			if(saveModel.size()==0) {
				std::stringstream sstr;
				sstr << (int)val;
				return sstr.str();
			} else {
				const Capabilities * cap = getCapabilities(saveModel);
				if(cap==NULL)
					throw bad_format(NULL,"Error: plist specialty type OutputSelector could not save due to bad saveModel "+saveModel);
				std::stringstream sstr;
				const char * name = capabilities.getFrameName(val);
				if(name==NULL) {
					// output isn't valid on the current model, can't map to others!
					cerr << "Warning: plist specialty type OutputSelector does not have an for offset " << (int)val << " can't map to model " << cap->getRobotName() << endl;
					sstr << capabilities.getRobotName()<<'/'<<(int)val;
					return sstr.str();
				}
				mappedValue = (int)cap->findFrameOffset(name);
				if(mappedValue==-1U) {
					// output can't be mapped to the saveModel, so we'll override the numeric and save the name
					cerr << "Warning: plist specialty type OutputSelector mapping to model " << saveModel << ", does not have " << name << endl;
					sstr << saveModel << '/' << name;
					return sstr.str();
				}
				sstr << saveModel << '/' << mappedValue;
				return sstr.str();
			}
		} else {
			const char * name = capabilities.getFrameName(val);
			if(name!=NULL) {
				return name;
			} else {
				std::stringstream sstr;
				if(saveModel.size()>0)
					sstr << saveModel << '/';
				else
					sstr << "INVALID" << '/';
				sstr << (int)val;
				return sstr.str();
			}
		}
	}
	
	void OutputSelector::setRange(unsigned int begin, unsigned int end) {
		rangeBegin=begin;
		rangeEnd=end;
		if(val!=UNUSED && (val<rangeBegin || val>=rangeEnd || val>=capabilities.getNumFrames()) ) {
			prevVal=val;
			val=UNUSED;
			fireValueChanged(prevVal==val);
			if(throwInvalid) {
				if(capabilities.getFrameName(val)!=NULL)
					throw bad_value(capabilities.getFrameName(val),val);
				else
					throw bad_value(val,true);
			}
			cerr << "Error: plist specialty type OutputSelector value " << prevVal << " was invalidated by change to range (" << rangeBegin <<','<< std::min(rangeEnd,capabilities.getNumFrames()) << ']' << endl;
		}
	}
	
	//! implements the clone function for Primitive<char>
	PLIST_CLONE_IMP(OutputSelector,new OutputSelector);
	

	

	

	Angle::Format Angle::defaultFormat=Angle::FORMAT_DEGREE;
	Angle::Pedantic Angle::pedantic=Angle::PEDANTIC_FULL;
	
	void Angle::loadXML(xmlNode* node) {
		if(node==NULL)
			return;
		xmlChar* cont=xmlNodeGetContent(node);
		xmlChar * att=NULL;
		try {
			if(xNodeHasName(node,"string") || xNodeHasName(node,"integer") || xNodeHasName(node,"real")) {
				att = xmlGetProp(node,(const xmlChar*)"unit");
				if(att==NULL) {
					set((char*)cont);
				} else {
					prevVal=val;
					if(xmlStrcasecmp(att,(const xmlChar*)"rad")==0 || xmlStrcasecmp(att,(const xmlChar*)"radian")==0 || xmlStrcasecmp(att,(const xmlChar*)"radians")==0) {
						char * endp=NULL;
						val=strtof((char*)cont,&endp);
						if(*endp!='\0') {
							val=prevVal;
							throw bad_format(node,"plist specialty type Angle expects a numeric value");
						}
						loadedFormat=FORMAT_RADIAN;
					} else if(xmlStrcasecmp(att,(const xmlChar*)"°")==0 || xmlStrcasecmp(att,(const xmlChar*)"deg")==0 || xmlStrcasecmp(att,(const xmlChar*)"degree")==0 || xmlStrcasecmp(att,(const xmlChar*)"degrees")==0) {
						char * endp=NULL;
						val=strtof((char*)cont,&endp);
						if(*endp!='\0') {
							val=prevVal;
							throw bad_format(node,"plist specialty type Angle expects a numeric value");
						}
						val=val*(decltype(val))M_PI/180;
						loadedFormat=FORMAT_DEGREE;
					} else if(xmlStrcasecmp(att,(const xmlChar*)"π")==0 || xmlStrcasecmp(att,(const xmlChar*)"pi")==0) {
						char * endp=NULL;
						val=strtof((char*)cont,&endp);
						if(*endp!='\0') {
							val=prevVal;
							throw bad_format(node,"plist specialty type Angle expects a numeric value");
						}
						val*=(decltype(val))M_PI;
						loadedFormat=FORMAT_PI;
					} else if(xmlStrcasecmp(att,(const xmlChar*)"%")==0 || xmlStrcasecmp(att,(const xmlChar*)"percent")==0) {
						char * endp=NULL;
						val=strtof((char*)cont,&endp);
						if(*endp!='\0') {
							val=prevVal;
							throw bad_format(node,"plist specialty type Angle expects a numeric value");
						}
						val=val*2*(decltype(val))M_PI/100;
						loadedFormat=FORMAT_PERCENT;
					} else {
						throw bad_format(node,"Error: plist specialty type Angle encountered unknown unit specification");
					}
					fireValueChanged(prevVal==val);
				}
			} else {
				throw bad_format(node,"Error: plist specialty type Angle must be either a string, real, or integer");
			} 
		} catch(const bad_format& err) {
			xmlFree(att);
			xmlFree(cont);
			throw (err.getNode()!=NULL) ? err : bad_format(node,err.what());
		} catch(...) {
			xmlFree(att);
			xmlFree(cont);
			throw;
		}
		xmlFree(att);
		xmlFree(cont);
	}
	
	void Angle::saveXML(xmlNode* node) const {
		if(node==NULL)
			return;
		Format fmt = saveFormat;
		if(fmt==FORMAT_SAME)
			fmt = (loadedFormat==FORMAT_AUTO) ? defaultFormat : loadedFormat;
		if(fmt==FORMAT_AUTO || fmt==FORMAT_SAME) {
			decltype(val) deg = val/(decltype(val))M_PI*180;
			if(std::abs(std::floor(deg)-deg)<1e-5)
				fmt=FORMAT_DEGREE;
		}
		switch(pedantic) {
			case PEDANTIC_FULL: { // use 'string' element for non-radian values
				xmlUnsetProp(node,(const xmlChar*)"unit");
				if(fmt==FORMAT_RADIAN || fmt==FORMAT_NONE) {
					xmlNodeSetName(node,(const xmlChar*)"real");
					std::stringstream str;
					str << val;
					xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
				} else {
					xmlNodeSetName(node,(const xmlChar*)"string");
					xmlNodeSetContent(node,(const xmlChar*)get().c_str());
				}
			} break;
			
			case PEDANTIC_VALID: { // use 'unit' attribute on 'real' element to specify units
				xmlNodeSetName(node,(const xmlChar*)"real");
				std::stringstream str;
				switch(fmt) {
					case FORMAT_NONE: {
						xmlUnsetProp(node,(const xmlChar*)"unit");
						str << val;
					} break;
					case FORMAT_RADIAN: {
						xmlSetProp(node,(const xmlChar*)"unit",(const xmlChar*)"rad");
						str << val;
					} break;
					case FORMAT_DEGREE: {
						xmlSetProp(node,(const xmlChar*)"unit",(const xmlChar*)"deg");
						str << (val/M_PI*180);
					} break;
					case FORMAT_PI: {
						xmlSetProp(node,(const xmlChar*)"unit",(const xmlChar*)"pi");
						str << (val/M_PI);
					} break;
					case FORMAT_PERCENT: {
						xmlSetProp(node,(const xmlChar*)"unit",(const xmlChar*)"%");
						str << (val/(2*M_PI)*100);
					} break;
					default: {
						throw std::logic_error("unhandled format");
					} break;
				}
				xmlNodeSetContent(node,(const xmlChar*)str.str().c_str());
			} break;
				
			case PEDANTIC_CUTE: { // use 'real' element for everything
				xmlUnsetProp(node,(const xmlChar*)"unit");
				xmlNodeSetName(node,(const xmlChar*)"real");
				xmlNodeSetContent(node,(const xmlChar*)get().c_str());
			} break;
				
		}
	}
	
	void Angle::set(const std::string& str) {
		prevVal=val;
		string strval = string_util::trim(str);
		size_t p;
		if((p=strval.find("%"))!=string::npos) {
			string strnum = strval.substr(0,p);
			char * endp=NULL;
			val=strtof(strnum.c_str(),&endp);
			if(endp==strnum.c_str()) {
				val=prevVal;
				throw bad_format(NULL,"plist specialty type Angle expects a numeric value");
			}
			if(*endp!='\0')
				std::cerr << "WARNING: plist specialty type Angle ignoring extra characters preceeding '%': " << endp << std::endl;
			if(p!=strval.size()-strlen("%"))
				std::cerr << "WARNING: plist specialty type Angle ignoring extra characters following '%': " << strval.substr(p+strlen("%")) << std::endl;
			val=val*2*(decltype(val))M_PI/100;
			loadedFormat=FORMAT_PERCENT;
			
		} else if((p=strval.find("π"))!=string::npos) {
			bool neg=false;
			if(strval[0]=='-') {
				neg=true;
				strval=strval.substr(1);
				p=strval.find("π");
			}
			float num,den=1;
			if(p==0)
				num=1;
			else {
				string strnum = strval.substr(0,p);
				char * endp=NULL;
				num=strtof(strnum.c_str(),&endp);
				if(endp==strnum.c_str())
					throw bad_format(NULL,"plist specialty type Angle expects a numeric value in π numerator");
				if(*endp!='\0')
					std::cerr << "WARNING: plist specialty type Angle ignoring extra characters preceeding 'π': " << endp << std::endl;
			}
			strval=string_util::trim(strval.substr(p+strlen("π")));
			if(strval[0]=='/') {
				strval=strval.substr(1);
				if(strval.size()==0) {
					std::cerr << "WARNING: plist specialty type Angle ignoring trailing '/' following 'π'" << std::endl;
				} else {
					char * endp=NULL;
					den=strtof(strval.c_str(),&endp);
					if(endp==strval.c_str())
						throw bad_format(NULL,"plist specialty type Angle expects a numeric value in π denominator");
					if(*endp!='\0')
						std::cerr << "WARNING: plist specialty type Angle ignoring extra characters following denominator: " << endp << std::endl;
				}
			} else if(strval.size()!=0) {
				std::cerr << "WARNING: plist specialty type Angle ignoring extra characters following 'π': " << strval.substr(p+strlen("π")) << std::endl;
			}
			val = neg ? -num*(decltype(val))M_PI/den : num*(decltype(val))M_PI/den;
			loadedFormat=FORMAT_PI;
			
		} else if((p=strval.find("°"))!=string::npos) {
			string strnum = strval.substr(0,p);
			char * endp=NULL;
			val=strtof(strnum.c_str(),&endp);
			if(endp==strnum.c_str()) {
				val=prevVal;
				throw bad_format(NULL,"plist specialty type Angle expects a numeric value");
			}
			if(*endp!='\0')
				std::cerr << "WARNING: plist specialty type Angle ignoring extra characters preceeding '°': " << endp << std::endl;
			if(p!=strval.size()-strlen("°"))
				std::cerr << "WARNING: plist specialty type Angle ignoring extra characters following '°': " << strval.substr(p+strlen("%")) << std::endl;
			val=val*(decltype(val))M_PI/180;
			loadedFormat=FORMAT_DEGREE;
			
		} else {
			if(strval=="∞" && std::numeric_limits<PLISTREAL>::has_infinity) {
				val=std::numeric_limits<PLISTREAL>::infinity();
			} else {
				char * endp=NULL;
				val=strtof(strval.c_str(),&endp);
				if(endp==strval.c_str()) {
					val=prevVal;
					throw bad_format(NULL,"plist specialty type Angle expects a numeric value");
				}
				if(*endp!='\0')
					std::cerr << "WARNING: plist specialty type Angle ignoring extra characters following value: " << endp << std::endl;
			}
			loadedFormat=FORMAT_RADIAN;
			
		}
		fireValueChanged(prevVal==val);
	}
	
	std::string Angle::get() const {
		Format fmt = saveFormat;
		if(fmt==FORMAT_SAME)
			fmt = (loadedFormat==FORMAT_AUTO) ? defaultFormat : loadedFormat;
		if(fmt==FORMAT_AUTO || fmt==FORMAT_SAME) {
			decltype(val) deg = val*180/(decltype(val))M_PI;
			if(std::abs(std::floor(deg)-deg)<1e-5)
				fmt=FORMAT_DEGREE;
		}
		std::stringstream str;
		switch(fmt) {
			case FORMAT_NONE:
			case FORMAT_RADIAN: {
				str << val;
			} break;
			case FORMAT_DEGREE: {
				str << (val/M_PI*180) << "°";
			} break;
			case FORMAT_PI: {
				float x = val/(decltype(val))M_PI;
				// is it an even multiple of pi?
				if(std::abs(rint(x)-x)<1e-5) {
					if(x==1) 
						return "π";
					else if(x==-1)
						return "-π";
					else
						str << x << "π";
				} else for(int i=2; i<16; ++i) {
					// nope, then test for good fractions
					x = val/(decltype(val))M_PI*i;
					if(std::abs(rint(x)-x)<1e-5 && std::abs(x)>.9) {
						if(std::abs(rint(x)-1)<1e-5) {
							str << "π/" << i;
						} else if(std::abs(rint(x)+1)<1e-5) {
							str << "-π/" << i;
						} else {
							str << x << "π/" << i;
						}
						break;
					}
				}
				if(str.str().size()==0) {
					// no good fraction, just dump it
					str << val/M_PI << "π";
				}
			} break;
			case FORMAT_PERCENT: {
				str << (val/(2*M_PI)*100) << "%";
			} break;
			default: {
				throw std::logic_error("unhandled format");
			} break;
		}
		return str.str();
	}
	
	//! implements the clone function for Primitive<char>
	PLIST_CLONE_IMP(Angle,new Angle);
	
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
