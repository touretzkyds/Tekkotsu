#include "XMLLoadSave.h"
#include <iostream>
#include <string>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h>
#include <libxml/xmlsave.h>
#include <libxml/xmlversion.h>
#include <errno.h>

#if LIBXML_VERSION < 20617
/**
 * xmlSaveOption:
 *
 * This is the set of XML save options that can be passed down
 * to the xmlSaveToFd() and similar calls.
 */
typedef enum {
	XML_SAVE_FORMAT     = 1<<0  /* format save output */
} xmlSaveOption;
#endif

using namespace std;

unsigned int XMLLoadSave::AutoInit::libxmlrefc=0;

XMLLoadSave::AutoInit::AutoInit() {
	if(libxmlrefc==0) {
		//cout << "libxmlinit" << endl;
		xmlInitParser();
		xmlSubstituteEntitiesDefault(1);
		xmlKeepBlanksDefault(1);
		xmlLineNumbersDefault(1);
		xmlIndentTreeOutput = 1;
	}
	libxmlrefc++;
}

XMLLoadSave::AutoInit::~AutoInit() {
	libxmlrefc--;
	if(libxmlrefc==0) {
		xmlCleanupParser();
		//cout << "libxmldest" << endl;
	}
}


#if LIBXML_VERSION >= 20623
/**
 * xmlEscapeEntities: taken from libxml2/xmlsave.c, release 2.6.31
 * this version of the function is modified to avoid escaping entities which don't require it
 * (e.g. unicode characters)  This provides human-readable foreign language and symbols...
 *
 * @out:  a pointer to an array of bytes to store the result
 * @outlen:  the length of @out
 * @in:  a pointer to an array of unescaped UTF-8 bytes
 * @inlen:  the length of @in
 *
 * Take a block of UTF-8 chars in and escape them. Used when there is no
 * encoding specified.
 *
 * Returns 0 if success, or -1 otherwise
 * The value of @inlen after return is the number of octets consumed
 *     if the return value is positive, else unpredictable.
 * The value of @outlen after return is the number of octets consumed.
 */
static int xmlEscapeMinimalEntities(unsigned char* out, int *outlen, const xmlChar* in, int *inlen) {
	unsigned char* outstart = out;
	const unsigned char* base = in;
	unsigned char* outend = out + *outlen;
	const unsigned char* inend;
	int val;

	inend = in + (*inlen);
	
	while ((in < inend) && (out < outend)) {
		if (*in == '<') {
			if (outend - out < 4) break;
			*out++ = '&';
			*out++ = 'l';
			*out++ = 't';
			*out++ = ';';
			in++;
			continue;
		} else if (*in == '>') {
			if (outend - out < 4) break;
			*out++ = '&';
			*out++ = 'g';
			*out++ = 't';
			*out++ = ';';
			in++;
			continue;
		} else if (*in == '&') {
			if (outend - out < 5) break;
			*out++ = '&';
			*out++ = 'a';
			*out++ = 'm';
			*out++ = 'p';
			*out++ = ';';
			in++;
			continue;
		} else if (((*in >= 0x20) && (*in < 0x80)) ||
				   (*in == '\n') || (*in == '\t')) {
			/*
			 * default case, just copy !
			 */
			*out++ = *in++;
			continue;
		} else if (*in >= 0x80) {
			/*
			 * We assume we have UTF-8 input.
			 */
			if (outend - out < 10) break;

			if (*in < 0xC0) {
				std::cerr << "XMLLoadSave::xmlEscapeMinimalEntities encountered non-UTF8 data: " << *in << std::endl;
				in++;
				goto error;
			} else if (*in < 0xE0) {
				if (inend - in < 2) break;
				val = (in[0]) & 0x1F;
				val <<= 6;
				val |= (in[1]) & 0x3F;
				*out++ = *in++;
				*out++ = *in++;
			} else if (*in < 0xF0) {
				if (inend - in < 3) break;
				val = (in[0]) & 0x0F;
				val <<= 6;
				val |= (in[1]) & 0x3F;
				val <<= 6;
				val |= (in[2]) & 0x3F;
				*out++ = *in++;
				*out++ = *in++;
				*out++ = *in++;
			} else if (*in < 0xF8) {
				if (inend - in < 4) break;
				val = (in[0]) & 0x07;
				val <<= 6;
				val |= (in[1]) & 0x3F;
				val <<= 6;
				val |= (in[2]) & 0x3F;
				val <<= 6;
				val |= (in[3]) & 0x3F;
				*out++ = *in++;
				*out++ = *in++;
				*out++ = *in++;
				*out++ = *in++;
			} else {
				std::cerr << "XMLLoadSave::xmlEscapeMinimalEntities encountered invalid UTF8 data " << *in << std::endl;
				in++;
				goto error;
			}
			if (!IS_CHAR(val)) {
				std::cerr << "XMLLoadSave::xmlEscapeMinimalEntities encountered unknown UTF8 data " << *in << std::endl;
				goto error;
			}

		} else if (IS_BYTE_CHAR(*in)) {
			if (outend - out < 6) break;
			*out++ = *in++;
		} else {
			xmlGenericError(xmlGenericErrorContext,
				"xmlEscapeEntities : char out of range\n");
			in++;
			goto error;
		}
	}
	*outlen = out - outstart;
	*inlen = in - base;
	return(0);
error:
	*outlen = out - outstart;
	*inlen = in - base;
	return(-1);
}
#endif

XMLLoadSave::XMLLoadSave()
  : xmldocument(NULL), compressionLevel(-1), autoFormat(true), libxmlInit()
{}

XMLLoadSave::XMLLoadSave(const XMLLoadSave& xls)
  : LoadSave(xls), xmldocument(xls.xmldocument==NULL?NULL:xmlCopyDoc(xls.xmldocument,true)), compressionLevel(xls.compressionLevel), autoFormat(xls.autoFormat), libxmlInit()
{}

XMLLoadSave& XMLLoadSave::operator=(const XMLLoadSave& xls) {
	LoadSave::operator=(xls);
	if(xls.xmldocument==NULL)
		clearParseTree();
	else
		setParseTree(xmlCopyDoc(xls.xmldocument,true));
	compressionLevel = xls.compressionLevel;
	autoFormat=xls.autoFormat;
	return *this;
}

XMLLoadSave::~XMLLoadSave() {
	clearParseTree();
}

void XMLLoadSave::reportError(const std::string& context, const bad_format& err) const {
	cerr << context << endl;
	cerr << "  " << err.what() << endl;
	if(err.getNode()!=NULL) {
		xmlChar* path=xmlGetNodePath(err.getNode());
		xmlChar* uri = xmlNodeGetBase(err.getNode()->doc,err.getNode());
		std::string file;
		if(uri!=NULL && uri[0]!='\0')
			file = std::string(" of ") + (char*)uri + std::string(":");
		else
			file = " at line ";
		xmlFree(uri);
		cerr << "  Error was flagged during processing" << file << xmlGetLineNo(err.getNode()) << ":\n"
		<< "    " << (char*)path << " '" << (char*)xmlNodeGetContent(err.getNode()) << '\'' << endl;
		xmlFree(path);
	}
}

unsigned int XMLLoadSave::getBinSize() const {
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
		xmlChar* buf=NULL;
		int size=0;
		// due to entity escaping unicode characters, this may predict a larger size than we actually use...
		// that's ok, this function is allowed to over-estimate size requirements
		xmlDocDumpFormatMemory(xmldocument, &buf, &size, autoFormat);
		xmlFree(buf);
		return size;
	} catch(const bad_format& err) {
		reportError("During calculation of size:",err);
		return 0;
	}
}
unsigned int XMLLoadSave::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	if(xmldocument!=NULL) {
		xmlFreeDoc(xmldocument);
		xmldocument=NULL; // in case we error out below
	}
	
	//does actual low-level XML parsing
	xmlParserCtxt* ctxt=xmlCreateMemoryParserCtxt(buf,len);
	if(ctxt==NULL) {
		cerr << "Error: " << (filename ? filename : "") << " XMLLoadSave could not create memory parser context" << endl;
		return 0;
	}
	xmldocument = xmlParseDocument(ctxt)==0?ctxt->myDoc:NULL;
	if (xmldocument == NULL ) {
		cerr << "Error: XMLLoadSave buffer not parsed successfully. (xml syntax error)\n"
		     << "       Attempting to recover..." << endl;
		xmlFreeParserCtxt(ctxt);
		ctxt=xmlCreateMemoryParserCtxt(buf,len);
		if(ctxt==NULL) {
			cerr << "Error: XMLLoadSave could not create memory parser context" << endl;
			return 0;
		}
		ctxt->recovery=1;
		xmldocument = xmlParseDocument(ctxt)==0?ctxt->myDoc:NULL;
		if(xmldocument==NULL) {
			cerr << "Error: XMLLoadSave recovery failed." << endl;
			xmlFreeParserCtxt(ctxt);
			return 0;
		}
	}
	unsigned int size=ctxt->nbChars;
	xmlFreeParserCtxt(ctxt);
	
	try {
		xmlNodePtr cur = FindRootXMLElement(xmldocument);
		loadXML(cur);
		return size;
	} catch(const bad_format& err) {
		reportError("During load of memory buffer:",err);
		xmlFreeDoc(xmldocument);
		xmldocument=NULL;
		return 0;
	}
}
unsigned int XMLLoadSave::saveBuffer(char buf[], unsigned int len) const {
	xmlBufferPtr xmlbuf = NULL;
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
#if LIBXML_VERSION < 20623
		// versions prior to 2.6.23 don't have saveToBuffer implemented!
		(void)xmlbuf; // avoid warning, we don't use in this case...
		xmlChar* xbuf=NULL;
		int size=0;
		xmlDocDumpFormatMemory(xmldocument, &xbuf, &size, autoFormat);
		if((unsigned int)size<=len)
			memcpy(buf,xbuf,size);
		else {
			cerr << "Error: XMLLoadSave::saveBuffer xmlDocDumpFormatMemory returned larger region than the target buffer" << endl;
			size=0;
		}
		xmlFree(xbuf);
		return size;
#else
		xmlbuf = xmlBufferCreate();
		xmlSaveCtxtPtr ctxt = xmlSaveToBuffer(xmlbuf, NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		xmlSaveSetEscape(ctxt,xmlEscapeMinimalEntities);
		xmlSaveSetAttrEscape(ctxt,xmlEscapeMinimalEntities);
		size_t size = xmlSaveDoc(ctxt,xmldocument);
		xmlSaveClose(ctxt);
		ctxt=NULL;
		if(size==(size_t)-1) {
			cerr << "Error: XMLLoadSave::saveBuffer: xmlSaveDoc returned -1" << endl;
			return 0;
		}
		size=xmlBufferLength(xmlbuf);
		if(size<=len)
			memcpy(buf,xmlBufferContent(xmlbuf),size);
		else {
			cerr << "Error: XMLLoadSave::saveBuffer xmlSaveDoc returned larger region than the target buffer" << endl;
			size=0;
		}
		xmlBufferFree(xmlbuf);
		xmlbuf=NULL;
		return size;
#endif
	} catch(const bad_format& err) {
		reportError("During save to memory buffer:",err);
		xmlBufferFree(xmlbuf);
		return 0;
	} catch(...) {
		xmlBufferFree(xmlbuf);
		throw;
	}
}

unsigned int XMLLoadSave::loadFile(const char* filename) {
	if(xmldocument!=NULL) {
		xmlFreeDoc(xmldocument);
		xmldocument=NULL; // in case we error out below
	}
	
	//does actual low-level XML parsing
	xmlParserCtxt* ctxt=xmlCreateFileParserCtxt(filename);
	if(ctxt==NULL) {
		cerr << "Error: XMLLoadSave could not create parser context for '"<< filename << "'" << endl;
		return 0;
	}
	xmldocument = xmlParseDocument(ctxt)==0?ctxt->myDoc:NULL;
	if (xmldocument == NULL ) {
		cerr << "Error: XMLLoadSave document '" << filename << "' not parsed successfully. (file not found or xml syntax error)\n"
		<< "       Attempting to recover..." << endl;
		xmlFreeParserCtxt(ctxt);
		ctxt=xmlCreateFileParserCtxt(filename);
		if(ctxt==NULL) {
			cerr << "Error: XMLLoadSave could not create parser context for '"<< filename << "'" << endl;
			return 0;
		}
		ctxt->recovery=1;
		xmldocument = xmlParseDocument(ctxt)==0?ctxt->myDoc:NULL;
		if(xmldocument==NULL) {
			cerr << "Error: XMLLoadSave document '" << filename << "' recovery failed." << endl;
			xmlFreeParserCtxt(ctxt);
			return 0;
		}
	}
	unsigned int size=ctxt->nbChars;
	xmlFreeParserCtxt(ctxt);
	
	try {
		xmlNodePtr cur = FindRootXMLElement(xmldocument);
		loadXML(cur);
		return size;
	} catch(const bad_format& err) {
		string context("During load of '");
		context+=filename;
		context+="':";
		reportError(context,err);
		xmlFreeDoc(xmldocument);
		xmldocument=NULL;
		return 0;
	}
}
unsigned int XMLLoadSave::saveFile(const char* filename) const {
	xmlBufferPtr xmlbuf = NULL;
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
#if LIBXML_VERSION < 20623
		// versions prior to 2.6.23 don't have saveToBuffer implemented!
		// could use xmlSaveToFilename and fake the return size, but I'd rather be correct and
		// give up on un-escaping fancy unicode characters
		(void)xmlbuf; // avoid warning, we don't use in this case...
		int size=xmlSaveFormatFile (filename, xmldocument, autoFormat);
		if(size==-1)
			cerr << "Error: XMLLoadSave::saveFile: xmlSaveFormatFile(\"" << filename << "\",...) returned -1" << endl;
		return size==-1?0:size;
#else
		FILE* f = fopen(filename,"w");
		if(f==NULL) {
			std::cerr << "*** WARNING XMLLoadSave::saveFile: could not open file for saving \"" << filename << "\"" << std::endl;
			return 0;
		}
		// xmlSaveDoc doesn't properly return written size, so use buffers instead of xmlSaveToFilename:
		xmlbuf = xmlBufferCreate();
		xmlSaveCtxtPtr ctxt = xmlSaveToBuffer(xmlbuf, NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		//xmlSaveCtxtPtr ctxt = xmlSaveToFilename(filename, NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		xmlSaveSetEscape(ctxt,xmlEscapeMinimalEntities);
		xmlSaveSetAttrEscape(ctxt,xmlEscapeMinimalEntities);
		size_t size = xmlSaveDoc(ctxt,xmldocument);
		xmlSaveClose(ctxt);
		ctxt=NULL;
		if(size==(size_t)-1) {
			cerr << "Error: XMLLoadSave::saveFile: xmlSaveDoc(\"" << filename << "\",...) returned -1" << endl;
			fclose(f);
			return 0;
		}
		size=xmlBufferLength(xmlbuf);
		size_t wrote=fwrite(xmlBufferContent(xmlbuf),1,xmlBufferLength(xmlbuf),f);
		if(wrote!=size)
			std::cerr << "*** WARNING XMLLoadSave::saveFile: short write (wrote " << wrote << ", expected " << size << ")" << std::endl;
		int err=fclose(f);
		if(err!=0) {
			std::cerr << "*** WARNING XMLLoadSave::saveFile: error '" << strerror(errno) << "' while closing " << filename << std::endl;
			return 0;
		}
		xmlBufferFree(xmlbuf);
		xmlbuf=NULL;
		return size;
#endif
	} catch(const bad_format& err) {
		string context("During save to '");
		context+=filename;
		context+="':";
		reportError(context,err);
		xmlBufferFree(xmlbuf);
		return 0;
	} catch(...) {
		xmlBufferFree(xmlbuf);
		throw;
	}
}

unsigned int XMLLoadSave::loadFileStream(FILE* f, const char* filename) {
	if(xmldocument!=NULL) {
		xmlFreeDoc(xmldocument);
		xmldocument=NULL; // in case we error out below
	}
	
	//does actual low-level XML parsing
	//This is a little sketchy trying to shoehorn a SAX style call, but it seems to work
	xmlParserCtxt* ctxt=xmlCreateIOParserCtxt(NULL,NULL,fileReadCallback,fileCloseCallback,f,XML_CHAR_ENCODING_UTF8);
	if(ctxt==NULL) {
		cerr << "Error: XMLLoadSave could not create file stream parser context" << endl;
		return 0;
	}
	ctxt->recovery=1;
	xmldocument = (xmlParseDocument(ctxt)==0) ? ctxt->myDoc : NULL;
	unsigned int size=ctxt->nbChars;
	bool wellFormed=ctxt->wellFormed;
	xmlFreeParserCtxt(ctxt);
	if (xmldocument==NULL) {
		cerr << "Error: XMLLoadSave file stream not parsed successfully. (xml syntax error)\n" << endl;
		return 0;
	}
	if(!wellFormed)
		cerr << "Warning: XMLLoadSave file stream was not well formed (but was recovered)." << endl;
	
	try {
		xmlNodePtr cur = FindRootXMLElement(xmldocument);
		loadXML(cur);
		return size;
	} catch(const bad_format& err) {
		reportError("During load of file stream "+std::string(filename)+":",err);
		xmlFreeDoc(xmldocument);
		xmldocument=NULL;
		return 0;
	}
}
unsigned int XMLLoadSave::saveFileStream(FILE* f) const {
	xmlBufferPtr xmlbuf = NULL;
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
#if LIBXML_VERSION < 20623
		// versions prior to 2.6.23 don't have saveToBuffer implemented!
		// could use xmlSaveToFilename and fake the return size, but I'd rather be correct and
		// give up on un-escaping fancy unicode characters
		(void)xmlbuf; // avoid warning, we don't use in this case...
		int size=xmlDocFormatDump(f, xmldocument, autoFormat);
		if(size==-1)
			cerr << "Error: XMLLoadSave::saveFileStream: xmlDocFormatDump(...) returned -1" << endl;
		return size==-1?0:size;
#else
		// xmlSaveDoc doesn't properly return written size, so use buffers instead of xmlSaveToFd:
		xmlbuf = xmlBufferCreate();
		xmlSaveCtxtPtr ctxt = xmlSaveToBuffer(xmlbuf, NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		//xmlSaveCtxtPtr ctxt = xmlSaveToFd([...], NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		xmlSaveSetEscape(ctxt,xmlEscapeMinimalEntities);
		xmlSaveSetAttrEscape(ctxt,xmlEscapeMinimalEntities);
		size_t size = xmlSaveDoc(ctxt,xmldocument);
		xmlSaveClose(ctxt);
		ctxt=NULL;
		if(size==(size_t)-1) {
			cerr << "Error: XMLLoadSave::saveFileStream: xmlSaveDoc returned -1" << endl;
			return 0;
		}
		size=xmlBufferLength(xmlbuf);
		size_t wrote=fwrite(xmlBufferContent(xmlbuf),1,xmlBufferLength(xmlbuf),f);
		if(wrote!=size)
			std::cerr << "*** WARNING XMLLoadSave::saveFileStream: short write (wrote " << wrote << ", expected " << size << ")" << std::endl;
		xmlBufferFree(xmlbuf);
		xmlbuf=NULL;
		return size;
#endif
	} catch(const bad_format& err) {
		reportError("During save to file stream:",err);
		xmlBufferFree(xmlbuf);
		return 0;
	} catch(...) {
		xmlBufferFree(xmlbuf);
		throw;
	}
}

//! Extends the parser context to include a completion flag and the input stream, and encapsulate the setup
struct StreamParser {
	// This must go first to masquerade as a xmlParserCtxt
	// Tried to use inheritance, but get a warning about synthesized constructor, rather just encapsulate instead
	// I know it's ugly(ier), don't hate me, it removes the warning so now no one will ever be compelled to look here anyway.
	xmlParserCtxt _ctxt;
	
	//! constructor
	StreamParser(std::istream& in, bool asFragment) : _ctxt(), handler(), ctxt(NULL), complete(false), is(&in), used(0) {
		if(!asFragment) {
			ctxt = xmlCreatePushParserCtxt(NULL,NULL,NULL,0,NULL);
		} else {
			xmlSAXVersion(&handler,1); // using SAX version 1... bad?  endElement is much simpler than endElementNs
			handler.endElement = endElementCallback;
			ctxt = xmlCreatePushParserCtxt(&handler,NULL,NULL,0,NULL);
		}
		if(ctxt==NULL) {
			cerr << "Error: XMLLoadSave could not create file stream parser context" << endl;
			throw std::bad_alloc();
		}
		ctxt->recovery=1;
		ctxt->userData=this;
		_ctxt=(*ctxt); // now steal all the associated structures
	}
	//! destructor, delete the context
	~StreamParser() { xmlFreeParserCtxt(ctxt); ctxt=NULL; }
	
	operator xmlParserCtxt*() { return &_ctxt; }
	
	//! stores callbacks, uses defaults to build tree, except endElement which forwards to our local version
	xmlSAXHandler handler;
	//! the original libxml structure we're mimicing, stored so we can free it in destructor (and thus the associated data structures)
	xmlParserCtxt* ctxt;
	
	//! marked true once the root element is closed
	bool complete;
	//! the stream we're reading from (so we can putback extra data for the next read)
	std::istream* is;
	//! should be updated by pusher to note how much data was loaded by the stream
	/*! (probably can't putback more than this, but also allows us to track usage byte count accurately) */
	unsigned int used;
	
	//! forwards callback to doEndElement() so we don't have to prepend @a ctx on every usage
	static void endElementCallback(void * ctx, const xmlChar * name) { reinterpret_cast<StreamParser*>(ctx)->doEndElement(name); }
	
	//! check if this is ending the root element, and putback unused data if it is, then mark the input complete
	void doEndElement(const xmlChar * curName) {
		//std::cerr << "Root is " << xmlDocGetRootElement(myDoc) << ' ' << node << ' ' << nodeNr << ' ' << nameNr << ' ' << spaceNr << std::endl;
		if(_ctxt.nameNr==1) {
			for(const xmlChar* x=_ctxt.input->end; x!=_ctxt.input->cur && *is; )
				is->putback(*--x);
			//std::cout << "put back " << input->end - input->cur << std::endl;
			used-=(_ctxt.input->end - _ctxt.input->cur);
			_ctxt.input->cur=_ctxt.input->end; // this prevents libxml from seeing the extra data, also seems to signal libxml not to try to get more data if letting the library pull
			complete=true;
		}
		xmlSAX2EndElement(this,curName);
	}
private:
	StreamParser(const StreamParser&); //!< don't call
	StreamParser& operator=(const StreamParser&); //!< don't call
};

unsigned int XMLLoadSave::loadStream(std::istream& is, bool asFragment /*=false*/) {
	if(xmldocument!=NULL) {
		xmlFreeDoc(xmldocument);
		xmldocument=NULL; // in case we error out below
	}
	
	unsigned int totalUsed=0;
	{
		StreamParser parser(is,asFragment);
		const size_t BUFSIZE=4*1024;
		char buf[BUFSIZE];
		while(!parser.complete && is.read(buf,1)) { // block for more data
			is.readsome(buf+1,BUFSIZE-1); // read rest of the packet (doesn't necessarily fill buffer, just what's available...)
			parser.used = is.gcount()+1;
			//std::cout << this << " buffer is '" << std::string(buf,std::min(100,used)) << "'" << std::endl;
			int err = xmlParseChunk(parser,buf,parser.used,false);
			if(err!=0) {
				std::cerr << "XMLLoadSave::loadStream encountered XML error " << err << std::endl;
				return 0;
			}
			totalUsed+=parser.used;
		}
		if(!is || !parser.complete)
			return 0;
		int err = xmlParseChunk(parser,NULL,0,true);
		if(err!=0) {
			std::cerr << "XMLLoadSave::loadStream encountered XML error " << err << std::endl;
			return 0;
		}
		if(!parser._ctxt.wellFormed)
			cerr << "Warning: XMLLoadSave file stream was not well formed (but was recovered)." << endl;
		
		xmldocument = parser._ctxt.myDoc;
		if(xmldocument==NULL) {
			cerr << "ERROR: XMLLoadSave parsing completed but document is still NULL!" << endl;
			return 0;
		}
	}
	
	try {
		xmlNodePtr cur = FindRootXMLElement(xmldocument);
		loadXML(cur);
		return totalUsed;
	} catch(const bad_format& err) {
		reportError("During load of file stream:",err);
		xmlFreeDoc(xmldocument);
		xmldocument=NULL;
		return 0;
	}
}
unsigned int XMLLoadSave::saveStream(std::ostream& os, bool asFragment) const {
	xmlBufferPtr xmlbuf = NULL;
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
#if LIBXML_VERSION < 20623
		// versions prior to 2.6.23 don't have saveToBuffer implemented!
		(void)xmlbuf; // avoid warning, we don't use in this case...
		xmlChar* xbuf=NULL;
		int size=0;
		xmlDocDumpFormatMemory(xmldocument, &xbuf, &size, autoFormat);
		os.write((const char*)xbuf,size);
		xmlFree(xbuf);
		return size;
#else
		xmlbuf = xmlBufferCreate();
		xmlSaveCtxtPtr ctxt = xmlSaveToBuffer(xmlbuf, NULL, (autoFormat ? XML_SAVE_FORMAT : 0));
		xmlSaveSetEscape(ctxt,xmlEscapeMinimalEntities);
		xmlSaveSetAttrEscape(ctxt,xmlEscapeMinimalEntities);
		size_t size = asFragment ? xmlSaveTree(ctxt,xmlDocGetRootElement(xmldocument)) : xmlSaveDoc(ctxt,xmldocument);
		xmlSaveClose(ctxt);
		ctxt=NULL;
		if(size==(size_t)-1) {
			cerr << "Error: XMLLoadSave::saveBuffer: xmlSaveDoc/xmlSaveTree returned -1" << endl;
			return 0;
		}
		size=xmlBufferLength(xmlbuf);
		os.write((const char*)xmlBufferContent(xmlbuf),size);
		xmlBufferFree(xmlbuf);
		return size;
#endif
	} catch(const bad_format& err) {
		reportError("During save to memory buffer:",err);
		xmlBufferFree(xmlbuf);
		return 0;
	} catch(...) {
		xmlBufferFree(xmlbuf);
		throw;
	}
}

void XMLLoadSave::clearParseTree() {
	xmlFreeDoc(xmldocument);
	xmldocument=NULL;
}
void XMLLoadSave::setParseTree(xmlDoc* doc) const {
	if(doc==xmldocument)
		return;
	xmlFreeDoc(xmldocument);
	xmldocument=doc;
}
xmlDoc* XMLLoadSave::stealParseTree(xmlDoc* newdoc/*=NULL*/) const {
	xmlDoc * oldDoc = xmldocument;
	xmldocument=newdoc;
	return oldDoc;
}
void XMLLoadSave::readParseTree() {
	if(xmldocument==NULL)
		return;
	try {
		xmlNodePtr cur = FindRootXMLElement(xmldocument);
		loadXML(cur);
	} catch(const bad_format& err) {
		reportError("During XMLLoadSave::readParseTree:",err);
		xmlFreeDoc(xmldocument);
		xmldocument=NULL;
	}
}
void XMLLoadSave::writeParseTree() const {
	try {
		if(xmldocument==NULL)
			setParseTree(xmlNewDoc((const xmlChar*)"1.0"));
		if(compressionLevel>=0)
			xmlSetDocCompressMode(xmldocument,compressionLevel);
		xmlNode * cur = FindRootXMLElement(xmldocument);
		saveXML(cur);
	} catch(const bad_format& err) {
		reportError("During writeParseTree:",err);
	}
}

void XMLLoadSave::setCompression(int level) {
	compressionLevel=level;
	if(xmldocument!=NULL)
		xmlSetDocCompressMode(xmldocument,compressionLevel);
}

xmlNode* XMLLoadSave::FindRootXMLElement(xmlDoc* doc) const {
	if(doc==NULL)
		return NULL;
	xmlNode* cur=xmlDocGetRootElement(doc);
	if(cur==NULL) {
		//empty file
		cur = xmlNewNode(NULL,(const xmlChar*)"");
		xmlFree(xmlDocSetRootElement(doc,cur));
	} 
	return cur;
}

int XMLLoadSave::fileReadCallback(void* f,char* buf, int len) {
	return ferror((FILE*)f) ? -1 : (int)fread(buf,sizeof(char),len,(FILE*)f);
}
int XMLLoadSave::fileCloseCallback(void* f) {
	return fclose((FILE*)f) ? -1 : 0;
}

xmlNode* XMLLoadSave::skipToElement(xmlNode* cur) {
	while(cur!=NULL && cur->type!=XML_ELEMENT_NODE)
		cur=cur->next;
	return cur;
}

xmlNode* XMLLoadSave::skipToElement(xmlNode* cur, std::string& comment) {
	comment.clear();
	while(cur!=NULL && cur->type!=XML_ELEMENT_NODE) {
		if(cur->type==XML_COMMENT_NODE) {
			xmlChar* cont=xmlNodeGetContent(cur);
			comment=(char*)cont; //only take last comment in series
			xmlFree(cont);
		}
		cur=cur->next;
	}
	return cur;
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
