//-*-c++-*-
#ifndef INCLUDED_XMLLoadSave_h_
#define INCLUDED_XMLLoadSave_h_

#include "LoadSave.h"

/*! @cond INTERNAL */
// libxml2 forward declarations
extern "C" {
	struct _xmlNode;
	struct _xmlDoc;
	struct _xmlAttr;
	typedef _xmlNode xmlNode;
	typedef _xmlDoc xmlDoc;
	typedef _xmlAttr xmlAttr;
}
/*! @endcond */

//! XMLLoadSave adds functions for XML format serialization, although if you write binary LoadSave functions as well, you can do either
class XMLLoadSave : public virtual LoadSave {
public:
	
	//! an exception to be thrown when a bad XML file is parsed, allows file position information to be passed to the user
	class bad_format : public std::exception {
	public:
		//!constructor
		bad_format(xmlNode * node) throw() : std::exception(), node_(node), msg_("invalid format in xml data") {}
		//!constructor
		bad_format(xmlNode * node, const std::string& msg) throw() : std::exception(), node_(node), msg_(msg) {}
		//!copy constructor
		bad_format(const bad_format& bf) : std::exception(bf), node_(bf.node_), msg_(bf.msg_) {}
		//!assignment
		bad_format& operator=(const bad_format& bf) { std::exception::operator=(bf); node_=bf.node_; msg_=bf.msg_; return *this; }
		virtual ~bad_format() throw() {} //!< destructor
		virtual const char * what() const throw() { return msg_.c_str(); } //!< standard interface for getting the exception's message string
		virtual xmlNode * getNode() const throw() { return node_; } //!< returns the xmlNode at which the error occurred
	protected:
		xmlNode * node_; //!< the node of the error
		std::string msg_; //!< message regarding the type of error
	};
	
	XMLLoadSave(); //!< constructor
	XMLLoadSave(const XMLLoadSave& xls); //!< copy constructor
	XMLLoadSave& operator=(const XMLLoadSave& xls); //!< assignment operator
	virtual ~XMLLoadSave(); //!< destructor
	
	//! This is called when the subclass needs to update its values from those values in the parse tree
	/*! @a node is the current node in the tree -- it may be the root, but it
	 *  may also be a subnode within the tree if a recursive structure is used */
	virtual void loadXML(xmlNode* node)=0;
	//! This is called when XMLLoadSave needs to have the subclass update values in the tree currently in memory -- may already be filled out by previous contents
	/*! @a node is the current node in the tree -- it may be the root, but it
	 *  may also be a subnode within the tree if a recursive structure is used */
	virtual void saveXML(xmlNode * node) const=0;
	
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;

	virtual unsigned int loadFile(const char* filename);
	virtual unsigned int saveFile(const char* filename) const;
	
	virtual unsigned int loadFileStream(FILE* f, const char* filename=NULL);
	virtual unsigned int saveFileStream(FILE* f) const;

	//! loads from a std::istream, optionally cleaning up stream to allow additional loads if @a asFragment is true
	virtual unsigned int loadStream(std::istream& is, bool asFragment=false);
	
	//! saves to a std::ostream, optionally skipping xml document header if @a asFragment is true
	virtual unsigned int saveStream(std::ostream& os, bool asFragment) const;
	virtual unsigned int saveStream(std::ostream& os) const { return saveStream(os,false); }
	
	//! resets the parse tree currently in memory so that a future save will write a "fresh" copy from scratch.
	/*! You may also want to call this in order to save memory after a Load so that
	 *  the parse tree information isn't retained in memory if you don't care about
	 *  retaining the file formatting */
	virtual void clearParseTree();

	//! assigns a parse tree which you have obtained from some other source
	/*! This doesn't update the contents of the subclass's values.  The values in
	 *  @a doc will be overwritten by those in the subclass on the next Save.
	 *  If you wish to have the subclass's values updated from @a doc, call
	 *  readParseTree() after calling this. */
	virtual void setParseTree(xmlDoc* doc) const;
	
	//! returns the current parse tree, either from a previous Load, Save, or setParseTree()
	virtual xmlDoc* getParseTree() const { return xmldocument; }
	
	//! sets the object's parse tree to NULL and returns the former tree, which is then caller's responisibility
	virtual xmlDoc* stealParseTree(xmlDoc* newdoc=NULL) const;
	
	//! calls loadXML with the root of the current parse tree
	virtual void readParseTree();
	//! calls saveXML with the root of the current parse tree
	virtual void writeParseTree() const;
	
	//! allows you to set the compression level used by libxml2 -- this corresponds to the compression level setting of zlib, i.e. 0-9
	virtual void setCompression(int level);
	//! returns the current compression level setting used by libxml2 (via zlib)
	virtual int getCompression() const { return compressionLevel; }
	
protected:
	//! returns the root element of the xml document
	virtual xmlNode* FindRootXMLElement(xmlDoc* doc) const;

	//! cleans up after an error occurs and sends a message to cerr
	void reportError(const std::string& context, const bad_format& err) const;

	//! called by libxml2 when it's ready for more data to be read from the source (in this case, a file)
	static int fileReadCallback(void* f,char* buf, int len);
	//! called by libxml2 when it's done reading data
	static int fileCloseCallback(void* f);
	
	//! returns the next element following @a cur
	/*! You should call this with node->next, otherwise you'll just keep getting the same node back again */
	static xmlNode* skipToElement(xmlNode* cur);

	//! returns the next element following @a cur; comment is set to the concatination of any comments is encountered in the interim
	/*! You should call this with node->next, otherwise you'll just keep getting the same node back again */
	static xmlNode* skipToElement(xmlNode* cur, std::string& comment);

	//! the cached xml parse tree -- contains additional formatting information such as white space and comments
	mutable xmlDoc* xmldocument;

	//! the current compression level, or level to be used for the next save
	int compressionLevel;
	
	//! if true, the saved document will use automatic indenting and formatting
	bool autoFormat;

	//static const char * const base64="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	
	//! allows automatic (de)initialization of libxml when the first or last XMLLoadSave class is created or destroyed
	class AutoInit {
	public:
		AutoInit(); //!< if #libxmlrefc is 0, calls libxml2 initialization and configuration functions
		~AutoInit(); //!< if #libxmlrefc is 1, calls libxml2 destruction functions
	protected:
		static unsigned int libxmlrefc; //!< current number of XMLLoadSave subclass instances
	} libxmlInit; //!< allows tracking of libxml usage so the library can be initialized and destructed automatically
	
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
