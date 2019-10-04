//-*-c++-*-
#ifndef INCLUDED_FileInputControl_h_
#define INCLUDED_FileInputControl_h_

#include "FileBrowserControl.h"
#include <string>

//! Upon activation, allows user to browse files and select one; stores path in a string
class FileInputControl : public FileBrowserControl {
 public:
	//! Constructor
	FileInputControl()
		: FileBrowserControl("Select file","Select a file","/"), file(NULL), myfile(), acceptNonExistant(false)
	{}

	//! Constructor
	FileInputControl(const std::string& nm, const std::string& desc, const std::string& path, std::string* store=NULL)
		: FileBrowserControl(nm,desc,path), file(store), myfile(), acceptNonExistant(false)
	{}

	//! returns the path to file last selected; use takeInput() to assign to this
	virtual const std::string& getLastInput() { return myfile; }

	//! clears the last input (i.e. so you can easily tell later if new input is entered)
	virtual void clearLastInput() { selectedFile(""); }

	//! pass pointer to an external string you wish to have set when a file is selected; NULL otherwise
	virtual void setStore(std::string* store) { file=store; }
	
	//! sets #acceptNonExistant
	virtual void setAcceptNonExistant(bool b) { acceptNonExistant=b; }
	//! returns #acceptNonExistant
	virtual bool getAcceptNonExistant() const { return acceptNonExistant; }
	
protected:
	virtual ControlBase* selectedFile(const std::string& f) {
		myfile=f;
		if(file!=NULL)
			*file=f;
		return NULL;
	}
	virtual ControlBase* invalidInput(const std::string& msg, bool ambiguous) {
		if(!acceptNonExistant)
			return FileBrowserControl::invalidInput(msg,ambiguous);
		return selectedFile(makePath(msg));
	}

	std::string* file;  //!< if we're supposed to store in an external string, this will point to it, otherwise NULL
	std::string myfile; //!< stores last file selected
	bool acceptNonExistant; //!< if true, will set #file and #myfile to "invalid" inputs as well -- i.e. inputs that don't correspond to any current file (so user can request a new one)

private:
	FileInputControl(const FileInputControl& ); //!< don't call
	FileInputControl& operator=(const FileInputControl& ); //!< don't call
};

/*! @file
 * @brief Defines FileInputControl, which allows the user to browse files and select one, which is then stored in a string
 * @author ejt (Creator)
 */

#endif
