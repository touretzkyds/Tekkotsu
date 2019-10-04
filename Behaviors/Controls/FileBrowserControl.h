//-*-c++-*-
#ifndef INCLUDED_FileBrowserControl_h_
#define INCLUDED_FileBrowserControl_h_

#include "ControlBase.h"
#include <string>
#include <vector>

//! Displays the contents of a directory in a control menu, probably useful as a baseclass for other controls
/*! Causes the selectedFile() function to be called on the root FileBrowserControl with the selected file */
class FileBrowserControl : public ControlBase {
public:
	//!@name Constructors/Destructors
	//!constructor
	FileBrowserControl() : ControlBase(), recurse(true), root(), paths(), filter("*") {init("");}
	//!constructor pass name and root path
	FileBrowserControl(const std::string& nm, const std::string& desc, const std::string& path) : ControlBase(nm,desc), recurse(true), root(), paths(), filter("*") { init(path); }
	//@}
	
	//!@name ControlBase Inheritance
	virtual ControlBase * activate(MC_ID display, Socket * gui);
	virtual ControlBase* doSelect();
	virtual ControlBase * takeInput(const std::string& msg);
	//@}

	//!@name Accessors
	void setRecurse(bool r) { recurse=r; }                 //!< sets #recurse
	bool getRecurse() const { return recurse; }            //!< returns #recurse

	void setRoot(const std::string& path);                 //!< sets #root, clears #paths
	std::string getRoot() const { return root; }           //!< returns #root

	void appendPath(const std::string& path);                 //!< adds entries to #paths (location relative to #root)
	
	void setFilter(const std::string& filt) {filter=filt;} //!< sets #filter; remember can only use one wildcard, e.g. *.ext or filename.ext or filename*
	//@}
	
protected:

	//! the big money function - by default calls the parent if it exists, otherwise nothing
	/*! returning NULL means deactivate, this (default) to stay put, or a different Control if you want a submenu */
	virtual ControlBase* selectedFile(const std::string&) { return this; }
	
	//! returns the path from root as a string, with no trailing '/'
	std::string makePath();
	
	//! returns the path from root as a string, appends filename
	std::string makePath(const std::string& filename);
	
	//! returns true if @a file matches @a filt
	static bool match(const std::string& file, const std::string& filt);

	//!rescans current directory and builds menus
	void rebuildmenu();

	//!sets a junk menu item to mark this as having submenus, and sets root to path
	void init(std::string path) { pushSlot(NULL); setRoot(path); }

	bool recurse;                   //!< if true (default), will show directories; if false, subdirectories are hidden
	std::string root;               //!< the path to browse, default "/"
	std::vector<std::string> paths; //!< list of directories from root
	
	std::string filter;             //!< default "*", only display matching files; only can use one wildcard, e.g. *.ext or filename.ext or filename*

private:
//	FileBrowserControl(const FileBrowserControl& ); //!< don't call
//	FileBrowserControl& operator=(const FileBrowserControl& ); //!< don't call
};

/*! @file
 * @brief Describes FileBrowserControl, which displays the contents of a directory
 * @author ejt (Creator)
 */

#endif
