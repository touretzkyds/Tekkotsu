//-*-c++-*-
#ifndef INCLUDED_ConfigurationEditor_h
#define INCLUDED_ConfigurationEditor_h

#include "Behaviors/Controls/ControlBase.h"
#include "Behaviors/Controls/FileInputControl.h"
#include "Behaviors/Controls/StringInputControl.h"
#include "Shared/plistCollections.h"

//! Provides interactive editing and loading/saving of a plist::Collection
class ConfigurationEditor : public ControlBase {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
public:
	//! default constructor
	ConfigurationEditor(plist::Collection * rootCollection=NULL)
		: ControlBase("ConfigurationEditor","Provides interactive editing and loading/saving of a plist::Collection"),
		root(rootCollection), path(), load(rootCollection), save(rootCollection)
	{init();}
	//! constructor which allows a custom name
	ConfigurationEditor(const std::string& n, plist::Collection * rootCollection=NULL)
		: ControlBase(n,"Provides interactive editing and loading/saving of a plist::Collection"),
		root(rootCollection), path(), load(rootCollection), save(rootCollection)
	{init();}
	//! constructor which allows a custom name and tooltip
	ConfigurationEditor(const std::string& n, const std::string& d, plist::Collection * rootCollection=NULL)
		: ControlBase(n,d), root(rootCollection), path(), load(rootCollection), save(rootCollection)
	{init();}

	//! destructor, have to do our custom clearSlots before the superclass tries to
	virtual ~ConfigurationEditor() { clearSlots(); }

protected:
	//! common initialization regardless of constructor
	virtual void init();


	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	//! called when a child has deactivated and this control should refresh its display, or some other event (such as the user pressing the refresh button) has happened to cause a refresh to be needed
	virtual void refresh();
	
	//! need to override clearing so we don't delete #load and #save
	virtual void clearSlots();
	
	//! sets the root of the collection being managed
	virtual void setRootCollection(plist::Collection* rootCollection);
	
	//! sets the path to the sub-collection being managed
	virtual void setPath(const std::string& p);
	
	//! returns a dictionary of plist objects which may be added as new entries to collections being edited
	static plist::Dictionary& getObjectTemplates();
	
	
	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	plist::Collection * root; //!< the root of collection being edited (i.e. may be the parent or other ancestor of the collection this editor is targeting, see #path)
	std::string path; //!< the names of the sub-collections  from #root to get to the collection this instance is actually targeting
	
	//! provides a file browser to load a plist from the file system
	class LoadSettings : public FileInputControl {
	public:
		//! constructor, if rootCollection is NULL, you must call setRootCollection() with a non-NULL value at some point before user tries to activate the control
		LoadSettings(plist::Collection * rootCollection) : FileInputControl("Load...","Load settings from disk","config"), rootcol(rootCollection) {}
		//! assigns #rootcol
		virtual void setRootCollection(plist::Collection* rootCollection) { rootcol=rootCollection; }
	protected:
		virtual ControlBase* selectedFile(const std::string& f);
		plist::Collection * rootcol; //!< the collection to be saved (generally the top level of the collection, even if currently editing within a sub-collection)
	private:
		LoadSettings(const LoadSettings&); //!< not supported
		LoadSettings& operator=(const LoadSettings&); //!< not supported
	};
	LoadSettings load; //!< reuse the same load instance instead of regenerating a new one each refresh (saves user's "working directory")
	
	//! provides a file browser to save a plist from the file system
	class SaveSettings : public FileInputControl {
	public:
		//! constructor, if rootCollection is NULL, you must call setRootCollection() with a non-NULL value at some point before user tries to activate the control
		SaveSettings(plist::Collection * rootCollection) : FileInputControl("Save...","Save settings to disk","config"), rootcol(rootCollection) { setAcceptNonExistant(true); }
		//! assigns #rootcol
		virtual void setRootCollection(plist::Collection* rootCollection) { rootcol=rootCollection; }
	protected:
		virtual ControlBase* selectedFile(const std::string& f);
		plist::Collection * rootcol; //!< the collection to be saved (generally the top level of the collection, even if currently editing within a sub-collection)
	private:
		SaveSettings(const SaveSettings&); //!< not supported
		SaveSettings& operator=(const SaveSettings&); //!< not supported
	};
	SaveSettings save; //!< reuse the same save instance instead of regenerating a new one each refresh (saves user's "working directory")
	
	//! displays a list of template objects (see ConfigurationEditor::getObjectTemplates()) which can be added to a target collection
	class AddCollectionEntry : public ControlBase {
	public:
		AddCollectionEntry(plist::Collection& target) : ControlBase("Add New Entry"), tgt(&target) {}
		virtual void refresh();
		virtual void deactivate() { clearSlots(); ControlBase::deactivate(); }
	protected:
		plist::Collection * tgt;
	private:
		AddCollectionEntry(const AddCollectionEntry&); //!< not supported
		AddCollectionEntry& operator=(const AddCollectionEntry&); //!< not supported
	};
	
	//! provides interface to set up a new collection entry before inserting it
	class NewCollectionEntry : public ControlBase {
	public:
		NewCollectionEntry(const std::string& n, plist::Collection& target, plist::ObjectBase& templ) : ControlBase(n), tgt(&target), obj(&templ) {}
		virtual void refresh();
		virtual void deactivate() { clearSlots(); ControlBase::deactivate(); }
		virtual ControlBase * doSelect();
	protected:
		plist::Collection * tgt;
		plist::ObjectBase * obj;
	private:
		NewCollectionEntry(const NewCollectionEntry&); //!< not supported
		NewCollectionEntry& operator=(const NewCollectionEntry&); //!< not supported
	};
	
	//! based on a string input control for easier mixing with other primitives (which just use a basic string input control as an editor)
	/*! If selected, provides a submenu with symbolic options to better prompt the user with valid values */
	class NamedEnumerationEditor : public StringInputControl {
	public:
		//! constructor
		NamedEnumerationEditor(const std::string& n, const std::string& prompt, plist::NamedEnumerationBase& target) : StringInputControl(n,prompt), tgt(&target) {}
		virtual void refresh();
		//! purposely clearing slots on deactivate to avoid having sub-menus -- don't want to appear as a collection
		virtual void deactivate() { clearSlots(); StringInputControl::deactivate(); }
		//! handles selecting one of the symbolic values
		virtual ControlBase * doSelect();
		//! override to only allow one hilight (first in the list)
		virtual void setHilights(const std::vector<unsigned int>& hi) {
			if(hi.empty())
				StringInputControl::setHilights(hi);
			else // go straight to control base
				ControlBase::setHilights(std::vector<unsigned int>(1,hi.front()));
		}
	protected:
		plist::NamedEnumerationBase * tgt; //!< the value being edited
	private:
		NamedEnumerationEditor(const NamedEnumerationEditor&); //!< not supported
		NamedEnumerationEditor& operator=(const NamedEnumerationEditor&); //!< not supported
	};
	

	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	ConfigurationEditor(const ConfigurationEditor&); //!< you can override, but don't call this...
	ConfigurationEditor& operator=(const ConfigurationEditor&); //!< you can override, but don't call this...
};

/*! @file
 * @brief Defines ConfigurationEditor, which provides interactive editing and loading/saving of a plist::Collection
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
#endif
