//-*-c++-*-
#ifndef INCLUDED_WalkEdit_h
#define INCLUDED_WalkEdit_h

#include "Behaviors/Controls/ControlBase.h"
#include <stack>

//! Editor for the CMPack-based Aibo walk engine
class WalkEdit : public ControlBase {
public:
	//! default constructor
	WalkEdit() : ControlBase("Walk Edit","Editor for the CMPack-based Aibo walk engine"), menuStack() { init(); }

	//! destructor
	virtual ~WalkEdit() {}

protected:
	//! initialization
	virtual void init();
	
	//! subsequent addItem() calls will be placed in the specified menu until matching endSubMenu()
	void startSubMenu(const std::string& nm, const std::string& desc);
	
	//! adds a specified control to the current submenu
	void addItem(ControlBase* control);
	
	//! closes the current submenu, moves to its parent
	void endSubMenu();
	
	//! maintains stack of current submenus when building via init()
	std::stack<ControlBase*> menuStack;

private:
	WalkEdit(const WalkEdit&); //!< you can override, but don't call this...
	WalkEdit& operator=(const WalkEdit&); //!< you can override, but don't call this...
};

/*! @file
 * @brief Describes WalkEdit, which provides an editor for the CMPack-based Aibo walk engine
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
#endif
