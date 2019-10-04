//-*-c++-*-
#ifndef INCLUDED_HelpControl_h_
#define INCLUDED_HelpControl_h_

#include "NullControl.h"

//! Recurses through the menu system and outputs the name and description of each item
class HelpControl : public NullControl{
public:
	//!< constructor
	HelpControl(ControlBase* r, unsigned int depth=-1U)
		: NullControl("Help","Recurses through the menu system and outputs the name and description of each item"),
			maxDepth(depth), root(r)
	{}

	//! displays global Controller commands (hardcoded as strings, will need updates) as well as recursing the menu system (dynamic)
	virtual ControlBase * activate(MC_ID disp_id, Socket * gui);

	//! displays the menu items of @a r and their descriptions, recursing on submenus
	/*! @a prefix is what should be displayed before each menu item (like a bullet point)
	 *  this is itself prefixed by 2 spaces for each level of recursion.  Word wrapping
	 *  is performed to maintain the clean indenting */
	void report(ControlBase* r, const std::string& prefix, unsigned int max_depth);

	//! sets #maxDepth
	void setReportDepth(unsigned int depth) { maxDepth=depth; }

	//! gets #maxDepth
	unsigned int getReportDepth() const { return maxDepth; }

protected:
	static const unsigned int term_width=80; //!< number of character to word wrap the display
	
	unsigned int maxDepth; //!< default maximum recursion depth for reporting from activation

	ControlBase * root; //!< stores root node to begin recursion (this item is not displayed)
	
private:
	HelpControl(const HelpControl&); //!< don't call
	HelpControl operator=(const HelpControl&); //!< don't call
};

/*! @file
 * @brief Describes HelpControl, which recurses through the menu system and outputs the name and description of each item
 * @author ejt (Creator)
 */

#endif
