//-*-c++-*-
#ifndef INCLUDED_ToggleControl_h_
#define INCLUDED_ToggleControl_h_

#include "NullControl.h"
#include "Shared/ReferenceCounter.h"

//! a simple control for turning things on and off
/*! Can also be used for radio buttons - select one of a group
 *
 *  By using the #externalStore (setStore()/getStore()), you can use
 *  this instead of a ValueEditControl<bool> */
class ToggleControl : public NullControl {
public:
	//! a little class for managing the currently active ToggleControl to allow radio buttons
	class RadioGroup : public ReferenceCounter {
	public:
		//! constructor
		explicit RadioGroup(bool must_have_one=true) : ReferenceCounter(), cur(NULL), enforced(must_have_one) {}
		//! call this when a different ToggleControl wants to take over
		/*! implementation is trickier than you might think! */
		virtual void activate(ToggleControl * next) {
			bool was_enforced=enforced; //turn off enforcing so current control can turn off...
			enforced=false;
			if(cur!=NULL) {
				ToggleControl * tmp=cur;
				cur=NULL; // set this to NULL first to prevent infinite recursion
				tmp->setStatus(false);
			}
			enforced=was_enforced;
			cur=next;
		}
		//! returns the currently active control
		virtual ToggleControl * getActive() const { return cur; }
		//! can change the "must have one" setting (#enforced)
		virtual void setEnforced(bool must_have_one) { enforced=must_have_one; }
		//! returns the "must have one" setting (#enforced)
		virtual bool getEnforced() const { return enforced; }
	protected:
		ToggleControl * cur; //!< the currently active control, or NULL
		bool enforced; //!< if true, the current control cannot turn off, a new one must be activated
	private:
		RadioGroup(const RadioGroup& ); //!< don't call
		RadioGroup& operator=(const RadioGroup& ); //!< don't call
	};

	//!@name Constructors
	//!
	ToggleControl() : NullControl("[ ] "), rg(NULL), externalStore(NULL) {}
	ToggleControl(const std::string& n, RadioGroup * rad=NULL) : NullControl("[ ] "+n), rg(NULL), externalStore(NULL) { setRadioGroup(rad); }
	ToggleControl(const std::string& n, const std::string& d, RadioGroup * rad=NULL) : NullControl("[ ] "+n,d), rg(NULL), externalStore(NULL) { setRadioGroup(rad); }
	//@}
	~ToggleControl() { setRadioGroup(NULL); } //!< destructor

	virtual ControlBase * activate(MC_ID mcid, Socket * disp) { toggleStatus(); return NullControl::activate(mcid,disp); }
	virtual ControlBase * doSelect() { toggleStatus(); return NullControl::doSelect(); }

	virtual ControlBase& setName(const std::string& n) { name=std::string("[")+name.substr(1,1)+std::string("] ")+n; return *this; }

	//! calls setStatus() with the not of getStatus()
	virtual void toggleStatus() {
		setStatus(!getStatus());
	}

	//! if status is a ' ', it'll be replaced with @a c, otherwise a space.
	virtual void toggleStatus(char c) {
		if(getStatus())
			setStatus(false);
		else
			setStatus(c);
	}

	//! status will toggle between the two arguments; if current status is neither, the first is used
	virtual void toggleStatus(char c1,char c2) {
		if(getStatusChar()==c1)
			setStatus(c2);
		else
			setStatus(c1);
	}

	//! a true will put a 'X' for the status; false shows ' '
	virtual void setStatus(bool check) {
		setStatus(check?'X':' ');
	}
	
	//! pass the character to put as the status
	virtual void setStatus(char c) {
		if(rg!=NULL) {
			if(c==' ') {
				if(rg->getEnforced())
					return;
				rg->activate(NULL);
			} else {
				rg->activate(this);
			}
		}
		name[1]=c;
		if(externalStore!=NULL)
			*externalStore=getStatus();
	}

	//! returns true if there's a non-space as the status
	virtual bool getStatus() const {
		return getStatusChar()!=' ';
	}

	//! returns the current status char
	virtual char getStatusChar() const {
		return name[1];
	}

	//! removes itself from current RadioGroup, and adds itself to @a rad if non-NULL
	virtual void setRadioGroup(RadioGroup * rad) {
		if(rg!=NULL)
			rg->removeReference();
		if(rad!=NULL)
			rad->addReference();
		rg=rad;
		if(getStatus() && rg!=NULL)
			rg->activate(this);
	}

	//! returns the current RadioGroup
	virtual RadioGroup * getRadioGroup() const { return rg; }

	virtual void setStore(bool * s) { externalStore=s; *s=getStatus(); }  //!< sets #externalStore (and updates its current setting)
	virtual bool * getStore() const { return externalStore; } //!< returns #externalStore

protected:
	RadioGroup * rg; //!< pointer to an optional radio group to allow one-of-many selections
	
	bool * externalStore; //!< an external bit which should be kept in sync with current setting

private:
	ToggleControl(const ToggleControl& ); //!< don't call
	ToggleControl& operator=(const ToggleControl& ); //!< don't call
};

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
