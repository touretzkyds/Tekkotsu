//-*-c++-*-
#ifndef INCLUDED_VisualRoutinesBehavior_h_
#define INCLUDED_VisualRoutinesBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "DualCoding/VRmixin.h"

namespace DualCoding {

/*! @brief Base class from which visual-routines based behaviors
 *  (which all share a common SketchSpace) inherit */
class VisualRoutinesBehavior : public BehaviorBase, public VRmixin {
public:
	virtual void start();
	virtual void stop();
	
protected:
	//! constructor, @a name is used as both instance name and class name
	explicit VisualRoutinesBehavior() : BehaviorBase(), VRmixin() {}
	
	//! constructor, @a name is used as both instance name and class name
	explicit VisualRoutinesBehavior(const std::string &name) : BehaviorBase(name), VRmixin() {}
	
	//! destructor, does nothing
	virtual ~VisualRoutinesBehavior(void) {}
	
private:
	// dummy functions to satisfy the compiler
	VisualRoutinesBehavior (const VisualRoutinesBehavior&);  //!< never call this
	VisualRoutinesBehavior& operator=(const VisualRoutinesBehavior&); //!< never call this
};

} // namespace

#endif
