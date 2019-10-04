//-*-c++-*-
#ifndef INCLUDED_VisualRoutinesStateNode_h_
#define INCLUDED_VisualRoutinesStateNode_h_

#include "Behaviors/StateNode.h"
#include "DualCoding/VRmixin.h"

namespace DualCoding {

  /*! @brief Base class from which visual-routines based state nodes
   *  (which all share a common SketchSpace) inherit */
  class VisualRoutinesStateNode : public StateNode, public VRmixin {
  public:
    virtual void start();
    virtual void stop();
	
  protected:
    //! constructor, class name from typeid is used as instance name
    explicit VisualRoutinesStateNode() : StateNode(), VRmixin() {}

    //! constructor, @a name is used as both instance name and class name
    explicit VisualRoutinesStateNode(const std::string &name) : StateNode(name), VRmixin() {}
	
  private:
    // dummy functions to satisfy the compiler
    VisualRoutinesStateNode (const VisualRoutinesStateNode&);  //!< never call this
    VisualRoutinesStateNode& operator=(const VisualRoutinesStateNode&); //!< never call this
  };

} // namespace

#endif
