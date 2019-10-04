//-*-c++-*-
#ifndef INCLUDED_PostureNode_h_
#define INCLUDED_PostureNode_h_

#include "MCNode.h"
#include "Events/EventRouter.h"
#include "Motion/PostureMC.h"
#include "Shared/MarkScope.h"

//!default name for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defPostureNodeName[];
//!default description for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defPostureNodeDesc[];

//! A simple StateNode that executes a PostureMC motion command
/*! Caches the posture file in a private PostureEngine because the
 *  motion command might be shared with other functions that are
 *  using it for other purposes */
class PostureNode : public MCNode<PostureMC,defPostureNodeName,defPostureNodeDesc,true> {
public:
	
  //! Constructor: takes optional instance name and filename.
  /*! Caches the posture file in a private PostureEngine because the
    motion command might be shared with other functions that are
    using it for other purposes */
  PostureNode(const std::string &nodename=defPostureNodeName, const std::string &filename=std::string());
	
  virtual void preStart();

  //! loads the specified file into #posture, note this @em doesn't affect the current PostureMC, just the cached one which will be loaded into it on next activation.  See getPosture(), getMC_ID()
  virtual void loadFile(const std::string &filename);
	
  //! accessor for #posture, note this @em doesn't affect the current PostureMC, just the cached one which will be loaded into it on next activation.  See getMC_ID()
  virtual PostureEngine& getPosture() { return posture; }
  //! accessor for #posture, note this @em doesn't return the current PostureMC, just the cached one which will be loaded into it on next activation.  See getMC_ID()
  virtual const PostureEngine& getPosture() const { return posture; }
	
protected:
  //! The internal cache of joint positions, copied to the motion command when activated.
  /*! This allows the motion command to be shared by other nodes/behaviors, which might modify
   *  the posture on an ongoing basis. */
  PostureEngine posture;
};

/*! @file
 * @brief Defines PostureNode, a simple StateNode that runs a PostureMC motion command
 * @author dst
 */

#endif
