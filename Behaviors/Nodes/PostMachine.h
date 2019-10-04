#ifndef _Included_PostMachine_h_
#define _Included_PostMachine_h_

#include "Behaviors/StateNode.h"

//! This node posts a completion event; it is used to construct dummy behaviors
class PostStateCompletion : public StateNode {
public:
  PostStateCompletion(const std::string &name="PostStateCompletion") : StateNode(name) {}
  virtual void doStart() { postStateCompletion(); }
};

//! This node can be included in an embedded state machine to make its parent post a completion event
class PostMachineCompletion : public StateNode {
public:
  PostMachineCompletion(const std::string &name="PostMachineCompletion") : StateNode(name) {}
  virtual void doStart() { postParentCompletion(); }
};

//! This node can be included in an embedded state machine to make its parent post a success event
class PostMachineSuccess : public StateNode {
public:
  PostMachineSuccess(const std::string &name="PostMachineSuccess") : StateNode(name) {}
  virtual void doStart() { postParentSuccess(); }
};

//! This node can be included in an embedded state machine to make its parent post a failure event
class PostMachineFailure : public StateNode {
public:
  PostMachineFailure(const std::string &name="PostMachineFailure") : StateNode(name) {}
  virtual void doStart() { postParentFailure(); }
};

#endif
