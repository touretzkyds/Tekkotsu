#include "PostureNode.h"

PostureNode::PostureNode(const std::string &nodename, const std::string &filename)
  : MCNode<PostureMC,defPostureNodeName,defPostureNodeDesc,true>(nodename), posture() {
  if ( filename.size() > 0 )
    posture.loadFile(filename.c_str());
}

void PostureNode::preStart() {
  MCNode<PostureMC,defPostureNodeName,defPostureNodeDesc,true>::preStart();
  getMC()->setAverage(posture,1);  // copy cached posture into the motion command
}

void PostureNode::loadFile(const std::string &filename) {
  posture.loadFile(filename.c_str());
  getMC()->setAverage(posture,1);
}

