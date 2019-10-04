//-*-c++-*-
#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK

#include "Behaviors/Nodes/WalkNode.h"
#include "DualCoding/VRmixin.h"
#include "Shared/MarkScope.h"
#include "Shared/Config.h"

bool WalkRequest::operator==(const WalkRequest& other) const {
  return ( x == other.x && y == other.y && a == other.a
	   && time == other.time && velocityMode == other.velocityMode
	   && filename == other.filename);
}

std::ostream& operator<<(std::ostream &os, const WalkRequest &req) {
  os << "WalkRequest[x=" << req.x << ", y=" << req.y << ", a=" << req.a
     << ", time=" << req.time 
     << ", velocityMode=" << (req.velocityMode ? "true" : "false") 
     << ", storedVelocities=" << (req.storedVelocities ? "true" : "false") 
     << "]" << std::endl;
  return os;
}

void WalkNode::preStart() {
  MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>::preStart();
  if ( const DataEvent<WalkRequest> *ev = dynamic_cast<const DataEvent<WalkRequest>*>(event) )
    req = ev->getData();
  if ( !req.filename.empty() )
    getMC()->loadFile(::config->motion.makePath(req.filename).c_str());
  // Must call the parent start() method first, because that
  // will call our doStart(), which may lead to user code
  // changing the displacement or velocity values; we'll check
  // for those changes afterwards.
  DualCoding::VRmixin::isWalkingFlag = true;
}

void WalkNode::postStart() {
  MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>::postStart();
  if ( req.storedVelocities ) {
    if ( !req.velocityMode ) {
      getMC()->setTargetDisplacement(req.x, req.y, req.a, req.time);
    } else if ( req.time<0 ) {
      getMC()->setTargetVelocity(req.x, req.y, req.a);
    } else {
      getMC()->setTargetVelocity(req.x, req.y, req.a, req.time);
    }
  }
}

void WalkNode::stop() {
  getMC()->zeroVelocities();
  DualCoding::VRmixin::isWalkingFlag = false;
  MCNode<WalkMC,defWalkNodeName,defWalkNodeDesc>::stop();
}

void WalkNode::setDisplacement(float xdist, float ydist, float adist, float time/*=0*/) {
  req = WalkRequest(xdist,ydist,adist,time,false,req.filename);
  if(isActive())
    getMC()->setTargetDisplacement(req.x, req.y, req.a, req.time);
}

void WalkNode::setVelocity(float xvel, float yvel, float avel) {
  req = WalkRequest(xvel,yvel,avel,req.filename);
  if(isActive())
    getMC()->setTargetVelocity(req.x, req.y, req.a);
}

void WalkNode::setVelocity(float xvel, float yvel, float avel, float time) {
  req = WalkRequest(xvel,yvel,avel,time,true,req.filename);
  if(isActive())
    getMC()->setTargetVelocity(req.x, req.y, req.a, req.time);
}

#endif
