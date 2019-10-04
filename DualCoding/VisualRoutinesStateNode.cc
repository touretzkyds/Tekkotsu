#include "VisualRoutinesStateNode.h"

namespace DualCoding {

//----------------------------------------------------------------

void VisualRoutinesStateNode::start() {
  if ( !started ) {
    startCrew();
    StateNode::start();
  }
}

void VisualRoutinesStateNode::stop() {
  if ( started ) {
    StateNode::stop();
    stopCrew();
  }
}

} // namespace
