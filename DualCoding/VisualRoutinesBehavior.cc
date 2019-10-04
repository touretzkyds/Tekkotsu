#include "VisualRoutinesBehavior.h"

namespace DualCoding {

//----------------------------------------------------------------

void VisualRoutinesBehavior::start() {
  if ( !started ) {
    startCrew();
    BehaviorBase::start();
  }
}

void VisualRoutinesBehavior::stop() {
  if ( started ) {
    BehaviorBase::stop();
    stopCrew();
  }
}

} // namespace
