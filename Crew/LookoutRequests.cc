#include "Crew/LookoutRequests.h"

namespace DualCoding {

const float LookoutScanRequest::defSpd = 0.0015f;

LookoutScanRequest::~LookoutScanRequest() {
  for (std::vector<Task*>::const_iterator it = tasks.begin();
       it != tasks.end(); it++)
    delete *it;
}

const char* const LookoutRequestBase::headMotionTypeNames[numHeadMotionTypes] = {
  "noMotion",
  "pointAt",
  "scan",
  "track",
  "search"
};


LookoutTrackRequest& LookoutTrackRequest::operator=(const LookoutTrackRequest &other) {
  *this = other;
  targetShape = other.targetShape;
  minBlobArea = other.minBlobArea;
  exitTest = other.exitTest;
  return *this;
}

} // namespace
