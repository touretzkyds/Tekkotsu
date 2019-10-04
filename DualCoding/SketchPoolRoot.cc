//-*-c++-*-

#include "SketchSpace.h"

using namespace std;

namespace DualCoding {

SketchPoolRoot::~SketchPoolRoot() {}

int SketchPoolRoot::getRefreshCounter() const {
  return space->getRefreshCounter();
}

const std::string& SketchPoolRoot::getSpaceName() const {
  return space->name;
}

} // namespace

