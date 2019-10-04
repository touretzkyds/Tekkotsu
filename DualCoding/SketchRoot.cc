//-*-c++-*-

#include "SketchRoot.h"
#include "Sketch.h"

namespace DualCoding {

SketchSpace& SketchRoot::rootGetSpace() const {
  const Sketch<bool>& faker = *reinterpret_cast<const Sketch<bool>*>(this);
  return *(faker.data->space);
}

const SketchDataRoot* SketchRoot::rootGetData() const {
  const Sketch<bool>* faker = reinterpret_cast<const Sketch<bool>*>(this);
  return reinterpret_cast<const SketchDataRoot*>(faker->data);
}

void SketchRoot::reset() {
  idCounter = 0;
}

int SketchRoot::idCounter = 0;

std::ostream& operator<<(std::ostream &os, const SketchRoot &r) {
  const SketchDataRoot* data = r.rootGetData();
  if ( data != NULL )
    os << "Sketch<" << SketchTypeNames[data->getType()] << ">(\""
       << data->getName() << "\",id=" << data->getId() << ")";
  else
    os << "Sketch()";
  return os;
}

} // namespace
