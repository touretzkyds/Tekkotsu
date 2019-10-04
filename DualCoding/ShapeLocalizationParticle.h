//-*-c++-*-
#ifndef _SHAPELOCALIZATIONPARTICLE_H_
#define _SHAPELOCALIZATIONPARTICLE_H_

#include "DualCoding/ShapeRoot.h"
#include "LocalizationParticleData.h"

namespace DualCoding {

template<>
class Shape<LocalizationParticleData> : public ShapeRoot {
public:
  SHAPESTUFF_H(LocalizationParticleData);   // defined in ShapeRoot.h

};

} // namespace

#endif
