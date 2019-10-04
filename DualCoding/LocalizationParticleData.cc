//-*-c++-*-

#include <math.h>

#include "LocalizationParticleData.h"
#include "ShapeLocalizationParticle.h"
#include "Sketch.h"

using namespace std;

namespace DualCoding {

LocalizationParticleData::LocalizationParticleData(ShapeSpace& _space, 
						   const ParticleFilter<LocalizationParticle>::particle_collection &_particles,
						   ParticleFilter<LocalizationParticle>::index_t _index) :
  BaseData(_space, getStaticType()), particles(&_particles), index(_index) {
  obstacle = false;
}

LocalizationParticleData::LocalizationParticleData(const LocalizationParticleData &other) :
  BaseData(other), particles(other.particles), index(other.index) {}

DATASTUFF_CC(LocalizationParticleData);

Point LocalizationParticleData::getCentroid() const {
  return Point((*particles)[index].x, (*particles)[index].y, 0, space->getRefFrameType());
}

bool LocalizationParticleData::updateParams(const ShapeRoot&, bool) { return true; }

void LocalizationParticleData::printParams() const {
  cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
  cout << "  location{" << getCentroid().coordX() << ", " << getCentroid().coordY() << "}"
       << ", orientation = " << getOrientation() << endl;
}

bool LocalizationParticleData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<LocalizationParticleData>& other_particle = ShapeRootTypeConst(other,LocalizationParticleData);
  float dist = getCentroid().distanceFrom(other_particle->getCentroid());
  return dist < 20; // *** DST hack
}

LocalizationParticleData& LocalizationParticleData::operator=(const LocalizationParticleData &other) {
  if ( this != &other ) {
    BaseData::operator=(other);
    particles = other.particles;
    index = other.index;
  }
  return *this;
}

//! Render into a sketch space and return a pointer. (Private.)
Sketch<bool>* LocalizationParticleData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  fmat::Column<3> ctr(getCentroid().getCoords());
  SkS.applyTmat(ctr);
  int const cx = int(ctr[0]);
  int const cy = int(ctr[1]);
  Sketch<bool>& draw_result = 
    *new Sketch<bool>(SkS, "render("+getName()+")");
  draw_result = false;
  draw_result(cx,cy) = true;  
  return &draw_result;
}


} // namespace

