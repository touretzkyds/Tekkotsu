//-*-c++-*-
#ifndef _LOCALIZATIONPARTICLEDATA_H_
#define _LOCALIZATIONPARTICLEDATA_H_

#include "Shared/Measures.h"    // coordinate_t; AngPi data member
#include "Shared/fmatSpatial.h"

#include "DualCoding/BaseData.h"    // superclass
#include "DualCoding/Point.h"       // Point data member
#include "DualCoding/ShapeTypes.h"  // localizationParticleDataType

#include "Localization/ShapeBasedParticleFilter.h"

namespace DualCoding {

class ShapeRoot;
class SketchSpace;
template<typename T> class Sketch;

//! Shape represention of a localization particle
/*! Rather than caching data locally, we store a pointer to the
  particle vector and the index of the particle. This way, each time
  the user refreshes the sketchGUI we will fetch the latest values
  for the particle.
*/
class LocalizationParticleData : public BaseData {
private:
  const ShapeBasedParticleFilter::particle_collection *particles;
  ShapeBasedParticleFilter::index_t index;

public:

  //! Constructor
  LocalizationParticleData(ShapeSpace &_space,
			   const ParticleFilter<LocalizationParticle>::particle_collection &_particles,
			   ParticleFilter<LocalizationParticle>::index_t _index);

  //! Copy constructor
  LocalizationParticleData(const LocalizationParticleData &other);

  static ShapeType_t getStaticType() { return localizationParticleDataType; }

  DATASTUFF_H(LocalizationParticleData);
  
  //! Centroid. (Virtual in BaseData.)
  virtual Point getCentroid() const;
  
  AngTwoPi getOrientation() const { return (*particles)[index].theta; }

  float getWeight() const { return (*particles)[index].weight;}

  virtual bool isMatchFor(const ShapeRoot& other) const;

  virtual bool updateParams(const ShapeRoot&, bool force=false);

  //! Print information about this shape. (Virtual in BaseData.)
  virtual void printParams() const;
  
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified) {}

  virtual void projectToGround(const fmat::Transform&, const PlaneEquation&) {}

  virtual unsigned short getDimension() const { return 2; }

  LocalizationParticleData& operator=(const LocalizationParticleData &other);

private:
  //! Render into a sketch space and return pointer. (Private.)
  virtual Sketch<bool>* render() const;


};

} // namespace

#endif
