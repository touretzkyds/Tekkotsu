#ifndef INCLUDED_LocalizationParticle_h
#define INCLUDED_LocalizationParticle_h

#include "Shared/ParticleFilter.h"
#include "Shared/Measures.h"
#include "Shared/zignor.h"
#include <iostream>
#include <cmath>

template<typename ParticleT> class LocalizationParticleDistributionPolicy;

//! Each Particle represents a hypothesis about the robot's 2D position and heading
class LocalizationParticle : public ParticleBase<LocalizationParticle> {
public:
  float x; //!< X position of robot in the world
  float y; //!< Y position of robot in the world
  AngTwoPi theta; //!< Orientation of robot in world
	
  //! defines a default DistributionPolicy for the particle type
  typedef LocalizationParticleDistributionPolicy<LocalizationParticle> DistributionPolicy;
	
  //! constructor
  LocalizationParticle() : ParticleBase<LocalizationParticle>(), x(0), y(0), theta(0) {}
	
  //! constructor, allows you to define the particle's position
  LocalizationParticle(float xp, float yp, AngTwoPi tp) : ParticleBase<LocalizationParticle>(), x(xp), y(yp), theta(tp) {}
	
  //! returns a straightforward sum squared error of each of the fields
  float sumSqErr(const LocalizationParticle& lp) const {
    //const LocalizationParticle& lp = static_cast<const LocalizationParticle&>(p);
    float dx=x-lp.x;
    float dy=y-lp.y;
    float dt=theta-lp.theta;
    return dx*dx+dy*dy+dt*dt;
  }

};

//! Provides parameters and methods for randomizing and tweaking LocalizationParticles
template<typename ParticleT>
class LocalizationParticleDistributionPolicy : public ParticleFilter<ParticleT>::DistributionPolicy {
public:
  typedef ParticleT particle_type;  //!< just for convenience
  typedef typename ParticleFilter<ParticleT>::index_t index_t; //!< just for convenience
	
  float mapMinX; //!< specifies the low end of x coordinates during randomize()
  float mapWidth; //!< along with #mapMinX, specifies the range of x coordinates to be used during randomize()
  float mapMinY; //!< specifies the low end of y coordinates during randomize()
  float mapHeight; //!< along with #mapMinY, specifies the range of y coordinates to be used during randomize()
  float positionVariance; //!< controls how much the x and y parameters will be modified during jiggle()
  float orientationVariance; //!< controls how much the orientation (theta) parameter will be modified during jiggle()
	
  //! constructor -- by default, coordinates will range from -1000 to 1000 for x and y, with variance of 50 and 0.18 for position and orientation
  LocalizationParticleDistributionPolicy()
    : mapMinX(-1000), mapWidth(2000), mapMinY(-1000), mapHeight(2000),
      positionVariance(50), orientationVariance(0.18f)
  {}
	
  virtual void randomize(particle_type* begin, index_t num) {
    particle_type* end=&begin[num]; 
    while(begin!=end) { 
      begin->x = float(rand())/RAND_MAX * mapWidth + mapMinX;
      begin->y = float(rand())/RAND_MAX * mapHeight + mapMinY;
      begin->theta = direction_t(rand())/RAND_MAX * 2 * direction_t(M_PI);
      //(begin++)->randomize();
      ++begin;
    }
  }
  virtual void jiggle(float var, particle_type* begin, index_t num) {
    if(var==0)
      return;
    particle_type* end=&begin[num]; 
    while(begin!=end) {
      begin->x+=(float)DRanNormalZig32()*positionVariance*var;
      begin->y+=(float)DRanNormalZig32()*positionVariance*var;
      begin->theta+=(float)DRanNormalZig32()*orientationVariance*var;
      //(begin++)->jiggle(var);
      ++begin;
    }
  }
};

//! dump a particle's state
inline std::ostream& operator << (std::ostream& os, const LocalizationParticle &p) {
  os << "Particle(p=" << p.weight
     << ", dx=" << p.x
     << ", dy=" << p.y
     << ", th=" << p.theta
     << ")";
  return os;
}



/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
