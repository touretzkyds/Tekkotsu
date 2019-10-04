#ifndef _ShapeSensorModel_h_
#define _ShapeSensorModel_h_

#include "LocalizationParticle.h"

class PfRoot;
class PfLine;

namespace DualCoding {
	class ShapeSpace;
}

// Contents:  LocalShapeEvaluator, CameraShapeEvaluator, and
//   ShapeSensorModel (child of ParticleFilter<ParticleT>::SensorModel)

//! Provides evaluation of the match between local or camera and world given a candidate particle
/*! The reason for separating LocalShapeEvaluator and ShapeSensorModel?  Partly so the
 *  fairly lengthy evaluation code can go in the .cc file to avoid repeated recompilation, but also to
 *  allow inheritance (e.g. ShapeSLAMParticleEvaluator) as a clean way to extend the 
 *  evaluation code for particle sub-types. */
class LocalShapeEvaluator {
public:
  //! constructor, pass the local and world shape spaces, these will be used to initialize the appropriate particle-independent fields of the class
  LocalShapeEvaluator(DualCoding::ShapeSpace &localShS, DualCoding::ShapeSpace &worldShS);
  virtual ~LocalShapeEvaluator() {} //!< destructor
		
  //! the heart of the class, call with a particle, will adjust the weight
  void evaluate(LocalizationParticle& part);

  //! the real work is done here; shared with SLAM version
  void evaluateWorkhorse (LocalizationParticle& p, const unsigned int nLocals,
			  float particleViewX[], float particleViewY[], float particleViewX2[], float ParticleViewY2[],
			  int localMatches[], float localScores[]);

  //! update the particle weight after computing local match scores (and possibly additions/deletions if SLAM)
  void updateWeight(LocalizationParticle &p, int const localMatches[], float const localScores[]);
		
  std::vector<PfRoot*> localLms; //!< a vector of the landmarks in the local space
  std::vector<PfRoot*> worldLms; //!< a vector of landmarks in the world space
		
  static float const maxDist;  //!< maximum distance for a landmark to be useful in distance error calculation;  value should be < 1e10
  static float const stdevSq;  //!< controls how much weight is given to "near-misses"
		
  //! helper function which calculates the distance between a point and a line along a perpendicular
  static float distanceFromLine(coordinate_t x0, coordinate_t y0, PfLine &wline);
	
  //!< computes a (non-normalized) gaussian distribution
  /*! normalization isn't needed because the scale factor is constant across particles, and so
   *  doesn't matter for purposes of comparison between particles */
  inline float normpdf(float const distsq) { return std::exp(-distsq/stdevSq); }
};
	
class CameraShapeEvaluator {
public:
  //! constructor, pass the camera and world shape spaces, these will be used to initialize the appropriate particle-independent fields of the class
  CameraShapeEvaluator(DualCoding::ShapeSpace &camShS, DualCoding::ShapeSpace &worldShS):
    pRandom(0.2f), alpha(1), xvar(3000), yvar(1000), cShS(camShS), wShS(worldShS)
  {}
		
  virtual ~CameraShapeEvaluator(){}
		
  // compute the likelihood of a given sensor reading
  virtual void computeLikelihood(LocalizationParticle& particle);
		
  float pRandom;                        //!< Probability sensor reading is random noise
  float alpha;                          //!< Attentuate weight of each individual marker match
		
  float xvar;                           //!< variance of sensor reading in x direction (image)
  float yvar;                           //!< variance of sensor reading in y direction (image)
  DualCoding::ShapeSpace &cShS;			//!< Camera shape space
  DualCoding::ShapeSpace &wShS;			//!< World shape space

  //!< computes a (non-normalized) gaussian distribution
  /*! normalization isn't needed because the scale factor is constant across particles, and so
   *  doesn't matter for purposes of comparison between particles */
  inline float normpdf(float const dist, float variance) { return std::exp(-dist*dist/variance); }
};

//================ ShapeSensorModel ================
//! this wraps the ParticleShapeEvaluator in a ParticleFilter::SensorModel so it can be used by the particle filter
/*! The reason for separating ParticleShapeEvaluator and ShapeSensorModel?  Partly so the
 *  fairly length evaluation code can go in the .cc file to avoid repeated recompilation, but also to
 *  allow inheritance (e.g. SLAMShapesParticleEvaluator) as a clean way to extend the 
 *  evaluation code for particle sub-types.  Ideally, I'd like to combine these classes. */

template<typename ParticleT>
class ShapeSensorModel : public ParticleFilter<ParticleT>::SensorModel {
public:
  typedef typename ParticleFilter<ParticleT>::SensorModel::index_t index_t; //!< convenience typedef
  typedef typename ParticleFilter<ParticleT>::SensorModel::particle_collection particle_collection; //!< convenience typedef
  typedef typename ParticleFilter<ParticleT>::SensorModel::particle_type particle_type; //!< convenience typedef
	
  //! constructor, the standard deviation on matches defaults to 60, but you can always reassign #stdevSq directly
  ShapeSensorModel(DualCoding::ShapeSpace &camShS, DualCoding::ShapeSpace &localShS, DualCoding::ShapeSpace &worldShS) :
    cShS(camShS), lShS(localShS), wShS(worldShS)
  {}
	
  //! Applies the ParticleShapeEvaluator across a collection of particles
  virtual void evaluate(particle_collection& particles, particle_type &estimate) {
    // We could handle some shapes in camera space and others in local space, but for now we'll handle everything in local space.
    updateFromLocal(particles, estimate);
  }
	
  virtual void updateFromLocal(particle_collection& particles, particle_type &estimate) {
    std::cout << "Particle filter updateFromLocal(): old est="
              << estimate.x << "," << estimate.y << " hdg=" << estimate.theta << " wt=" << estimate.weight
              << std::endl;
    LocalShapeEvaluator localEval(lShS,wShS);
    float bestWeight = -FLT_MAX;
    typename particle_collection::size_type bestIndex = 0;
    double tx=0, ty=0, tcos=0, tsin=0;
    double totalWeight = 0;
    for(typename particle_collection::size_type p=0; p<particles.size(); ++p) {
      localEval.evaluate(particles[p]);
      if (particles[p].weight > bestWeight) {
				bestWeight = particles[p].weight;
				bestIndex = p;
      }
      // double w = exp((double)particles[p].weight);
      //
      // Using exponential weights (see line above) causes almost all
      // the weight to be put on the best particle; the rest have very
      // little influence.  And the "best" particle can be an outlier.
      // So instead we're going to use the log weights, except that
      // smaller weight values should have GREATER influence, so we'll
      // take the negative reciprocal.
      double w = -1 / particles[p].weight;  // see explanation above
      totalWeight += w;
      tx += particles[p].x * w;
      ty += particles[p].y * w;
      tcos += cos(particles[p].theta) * w;
      tsin += sin(particles[p].theta) * w;
    }
    if ( totalWeight > 1e-200 ) {
      estimate.x = tx / totalWeight;
      estimate.y = ty / totalWeight;
      estimate.theta = atan2(tsin/totalWeight, tcos/totalWeight);
    } else { // avoid divide-by-zero error: can't use totalWeight
      estimate.x = particles[bestIndex].x;
      estimate.y = particles[bestIndex].y;
      estimate.theta = particles[bestIndex].theta;
    }
		estimate.weight = bestWeight;
    std::cout << "Particle filter: new totalWeight=" << totalWeight << " bestIndex=" << bestIndex << " new est="
              << estimate.x << "," << estimate.y << " hdg=" << estimate.theta << " wt=" << estimate.weight
              << std::endl;
  }

  virtual void updateFromCamera(particle_collection& particles, particle_type& estimate) {
    CameraShapeEvaluator cameraEval(cShS,wShS);
    for(typename particle_collection::size_type p=0; p<particles.size(); ++p) {
      // resample should already do this....
      // but it doesn't seem to work
      //      particles[p].weight = 0;
			
      // evaluate weight for particle p
      cameraEval.computeLikelihood(particles[p]);
    }
  }
	
  DualCoding::ShapeSpace& getcamShS() const { return lShS; }
  DualCoding::ShapeSpace& getLocalShS() const { return lShS; }
  DualCoding::ShapeSpace& getWorldShS() const { return wShS; }
	
protected:
  DualCoding::ShapeSpace &cShS;			//!< Camera shape space
  DualCoding::ShapeSpace &lShS;			//!< Local shape space
  DualCoding::ShapeSpace &wShS;			//!< World shape space
};

#endif
