#ifndef INCLUDED_ParticleFilter_h
#define INCLUDED_ParticleFilter_h

#include <vector>
#include <algorithm>
#include <iostream>
#include <cfloat>
#include <cmath>

//! Provides a common base class for particles used by the ParticleFilter
/*! Each particle represents a hypothesis regarding a position in state space.
 *  The state space being modeled is defined by the fields tracked by
 *  the particles, a sensor model which evaluates the 'health' of each particle
 *  based on incoming information from the world, and a motion model
 *  which updates particles based on the robot's own changes to the world.
 *
 *  For a common example, see the LocalizationParticle for 
 *  tracking a robot's position and orientation in a 2D world.
 *
 *  The default LowVarianceResamplingPolicy has two requirements for
 *  particles.  One requirement is that all particle implementations must
 *  define a 'DistributionPolicy' (usually via typedef within your class) so
 *  that the resampler can create randomly generated particles and
 *  modify existing ones. (see ParticleFilter::DistributionPolicy)
 *
 *  The second requirement is that all particles provide a public 'weight'
 *  field so that particles can be compared.  The recommended way to do
 *  this is to inherit from this ParticleBase base class.  However, since
 *  templates are used to specify the particle type to the particle filter,
 *  you can use an unaffiliated class as long as it provides a weight member.
 *  (However, inheritance is still recommended so you'll automatically pick up
 *  any changes or new requirements made to this base class.)
 *
 *  The final requirement of the ParticleFilter itself is to provide a
 *  sumSqErr() function so that a confidence interval can be computed.
 *  However, the meaning of the value returned by this function is entirely
 *  up to you.  The base class provides a prototype for the function, but
 *  its implementation is abstract.*/

template<class ParticleT> class ParticleBase {
 public:
  //! constructor
  ParticleBase() : weight(0) {}
    //! destructor
    virtual ~ParticleBase() {};
	
    //! returns the sum squared error between this particle and @a p
    /*! This is only used to compute the confidence of the particle filter,
     *  you may want to weight some dimensions differently if they tend
     *  to have smaller values or are more important.  How you interpret
     *  ParticleFilter::confidence() depends on how this function is implemented. */
    virtual float sumSqErr(const ParticleT& p) const=0;
	
    //! indicates the 'health' of the particle -- the bigger the value, the better this particle
    /*! Generally weights are indicative of probability, but are often unnormalized
     *  since the normalization factor is constant across particles and thus doesn't
     *  affect matters of relative comparison.
     *
     *  Further, weights tend to be very small as the accumulation of a number of
     *  sensor values tend to be each somewhat unlikely, and taken together
     *  the particle's weight shrinks exponentially.  Thus it useful to work in
     *  log space to avoid numeric underflow on the range of a floating point value.
     *  This also has the advantage of transforming multiplication operations to
     *  slightly quicker addition operations.  The default LowVarianceResamplingPolicy
     *  has a logWeights member so you can indicate whether weight values
     *  should be interpreted logarithmically (i.e. negative values)
     *  or linearly (e.g. positive (and generally very small) values). (default is logarithmic) */
    float weight;
};

//! Implements a particle filter with support for a variety of applications through the usage of arbitrary combination of a variety of models and policies
/*! The particle type is passed as a template parameter, which provides the implementation
 *  advantage of being able to directly store arrays of particles as contiguous blocks in memory.  This allows
 *  better cache coherency and enables platform-specific acceleration tricks, such as SIMD calls.
 *
 *  There are a number of embedded classes which together form the implementation of the
 *  particle filter.  The advantage of this architecture is that you can mix and match any
 *  combination of modules to get the features needed for your application.
 *  - SensorModel: pass one of these to updateSensors in order to evaluate the particles
 *    as new information is discovered.  You may have several different sensors at the same
 *    time, simply create a model for each type of sensor, and pass it to the filter when updated.
 *  - MotionModel: modifies particle states based on the expected outcome of
 *    any controls you might have over the system.  See DeadReckoningBehavior for an example.
 *    Generally, you will install one motion model, and this model will be given a opportunity
 *    to update expected particle state before each sensor update.  (unless you pass 'false'
 *    to updateSensors()).  MotionModel can be NULL if you have no control over the system.
 *  - DistributionPolicy: defines how to generate random particles and "jiggle" existing ones.
 *    The default DistributionPolicy is usually specified via a typedef in the particle itself, and
 *    is stored as a property of the ResamplingPolicy (next item) since that is what will use it.
 *  - ResamplingPolicy: Following a sensor update, you may wish to re-evaluate the particles
 *    in use, making copies of the "good" particles, and dropping those which are not matching
 *    sensor values.  If you receive a group of different sensor readings, you may want to
 *    hold off on resampling until they have all been applied for better evaluation of the particles
 *    before selecting which to replicate.  Similarly, if your sensors are noisy, you may want to
 *    take several readings before allowing resampling so you don't kill off all the "good" particles
 *    based on a bad reading.  Pass 'false' to updateSensors() or use the delayCount parameter of
 *    LowVarianceResamplingPolicy.  The resampling policy can be 'NULL' if you never want to
 *    resample, but it defaults to an instance of LowVarianceResamlingPolicy.
 *
 *  Generally, preparing to use a particle filter requires these prerequisites:
 *  - write your particle class and its associated distribution policy
 *    (see LocalizationParticle and LocalizationParticleDistributionPolicy)
 *  - write a sensor model to evaluate particles using sensors you'll have available
 *    (see DualCoding::ShapeSensorModel)
 *  - write a motion model if you have any knowledge of modifications to the state
 *    (see HolonomicMotionModel and DeadReckoningBehavior)
 *
 *  Once these are available, usage goes something like this:
 *  -# create particle filter, optionally passing motion model and/or resampling policy
 *  -# customize parameters for resampling and distribution policies
 *  -# while active (note these are all "as needed", in no particular order):
 *     - update motion model whenever there's a change in controls (e.g. call setVelocity() on
 *        a HolonomicMotionModel)
 *     - create/pass SensorModel(s) for any measurements obtained (e.g. call updateSensors()
 *        on the particle filter)
 *     - call getEstimate() on the particle filter to obtain current state estimate
 *
 *  Remember that the particle filter takes responsibility for deallocating all policies and the
 *  motion model when they are removed.  Do not attempt to reuse them between particle
 *  filters. SensorModels are the only exception -- they are @e not retained between calls
 *  to updateSensors, so you can reuse them.
 */

template<typename ParticleT>
class ParticleFilter {
 public:
  typedef ParticleT particle_type; //!< redefinition here allows reference to the particle type even if the template parameter may be abstracted away due to a typedef
  typedef typename std::vector<particle_type> particle_collection; //!< the collection type we'll be using to store the particles
  typedef typename particle_collection::size_type index_t; //!< index type for refering to particles within the collection
	
  //! A sensor model is used to update particle weights to account based on each particle's ability to explain observations taken from the system
  class SensorModel {
  public:
    typedef ParticleT particle_type; //!< redefinition here allows reference to the particle type even if the template parameter may be abstracted away due to a typedef
    typedef typename std::vector<particle_type> particle_collection; //!< the collection type we'll be using to store the particles
    typedef typename particle_collection::size_type index_t; //!< index type for refering to particles within the collection
    virtual ~SensorModel() {} //!< destructor (no-op for base class)
		
    //! once passed to the particle filter's updateSensors(), this will be called to allow the sensor model to update the 'weight' member of each particle
    /*! @param particles the current set of particles to be evaluated
     *  @param[out] estimate the weighted mean of the particle values
     *
     *  Remember to @e update each particle's weight, don't overwrite it.  In other words,
     *  you want to combine (e.g. add or multiply) the weight from the current sensor evaluation
     *  with the weight currently stored in each particle, don't just replace it.  This is because
     *  there may be several sensor updates between resamplings so that particles can be
     *  more accurately evaluated. */
    virtual void evaluate(particle_collection& particles, particle_type& estimate)=0;
  };

  //! A motion model is retained by the particle filter to query before evaluating sensor measurements so all known influences are accounted for before testing the particles
  /*! It's a good idea to apply noise to the motion model depending on the precision of the model.
   *  This allows the particle cluster to spread over time until new information is obtained to
   *  to evaluate how accurate the motion really was, at which point resampling will collapse
   *  the cluster back down again. */
  class MotionModel {
  public:
    typedef ParticleT particle_type; //!< redefinition here allows reference to the particle type even if the template parameter may be abstracted away due to a typedef
    typedef typename std::vector<particle_type> particle_collection; //!< the collection type we'll be using to store the particles
    typedef typename particle_collection::size_type index_t; //!< index type for refering to particles within the collection
    virtual ~MotionModel() {} //!< destructor
		
    //! The particle filter will call these when it wants to update particle state values to account for known influences
    /*! See the class notes regarding the usefulness of adding noise to the control parameters (or their effects) */
    virtual void updateMotion(particle_collection& particles, particle_type &estimate)=0;
  };

  //! A distribution policy provides the ability to randomize ("redistribute") or tweak the values of a group of particles
  /*! Unlike the other particle filter helper classes, the functions for the distribution policy
   *  operate on a subset of the particles at a time.
   *  You may wonder why the randomize() and jiggle() functions aren't simply made methods
   *  of the ParticleBase class.  The main reason is that these functions may need additional 
   *  parameters, such as specification of how large an area to distribute over, and these
   *  parameters are static across particles.  However, if they were actually static members
   *  of the particle class, then the values would be shared by all particle filters.  By making
   *  a separate class to hold the parameters and apply the one-to-many relationship, you
   *  can have multiple particle filters with the same type of particle, and each filter can have
   *  different parameter values controlling distribution of its particles.
   *
   *  Note that the DistributionPolicy is actually a property of the resampling policy, not
   *  directly of the particle filter itself. */
  class DistributionPolicy {
  public:
    typedef ParticleT particle_type; //!< redefinition here allows reference to the particle type even if the template parameter may be abstracted away due to a typedef
    typedef typename std::vector<particle_type> particle_collection; //!< the collection type we'll be using to store the particles
    typedef typename particle_collection::size_type index_t; //!< index type for refering to particles within the collection
    virtual ~DistributionPolicy() {} //!< destructor
		
    //! This should redistribute the particles over a large area, independently of the particle's current value
    /*! Randomization occurs whenever the particle filter doesn't have any usable particles for
     *  replication, either because the particle filter has just been created and doesn't have any
     *  information yet, or because new sensor readings have invalidated all of the current particles. */
    virtual void randomize(particle_type* begin, index_t num)=0;// { particle_type* end=begin+num; while(begin!=end) (begin++)->randomize(); }
		
    //! This should slightly modify the particles' state values
    /*! @param var indicates the scale of the variance desired -- multiply whatever variance you use for modifying each state parameter by this value
     *  @param begin the first particle in the array
     *  @param num the number of particles to apply the operation to
     *
     *  This function is called on particles which have been replicated from an existing
     *  particle to explore the space around that particle.  The more accurate your
     *  sensors and particle evaluation, the smaller the jiggle variance can be. */
    virtual void jiggle(float var, particle_type* begin, index_t num)=0;// { particle_type* end=begin+num; while(begin!=end) (begin++)->jiggle(var); }
  };
	
  //! The resampling policy focuses the particle filter on those particles which are performing well, and dropping those which are poorly rated
  /*! Resampling should replicate particles proportionally to how well their weights compare
   *  to other particles in the filter.  The process is similar to a genetic algorithm.
   *  This process typically does not vary between applications,
   *  so you will probably want to use the supplied LowVarianceResamplingPolicy, and
   *  simply tweak parameters as needed.
   *
   *  The ResamplingPolicy interface includes a DistributionPolicy so particles can be
   *  randomly generated or modified in an abstract manner. */
  class ResamplingPolicy {
  public:
    typedef ParticleT particle_type; //!< redefinition here allows reference to the particle type even if the template parameter may be abstracted away due to a typedef
    typedef typename std::vector<particle_type> particle_collection; //!< the collection type we'll be using to store the particles
    typedef typename particle_collection::size_type index_t; //!< index type for refering to particles within the collection
		
    //! constructor, creates a DistributionPolicy based on the particle_type's own DistributionPolicy typedef
    ResamplingPolicy() : dist(new typename particle_type::DistributionPolicy) {}
      //! constructor, pass your own custom distribution policy (responsibility for deallocation is assumed by the ResamplingPolicy)
      explicit ResamplingPolicy(DistributionPolicy * distPolicy) : dist(distPolicy) {}
	//! destructor
	virtual ~ResamplingPolicy() { delete dist; };
	//! the particle filter will call resample() when the particles have been evaluated and are ready to be selected
	virtual void resample(particle_collection& particles)=0;
	//! replaces #dist with a new distribution policy.  If you pass NULL, #dist will be reset to the particle_type's default distribution policy, as specified by a 'DistributionPolicy' typedef within the particle class
	virtual void setDistributionPolicy(DistributionPolicy * distPolicy) {
	  delete dist;
	  dist = (distPolicy!=NULL) ? distPolicy : new typename particle_type::DistributionPolicy;
	}
	//! returns the currently active distribution policy (#dist)
	virtual DistributionPolicy& getDistributionPolicy() const { return *dist; }
  protected:
	//! a pointer to the current distribution policy, which cannot be NULL
	DistributionPolicy * dist;
  private:
	ResamplingPolicy(const ResamplingPolicy& rp); //!< copy unsupported
	ResamplingPolicy& operator=(const ResamplingPolicy& rp); //!< assignment unsupported
  };
	
  //! This class provides a generic, default ResamplingPolicy.  It is based on the low variance resampling policy algorithm found in "Probabilistic Robotics" by Sebastian Thrun, Wolfram Burgard, Dieter Fox
  /*! This class is called "low variance" because it will maintain particle modalities in the face of
   *  uniform weighting.  This means that if resamples are triggered when no new information
   *  is available, every particle is resampled for the next generation.  This prevents the eventual
   *  convergence of particle clusters over time.
   *
   *  However, this implementation provides a #varianceScale parameter for adding variance
   *  to the particle's state on each generation, which can be useful for more rapidly exploring
   *  the state space around a "winning" particle.  Ideally, it is better to use a very low resampling
   *  variance, and rely on noise in the motion model and a broad probability distribution in
   *  the sensor model to allow particles to spread.  #varianceScale is really a crutch to manage
   *  an overconfident sensor model (one which weights "correct" particles with sharply higher values).
   *  
   *  The #varianceScale parameter defaults to a negative value, which indicates the
   *  resampling variance will be scaled with particle weight to provide broader sampling when
   *  particle weights are poor, and tighten sampling when particles are tracking accurately.  This
   *  requires setting a value for #minAcceptableWeight, described next.
   *
   *  The other trick this implementation provides is specification of a minimum acceptable
   *  particle weight (#minAcceptableWeight).  If the best particle's weight is below this value,
   *  new, randomly generated particles will be created, up to #maxRedistribute percent of
   *  the particles on a round of resampling.  This handles situations where the actual state
   *  has somehow jumped out of the region being sampled by the particles, and the filter is "lost".
   *  Without some further information (i.e. fixing the MotionModel to predict the "kidnapping"),
   *  this can provide automatic re-acquisition of the position in state space (at the cost of
   *  spawning new modalities).
   *
   *  Finally, #resampleDelay is provided to limit actual resampling to one in every #resampleDelay
   *  attempts.  This allows you to only resample at a lower frequency than the sensor model,
   *  without having to manually track the number of sensor samples and pass a parameter to
   *  the ParticleFilter::updateSensors() to limit resampling yourself.  The reason you would
   *  want to limit the resampling frequency is to better evaluate the particles before selecting
   *  them for replication or pruning -- if your sensors are noisy and you resample too often,
   *  bad values will kill off good particles on a regular basis, causing the filter to continually be "lost".
   *
   *  This policy can interpret weights in either "log space" or "linear space".  It defaults to "log space",
   *  but if your sensor model is providing linear weights, set #logWeights to false.
   */
  class LowVarianceResamplingPolicy : public ResamplingPolicy {
  public:
    //! constructor
    LowVarianceResamplingPolicy()
      : varianceScale(-2), maxRedistribute(1/2.f), minAcceptableWeight(-30),
      logWeights(true), resampleDelay(0), newParticles(), resampleCount(0)
      {}
      virtual void resample(particle_collection& particles);
		
      //! returns true if the next call to resample will trigger a "real" resampling (is #resampleCount greater than #resampleDelay?)
      bool nextResampleIsFull() { return resampleCount>=resampleDelay; }
		
      //! If non-negative, passed to the DistributionPolicy's jiggle() for replicated particles; otherwise an "automatic" value is used which inversely scales with the best particle weight
      /*! A negative value is still used to control the maximum magnitude of the resampling variance.
       *  It's better to keep this small (or zero) and rely on the sensor and motion model's noise parameters */
      float varianceScale;
      //! A percentage (0-1) of the particles which can be randomly re-distributed on a single resampling if the best particle's weight is below #minAcceptableWeight
      float maxRedistribute;
      //! The lowest weight per resample attempt to consider "acceptable"
      /*! This is scaled by resampleDelay when being compared to particle weights, so that
       *  you don't have to adjust this parameter when you increase resampleDelay.
       *  As the best particle weight drops below this value, more particles will be randomly
       *  redistributed, up to #maxRedistribute. */
      float minAcceptableWeight; 
      //! This controls the interpretation of particle weights.  If true, they are interpreted as "log space", otherwise "linear space"
      bool logWeights;
      //! This indicates how many resampling attempts should be skipped before actually doing it.  See class notes for rationale.
      unsigned int resampleDelay;
  protected:
      particle_collection newParticles; //!< temporary scratch space as particles are created
      unsigned int resampleCount; //!< the number of resampling attempts which have occurred.
  };
	
	
  //! Constructor for the particle filter, specify number of particles and optionally pass a motion model and resampling policy
  /*! The particle filter assumes responsibility for eventual deallocation of the motion model and resampling policy */
  explicit ParticleFilter(unsigned int numParticles, MotionModel* mm=NULL, ResamplingPolicy* rs=new LowVarianceResamplingPolicy)
    : particles(numParticles), estimate(), variance(), varianceValid(false), motion(mm), resampler(rs), hasEvaluation(false)
    {
      if(numParticles>0)
	resetFilter(particles.front().weight);
    }
	
    //! Destructor
    virtual ~ParticleFilter() { delete motion; delete resampler; }
	
    //! Returns the current motion model (#motion)
    virtual MotionModel * getMotionModel() const { return motion; }
    //! Reassigns the motion model, deleting the old one; motion model can be NULL
    virtual void installMotionModel(MotionModel* mm) { delete motion; motion=mm; }
	
    //! Returns the current resampling policy (#resampler)
    virtual ResamplingPolicy * getResamplingPolicy() const { return resampler; }
    //! Reassigns the resampling policy, deleting the old one; resampling policy can be NULL (although not recommended...)
    virtual void installResamplingPolicy(ResamplingPolicy* rs) { delete resampler; resampler=rs; }
	
    //! Sets the resampling policy's resampleDelay, which controls how many sensor updates to process before resampling the particles; policy must be a LowVarianceResamplingPolicy
    virtual void setResampleDelay(unsigned int d) {
      LowVarianceResamplingPolicy* p = dynamic_cast<LowVarianceResamplingPolicy*>(getResamplingPolicy());
      if ( p )
	p->resampleDelay = d;
      else
	std::cout << "Warning: setResampleDelay found getResamplingPolicy() returns wrong type policy; resampleDelay not set." << std::endl;
    }
	
    //! Sets the resampling policy's minimum acceptable weight for a particle; policy must be a LowVarianceResamplingPolicy
    virtual void setMinAcceptableWeight(float w) {
      LowVarianceResamplingPolicy* p = dynamic_cast<LowVarianceResamplingPolicy*>(getResamplingPolicy());
      if ( p )
	p->minAcceptableWeight = w;
      else
	std::cout << "Warning: setMinAcceptableWeight found getResamplingPolicy() returns wrong type policy; minAcceptableWeight not set." << std::endl;
    }
	
    //! If getResamplingPolicy() returns a LowVarianceResamplingPolicy instance, this will set LowVarianceResamplingPolicy::maxRedistribute; otherwise will display a warning
    virtual void setMaxRedistribute(float r) {
      LowVarianceResamplingPolicy* p = dynamic_cast<LowVarianceResamplingPolicy*>(getResamplingPolicy());
      if ( p )
	p->maxRedistribute = r;
      else
	std::cout << "Warning: setMaxRedistribute found getResamplingPolicy() returns wrong type policy; maxRedistribute not set." << std::endl;
    }
	
    //! If getResamplingPolicy() returns a LowVarianceResamplingPolicy instance, this will set LowVarianceResamplingPolicy::varianceScale; otherwise will display a warning
    virtual void setVarianceScale(float s) {
      LowVarianceResamplingPolicy* p = dynamic_cast<LowVarianceResamplingPolicy*>(getResamplingPolicy());
      if ( p )
	p->varianceScale = s;
      else
	std::cout << "Warning: setVarianceScale found getResamplingPolicy() returns wrong type policy; varianceScale not set." << std::endl;
    }
	

    //! Allows you to manually request a position update -- you might want to call this before using getEstimate's state information
    virtual void updateMotion() {
      if(motion!=NULL)
	motion->updateMotion(particles,estimate);
      varianceValid = false;
    }
	
    //! Applies the sensor model's evaluation to the particles, optionally updating the motion model and resampling first
    /*! If you are applying a group of sensor readings, you probably only want to update motion for the first one
     *  (since no motion is occuring between the readings if they were taken at the same time), and may
     *  want to hold off on resampling until the end (so the particles are better evaluated).
     *  If using the default LowVarianceResamplingPolicy, see also LowVarianceResamplingPolicy::resampleDelay. */
    virtual void updateSensors(SensorModel& sm, bool updateMot=true, bool doResample=true) {
      if(updateMot)
	updateMotion();
      if(doResample && hasEvaluation)
	resample();
      sm.evaluate(particles, estimate);
      hasEvaluation=true;
      varianceValid = false;
    }
	
    //! A manual call to trigger resampling
    virtual void resample() {
      if(resampler!=NULL)
	resampler->resample(particles);
      hasEvaluation=false;
    }
	
    //! Assigns the specified weight value to all of the particles
    virtual void resetWeights(float w) {
      for(typename particle_collection::iterator it=particles.begin(); it!=particles.end(); ++it)
	it->weight=w;
      hasEvaluation=false;
    }

    //! Requests that the resampler's distribution policy randomly distribute all of the particles, and reset weights to @a w.
    /*! You might want to do this if you believe you have been "kidnapped" by some unmodeled motion
     *  to a new area of state space, and need to restart the filter to determine the new location. */
    virtual void resetFilter(float w) {
      if(resampler!=NULL)
				resampler->getDistributionPolicy().randomize(&particles[0],particles.size());
      resetWeights(w);
    }

    virtual const particle_type& getEstimate() const { return estimate; } //!< Returns the weighted mean of all the #particles

    //! Returns the variance of the #particles
    virtual const particle_type& getVariance() {
      if ( !varianceValid )
	computeVariance();
      return variance;
    }

    //! Computes the variance of the particles; no-op here: override in subclass
    virtual void computeVariance() { varianceValid = true; }

    virtual particle_collection& getParticles() { return particles; } //!< Returns a reference to #particles itself (if you want to modify the particles, generally better to formulate it in terms of a sensor model or motion model for consistency)
    virtual const particle_collection& getParticles() const { return particles; } //!< Returns a reference to #particles itself (if you want to modify the particles, generally better to formulate it in terms of a sensor model or motion model for consistency)

    //! if you know the position in state space, pass it here, along with a positive varianceScale if you want some jiggle from the distribution policy
    virtual void setPosition(const particle_type& pos, float jiggleVariance=0) {
      particles.assign(particles.size(),pos);
      if ( jiggleVariance > 0 )
	resampler->getDistributionPolicy().jiggle(jiggleVariance,&particles.front(),particles.size());
    }
	
    //! Returns a confidence interval based on the particle_type's sumSqErr implementation (see ParticleBase::sumSqErr())
    virtual float getConfidenceInterval() const {
      float d=0;
      for(typename particle_collection::const_iterator it=particles.begin(); it!=particles.end(); ++it)
	d += it->sumSqErr(estimate);
      return std::sqrt(d/(particles.size()-1));
    }
    //! Adjusts the size of the particle collection -- more particles gives better coverage, but more computation
    /*! You may wish to shrink the number of particles when the confidence interval is small or
     *  particle weights are high, and increase particles when the filter is getting "lost". */
    virtual void resizeParticles(unsigned int numParticles) {
      std::cerr << "Resizing particles from " << particles.size() << " to " << numParticles << std::endl;
      if(numParticles > particles.size()) {
	index_t oldsize=particles.size();
	particles.resize(numParticles);
	if(resampler!=NULL)
	  resampler->getDistributionPolicy().randomize(&particles[oldsize],numParticles-oldsize);
			
      } else if(numParticles < particles.size()) {
	std::vector<particle_type*> sorted(particles.size());
	for(unsigned int i=0; i<particles.size(); ++i)
	  sorted[i]=&particles[i];
	std::sort(sorted.begin(),sorted.end(),weightLess);
	particle_collection newParticles;
	newParticles.reserve(numParticles);
	for(unsigned int i=sorted.size()-numParticles-1; i<sorted.size(); ++i)
	  newParticles.push_back(*sorted[i]);

	// The swap below will mess up the particle shapes in the sketchGUI

	particles.swap(newParticles);
      }
    }
	
 protected:
 public:// for debugging
	//!< used for sorting particles in resizeParticles() to drop the least weighted particles first
    static bool weightLess(const particle_type* a, const particle_type* b) { return a->weight < b->weight; }
	
    particle_collection particles; //!< Storage of the particles (no particular order)
    particle_type estimate; //!< Weighted mean of all the particles; weight is the weight of the best particle
    particle_type variance; //!< variance of the particles
    bool varianceValid; //!< True if the particles haven't changed since the variance was last computed
    MotionModel * motion; //!< motion model, can be NULL if you have no control or knowledge of changes in the system
    ResamplingPolicy * resampler; //!< resampling policy refocuses filter on "good" particles, can be NULL but filter won't work well without a resampler
    bool hasEvaluation; //!< set to true following each call to updateSensors, and false following resample() or resetWeights(); avoids repeated resamplings
	
 private:
    ParticleFilter(const ParticleFilter&); //!< don't call (copy constructor)
    ParticleFilter& operator=(const ParticleFilter&); //!< don't call (assignment operator)
};

template<typename ParticleT>
void ParticleFilter<ParticleT>::LowVarianceResamplingPolicy::resample(particle_collection& particles) {
  if ( resampleCount++ < resampleDelay )
    return;
  resampleCount=0;
  // std::cerr << "RESAMPLE UNDERWAY" << std::endl;
  // std::cerr << "Best particle is " << bestIndex << " @ " << particles[bestIndex].weight << std::endl;
	
  // we reuse newParticles each time, doing an STL O(1) swap to quickly exchange contents
  // have to make sure it's still the right size though...
  if(newParticles.size()<particles.size() || newParticles.size()>particles.size()*2)
    newParticles.resize(particles.size());
  if(particles.size()==0)
    return;
	
  // This part figures out how many particles we're going to globally redistribute,
  // leaving the rest for resampling
  float bestWeight = -FLT_MAX;
  for (size_t i=0; i<particles.size(); i++)
    if ( particles[i].weight > bestWeight )
      bestWeight = particles[i].weight;
  float redistributeRatio = 1;
  if ( logWeights ) {
    float min=minAcceptableWeight*(resampleDelay+1);
    if ( bestWeight > min ) {
      redistributeRatio = 1 - (min-bestWeight)/min;
      if ( redistributeRatio > 1 )
	redistributeRatio=1;
    }
  }
  else { // not logWeights
    float min=std::pow(minAcceptableWeight,(float)(resampleDelay+1));
    if ( bestWeight > min )
      redistributeRatio = (1-bestWeight/min);
  }
  unsigned int numRedistribute = (unsigned int)(particles.size() * redistributeRatio * maxRedistribute);
  std::cout << "Particle filter resampling: minAcceptable=" << minAcceptableWeight << "  best=" << bestWeight
						<< "  redistRatio=" << redistributeRatio << "  numRedist=" << numRedistribute << std::endl;
	
  // now do resampling, writing into newParticles
  const unsigned int numResample=newParticles.size()-numRedistribute;
  //std::cerr << "best " << bestIndex << " @ " << bestWeight << " numRedist. " << numRedistribute << " of " << particles.size() << std::endl;
  if(numResample>0) {
    // add up the cumulative weights for each particle...
    std::vector<float> weights(particles.size());
    if(logWeights) {
      weights[0]=std::exp(particles.front().weight-bestWeight);
      for (unsigned int i=1; i < particles.size(); i++)
	weights[i] = weights[i-1] + std::exp(particles[i].weight-bestWeight);
    } else {
      weights[0]=particles.front().weight/bestWeight;
      for (unsigned int i=1; i < particles.size(); i++)
	weights[i] = weights[i-1] + particles[i].weight/bestWeight;
    }
    if(weights.back()<=0) {
      std::cerr << "Warning particle filter attempted resampling with weight total " << weights.back() << std::endl;
      return;
    }
		
    float r = weights.back() / numResample; // last element of weights/number of particles
    float offset = r*float(rand())/RAND_MAX;
    unsigned int pos = 0;
		
    for (unsigned int i=0; i < numResample; i++){
      float target = offset+r*i; // multiply instead of repeatedly adding to avoid rounding issues
      while (target >= weights[pos])
				pos++;
      // copy over particle (we'll "jiggle" it later if desired)
      newParticles[i]=particles[pos];
    }
		
    // now jiggle all of the particles we've resampled
    if(varianceScale!=0) {
      float vs;
      if(varianceScale>=0)
	vs=varianceScale; // use the user's setting
      // otherwise varianceScale is negative, we'll try to pick a variance scale based on how well we're doing
      else if(redistributeRatio>0)
	vs=1+redistributeRatio*(-varianceScale-1);
      else {
	if(logWeights) {
	  float min=minAcceptableWeight*(resampleDelay+1);
	  vs=bestWeight/min;
	} else {
	  float min=std::pow(minAcceptableWeight,(float)(resampleDelay+1));
	  vs=(1-bestWeight)/(1-min);
	}
	//vs=std::pow(vs,4.f);
      }
      ResamplingPolicy::dist->jiggle(vs,&newParticles[0],numResample);
    }
  }
	
  // finished resampling, redistribute the remaining particles (if needed due to falling below minAcceptableWeight)
  ResamplingPolicy::dist->randomize(&newParticles[numResample],numRedistribute);
	
  // reset weights
  float const initialWeight = logWeights ? 0 : 1;
  for(typename particle_collection::iterator it=newParticles.begin(); it!=newParticles.end(); ++it)
    it->weight = initialWeight;
	
  particles.swap(newParticles); // all done!  swap the particle lists
}

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
