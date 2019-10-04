#ifndef _LOADED_ShapeBasedParticleFilter_h_
#define _LOADED_ShapeBasedParticleFilter_h_

#include <vector>
#include <iostream>
#include <cmath>

#include "DualCoding/ShapePolygon.h"
#include "Localization/LocalizationParticle.h"

#include "Shared/RobotInfo.h"
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
#  include "Localization/CreateMotionModel.h"
#elif defined(TGT_IS_KOBUKI)
#  include "Localization/KobukiMotionModel.h"
#else
#  include "Localization/DeadReckoningBehavior.h"
#endif

#include "ShapeSensorModel.h"

namespace DualCoding {
	
  // Defined in this file: first ShapeParticleDistributionPolicy, then
  // ShapeBasedParticleFilter, which uses this policy
  

  //================ ShapeParticleDistributionPolicy ================

  template<typename ParticleT>
  class ShapeParticleDistributionPolicy : public LocalizationParticleDistributionPolicy<ParticleT> {
  public:
    typedef ParticleT particle_type;  //!< just for convenience
    typedef typename ParticleFilter<particle_type>::index_t index_t; //!< just for convenience

    ShapeParticleDistributionPolicy() : LocalizationParticleDistributionPolicy<ParticleT>(), worldBounds() {}

      // Override LocalizationParticleDistributionPolicy::randomize() so we can restrict particles
      // to fit within the worldBounds polygon if one is defined.
    virtual void randomize(particle_type* begin, index_t num) {
      particle_type* end=&begin[num]; 
      for(particle_type* p=begin; p!=end; p++) { 
				while (1) { // loop until particle is acceptable
					p->x = float(rand())/RAND_MAX * LocalizationParticleDistributionPolicy<ParticleT>::mapWidth + 
						LocalizationParticleDistributionPolicy<ParticleT>::mapMinX;
					p->y = float(rand())/RAND_MAX * LocalizationParticleDistributionPolicy<ParticleT>::mapHeight + 
						LocalizationParticleDistributionPolicy<ParticleT>::mapMinY;
					if ( !worldBounds.isValid() || worldBounds->isInside(Point(begin->x,begin->y)) )
						break;
				}
				p->theta = direction_t(rand())/RAND_MAX * 2 * direction_t(M_PI);
      }
    }

    virtual void jiggle(float var, particle_type* begin, index_t num) {
      if(var==0)
	return;
      particle_type* end=&begin[num]; 
      while(begin!=end) {
				float dx=0, dy=0;
				for (int i=0; i<4; i++) {
					float rx = (float)DRanNormalZig32()*LocalizationParticleDistributionPolicy<ParticleT>::positionVariance*var;
					float ry = (float)DRanNormalZig32()*LocalizationParticleDistributionPolicy<ParticleT>::positionVariance*var;
					if ( !worldBounds.isValid() || worldBounds->isInside(Point(begin->x+rx, begin->y+ry)) ) {
						dx = rx;
						dy = ry;
						break;
					}
				}
				begin->x += dx;
				begin->y += dy;
				begin->theta+=(float)DRanNormalZig32()*LocalizationParticleDistributionPolicy<ParticleT>::orientationVariance*var;
				++begin;
      }
    }

		virtual void setWorldBounds(float minX, float width, float minY, float height) {
			LocalizationParticleDistributionPolicy<ParticleT>::mapMinX = minX;
			LocalizationParticleDistributionPolicy<ParticleT>::mapWidth = width;
			LocalizationParticleDistributionPolicy<ParticleT>::mapMinY = minY;
			LocalizationParticleDistributionPolicy<ParticleT>::mapHeight = height;
		}

    virtual void setWorldBounds(const Shape<PolygonData> bounds) {
      worldBounds = bounds;
      if ( worldBounds.isValid() ) {
				BoundingBox2D b(worldBounds->getBoundingBox());
				setWorldBounds(b.min[0], b.getDimension(0), b.min[1], b.getDimension(1));
      }
    }

  protected:
    Shape<PolygonData> worldBounds;	//!< If valid shape, particles must lie inside it.
  };

  //================ ShapeBasedParticleFilter ================

  //! Bundles a motion model (DeadReckoningBehavior or CreateMotionModel) and a ShapeSensorModel for easy use of a shape-based particle filter for localization

  class ShapeBasedParticleFilter : public ParticleFilter<LocalizationParticle> {
  public:
    //! constructor, must pass local and world shape spaces, which will be used in future calls to update()
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)
    ShapeBasedParticleFilter(ShapeSpace &camShS, ShapeSpace &localShS, ShapeSpace &worldShS, unsigned int numParticles=2000)
      : ParticleFilter<LocalizationParticle>(numParticles, new CreateMotionModel<LocalizationParticle>),
	sensorModel(new ShapeSensorModel<LocalizationParticle>(camShS,localShS,worldShS))
    {
      getResamplingPolicy()->setDistributionPolicy(new ShapeParticleDistributionPolicy<LocalizationParticle>);
      if(BehaviorBase* motBeh = dynamic_cast<BehaviorBase*>(motion))
	motBeh->start();
    }
#elif defined(TGT_IS_KOBUKI)
    ShapeBasedParticleFilter(ShapeSpace &camShS, ShapeSpace &localShS, ShapeSpace &worldShS, unsigned int numParticles=2000)
      : ParticleFilter<LocalizationParticle>(numParticles, new KobukiMotionModel<LocalizationParticle>),
	sensorModel(new ShapeSensorModel<LocalizationParticle>(camShS,localShS,worldShS))
    {
      getResamplingPolicy()->setDistributionPolicy(new ShapeParticleDistributionPolicy<LocalizationParticle>);
      if(BehaviorBase* motBeh = dynamic_cast<BehaviorBase*>(motion))
	motBeh->start();
    }
#else
    ShapeBasedParticleFilter(ShapeSpace &camShS, ShapeSpace &localShS, ShapeSpace &worldShS, unsigned int numParticles=2000)
      : ParticleFilter<LocalizationParticle>(numParticles, new DeadReckoningBehavior<LocalizationParticle>),
	sensorModel(new ShapeSensorModel<LocalizationParticle>(camShS,localShS,worldShS))
    {
      // std::cout<<"ShapeBasedParticleFilter using DeadReckoningBehavior\n";
      getResamplingPolicy()->setDistributionPolicy(new ShapeParticleDistributionPolicy<LocalizationParticle>);
      if(BehaviorBase* motBeh = dynamic_cast<BehaviorBase*>(motion))
	motBeh->start();
    }
#endif
	

    //! destructor
    virtual ~ShapeBasedParticleFilter() { 
      if(BehaviorBase* motBeh = dynamic_cast<BehaviorBase*>(motion)) {
	motBeh->stop();
	// behaviors are reference counted, stopping removes our reference, set to NULL to avoid call to delete in ParticleFilter
	motion=NULL;
      }
      delete sensorModel;
    }
  
    virtual void updateFromLocal() {
      getSensorModel().updateFromLocal(particles, estimate);
    }
	
    virtual void updateFromCamera() {
      getSensorModel().updateFromCamera(particles, estimate);
    }
	
    //! accessor for #sensorModel
    virtual ShapeSensorModel<LocalizationParticle>& getSensorModel() const { return *sensorModel; }

    //! replaces the sensor model in use, the particle filter will take responsibility for deallocating the sensor model's memory when destructed or replaced
    virtual void setSensorModel(ShapeSensorModel<LocalizationParticle>* customSensorModel) {
      delete sensorModel; 
      sensorModel=customSensorModel;
    }

    using ParticleFilter<LocalizationParticle>::resetFilter;

    //! randomizes all the particles
    virtual void resetFilter();
	
    //! Adjusts the size of the particle collection -- more particles gives better coverage, but more computation
    /*! You may wish to shrink the number of particles when the confidence interval is small or
     *  particle weights are high, and increase particles when the filter is getting "lost". */
    virtual void resizeParticles(unsigned int numParticles);

    //! Synchs the estimate to the agent's current position and heading in worldShS. During odometry, the Pilot will use the estimate to update the agent's position. 
    virtual void synchEstimateToAgent();
  
    //! Resets particles to the specified position and orientation, and optionally jiggles them by variance
    virtual void setPosition(float const x, float const y, AngTwoPi const orientation, float variance=0);
    using ParticleFilter<LocalizationParticle>::setPosition;

		//! Computes the weighted variance in the position and heading estimates of the particle collection.
    virtual void computeVariance();

    //! Sets boundary within which particles should lie
    virtual void setWorldBounds(const Shape<PolygonData> &bounds);

    //! Sets boundary within which particles should lie
		virtual void setWorldBounds(float minX, float width, float minY, float height);

    //! Deletes particle shapes and "particle" GraphicsData object so we can prepare a new display
    static void deleteParticleDisplay(ShapeSpace &wShS);

    //! Displays particles on the world map using a GraphicsData shape; howmany can either be a percentage (<= 1.0) or a whole number
    virtual void displayParticles(float const howmany=100);

    //! Displays individual particles on the world map using LocalizationParticle shapes; howmany can either be a percentage (<= 1.0) or a whole number
    virtual void displayIndividualParticles(float const howmany=100);

  protected:
    ShapeSensorModel<LocalizationParticle> * sensorModel; //!< provides evaluation of particles
  
  private:
    ShapeBasedParticleFilter(const ShapeBasedParticleFilter&); //!< don't call (copy constructor)
    ShapeBasedParticleFilter& operator=(const ShapeBasedParticleFilter&); //!< don't call (assignment operator)
  }; 

} // namespace

#endif
