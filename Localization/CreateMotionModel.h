//-*-c++-*-
#ifndef _CreateMotionModel_h_
#define _CreateMotionModel_h_

#include "Shared/RobotInfo.h"

#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)

#include "Behaviors/BehaviorBase.h"
#include "DualCoding/VRmixin.h"
#include "Shared/zignor.h"
#include "Shared/Measures.h"
#include "Shared/ParticleFilter.h"
#include "Shared/WorldState.h"
#include "Vision/VisualOdometry/VisualOdometry.h"


//! Holds particle-independent odometry code so we can avoid templating and just recompile the cc file
class CreateOdometry {
private:
  float lastdist;
  AngTwoPi lastang;
  AngTwoPi lastOdoAng;

public:
  CreateOdometry() :
    lastdist(state->sensors[EncoderDistanceOffset]),
    lastang(state->sensors[EncoderAngleOffset]/180.0*(float)M_PI),
    lastOdoAng(0) {}

  virtual ~CreateOdometry() {}

  //! Calculate translation and rotation using both Create hardware odometry and visual odometry
  void getCreateOdometry(float &dd, AngSignPi &da);

  //! Compute distance update for a given linear distance and angular distance traveled
  virtual void computeCreateMotion(float len, float omega, float theta, float &dx, float &dy, float &dtheta);

  static constexpr float minUpdateDistance = 5.0f;
  static constexpr float minUpdateAngle = 0.05f;
};

template<class ParticleT>
class CreateMotionModel : public BehaviorBase, public ParticleFilter<ParticleT>::MotionModel, public CreateOdometry {
private:
  /* mean and variance for added gaussian noise */
  float dmean;
  AngTwoPi amean;
  float ddvar, davar, aavar, advar;

public:
  typedef typename ParticleFilter<ParticleT>::MotionModel::particle_type particle_type;
  typedef typename ParticleFilter<ParticleT>::MotionModel::particle_collection particle_collection;
  typedef typename particle_collection::size_type index_t;

  //! Constructor
  CreateMotionModel(float dm=0.1f, float am=0.2f, float ddv=0.001f, float dav=0.001f, float aav=0.001f, float adv=0.000001f) :
    BehaviorBase("CreateMotionModel"), ParticleFilter<ParticleT>::MotionModel(), CreateOdometry(),
    dmean(dm), amean(am), ddvar(ddv), davar(dav), aavar(aav), advar(adv)
    {}

  float sampleNormal(float mean, float var) {
    // N(m,s^2) ~ s*N(0,1) + m
    return mean + (float)DRanNormalZig32()*std::sqrt(var);
  }

  virtual void updateMotion(particle_collection& particles, particle_type& estimate) {
    float dd;
    AngSignPi da;
    getCreateOdometry(dd,da);

    // If we haven't moved much, don't bother updating the particles
    if (fabs(dd) < minUpdateDistance && fabs(da) < minUpdateAngle)
      return;

    // First update the estimate, using zero noise
    float dx, dy, dtheta;
    computeCreateMotion(dd, float(da), estimate.theta, dx, dy, dtheta);
    estimate.x += dx;
    estimate.y += dy;
    estimate.theta += dtheta;

    // Now  update the particles using our noise model
    for(typename particle_collection::iterator it=particles.begin(); it!=particles.end(); ++it) {
      // noise perturbed distance and angle travelled
      // Note: original code was:
      //
      //    float dnoise = sampleNormal(dmean*dd, ddvar*dd*dd + davar*da*da);
      //    float anoise = sampleNormal(amean*da, aavar*da*da + advar*dd*dd);
      //
      // but with current parameters, this caused a 10% distance
      // overshoot, while the actual robot seems to show a 3%
      // undershoot.  Also, for long turns (90 degrees) the angular
      // error was way too high.  So the version below gives zero mean
      // translational error; might replace this later with original
      // code plus better parameter settings.  Our trick doesn't work
      // for the angular component because if we don't command a turn,
      // the Create will always report 0 angular change so scaling by
      // da would zero the noise entirely.

      float dnoise, anoise;
      dnoise = dmean * dd * sampleNormal(0, ddvar*dd*dd + davar*da*da);
      anoise = float(amean) * sampleNormal(0, aavar*da*da + advar*dd*dd);
      float ddn = dd + dnoise;
      AngSignPi dan = float(da) + anoise;
      computeCreateMotion(ddn, float(dan), float(it->theta), dx, dy, dtheta);
      it->x += dx;
      it->y += dy;
      it->theta += dtheta;
    }
  }

};

#endif

#endif

