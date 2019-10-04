//-*-c++-*-
#ifndef INCLUDED_HolonomicMotionModel_h_
#define INCLUDED_HolonomicMotionModel_h_

#include "Shared/ParticleFilter.h"
#include "Localization/LocalizationParticle.h"
#include "Shared/get_time.h"
#include "Shared/zignor.h"
#include <cmath>

//! the main function -- to avoid numeric issues, treats paths that would result in a radius over 1e6 long as a straight line
/*! see HolonomicMotionModel class notes for more information on the math involved */
void computeHolonomicMotion(float xvel, float yvel, float avel, float time, float& xpos, float& ypos, float& apos);

//! This class can model the path of a holonomic robot in two dimensions (x and y)
/*! This class can also model a non-holonomic robot, just don't use the y parameter!
 *  
 *  The static function computeMotion() does the main computational grunt work.
 *  <table cellspacing=0 cellpadding=0 width="434" class="figures" align="center" border="0"><tr>
 *  <td class="figure"><img src="MotionModel.png"><div style="padding:10px;">
 *  Illustration of the equations used.  Constant angular and linear velocities result in a
 *  circular arc as shown.  The radius of the arc is directly determined by the ratio of the
 *  linear speed to the angular speed.  The final position after time @f$ \Delta t @f$ will be
 *  @f$ \delta @f$ along a bearing @f$ \theta/2 @f$, where @f$ \theta @f$ is the angle which has
 *  been turned during the interval of interest.
 *  </div></td></tr></table>
 *  
 *  You can call setPosition() to initialize the starting location or if the robot has been moved
 *  by some external means.  You should call setVelocity() whenever the robot's velocity changes.
 *  The class will internally store the position and time at which the velocity change happened
 *  so that later calls to getPosition() will return points along the new path.
 *
 *  This class can be used as a motion model in the ParticleFilter class, or independently if
 *  desired for general purpose motion estimation.  (see the standalone computeHolonomicMotion())
 *
 *  However, if you are looking for a motion model for ParticleFiltering, it may be more convenient
 *  to use DeadReckoningBehavior because you can start the
 *  behavior and it will subscribe to automatically receive LocomotionEvents from then on.
 *  If using HolonomicMotionModel directly with the particle filter, you
 *  would need to call setVelocity() yourself anytime the robot changes direction.
 *
 *  Variance parameters only come into play with updateMotion(), which is called on
 *  collections of particles.  The other functions all return "ideal" motion calculations.
 *  Be aware that when using in a particle filter, the position is reset on particle updates
 *  (updateMotion()).  In other words, the position returned by motion model is the
 *  offset achieved since the last particle update, @e not the position relative to
 *  world's origin, nor any other fixed point.
 *
 *  Caveat: acceleration is not handled by this model.  That would be a nice addition...
 *
 *  @see computeHolonomicMotion()
 */
template<typename ParticleT>
class HolonomicMotionModel : public ParticleFilter<ParticleT>::MotionModel {
public:
  typedef typename ParticleFilter<ParticleT>::MotionModel::particle_type particle_type;
  typedef typename ParticleFilter<ParticleT>::MotionModel::particle_collection particle_collection;
  typedef typename particle_collection::size_type index_t;

  //! constructor, with default noise parameters (xvar=yvar=50, avar=0.15f)
  HolonomicMotionModel()
    : ParticleFilter<ParticleT>::MotionModel(),
    xvel(0), yvel(0), avel(0), prevtime(get_time()), posx (0), posy(0), posa(0),
    xvar(.25f), yvar(.25f), avar(.25f), crossAxis(.05f), crossAngle(.001f) {}
	
  //! constructor, with noise parameters (pass 0's to make it an "ideal" motion model)
  /*! Variance parameters only come into play with updateMotion(), which is called on
   *  collections of particles.  The other functions all return "ideal" motion calculations. */
  HolonomicMotionModel(float xVariance, float yVariance, float aVariance)
    : ParticleFilter<ParticleT>::MotionModel(),
    xvel(0), yvel(0), avel(0), prevtime(get_time()), posx (0), posy(0), posa(0),
    xvar(xVariance), yvar(yVariance), avar(aVariance),
    crossAxis((xVariance+yVariance)/10), crossAngle(.001f)  {}
	
  //! called by the particle filter when the current position of each particle should be updated
  /*! This will reset the motion model to set the origin at the current location after the particles
   *  are updated, so that the next call to updateMotion() will supply the particles with the
   *  displacement which occurred since the last update */
  virtual void updateMotion(particle_collection& particles, particle_type& estimate) {
    unsigned int curt = get_time();
    if(curt==prevtime)
      return; // no time has passed!
    float dt = (curt - prevtime)/1000.f;
    prevtime=curt;
    // update estimate using zero noise
    posx=posy=posa=0;
    computeHolonomicMotion(xvel,yvel,avel,dt, posx,posy,posa);
    float ec = std::cos(estimate.theta);
    float es = std::sin(estimate.theta);
    estimate.x += posx*ec - posy*es;
    estimate.y += posx*es + posy*ec;
    estimate.theta += posa;

    // now update the particles
    if(xvar==0 && yvar==0 && avar==0) {
      // if we're using a noiseless motion model, can be a bit faster and avoid all the random number generation
      for(typename particle_collection::iterator it=particles.begin(); it!=particles.end(); ++it) {
	float c = std::cos(it->theta);
	float s = std::sin(it->theta);
	it->x += posx*c - posy*s;
	it->y += posx*s + posy*c;
	it->theta += posa;
      }
    } else {
      // otherwise have to do the noise generation too...
      for(typename particle_collection::iterator it=particles.begin(); it!=particles.end(); ++it) {
	posx=posy=posa=0;
	// this factor normalizes across update rates
	// (integrating many small updates otherwise yields lower variance in position than fewer large updates...)
	float norm=1/std::sqrt(dt);
	float xv=xvel*(1+(float)DRanNormalZig32()*xvar*norm) + (yvel*(float)DRanNormalZig32()*crossAxis*norm);
	float yv=yvel*(1+(float)DRanNormalZig32()*yvar*norm) + (xvel*(float)DRanNormalZig32()*crossAxis*norm);
	float av=avel*(1+(float)DRanNormalZig32()*avar*norm) + ((xvel+yvel)*(float)DRanNormalZig32()*crossAngle*norm);
	computeHolonomicMotion(xv,yv,av,dt, posx,posy,posa);
	float c = std::cos(it->theta);
	float s = std::sin(it->theta);
	it->x += posx*c - posy*s;
	it->y += posx*s + posy*c;
	it->theta += posa;
      }
    }
  }
	
  //! stores the current position into the arguments (based on get_time() vs the time the position was last set)
  void getPosition(float& outx, float& outy, float& outa) const {
    outx=posx;
    outy=posy;
    outa=posa;
    float dt = (get_time() - prevtime)/1000.f;
    computeHolonomicMotion(xvel,yvel,avel,dt, outx,outy,outa);
  }
	
//! stores the current position into the arguments (based on curtime vs the time the position was last set)
void getPosition(float& outx, float& outy, float& outa, unsigned int curtime) const {
  // store position of last velocity change:
  outx=posx;
  outy=posy;
  outa=posa;
  // how much time has passed since then?
  float dt = (curtime - prevtime)/1000.f;
  // compute current position along path
  computeHolonomicMotion(xvel,yvel,avel,dt, outx,outy,outa);
}
	
//! sets the current position to the specified values and updates the timestamp to the current time
void setPosition(float x, float y, float angle) { setPosition(x,y,angle,get_time()); }
	
//! sets the current position to the specified values and updates the timestamp to the specified time
void setPosition(float x, float y, float angle, unsigned int curtime) {
  posx = x;
  posy = y;
  posa = angle;
  prevtime = curtime;
}

//! stores the current velocity into the arguments (no noise is added, this just echos the values passed to setVelocity())
void getVelocity(float& outxv, float& outyv, float& outav) const {
  outxv=xvel;
  outyv=yvel;
  outav=avel;
}
	
//! sets the current velocity to the specified values and updates the position and timestamp to the current time
void setVelocity(float xv, float yv, float av) { setVelocity(xv,yv,av,get_time()); }
	
//! sets the current velocity to the specified values and updates the position and timestamp to the specified time
void setVelocity(float xv, float yv, float av, unsigned int curtime) {
  //std::cerr << "setVelocity("<<xv<<','<<yv<<','<<av<<','<<curtime<<')'<<std::endl;
  // first update current position
  float dt = (curtime - prevtime)/1000.f;
  computeHolonomicMotion(xvel,yvel,avel,dt, posx,posy,posa);
  // now store specified velocity
  xvel = xv; // forward velocity
  yvel = yv; // sideways speed
  avel = av; // turning speed
  prevtime = curtime;  // time of last event

  //cout << "Posx: " << posx << " Posy: " << posy << " Posangle: " << posa << endl;
  //cout << "PrevX: " << xvel << endl;
  //cout << "PrevY: " << yvel << endl;
  //cout << "PrevTime: " << prevtime << endl;
}
	
//! allows you to change the variance parameters (#xvar, #yvar, #avar)
void setVariance(float xv, float yv, float av) {
  xvar=xv; yvar=yv; avar=av;
}
//! allows you to change the cross-variance parameters (#crossAxis, #crossAngle)
void setCrossVariance(float axis, float angle) {
  crossAxis=axis;
  crossAngle=angle;
}
	
float getXVariance() const { return xvar; } //!< accessor for #xvar
float getYVariance() const { return yvar; } //!< accessor for #yvar
float getAVariance() const { return avar; } //!< accessor for #avar
float getAxisCrossVariance() const { return crossAxis; } //!< accessor for crossAxis
float getAngleCrossVariance() const { return crossAngle; } //!< accessor for crossAngle

protected:
float xvel; //!< current x velocity
float yvel; //!< current y velocity
float avel; //!< current angular velocity
unsigned int prevtime; //!< time (in milliseconds) that the position was last set
	
float posx; //!< x position at which #prevtime was set
float posy; //!< y position at which #prevtime was set
float posa; //!< orientation at which #prevtime was set
	
float xvar; //!< variance of x velocities as ratio of x speed, used when updating particle list (updateMotion())
float yvar; //!< variance of y velocities as ratio of y speed, used when updating particle list (updateMotion())
float avar; //!< variance of angular velocities as ratio of angular speed, used when updating particle list (updateMotion())
float crossAxis; //!< cross variance of x speed on y speed and vice versa
float crossAngle; //!< cross variance of x,y speed on angular speed
};

/*! @file
 * @brief Defines HolonomicMotionModel, which can model the path of a holonomic robot
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
