//-*-c++-*-
#ifndef INCLUDED_DeadReckoningBehavior_h_
#define INCLUDED_DeadReckoningBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/LocomotionEvent.h"
#include "Localization/HolonomicMotionModel.h"
#include "Shared/WorldState.h"

//! Subscribes to LocomotionEvents and attempts to track robot movement over time using a fairly generic HolonomicMotionModel
/*! Can be used as a ParticleFilter::MotionModel, or as a component of other behaviors.
 *
 *  If you want a regular report on position, the behavior will output current position on timer events */
template<class ParticleT>
class DeadReckoningBehavior : public BehaviorBase, public HolonomicMotionModel<ParticleT> {
public:
  //! constructor
  explicit DeadReckoningBehavior(const std::string& name="DeadReckoningBehavior")
    : BehaviorBase(name), HolonomicMotionModel<ParticleT>(), vel_x(), vel_y(), vel_a() {}
	
  //! constructor
  DeadReckoningBehavior(float xVariance, float yVariance, float aVariance)
    : BehaviorBase("DeadReckoningBehavior"), HolonomicMotionModel<ParticleT>(xVariance,yVariance,aVariance),
      vel_x(), vel_y(), vel_a() {}
	
  virtual void doStart() {
    vel_x = state->vel_x;
    vel_y = state->vel_y;
    vel_a = state->vel_a;
    HolonomicMotionModel<ParticleT>::setVelocity(vel_x,vel_y,vel_a);
    erouter->addListener(this, EventBase::locomotionEGID );
    //erouter->addTimer(this, 0, 500);
  }
	
  virtual void doEvent() {
    if (event->getGeneratorID() == EventBase::locomotionEGID) {
      const LocomotionEvent &locoevt = dynamic_cast<const LocomotionEvent&>(*event);
      float new_x = locoevt.x;
      float new_y = locoevt.y;
      float new_a = locoevt.a;
      if ( (new_x == 0 && new_y == 0 && new_a == 0 && (vel_x != 0 || vel_y != 0 || vel_a != 0)) // robot has just stopped
	   || fabs(new_x-vel_x) > 0.1       // translation velocity change > 0.1 mm/sec
	   || fabs(new_y-vel_y) > 0.1 
	   || fabs(new_a-vel_a) > 0.001 ) { // angular velocity change > 0.001 rad/sec
	// std::cout << "New DeadReckoning velocities: " << new_x << " " << new_y << " " << new_a << std::endl;
	vel_x = new_x;
	vel_y = new_y;
	vel_a = new_a;
	HolonomicMotionModel<ParticleT>::setVelocity(vel_x,vel_y,vel_a,locoevt.getTimeStamp());
      }
    } else if (event->getGeneratorID() == EventBase::timerEGID) {
      float tempx;
      float tempy;
      float tempa;
      HolonomicMotionModel<ParticleT>::getPosition(tempx, tempy, tempa);
      std::cout << "DEADPOS " << tempx << " " << tempy << " " << tempa << std::endl;
    }
  }
	
  static std::string getClassDescription() { return "Subscribes to LocomotionEvents and attempts to track robot movement over time"; }
  virtual std::string getDescription() const { return getClassDescription(); }
	
private:
  float vel_x, vel_y, vel_a;
};

/*! @file
 * @brief Defines DeadReckoningBehavior, which subscribes to LocomotionEvents and attempts to track robot movement over time using a fairly generic HolonomicMotionModel
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
