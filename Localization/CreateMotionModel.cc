#include "Shared/RobotInfo.h"

#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)

#include "CreateMotionModel.h"

void CreateOdometry::getCreateOdometry(float &dd, AngSignPi &da) {
  float dist = state->sensors[EncoderDistanceOffset];
  AngTwoPi ang = state->sensors[EncoderAngleOffset]/180.0*(float)M_PI;

  dd = dist - lastdist;
  da = float(ang - lastang);
  if (fabs(dd) < minUpdateDistance && fabs(da) < minUpdateAngle)
    return;

  lastdist = dist;
  lastang = ang;

  // *** Visual odometry is presently disabled; need to revisit this
  // now that the lastdist/lastang update bug was fixed on 4/3/2013.

  // Apply visual odometry for better rotation estimation.
  //
  // In Mirage, the simulated Create odometry is nearly perfect,
  // while visual odometry overestimates turns.  On the physical robot,
  // the Create's poor hardware odometry favors relying on visual
  // odometry.  But if visual odometry is failing, revert to the Create.
  // 
  if (fabs(da) > minUpdateAngle) {  // don't bother if we're not turning
    if ( DualCoding::VRmixin::imageOdometry != NULL ) {
      AngTwoPi odoAng = DualCoding::VRmixin::imageOdometry->getIntegratedAngle()/180.0*(float)M_PI;
      AngSignPi odoDa = AngSignPi(odoAng - lastOdoAng);
      lastOdoAng = odoAng;
      // std::cout << "old da = " << da << "   odoDa = " << odoDa;
      if ( state->sensors[GPSXOffset] == 0.0 )  // no GPS means we're not running in Mirage
	if ( da*odoDa > 0 && fabs(odoDa) > fabs(da) ) // ignore odoDa if sign doesn't match da
	  da = odoDa;
    }
  }
  // std::cout << "   new da = " << da << std::endl;
}

void CreateOdometry::computeCreateMotion(float len, float omega, float theta, float &dx, float &dy, float &dtheta) { 
  // check for linear motion
  if ( omega == 0 ) {
    dx = len*std::cos(theta);
    dy = len*std::sin(theta);
    dtheta = 0;
  }
  // curved motion, if len = 0, just rotation
  else {
    // solve for radius of circle
    // note that we don't need to use abs here
    // if len, omega are negative then we want r to be
    // negative because we are effectively flipping the circle
    float r = len / omega;
      
    // find new angles
    // would be this if theta was position around circle:
    //dx = r*cos(theta + omega) - r*cos(theta);
    //dy = r*sin(theta + omega) - r*sin(theta);
    // but it's actually the tangent to the circle,
    // so we shift by -pi/2 giving:
    dx = r*std::sin(theta + omega) - r*std::sin(theta);
    dy = -r*std::cos(theta + omega) + r*std::cos(theta);
    dtheta = omega;
  }
}

#endif
