#include "UPennWalkMC.h"

//This class is ported from University of Pennsylvania's 2004 Robosoccer entry, and falls under their license:
/*=========================================================================
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================= */

//better to put this here instead of the header
using namespace std; 

#include <math.h>

#ifndef PI
#define PI M_PI
#endif

const size_t MAX_WIDTH = 208;
const size_t MAX_HEIGHT = 160;
const size_t LAYERM_WIDTH = 104;
const size_t LAYERM_HEIGHT = 80;

const double FIELD_VIEW_H = 56.9*(PI/180);
const double FIELD_VIEW_V = 45.2*(PI/180);
const double FOCAL_LENGTH = 192.0; // in pixel units (.5*WIDTH/tan(.5*FIELD_VIEW_H))
const unsigned int IMAGE_WIDTH = 208;
const unsigned int IMAGE_HEIGHT = 160;

const double BODY_TILT = -13*PI/180; // Negative is head closer to ground

const double BODY_WIDTH = 134.4;
const double BODY_LENGTH = 130.0;

const size_t NUM_LEG = 4;
const size_t NUM_LEG_JOINT = 3*NUM_LEG;

// Should optimize by also storing LENGTH and ANGLE of each segment:
const double LEG_FORE_UPPER_Z = 69.5;
const double LEG_FORE_UPPER_Y = 9.0;
const double LEG_FORE_LOWER_Z = 76.4;  // From (71.5, 28.3) @ 30 deg.
const double LEG_FORE_LOWER_Y = -9.0;

const double LEG_HIND_UPPER_Z = 69.5;
const double LEG_HIND_UPPER_Y = 9.0;
const double LEG_HIND_LOWER_Z = 78.9; // From (76.5, 21.3) @ 30 deg.
const double LEG_HIND_LOWER_Y = -9.0;
//const double LEG_HIND_LOWER_Z = 80.0; // From (76.5, 21.3) @ 30 deg.
//const double LEG_HIND_LOWER_Y = 0.0;

const double NECK_TILT2_TO_CAMERA_Y = 81.0;
const double NECK_TILT2_TO_CAMERA_Z = -14.6;
const double NECK_TILT_TO_TILT2 = 80.0;
const double SHOULDER_TO_NECK_TILT_Y = 2.5;
const double SHOULDER_TO_NECK_TILT_Z = 19.5;

const double MIN_SHOULDER_HEIGHT = 50.0;

const double TURN_OFFSET = 75.0;



// Maximum parameters for walk:


// Imitating Paul's walk:

const double STANCE_BODY_TILT = 0*PI/180;
const double STANCE_SHOULDER_HEIGHT = 105.;

const double STANCE_FORE_X0 = 7.;
//const double STANCE_FORE_Y0 = 50.;
const double STANCE_FORE_Y0 = 60.;
const double STANCE_HIND_X0 = 2.;
const double STANCE_HIND_Y0 = -45.;

const int WALK_QUARTER_PERIOD = 3;
const double WALK_MAX_DISTANCE = 13.; // 390 (mm/sec) /30 frames = 13

const double WALK_FORE_LIFT_INITIAL = 25;
const double WALK_FORE_LIFT_FINAL = 45;

const double WALK_HIND_LIFT_INITIAL = 25;
const double WALK_HIND_LIFT_FINAL = 25;

const double WALK_FORE_XMIN = -10;
const double WALK_FORE_XMAX = 30;
const double WALK_FORE_YMIN = 25;
//const double WALK_FORE_YMAX = 85;
const double WALK_FORE_YMAX = 90;

const double WALK_HIND_XMIN = -15;
const double WALK_HIND_XMAX = 35;
const double WALK_HIND_YMIN = -70;
const double WALK_HIND_YMAX = -15;


/*
// Imitating 2004 US Open walk:
const double STANCE_BODY_TILT = -15*PI/180;
const double STANCE_SHOULDER_HEIGHT = 95.;

const double STANCE_FORE_X0 = 5.;
const double STANCE_FORE_Y0 = 70.;
const double STANCE_HIND_X0 = 10.;
const double STANCE_HIND_Y0 = -40.;

const int WALK_QUARTER_PERIOD = 4;
const double WALK_MAX_DISTANCE = 8.; // 390 (mm/sec) /30 frames = 13

const double WALK_FORE_LIFT_INITIAL = 20;
const double WALK_FORE_LIFT_FINAL = 30;
const double WALK_HIND_LIFT_INITIAL = 30;
const double WALK_HIND_LIFT_FINAL = 20;

const double WALK_FORE_XMIN = -20;
const double WALK_FORE_XMAX = 40;
const double WALK_FORE_YMIN = 0;
const double WALK_FORE_YMAX = 100;

const double WALK_HIND_XMIN = -20;
const double WALK_HIND_XMAX = 50;
const double WALK_HIND_YMIN = -150;
const double WALK_HIND_YMAX = 20;
*/


enum LegIdentifier {
  LEG_LEFT_FORE = 0,
  LEG_LEFT_HIND = 1,
  LEG_RIGHT_FORE = 2,
  LEG_RIGHT_HIND = 3
};

UPennWalkMC::UPennWalkMC()
	: MotionCommand(), xVel(0), yVel(0), aVel(0),
		// Default stance parameters:
		body_tilt(STANCE_BODY_TILT), shoulder_height(STANCE_SHOULDER_HEIGHT),
		fore_x0(STANCE_FORE_X0), fore_y0(STANCE_FORE_Y0),
		hind_x0(STANCE_HIND_X0), hind_y0(STANCE_HIND_Y0),
		// Default walk parameters:
		walk_phase(0),walk_phase_direction(1),
		walk_quarter_period(WALK_QUARTER_PERIOD), walk_max_distance(WALK_MAX_DISTANCE),
		walk_fore_lift_initial(WALK_FORE_LIFT_INITIAL), walk_fore_lift_final(WALK_FORE_LIFT_FINAL),
		walk_hind_lift_initial(WALK_HIND_LIFT_INITIAL), walk_hind_lift_final(WALK_HIND_LIFT_FINAL),
		walk_fore_xmin(WALK_FORE_XMIN), walk_fore_xmax(WALK_FORE_XMAX),
		walk_fore_ymin(WALK_FORE_YMIN), walk_fore_ymax(WALK_FORE_YMAX),
		walk_hind_xmin(WALK_HIND_XMIN), walk_hind_xmax(WALK_HIND_XMAX),
		walk_hind_ymin(WALK_HIND_YMIN), walk_hind_ymax(WALK_HIND_YMAX)
{
  for (int i = 0; i < 4; i++) {
    walk_current_x[i] = 0.0;
    walk_current_y[i] = 0.0;
  }
}

void
UPennWalkMC::SetLegJoints(double * x) {
#ifdef TGT_HAS_REK_LEGS
	motman->setOutput(this,LFrLegOffset+RotatorOffset,(float)x[LEG_LEFT_FORE*JointsPerLeg+0]);
	motman->setOutput(this,LFrLegOffset+ElevatorOffset,(float)x[LEG_LEFT_FORE*JointsPerLeg+1]);
	motman->setOutput(this,LFrLegOffset+KneeOffset,(float)x[LEG_LEFT_FORE*JointsPerLeg+2]);
	
	motman->setOutput(this,RFrLegOffset+RotatorOffset,(float)x[LEG_RIGHT_FORE*JointsPerLeg+0]);
	motman->setOutput(this,RFrLegOffset+ElevatorOffset,(float)x[LEG_RIGHT_FORE*JointsPerLeg+1]);
	motman->setOutput(this,RFrLegOffset+KneeOffset,(float)x[LEG_RIGHT_FORE*JointsPerLeg+2]);
	
	motman->setOutput(this,LBkLegOffset+RotatorOffset,(float)x[LEG_LEFT_HIND*JointsPerLeg+0]);
	motman->setOutput(this,LBkLegOffset+ElevatorOffset,(float)x[LEG_LEFT_HIND*JointsPerLeg+1]);
	motman->setOutput(this,LBkLegOffset+KneeOffset,(float)x[LEG_LEFT_HIND*JointsPerLeg+2]);
	
	motman->setOutput(this,RBkLegOffset+RotatorOffset,(float)x[LEG_RIGHT_HIND*JointsPerLeg+0]);
	motman->setOutput(this,RBkLegOffset+ElevatorOffset,(float)x[LEG_RIGHT_HIND*JointsPerLeg+1]);
	motman->setOutput(this,RBkLegOffset+KneeOffset,(float)x[LEG_RIGHT_HIND*JointsPerLeg+2]);
#endif
}

void
UPennWalkMC::SetStanceParameters(double bodyTilt, double shoulderHeight,
			    double foreX0, double foreY0,
			    double hindX0, double hindY0) {
  body_tilt = bodyTilt;
  shoulder_height = shoulderHeight;
  fore_x0 = foreX0;
  fore_y0 = foreY0;
  hind_x0 = hindX0;
  hind_y0 = hindY0;
}

void
UPennWalkMC::SetWalkSpeeds(int quarterPeriod, double maxDistance,
			  double foreLiftInitial, double foreLiftFinal,
			  double hindLiftInitial, double hindLiftFinal) {
  walk_quarter_period = quarterPeriod;
  walk_max_distance = maxDistance;
  walk_fore_lift_initial = foreLiftInitial;
  walk_fore_lift_final = foreLiftFinal;
  walk_hind_lift_initial = hindLiftInitial;
  walk_hind_lift_final = hindLiftFinal;
}

void
UPennWalkMC::SetWalkWorkspace(double foreXMin, double foreXMax,
			 double foreYMin, double foreYMax,
			 double hindXMin, double hindXMax,
			 double hindYMin, double hindYMax) {
  walk_fore_xmin = foreXMin;
  walk_fore_xmax = foreXMax;
  walk_fore_ymin = foreYMin;
  walk_fore_ymax = foreYMax;

  walk_hind_xmin = hindXMin;
  walk_hind_xmax = hindXMax;
  walk_hind_ymin = hindYMin;
  walk_hind_ymax = hindYMax;
}

// Calculate 12 leg joint angles from leg positions
// Note positions are relative to stance parameters
// so all zero inputs -> stance angles
void
UPennWalkMC::LegPositionsToAngles(double *a)
{
  double cosTilt = cos(body_tilt);
  double sinTilt = sin(body_tilt);

  double foreHeight = shoulder_height;
  double hindHeight = shoulder_height - BODY_LENGTH*sinTilt;

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    double posX=0, posY=0, posZ=0;
    double dUpperZ = 0.0, dUpperY = 0.0, dLowerZ = 0.0, dLowerY = 0.0;

    switch (iLeg) {
    case LEG_LEFT_FORE:
      // Left Fore: Reverse x
      a[0] -= fore_x0;
      a[1] += fore_y0;
      a[2] -= foreHeight;

      posX = -(a[0]);
      posY = (cosTilt*a[1]+sinTilt*a[2]);
      posZ = (-sinTilt*a[1]+cosTilt*a[2]);
      dUpperZ = LEG_FORE_UPPER_Z;
      dUpperY = LEG_FORE_UPPER_Y;
      dLowerZ = LEG_FORE_LOWER_Z;
      dLowerY = LEG_FORE_LOWER_Y;
      break;
    case LEG_LEFT_HIND:
      // Left Hind: Reverse x,y
      a[0] -= hind_x0;
      a[1] += hind_y0;
      a[2] -= hindHeight;

      posX = -(a[0]);
      posY = -(cosTilt*a[1]+sinTilt*a[2]);
      posZ = (-sinTilt*a[1]+cosTilt*a[2]);

      dUpperZ = LEG_HIND_UPPER_Z;
      dUpperY = LEG_HIND_UPPER_Y;
      dLowerZ = LEG_HIND_LOWER_Z;
      dLowerY = LEG_HIND_LOWER_Y;
      break;
    case LEG_RIGHT_FORE:
      // Right Fore:
      a[0] += fore_x0;
      a[1] += fore_y0;
      a[2] -= foreHeight;

      posX = (a[0]);
      posY = (cosTilt*a[1]+sinTilt*a[2]);
      posZ = (-sinTilt*a[1]+cosTilt*a[2]);

      dUpperZ = LEG_FORE_UPPER_Z;
      dUpperY = LEG_FORE_UPPER_Y;
      dLowerZ = LEG_FORE_LOWER_Z;
      dLowerY = LEG_FORE_LOWER_Y;
      break;
    case LEG_RIGHT_HIND:
      // Right Hind: Reverse y
      a[0] += hind_x0;
      a[1] += hind_y0;
      a[2] -= hindHeight;

      posX = (a[0]);
      posY = -(cosTilt*a[1]+sinTilt*a[2]);
      posZ = (-sinTilt*a[1]+cosTilt*a[2]);

      dUpperZ = LEG_HIND_UPPER_Z;
      dUpperY = LEG_HIND_UPPER_Y;
      dLowerZ = LEG_HIND_LOWER_Z;
      dLowerY = LEG_HIND_LOWER_Y;
      break;
    default:
      cerr << "UPennWalkMC::LegPositionsToAngles(): Unknown leg.\n" << endl;
    }

    double dUpper = sqrt(dUpperY*dUpperY+dUpperZ*dUpperZ);
    double angleUpper = tan(dUpperY/dUpperZ);  // Positive

    double dLower = sqrt(dLowerY*dLowerY+dLowerZ*dLowerZ);
    double angleLower = tan(dLowerY/dLowerZ);  // Negative

    double posSumSq = posX*posX+posY*posY+posZ*posZ;
    
    double cosJ3 = .5*(posSumSq-dUpper*dUpper-dLower*dLower)/(dUpper*dLower);
    cosJ3 = clip(cosJ3, -1.0, 1.0);
    // Correct for angle offsets of leg segments
    a[2] = acos(cosJ3)+angleUpper-angleLower;

    double aZ = -dUpperZ-dLower*cos(a[2]+angleLower);
    double aY = dUpperY+dLower*sin(a[2]+angleLower);

    double sinJ2 = -posX/aZ;
    sinJ2 = clip(sinJ2, -1.0, 1.0);
    a[1] = asin(sinJ2);

    double J1a = atan2(aZ*cos(a[1]), aY);
    double J1b = atan2(posZ, posY);
    double J1 = J1b-J1a;
    while (J1 > PI) J1 -= 2*PI;
    while (J1 < -PI) J1 += 2*PI;
    a[0] = J1;

    a += 3;
  }
}

void
UPennWalkMC::StandLegs(double x/*=0*/, double y/*=0*/, double z/*=0*/)
{
  static double leg_joints[NUM_LEG_JOINT];
  double *a = leg_joints;

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    double center_x = 0, center_y = 0;
    switch (iLeg) {
    case LEG_LEFT_FORE:
      center_x = -fore_x0;
      center_y = fore_y0;
      break;
    case LEG_LEFT_HIND:
      center_x = -hind_x0;
      center_y = hind_y0;
      break;
    case LEG_RIGHT_FORE:
      center_x = fore_x0;
      center_y = fore_y0;
      break;
    case LEG_RIGHT_HIND:
      center_x = hind_x0;
      center_y = hind_y0;
      break;
    default:
      cerr << "UPennWalkMC::StandLegs(): Unknown leg.\n" << endl;
    }

    a[0] = -x;
    a[1] = -y;
    a[2] = -z;

    walk_current_x[iLeg] = center_x - x;
    walk_current_y[iLeg] = center_y - y;

    // Advance pointer to next leg angles:
    a += 3;
  }

  LegPositionsToAngles(leg_joints);
  SetLegJoints(leg_joints);

}

int
UPennWalkMC::GetWalkPhase() {
  return walk_phase;
}

void
UPennWalkMC::SetWalkPhase(int phase) {
  walk_phase = phase;
}


void
UPennWalkMC::WalkLegs(double xWalk/*=0.0*/, double yWalk/*=0.0*/, double aWalk/*=0.0*/)
{
  // Check to see if legs should stand rather than walk
  if ((walk_phase == 0) &&
      (xWalk == 0.0) && (yWalk == 0.0) && (aWalk == 0.0)) {
    //StandLegs(0, 0, 0);
    return;
  }
  
  static double leg_joints[NUM_LEG_JOINT];
  double *a = leg_joints;
  bool switch_phase_direction = false;

  //  double afactor = .7*BODY_LENGTH*aWalk; // 1/2*sqrt(2)
  double afactor = .85*BODY_LENGTH*aWalk; // 1/2*sqrt(2)
  double rnorm = sqrt(afactor*afactor + xWalk*xWalk + yWalk*yWalk);
  if (rnorm > walk_max_distance) {
    double scale = walk_max_distance/rnorm;
    aWalk *= scale;
    xWalk *= scale;
    yWalk *= scale;
  }

  //  printf("WalkLegs: phase = %d, phase_direction = %d\n", walk_phase, walk_phase_direction);
  int phase_diff_from_switch = walk_quarter_period+walk_phase_direction*walk_phase;
  int phase_diff_to_switch = walk_quarter_period-walk_phase_direction*walk_phase;

  double half_width = .5*BODY_WIDTH;
  double half_length = .5*BODY_LENGTH*cos(body_tilt);

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    double center_x = 0, center_y = 0; // Stance position center
    double leg_offset_x = 0, leg_offset_y = 0; // Relative to center of body
    double leg_lift_initial = 0, leg_lift_final = 0; // Lift heights
    double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0; // Workspace limits relative to shoulder
    int leg_sign = 1;

    double current_x = walk_current_x[iLeg];
    double current_y = walk_current_y[iLeg];
    
    switch (iLeg) {
    case LEG_LEFT_FORE:
      leg_sign = 1;
      center_x = -fore_x0;
      center_y = fore_y0;

      leg_offset_x = -half_width+center_x;
      leg_offset_y = half_length+center_y;

      leg_lift_initial = walk_fore_lift_initial;
      leg_lift_final = walk_fore_lift_final;

      xmin = -walk_fore_xmax; xmax = -walk_fore_xmin;
      ymin = walk_fore_ymin; ymax = walk_fore_ymax;
      break;
    case LEG_LEFT_HIND:
      leg_sign = -1;
      center_x = -hind_x0;
      center_y = hind_y0;

      leg_offset_x = -half_width+center_x;
      leg_offset_y = -half_length+center_y;

      leg_lift_initial = walk_hind_lift_initial;
      leg_lift_final = walk_hind_lift_final;

      xmin = -walk_hind_xmax; xmax = -walk_hind_xmin;
      ymin = walk_hind_ymin; ymax = walk_hind_ymax;
      break;
    case LEG_RIGHT_FORE:
      leg_sign = -1;
      center_x = fore_x0;
      center_y = fore_y0;

      leg_offset_x = half_width+center_x;
      leg_offset_y = half_length+center_y;

      leg_lift_initial = walk_fore_lift_initial;
      leg_lift_final = walk_fore_lift_final;

      xmin = walk_fore_xmin; xmax = walk_fore_xmax;
      ymin = walk_fore_ymin; ymax = walk_fore_ymax;
      break;
    case LEG_RIGHT_HIND:
      leg_sign = 1;
      center_x = hind_x0;
      center_y = hind_y0;

      leg_offset_x = half_width+center_x;
      leg_offset_y = -half_length+center_y;

      leg_lift_initial = walk_hind_lift_initial;
      leg_lift_final = walk_hind_lift_final;

      xmin = walk_hind_xmin; xmax = walk_hind_xmax;
      ymin = walk_hind_ymin; ymax = walk_hind_ymax;
      break;
    default:
      cerr << "UPennWalkMC::WalkLegs(): Unknown leg.\n" << endl;
    }

    // Make relative to stance center:
    current_x -= center_x;
    current_y -= center_y;
    xmin -= center_x;
    xmax -= center_x;
    ymin -= center_y;
    ymax -= center_y;

    double dx, dy;

    // leg is up if leg_sign == walk_phase_direction
    if (leg_sign == walk_phase_direction) {
      // Leg is up
      if (phase_diff_from_switch > 0) {

	dx = xWalk + (cos(aWalk)-1)*leg_offset_x-sin(aWalk)*leg_offset_y;
	dy = yWalk + sin(aWalk)*leg_offset_x+(cos(aWalk)-1)*leg_offset_y;

	double destination_x = walk_quarter_period*dx;
	double destination_y = walk_quarter_period*dy;

      //      printf("Leg %d destination: %g, %g\n", iLeg, destination_x, destination_y);
	dx = (destination_x-current_x)/(phase_diff_to_switch+1);
	dy = (destination_y-current_y)/(phase_diff_to_switch+1);

	current_x += dx;
	current_y += dy;
      }

      current_x = clip(current_x, xmin, xmax);
      current_y = clip(current_y, ymin, ymax);

      a[0] = current_x;
      a[1] = current_y;
      a[2] = (phase_diff_from_switch*leg_lift_final+
	      phase_diff_to_switch*leg_lift_initial)/(2*walk_quarter_period);

    }
    else {
      // Leg is down

      leg_offset_x += current_x;
      leg_offset_y += current_y;
      dx = xWalk + (cos(aWalk)-1)*leg_offset_x-sin(aWalk)*leg_offset_y;
      dy = yWalk + sin(aWalk)*leg_offset_x+(cos(aWalk)-1)*leg_offset_y;

      current_x -= dx; // Travels opposite direction as dx
      current_y -= dy; // Travels opposite direction as dy

      a[0] = current_x;
      a[1] = current_y;
      a[2] = 0;

      // Check if leg is going outside workspace:
      // This check is after writing leg positions to give extra reach
      current_x = clip(current_x, xmin, xmax);
      current_y = clip(current_y, ymin, ymax);

      /*
      Old code to advance phase to speed up cycle
      if (walk_current_x[iLeg] < xmin) {
	walk_current_x[iLeg] = xmin;
	// Only switch direction after quarter_period has elapsed:
	if (walk_phase_direction*walk_phase > 0)
	  switch_phase_direction = true;
      }
      else if (walk_current_x[iLeg] > xmax) {
	walk_current_x[iLeg] = xmax;
	if (walk_phase_direction*walk_phase > 0)
	  switch_phase_direction = true;
      }
      if (walk_current_y[iLeg] < ymin) {
	walk_current_y[iLeg] = ymin;
	if (walk_phase_direction*walk_phase > 0)
	  switch_phase_direction = true;
      }
      else if (walk_current_y[iLeg] > ymax) {
	walk_current_y[iLeg] = ymax;
	if (walk_phase_direction*walk_phase > 0)
	  switch_phase_direction = true;
      }
      */

    }

    //    printf("iLeg = %d: %g %g %g\n", iLeg, a[0],a[1],a[2]);

    // Store current leg positions in coordinates relative to shoulder:
    walk_current_x[iLeg] = current_x+center_x;
    walk_current_y[iLeg] = current_y+center_y;

    // Shift pointer to next leg:
    a += 3;
  }

  /*
  if (switch_phase_direction)
  printf("WalkLegs: workspace switch: phase = %d, phase_direction = %d\n", walk_phase, walk_phase_direction);
  */

  // walk_phase goes oscillates between
  // -walk_quarter_period to +walk_quarter_period:
  walk_phase += walk_phase_direction;
  if ((walk_phase > walk_quarter_period) ||
      (walk_phase < -walk_quarter_period)) {
    switch_phase_direction = true;
  }

  if (switch_phase_direction) {
    // Switch legs:
    walk_phase = walk_phase_direction*walk_quarter_period;
    walk_phase_direction = -walk_phase_direction;
  }
  
  LegPositionsToAngles(leg_joints);
  SetLegJoints(leg_joints);

  // Update odometry:
  // Commenting out to put odometry control in Perl:
  //  Self.world->AddMotionUpdate(xWalk, yWalk, aWalk);
  
}




/*! @file
 * @brief Defines UPennWalkMC, which uses the UPennalizers' 2004 RoboCup code to compute walking gaits
 * @author UPennalizers 2004 (Creator)
 * @author ejt (Ported)
 *
 * The UPennalizers code was released under the GPL:\n
 *  ------------------------------------------------------------------------- \n
 *    This software is distributed under the GNU General Public License,      \n
 *    version 2.  If you do not have a copy of this licence, visit            \n
 *    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,       \n
 *    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed       \n
 *    in the hope that it will be useful, but WITHOUT ANY WARRANTY,           \n
 *    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.          \n
 *  ------------------------------------------------------------------------- \n
 */
