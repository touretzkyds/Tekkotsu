//This class is ported from Carnegie Mellon's 2001 Robosoccer entry, and falls under their license:
/*=========================================================================
    CMPack'02 Source Code Release for OPEN-R SDK v1.0
    Copyright (C) 2002 Multirobot Lab [Project Head: Manuela Veloso]
    School of Computer Science, Carnegie Mellon University
  -------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  -------------------------------------------------------------------------
    Additionally licensed to Sony Corporation under the following terms:

    This software is provided by the copyright holders AS IS and any
    express or implied warranties, including, but not limited to, the
    implied warranties of merchantability and fitness for a particular
    purpose are disclaimed.  In no event shall authors be liable for
    any direct, indirect, incidental, special, exemplary, or consequential
    damages (including, but not limited to, procurement of substitute
    goods or services; loss of use, data, or profits; or business
    interruption) however caused and on any theory of liability, whether
    in contract, strict liability, or tort (including negligence or
    otherwise) arising in any way out of the use of this software, even if
    advised of the possibility of such damage.
  =========================================================================
*/

#ifdef PLATFORM_LINUX
#include <stdlib.h>
#include <stdio.h>
#endif

#include <math.h>

#include "OldKinematics.h"
//#include "Wireless/Socket.h"
#include "Shared/RobotInfo.h"

// #define DEBUG
#define RAD(deg) (((deg) * M_PI ) / 180.0)
#define DEG(rad) (((rad) * 180.0) / M_PI )

#ifdef TGT_ERS7
// ERS-7 Leg Parameters
const vector3d f_body_to_shoulder       ( 65.00, 62.50,   0.00);
const vector3d f_leg_shoulder_to_knee   (  9.00,  4.70, -69.50);
const vector3d f_leg_knee_to_ball       (-11.24,  0.00, -76.07);

const vector3d h_body_to_shoulder       (-65.00, 62.50,   0.00);
const vector3d h_leg_shoulder_to_knee   ( -9.00,  4.70, -69.50);
const vector3d h_leg_knee_to_ball       ( 19.80,  0.00, -76.90);
#else
// ERS-210/220 Leg Parameters
const vector3d f_body_to_shoulder       ( 59.50, 59.20,   0.00);
const vector3d f_leg_shoulder_to_knee   ( 12.80,  0.50, -64.00);
const vector3d f_leg_knee_to_ball       (-18.88,  0.00, -66.00);

const vector3d h_body_to_shoulder       (-59.50, 59.20,   0.00);
const vector3d h_leg_shoulder_to_knee   (-12.80,  0.50, -64.00);
const vector3d h_leg_knee_to_ball       ( 29.36,  0.00, -70.88);
/*
	knee_to_ball was found to be possible to compute from diagrams, 4/11/04 ejt
	previous values by original author are as follows:
  18 : measured
  67.23 = sqrt(69.6^2 - 18^2)
  74.87 = sqrt(77^2 - 18^2)
  0.2616 = asin(18 / 69.6)
  0.2316 = asin(18 / 77)
	const vector3d f_leg_knee_to_ball       (-18.00,  0.00, -67.23);
	const vector3d h_leg_knee_to_ball       ( 18.00,  0.00, -74.87);
*/
#endif

/* ERS-110
const vector3d f_body_to_shoulder       ( 44.85, 26.50,   0.00);
const vector3d f_leg_shoulder_to_knee   ( 13.00,  5.50, -61.00);
const vector3d f_leg_knee_to_ball       ( -9.50,  1.06, -58.00); // ??

const vector3d h_body_to_shoulder       (-44.85, 26.50,   0.00);
const vector3d h_leg_shoulder_to_knee   (-13.00,  5.50, -61.00);
const vector3d h_leg_knee_to_ball       ( 19.00,  1.06, -69.00); // ??
*/

#ifdef PLATFORM_LINUX
void print(double *angles,vector3d pos)
{
  printf("A(%7.4f,%7.4f,%7.4f) P(%7.2f,%7.2f,%7.2f)\n",
	 angles[0],angles[1],angles[2],
	 pos.x,pos.y,pos.z);
}
#endif


const vector3d f_upper = f_leg_shoulder_to_knee;
const vector3d f_lower = f_leg_knee_to_ball;
const vector3d h_upper = h_leg_shoulder_to_knee;
const vector3d h_lower = h_leg_knee_to_ball;

//==== Globals ====//
#ifdef PLATFORM_LINUX
int g_leg;
vector3d g_target;
#endif

static int errors;

void KinClearErrors()
{
  errors = 0;
}

int KinGetErrors()
{
  return(errors);
}

double GetTrigAngle(double a,double b,double d,double mn,double mx,bool add)
// Analytic solution to a*sin(x) + b*cos(x) = d
{
  double theta;
  double t,f,c;
  int err;

  f = d / sqrt(a*a + b*b);
  err = 0;

  if(fabs(f) > 1.0){
#ifdef PLATFORM_LINUX
    /*
    printf("Out of range (distance=%g) leg=%d target=(%g,%g,%g)\n",f,
           g_leg,g_target.x,g_target.y,g_target.z);
    */
#endif
    f = (f > 0.0)? 1.0 : -1.0;
    err = 1;
  }

  t = atan2(a,b);
  c = acos(f);

  theta = add? (t + c) : (t - c);

  if(theta < mn){
#ifdef PLATFORM_LINUX
    /*
    printf("Out of range (angle to small) leg=%d target=(%g,%g,%g)\n",
           g_leg,g_target.x,g_target.y,g_target.z);
    */
#endif
    errors++;
    return(mn);
  }else if(theta > mx){
#ifdef PLATFORM_LINUX
    /*
    printf("Out of range (angle to large) leg=%d target=(%g,%g,%g)\n",
           g_leg,g_target.x,g_target.y,g_target.z);
    */
#endif
    errors++;
    return(mx);
  }else{
    errors += err;
    return(theta);
  }
}

/*
Angle Limits:
           ===Software====  ==Mechanical===
  Rotator  [-117.0, 117.0]  [-120.0, 120.0]
  Shoulder [ -11.0,  97.0]  [ -14.0, 100.0]
  Knee     [ -27.0, 147.0]  [ -30.0, 150.0]
*/

const double rotator_min  = RAD(-117.0);
const double rotator_max  = RAD( 117.0);
const double shoulder_min = RAD( -11.0);
const double shoulder_max = RAD(  97.0);
const double knee_max     = RAD( 147.0);
const double knee_min     = RAD( -27.0);

const double rotator_kmin  = RAD(-90.0);
const double rotator_kmax  = RAD( 90.0);
const double shoulder_kmin = shoulder_min;
const double shoulder_kmax = RAD( 90.0);
const double knee_kmax     = knee_max;
const double f_knee_kmin   = 0.2616;
const double h_knee_kmin   = 0.2316;

const double tail_min = RAD(-22);
const double tail_max = RAD( 22);

const double head_tilt_min = RAD(-88.5);
const double head_tilt_max = RAD( 43.0);
const double head_pan_min  = RAD(-89.6);
const double head_pan_max  = RAD( 89.6);
const double head_roll_min = RAD(-29.0);
const double head_roll_max = RAD( 29.0);


void GetLegAngles(double *angles,vector3d target,int leg)
{
  vector3d targ,pos;
  double knee,shoulder,rotator;
  double a,b,d,dist;
  bool front = leg < 2;

#ifdef PLATFORM_LINUX
  g_leg    = leg;
  g_target = target;
  // printf("GLA: target=(%g,%g,%g)\n",target.x,target.y,target.z);
#endif

  knee = shoulder = rotator = 0.0;

  if(leg % 2) target.y = -target.y;

  if(front){
    targ = target - f_body_to_shoulder;
    dist = targ.sqlength();

    // Calculate knee angle
    a = -2*(f_upper.x*f_lower.z - f_upper.z*f_lower.x);
    b =  2*(f_upper.x*f_lower.x + f_upper.z*f_lower.z);
    d = (dist - f_upper.sqlength() - f_lower.sqlength() - 2*f_upper.y*f_lower.y);
    knee = GetTrigAngle(a,b,d,f_knee_kmin,knee_kmax,true);

    // Calculate shoulder angle
    pos = f_leg_shoulder_to_knee + f_leg_knee_to_ball.rotate_y(-knee);
    shoulder = GetTrigAngle(-pos.z,pos.y,targ.y,shoulder_kmin,shoulder_kmax,false);

    // Calculate rotator angle
    // pos = pos.rotate_x(shoulder);
    pos.z = sin(shoulder)*pos.y + cos(shoulder)*pos.z;
    rotator = GetTrigAngle(-pos.z,pos.x,targ.x,rotator_min,rotator_max,target.z > 0.0);

#ifdef DEBUG
    // Test
		//    pos = (f_leg_shoulder_to_knee + f_leg_knee_to_ball.rotate_y(-knee))
		//           .rotate_x(shoulder).rotate_y(-rotator);
		//    printf("D(%f,%f,%f)\n",targ.x - pos.x,targ.y - pos.y,targ.z - pos.z);
#endif
  }else{
    targ = target - h_body_to_shoulder;
    dist = targ.sqlength();

    // Calculate knee angle
    a = 2*(h_upper.x*h_lower.z - h_upper.z*h_lower.x);
    b = 2*(h_upper.x*h_lower.x + h_upper.z*h_lower.z);
    d = (dist - h_upper.sqlength() - h_lower.sqlength() - 2*h_upper.y*h_lower.y);
    knee = GetTrigAngle(a,b,d,h_knee_kmin,knee_kmax,true);

    // Calculate shoulder angle
    pos = h_leg_shoulder_to_knee + h_leg_knee_to_ball.rotate_y(knee);
    shoulder = GetTrigAngle(-pos.z,pos.y,targ.y,shoulder_kmin,shoulder_kmax,false);

    // Calculate rotator angle
    pos.z = sin(shoulder)*pos.y + cos(shoulder)*pos.z;
    rotator = GetTrigAngle(-pos.z,-pos.x,-targ.x,rotator_min,rotator_max,target.z > 0.0);

    /*
    if(fabs(theta - rotator) > 0.01){
      printf("ERROR(%f - %f = %f)\n",theta,rotator,theta - rotator);
    }
    */

#ifdef DEBUG
    // Test
		//    pos = (h_leg_shoulder_to_knee + h_leg_knee_to_ball.rotate_y(angles[2]))
		//           .rotate_x(angles[1]).rotate_y(angles[0]);
		//    printf("D(%f,%f,%f)\n",targ.x - pos.x,targ.y - pos.y,targ.z - pos.z);
#endif
  }

  angles[0] = rotator;
  angles[1] = shoulder;
  angles[2] = knee;
}

void GetLegAngles(double *angles,vector3d target[4],
                  double body_angle,double body_height)
{
  vector3d p;
  int i;

  for(i=0; i<4; i++){
    p = target[i];
    p.z -= body_height;
    p = p.rotate_y(-body_angle);
    GetLegAngles(angles + 3*i,p,i);
  }
}

void GetLegAngles(double *angles,vector3d target[4],BodyPosition &bp)
{
  vector3d p;
  int i;

  for(i=0; i<4; i++){
    p = (target[i] - bp.loc).rotate_z(bp.angle.z).rotate_y(-bp.angle.y);
    GetLegAngles(angles + 3*i,p,i);
  }
}

void GetLegAngles(double *angles,vector3d target,BodyPosition &bp,int leg)
{
  vector3d p;

  p = (target - bp.loc).rotate_z(bp.angle.z).rotate_y(-bp.angle.y);
  GetLegAngles(angles,p,leg);
}


void GetLegPosition(vector3d& p, const double* ang, int leg)
{
	if(leg < 2){
		p = f_body_to_shoulder +
			(f_leg_shoulder_to_knee + f_leg_knee_to_ball.rotate_y(-ang[2]))
			.rotate_x(ang[1]).rotate_y(-ang[0]);
	}else{
		p = h_body_to_shoulder +
			(h_leg_shoulder_to_knee + h_leg_knee_to_ball.rotate_y( ang[2]))
			.rotate_x(ang[1]).rotate_y( ang[0]);
	}
	
	if(leg % 2)
		p.y = -p.y;
}
void GetLegPosition(vector3d& p, const double* ang, BodyPosition &bp,int leg)
{
	GetLegPosition(p,ang,leg);
	p=p.rotate_y(bp.angle.y).rotate_z(-bp.angle.z)+bp.loc;
}


void GetBodyLocation(vector3d &ball,vector3d &toe,const double *ang,int leg)
// project various parts of foot that could touch by given joint angles
{
  if(leg < 2){
    ball = f_body_to_shoulder +
           (f_leg_shoulder_to_knee + f_leg_knee_to_ball.rotate_y(-ang[2]))
           .rotate_x(ang[1]).rotate_y(-ang[0]);
  }else{
    ball = h_body_to_shoulder +
           (h_leg_shoulder_to_knee + h_leg_knee_to_ball.rotate_y( ang[2]))
           .rotate_x(ang[1]).rotate_y( ang[0]);
  }

  // TODO: toes
  toe = ball;

  if(leg % 2){
    ball.y = -ball.y;
    toe.y = -toe.y;
  }
}

#ifdef TGT_ERS7
//use the current setting for tilt joint, determine nod joint
bool getNodAndPan(double *angles,vector3d target) {
	target=target.rotate_y(angles[0]);

	double dx=target.x-neck_to_nod.x;
	double dy=target.y-neck_to_nod.y;
	double dz=target.z-neck_to_nod.z;
	double elv=atan2(dz,hypot(dx,dy)); //angle from xy plane at nod joint to target position
	double camAng=atan2(nod_to_camera.z,nod_to_camera.x); //angle from xy plane at nod joint to actual camera position
	double tgtDist=sqrt(dx*dx+dy*dy+dz*dz);
	double camDist=hypot(nod_to_camera.x,nod_to_camera.z);

	if(camDist>=tgtDist) {
		for(unsigned int i=0; i<3; i++)
			angles[i]=0;
		return false;
	}
	double angle_base_target_camera=asin(camDist*sin(-camAng)/tgtDist);
	angles[1]=atan2(target.y,target.x);
	angles[2]=elv-2*camAng-angle_base_target_camera;

	//sout->printf("elv=%g camAng=%g dx=%g dy=%g dz=%g abtc=%g nod=%g tilt=%g\n",elv,camAng,dx,dy,dz,angle_base_target_camera,angles[2],angles[0]);

	return true;
}
#endif


void GetHeadAngles(double *angles,vector3d target,
                   double body_angle,double body_height)
{
#ifdef TGT_ERS7
	//***********************************************
	//******************** ERS-7 ********************
	//***********************************************
	if(!finite(angles[0]))
		angles[0]=0;
  vector3d neck = body_to_neck.rotate_y(body_angle);
  target.z -= body_height + neck.z;

	//try using just the nod joint...
	if(!getNodAndPan(angles,target))
		return;

	//we could consider ourselves done if only using nod and pan, but
	//but should use tilt too if out of range of nod

	//if we can't pan far enough, change the tilt
	if(angles[1]<outputRanges[HeadOffset+PanOffset][MinRange] || 
		 angles[1]>outputRanges[HeadOffset+PanOffset][MaxRange]
		 ) {
		double elv=-atan2(target.x,target.z);
		elv=bound(elv,outputRanges[HeadOffset+TiltOffset][MinRange],outputRanges[HeadOffset+TiltOffset][MaxRange]);
		angles[0]=elv;
		if(!getNodAndPan(angles,target))
			return;
	}

	// Now we're going to see if the attempted nod is out of range and use the tilt joint if it is.
	
	// To find the tilt needed to get into view, we want to find the intersection of:
	// Hyperbola of intersection of viewing cone with an xz plane through target (y=target's y coordinate)
	//   ((y-h)/B)^2 - (x/A)^2 = 1
	//     h: y-intercept of cone
	//     A: distance from xz plane of target (target's y coordinate aka dy)
	//     B: drop of top of hyperbola from top of cone (dy*slope)
	// and the circle formed by points the target could be by rotating around y-axis:
	//   x^2 + y^2 == r^2
	//     r: distance from target to y-axis
	// This gives the following equation: (-Sqrt[x^2*B^2/A^2 + B^2] + h)^2 == r^2 - x^2
	// ...plug into Mathematica, solve for x, you get this mess: (% means plus/minus aka +/-)
	//   x = % sqrt( (A^4*(B^2+h^2-r^2) + A^2*B^2*(-B^2+h^2+r^2) % 2*sqrt(A^4*B^2*h^2*(A^4*+B^2*r^2+A^2*(B^2-h^2+r^2)))) / (A^2+B^2)^2 )
	// (4 solutions)
	// Or more simply, solve for y instead of x: ((y - h)^2/B^2 - 1)*A^2 == r^2 - y^2
	//   y = ( A^2*h % sqrt(B^2*(A^4+B^2*r^2+A^2*(B^2-h^2+r^2))) ) / (A^2+B^2)
	// (2 solutions)

	if(angles[2]<outputRanges[HeadOffset+NodOffset][MinRange] || 
		 angles[2]>outputRanges[HeadOffset+NodOffset][MaxRange]
		 ) {
		double nod;
		if(angles[2]<outputRanges[HeadOffset+NodOffset][MinRange])
			nod=outputRanges[HeadOffset+NodOffset][MinRange];
		else
			nod=outputRanges[HeadOffset+NodOffset][MaxRange];
		double rad=hypot(target.x,target.z); // aka 'r'
		double slope=tan(nod);
		double intcpt=neck_to_nod.z+nod_to_camera.z/cos(nod); // aka 'h'
		double A=target.y-neck_to_nod.y;
		double B=A*slope;
		double yval;
		if(angles[2]<outputRanges[HeadOffset+NodOffset][MinRange])
			// too low: use the '-' solution so we get the lower intercept:
			yval = ( A*A*intcpt - sqrt(B*B*(A*A*A*A+B*B*rad*rad+A*A*(B*B-intcpt*intcpt+rad*rad))) ) / (A*A+B*B);
		else
			// too high: use the '+' solution so we get the upper intercept:
			yval = ( A*A*intcpt + sqrt(B*B*(A*A*A*A+B*B*rad*rad+A*A*(B*B-intcpt*intcpt+rad*rad))) ) / (A*A+B*B);
		angles[0]=atan2(target.z,target.x)-asin(yval/rad);
		if(angles[0]<outputRanges[HeadOffset+TiltOffset][MinRange])
			angles[0]=outputRanges[HeadOffset+TiltOffset][MinRange];
		else if(angles[0]>outputRanges[HeadOffset+TiltOffset][MaxRange])
			angles[0]=outputRanges[HeadOffset+TiltOffset][MaxRange];
		getNodAndPan(angles,target);
		//sout->printf("rad=%g slope=%g intcpt=%g A=%g B=%g yval=%g tilt=%g\n",rad,slope,intcpt,A,B,yval,angles[0]);
	}

	/* this is the bad way:
	//ick:
	//solve: sqrt(dy^2+(rad*cos(rot)))==slope*(rad*sin(rot)+intcpt)  for rot
	//mathematica gives:
	// %acos(%sqrt((-dy2*rad2+slope2*rad2*(intcpt2-dy2+rad2)+slope4*(-intcpt2*rad2+rad4)
	//             % (2*sqrt(slope4*intcpt2*rad4*(dy2+rad2+slope2*(-intcpt2+dy2+rad2)))))/((1+slope2)*(1+slope2)*rad4)))
	// where '%' denotes plus/minus aka +/-
	double rad2=rad*rad;
	double rad4=rad2*rad2;
	double slope2=slope*slope;
	double slope4=slope2*slope2;
	double dy2=dy*dy;
	double intcpt2=intcpt*intcpt;
	double p1=-dy2*rad2+slope2*rad2*(intcpt2-dy2+rad2)+slope4*(-intcpt2*rad2+rad4);
	double p2=(2*sqrt(slope4*intcpt2*rad4*(dy2+rad2+slope2*(-intcpt2+dy2+rad2))));
	double p3=(1+slope2)*(1+slope2)*rad4;
	double telv=atan2(target.z,target.x);
	if(p1+p2>0)
	tilt=telv+acos(-sqrt((p1+p2)/p3));
	sout->printf("telv=%g rad=%g slope=%g intcpt=%g p1=%g p2=%g p3=%g tilt=%g\n",telv,rad,slope,intcpt,p1,p2,p3,tilt);
	*/

#else //TGT_ERS210,TGT_ERS220,TGT_ERS2xx

	//***********************************************
	//*************** ERS-210, ERS-220 **************
	//***********************************************

  double tilt,pan;

  vector3d neck;
  double height;

#ifdef PLATFORM_APERIOS
  //pprintf(TextOutputStream,"target (%g,%g,%g) body_angle %g body_height %g\n",
  //        target.x,target.y,target.z,
  //        body_angle,body_height);
#endif

  // translate target so it is relative to base of neck
  neck = body_to_neck.rotate_y(body_angle);
  height = body_height + neck.z;
  target.z -= height;
  
  double target_xz_dist; // distance in robot's xz plane of target

  target_xz_dist=hypot(target.x,target.z);

  // assumes that camera is aligned with base of neck
  // can only see if not too close to neck
  if(target_xz_dist > neck_to_camera.z && target.length() > neck_to_camera.length()) {
    double angle_base_target; // angle from base of neck to target xz
    double angle_base_target_camera; // angle between base and camera from target xz
    
    angle_base_target = atan2(target.z,target.x);
    angle_base_target_camera = asin(neck_to_camera.z/target_xz_dist);
    
    tilt = angle_base_target - angle_base_target_camera + body_angle;
    tilt = bound(tilt,head_tilt_min,head_tilt_max);
    
    double camera_dist_to_target;
    camera_dist_to_target = sqrt(target_xz_dist*target_xz_dist - 
                                 neck_to_camera.z*neck_to_camera.z);
    
    pan = atan2(target.y,camera_dist_to_target);
    pan = bound(pan,head_pan_min,head_pan_max);

#ifdef PLATFORM_APERIOS
    //pprintf(TextOutputStream,"txzd %g abt %g abtc %g tilt %g pan %g\n",
    //        target_xz_dist,angle_base_target,angle_base_target_camera,
    //        tilt,pan);
#endif
  }
  else {
    tilt = pan = 0.0;
  }

  angles[0] = tilt;
  angles[1] = pan;
  angles[2] = 0.0; // roll
	
#endif //TGT_ERS210,TGT_ERS220,TGT_ERS2xx
}

vector3d
RunForwardModel(double *angles, double body_angle, double body_height, vector3d point) {
  double tilt=0.0,pan=0.0,roll=0.0;

  tilt = angles[0];
  pan  = angles[1];
  roll = angles[2];

  point = point.rotate_x(roll);
  point += neck_to_camera;
  point = point.rotate_z(pan);
  point = point.rotate_y(-tilt+body_angle);

  vector3d neck;

  neck = body_to_neck;
  neck = neck.rotate_y(body_angle);
  neck.z += body_height;

  point.z += neck.z;

  return point;
}

// calculates the pose of the camera
// location  = location of camera in robot coordinates relative to point on ground under neck
// direction = unit vector pointing in direction of camera
// up        = unit vector pointing in direction of higher on image
// right     = unit vector pointing in direction of more right on image
void GetHeadPosition(vector3d &location, vector3d &direction, vector3d &up, vector3d &right,
                     double *angles, double body_angle, double body_height)
{
  location = RunForwardModel(angles, body_angle, body_height, vector3d(0.0,0.0,0.0));

  vector3d image_x,image_y,image_z;

  image_x = RunForwardModel(angles, body_angle, body_height, vector3d(0.0,-1.0, 0.0));
  image_y = RunForwardModel(angles, body_angle, body_height, vector3d(0.0, 0.0, 1.0));
  image_z = RunForwardModel(angles, body_angle, body_height, vector3d(1.0, 0.0, 0.0));

  direction = image_z - location;
  direction = direction.norm();

  up = image_y - location;
  up = up.norm();

  right = image_x - location;
  right = right.norm();
}

/*! @file
 * @brief Functions to provide kinematics calculations
 * @author CMU RoboSoccer 2001-2002 (Creator)
 * 
 * @verbinclude CMPack_license.txt
 */

