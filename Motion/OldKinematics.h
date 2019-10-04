//-*-c++-*-
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

#ifndef __CMPACK_KINEMATICS_H__
#define __CMPACK_KINEMATICS_H__

#include "Geometry.h"
#include "Shared/Util.h"

extern const double rotator_min ;
extern const double rotator_max ;
extern const double shoulder_min;
extern const double shoulder_max;
extern const double knee_max    ;
extern const double knee_min    ;

extern const double rotator_kmin ;
extern const double rotator_kmax ;
extern const double shoulder_kmin;
extern const double shoulder_kmax;
extern const double knee_kmax    ;
extern const double f_knee_kmin  ;
extern const double h_knee_kmin  ;

extern const double tail_min;
extern const double tail_max;

extern const double head_tilt_min;
extern const double head_tilt_max;
extern const double head_pan_min ;
extern const double head_pan_max ;
extern const double head_roll_min;
extern const double head_roll_max;

//! holds the current location of the body, as a delta from when walking started
/*! @todo get rid of this */
struct BodyPosition{
	//! constructor
	BodyPosition() : loc(), angle() {}
  vector3d loc;   //!< position of the center of the body
	vector3d angle; //!< angle of the center of the body
};

#ifdef TGT_ERS7
const vector3d body_to_neck    ( 67.50,  0.00,  19.50);
const vector3d neck_to_nod     ( 00.00,  0.00,  80.00);
const vector3d nod_to_camera   ( 81.06,  0.00, -14.60);
const vector3d neck_to_camera  ( 81.06,  0.00,  65.40); //only correct @nod==0
#else
const vector3d body_to_neck    ( 75.00,  0.00,  50.00);
const vector3d neck_to_camera  ( 66.60,  0.00,  48.00);
//const vector3d neck_to_camera  ( 65.00,  0.00,  48.00);
#endif

void KinClearErrors();
int KinGetErrors();

void GetLegAngles(double *angles,vector3d target,int leg);
void GetLegAngles(double *angles,vector3d target[4],
                  double body_angle,double body_height);
void GetLegAngles(double *angles,vector3d target[4],BodyPosition &bp);

void GetLegAngles(double *angles,vector3d target,BodyPosition &bp,int leg);
void GetLegPosition(vector3d& p,const double* ang,int leg);
void GetLegPosition(vector3d& p, const double* ang, BodyPosition &bp,int leg);

void GetBodyLocation(vector3d &ball,vector3d &toe,const double *ang,int leg);

// get the tilt,pan,roll angles to point the head towards the target assuming 
//   the given body_angle/body_height
void GetHeadAngles(double *angles,vector3d target,
                   double body_angle,double body_height);
// converts the camera relative position "point" to a robot centric (under base of neck) position
//  using the given head angles (tilt/pan/roll) and body_angle and body_height
vector3d RunForwardModel(double *angles, 
                         double body_angle, double body_height, 
                         vector3d point);
// gets the location of the camera and basis vectors corresponding to the directions of the camera's
//  z,-y,x corrdinate axis
void GetHeadPosition(vector3d &location, vector3d &direction, vector3d &up, vector3d &right,
                     double *angles, double body_angle, double body_height);


/*! @file
 * @brief Functions to provide kinematics calculations
 * @author CMU RoboSoccer 2001-2002 (Creator)
 * 
 * @verbinclude CMPack_license.txt
 */

#endif
// __KINEMATICS_H__
