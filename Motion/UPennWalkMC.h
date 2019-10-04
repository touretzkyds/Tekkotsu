//-*-c++-*-
#ifndef INCLUDED_UPennWalkMC_h_
#define INCLUDED_UPennWalkMC_h_

#include "Motion/MotionCommand.h"
#include "Motion/MotionManager.h"

//! Uses the UPennalizers' 2004 RoboCup code to compute walking gaits

/*! This class is ported from University of Pennsylvania's 2004 Robosoccer entry, and falls under their license:
    =========================================================================
    This software is distributed under the GNU General Public License,
	  version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
	  including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    ========================================================================= */

class UPennWalkMC : public MotionCommand {
public:
	//! constructor
	UPennWalkMC();

	virtual int updateOutputs() {
		/*if(sqrt(xVel*xVel+yVel*yVel+aVel*aVel)<.01)
			StandLegs(xVel,yVel,aVel);
			else */
		WalkLegs(xVel,yVel,aVel);
		return NumLegJoints;
	}

	virtual int isDirty() { return true; }
	virtual int isAlive() { return true; }

	void setTargetVelocity(float x, float y, float a) { xVel=-y; yVel=x; aVel=a; }

protected:
	void SetLegJoints(double * x);

	inline static double clip(double x, double min=-1.0, double max=1.0)
	{
		if (x < min) return min;
		if (x > max) return max;
		return x;
	}

	void SetStanceParameters(double bodyTilt, double shoulderHeight,
	                         double foreX0, double foreY0,
	                         double hindX0, double hindY0);
	
	void SetWalkSpeeds(int quarterPeriod, double maxDistance,
	                   double foreLiftInitial, double foreLiftFinal,
	                   double hindLiftInitial, double hindLiftFinal);
	
	void SetWalkWorkspace(double foreXMin, double foreXMax,
	                      double foreYMin, double foreYMax,
	                      double hindXMin, double hindXMax,
	                      double hindYMin, double hindYMax);
	
	//! Calculate 12 leg joint angles from leg positions
	/*! Note positions are relative to stance parameters
	 *  so all zero inputs -> stance angles */
	void LegPositionsToAngles(double *a);

	void StandLegs(double x=0, double y=0, double z=0);

	int  GetWalkPhase();

	void SetWalkPhase(int phase);

	void WalkLegs(double xWalk=0.0, double yWalk=0.0, double aWalk=0.0);

	float xVel;
	float yVel;
	float aVel;

	double body_tilt, shoulder_height;
	double fore_x0, fore_y0;
	double hind_x0, hind_y0;

	int walk_phase, walk_phase_direction, walk_quarter_period;
	double walk_max_distance;
	double walk_fore_lift_initial, walk_fore_lift_final;
	double walk_hind_lift_initial, walk_hind_lift_final;
	double walk_current_x[NumLegs], walk_current_y[NumLegs];
	double walk_fore_xmin, walk_fore_xmax, walk_fore_ymin, walk_fore_ymax;
	double walk_hind_xmin, walk_hind_xmax, walk_hind_ymin, walk_hind_ymax;

};

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

#endif

