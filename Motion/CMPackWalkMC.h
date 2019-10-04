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

#ifndef INCLUDED_CMPackWalkMC_h
#define INCLUDED_CMPackWalkMC_h

#include "MotionCommand.h"
#include "Geometry.h"
#include "OldKinematics.h"
#include "Path.h"
#include "Shared/get_time.h"
#include "OutputCmd.h"
#include "Shared/LoadSave.h"

//! A nice walking class from Carnegie Mellon University's 2001 Robosoccer team, modified to fit this framework, see their <a href="../CMPack_license.txt">license</a>
/*! Moves the feet through a looping path in order to walk - default parameters use
 *  a walk low to the ground so you don't walk over the ball.
 *
 *  There are around 50 parameters which control the walk - these are
 *  loaded from a file and can modify almost every aspect of the the
 *  gait.  It's a binary file format, I recommend using our Walk Edit
 *  menu to edit the parameters in real time and get immediate
 *  feedback.  It's a tricky job to find a good set of parameters.
 *
 *  And then, once you have it walking, there's a whole different
 *  problem of actually moving at the speed that's requested.  That's
 *  what the calibration parameters do - map the requested target
 *  speed to the command to pass to the engine so the resulting motion
 *  will hopefully match what you asked for.
 *
 *  You'll probably want to take a look at the setTargetVelocity()
 *  function to control the direction of the walk.
 *
 *  This class is in some dire need of some cleanup - we (Tekkotsu)
 *  didn't write it, have none the less hacked around and added stuff on top
 *  of it.  So pardon the mess, unless you're feeling ambitious to
 *  write your own ;)
 *
 *  This portion of the code falls under CMPack's license:
 *  @verbinclude CMPack_license.txt
 */
class CMPackWalkMC : public MotionCommand, public LoadSave {
public:
	typedef SplinePath<vector3d,float> splinepath; //!<for convenience
	typedef HermiteSplineSegment<vector3d,float> spline; //!<for convenience

	//! holds state about each leg's path
	struct LegWalkState {
		LegWalkState() : airpath(), air(0), cyc(0) {} //!< constructor
		spline airpath; //!< the path to follow
		bool air; //!< true if in the air
		float cyc; //!< % (0,1) of how far along the path we are
	};
	
	//! holds parameters about how to move each leg
	struct LegParam {
		LegParam() : neutral(), lift_vel(), down_vel(), lift_time(0), down_time(0) {} //!< constructor
		vector3d neutral; //!< defines the "neutral" point of each leg - where it is in midstep
		vector3d lift_vel; //!< give the velocities to use when raising the paw
		vector3d down_vel; //!< give the velocities to use when lowering the paw
		double lift_time; //!< the time (as percentage of WalkParam::period) in the cycle to lift (so you can set different offsets between the paws)
		double down_time; //!< the time (as percentage of WalkParam::period) in the cycle to put down (so you can set different offsets between the paws)
	};

	//! holds more general parameters about the walk
	struct WalkParam {
		WalkParam() : body_height(0), body_angle(0), hop(0), sway(0), period(0), useDiffDrive(0), sag(0), reserved() {} //!< constructor
		LegParam leg[4]; //!< a set of LegParam's, one for each leg
		double body_height; //!< the height to hold the body (mm)
		double body_angle; //!< the angle to hold the body (rad - 0 is level)
		double hop;  //!< sinusoidal hop amplitude
		double sway; //!< sinusoidal sway in y direction
		int period; //!< the time between steps
		int useDiffDrive; //!< if non-zero, diff-drive style turning is used instead of rotational turning
		float sag; //!< the amount to sagging to account for when a foot is lifted
		float reserved; //!< just live with it
	};

	//! holds information to correct for slippage, non-idealities
	struct CalibrationParam {
		CalibrationParam(); //!< constructor, sets calibration matricies to identity

		//! symbolic way to refer to each of the directions
		enum dimension_offset {
			forward, //!< forward (x)
			reverse, //!< backward (-x)
			strafe,  //!< sideways (y)
			rotate,  //!< spin (z/a)
			NUM_DIM  //!< number of directions we calibrate for
		};

		float f_calibration[3][11]; //!< matrix of calibration parameters; 3 columns for f,s,r speeds, 2 columns for abs s,r speeds, 1 gabor function, 1 squared planar speed, 3 columns for f*r,s*f,r*s, and 1 column for offset
		float b_calibration[3][11]; //!< matrix of calibration parameters; 3 columns for f,s,r speeds, 2 columns for abs s,r speeds, 1 gabor function, 1 squared planar speed, 3 columns for f*r,s*f,r*s, and 1 column for offset

		float max_accel[NUM_DIM]; //!< maximum achievable acceleration, 0 for infinite (mm/s^2)
		float max_vel[NUM_DIM]; //!< maximum achievable velocity (mm/s)
	};

	//! constructor
	CMPackWalkMC(const char* pfile=NULL);

	virtual void start(); //!< sends an activate LocomotionEvent
	virtual void stop();  //!< sends a deactivate LocomotionEvent

        //! Posts a LocomotionEvent. Also forces an output frame setting wheel velocities to zero; needed because if we remove a motion command there may be nothing left to zero the velocities.
        virtual void zeroVelocities();

	virtual int updateOutputs(); //!< calculates current positions of the legs or speeds of the wheels
	
	//! Returns true if we are walking. This modified isDirty allows the AIBO to slow down to a stop rather than stopping immediately.
	virtual int isDirty();
	virtual void setDirty() { isPaused = false; }
	
	//! Will prune if we've taken the requested number of steps.
	virtual int isAlive() { return step_count != 0; }

	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;
	virtual	unsigned int loadFile(const char* filename);
	virtual unsigned int saveFile(const char* filename) const;

	//! returns current velocity we're trying to go
	const vector3d& getTargetVelocity() { return target_vel_xya; }
	//! returns the velocity we're actually moving (subject to clipping at max_accel_xya), doesn't reflect value of getPaused()...
	const vector3d& getCurVelocity() const { return vel_xya;}
	//! returns the time at which we started traveling along the current vector, in milliseconds
	unsigned int getStartTravelTime() { return travelTime; }
	//! returns the amount of time we've been traveling along the current vector, in milliseconds
	unsigned int getTravelTime() { return get_time()-getStartTravelTime(); }

	//! set the direction to walk
	/*! @param dx forward velocity (millimeters per second)
	 *  @param dy left velocity (millimeters per second)
	 *  @param da counterclockwise velocity (radians per second) */
	void setTargetVelocity(double dx,double dy,double da);
	
	//! set the direction to walk
	/*! @param dx forward velocity (millimeters per second)
	 *  @param dy left velocity (millimeters per second)
	 *  @param da counterclockwise velocity (radians per second)
	 *  @param time (seconds) duration to walk before stopping and posting a status event; time will be 
	 *  rounded to the nearest step time. (This is internally converted to number of steps) */
	void setTargetVelocity(double dx,double dy,double da,double time);
	
	//! recalculates the target velocity so steps are of a given length to achieve the specified displacement in @a n steps
	/*! @param dx length of displacement in millimeters along body's x axis
	 *  @param dy length of displacement in millimeters along body's y axis
	 *  @param da amount to turn in radians, counter-clockwise viewed from above
	 *  @param time (seconds) duration in which to do the displacement (thus controls velocity); time will be 
	 *  rounded to the nearest step time. (This is internally converted to number of steps)
	 *
	 *  @see setTargetVelocity() */
	void setTargetDisplacement(double dx, double dy, double da, double time);

	void setTargetDisplacement(double dx, double dy, double da, double vx, double vy, double va) {}

	//! returns remaining steps (#step_count) (negative means infinite)
	int getRemainingSteps() const { return step_count; }

	
	float getStepThreshold() const { return step_threshold; } //!< returns the step threshold -- see #step_threshold
	void setStepThreshold(float st) { step_threshold=st; } //!< sets the step threshold -- see #step_threshold
	
	void setPaused(bool p) { isPaused=p; } //!< if set to true, will stop moving
	bool getPaused() const { return isPaused; } //!< if is true, we aren't moving
	void setHeight(double h) { wp.body_height=h; } //!< sets WalkParam::body_height of #wp
	double getHeight() { return wp.body_height; } //!< gets WalkParam::body_height of #wp
	void setAngle(double a) { wp.body_angle=a; } //!< sets WalkParam::body_angle of #wp
	double getAngle() { return wp.body_angle; } //!< gets WalkParam::body_angle of #wp
	void setHop(double h) { wp.hop=h; } //!< sets WalkParam::hop of #wp
	double getHop() { return wp.hop; } //!< gets WalkParam::hop of #wp
	void setSway(double h) { wp.sway=h; } //!< sets WalkParam::sway of #wp
	double getSway() { return wp.sway; } //!< gets WalkParam::sway of #wp
	void setPeriod(long p) { wp.period=p; } //!< sets WalkParam::period of #wp
	long getPeriod() { return wp.period; } //!< gets WalkParam::period of #wp
	void setSlowMo(float p) { slowmo=p; } //!< sets #slowmo
	float* getSlowMo() { return &slowmo; } //!< gets #slowmo

	//!used to specify value for #acc_style
	enum AccelerationStyle_t {
		CALIBRATION_ACCEL, //!< use the acceleration specified by the calibration parameters
		INF_ACCEL,         //!< ignore calibration acceleration (attempt infinite acceleration)
		DEFAULT_ACCEL      //!< reference the value of Config::motion_config::walk_acceleration
	};
	virtual void setAccelerationStyle(AccelerationStyle_t acc) { acc_style=acc; } //!< sets #acc_style
	virtual AccelerationStyle_t getAccelerationStyle() const { return acc_style; } //!< returns #acc_style

#ifdef TGT_HAS_LEGS
	//! returns the current leg position of leg @a i
	const vector3d& getLegPosition(LegOrder_t i) const { return legpos[i]; }
#endif

	struct WalkParam& getWP() { return wp; }; //!< returns the current walk parameter structure
	struct CalibrationParam& getCP() { return cp; }; //!< returns the current walk calibration parameter
	
	//! takes current leg positions from WorldState and tries to match the point in the cycle most like it
	void resetLegPos();

	static const float MAX_DX; //!< maximum forward velocity, for setTargetDisplacement
	static const float MAX_DY; //!< maximum sideways velocity, for setTargetDisplacement
	static const float MAX_DA; //!< maximum angular velocity, for setTargetDisplacement
#ifdef TGT_HAS_WHEELS
	static const float OPTIMAL_DA; //!< optimal angular velocity for Create odometry in setTargetDisplacement
#endif
	
	float getMaxXVel() const { return MAX_DX; } //!< more portable than directly accessing #MAX_DX
	float getMaxYVel() const { return MAX_DY; } //!< more portable than directly accessing #MAX_DY
	float getMaxAVel() const { return MAX_DA; } //!< more portable than directly accessing #MAX_DA

	static const vector3d max_accel_xya; //!< maximum acceleration of x, y, and a velocity

 protected:
	//! holds current joint commands
	OutputCmd cmds[NumOutputs][NumFrames];

	//! converts @a in to calibration parameters and multiplies through the calibration matrix
	static void applyCalibration(const float mat[3][11], const vector3d& in, vector3d& out);

 protected:
	//! does some setup stuff, calls loadFile(pfile)
	void init(const char* pfile);

	bool isPaused; //!< true if we are paused
  bool dirty;  //!< Ignored; needed to make WaypointWalkMC::setDirty() work

	WalkParam wp; //!< current walking parameters (note that it's not static - different CMPackWalkMC's can have different setting, handy...
	CalibrationParam cp; //!< calibration parameters for current walk.
	LegWalkState legw[NumLegs]; //!< current state of each leg
	vector3d legpos[NumLegs]; //!< current position of each leg
	splinepath body_loc; //!< the path the body goes through while walking (?)
	splinepath body_angle; //!< the path the body goes through while walking (?)
	vector3d liftPos[NumLegs]; //!< position each of the feet was last lifted
	vector3d downPos[NumLegs]; //!< position each of the feet is next going to be set down
	
	AccelerationStyle_t acc_style; //!< lets you switch between finite or infinite acceleration models

	int step_count; //!< number of steps to take before stopping; if negative, walk forever.
	float step_threshold; //!< point in a leg's cycle where the step counter should be decremented; 0 - leg lift, .25 - mid-air, .5 - leg down, .75 - mid-ground

	double last_cycle; //!< where the walk is in it its cycle, updated at the end of each call to updateOutputs()

	vector3d pos_delta; //!< how much we've moved
	float angle_delta; //!< how much we've turned
	
	unsigned int travelTime; //!< the time of the last call to setTargetVelocity - handy to check the time we've been traveling current vector
	int time; //!< time of last call to updateOutputs() (scaled by slowmo)
	int TimeStep; //!< time to pretend passes between each call to updateOutputs() - usually RobotInfo::FrameTime
	float slowmo; //!< scales time values to make the walk move in slow motion for analysis (or fast forward)

	// tss "SmoothWalk" addition follows
	/*! The CycleOffset variable is used to ensure that each time the AIBO
	 *  starts walking, it starts at the same point in the walk cycle as
	 *  where it stopped. This measure is intended to decrease the amount
	 *  of jerking (and hence deviation) that occurs when the AIBO starts
	 *  walking forward and then suddenly stops. */
	int CycleOffset;
	/*! Each CycleOffset corresponds to a different TimeOffset once the
	 *  robot starts walking again. Consider this example: the robot
	 *  stops 2/3 of the way through the cycle, then starts again 1/3
	 *  of the way through the cycle on the absolute clock. The time
	 *  offset to advance the clock to the right part of the cycle is
	 *  1/3 of a cycle, so we set TimeOffset to 1/3 cycle and add that to
	 *  every clock value used in the walk code. */
	int TimeOffset;
	/*! Every time we stop, we know we'll have a new CycleOffset, and we'll
	 *  need to compute a new TimeOffset. This boolean says as much. */
	bool NewCycleOffset;
// tss "SmoothWalk" addition ends

	vector3d target_disp_xya; //!< requested displacement
	vector3d vel_xya; //!< the current velocity we're moving
	vector3d target_vel_xya; //!< the velocity that was requested
	vector3d last_target_vel_xya; //!< the velocity that was last sent to motion
};

/* struct LegState{
	 long attr,reserved;
	 point3d pos;
	 double angles[3];
	 };
	 
	 struct HeadState{
	 long attr,reserved;
	 vector3d target;
	 double angles[3];
	 };
	 
	 struct BodyState{
	 BodyPosition pos;
	 LegState leg[4];
	 HeadState head;
	 }; 
*/

/*! @file
 * @brief Describes CMPackWalkMC, a MotionCommand for walking around
 * @author CMU RoboSoccer 2001-2002 (Creator)
 * @author ejt (ported)
 * @author PA Gov. School for the Sciences 2003 Team Project - Haoqian Chen, Yantian Martin, Jon Stahlman (modifications)
 * 
 * @verbinclude CMPack_license.txt
 */

#endif
