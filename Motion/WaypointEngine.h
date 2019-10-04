//-*-c++-*-
#ifndef INCLUDED_WaypointEngine_h_
#define INCLUDED_WaypointEngine_h_

#include "IPC/ListMemBuf.h"
#include "Shared/LoadSave.h"
#include "Shared/Config.h"
#include "Shared/get_time.h"
#include "WaypointList.h"
#include <cmath>

//! Provides computation and management of a desired path through a series of waypoints
/*! This is a generalized set of data structures and management code -
 *  it doesn't actually say anything about @e how you get from one
 *  waypoint to the other, it will just tell you where you should be
 *  going at any given time.
 *
 *  So, for instance, you may be interested in WaypointWalk, which
 *  will use a WalkMC to traverse the waypoints.  Future development
 *  may include a WaypointPush, to push an object along a path instead
 *  of just moving the body along a path.
 *
 *  Although general curves between waypoints are not supported, you
 *  can use either circular arcs or straight lines.
 *
 *  The Waypoint class holds the actual data about each waypoint.  You
 *  can specify waypoints in 3 ways: egocentric, offset, and absolute.
 *
 *  <table cellspacing=0 cellpadding=0 width="750" class="figures" align="center" border="0"><tr>
 *  <td class="figure"><img src="Waypoint_Ego.png"><div style="padding:10px;">
 *    @b Egocentric: the x and y parameters are relative to the body
 *    itself; x is always forward and y is always left.  Handy for
 *    turtle/logo style specification of instructions
 *  </div></td><td class="figure"><img src="Waypoint_Off.png"><div style="padding:10px;">
 *    @b Offset: the x and y parameters are relative to the current body
 *    position, but not its heading.
 *  </div></td><td class="figure"><img src="Waypoint_Abs.png"><div style="padding:10px;">
 *    @b Absolute: the x and y parameters are direct coordinates
 *  </div></td></tr></table>
 *
 *  These specify the @e position of the next waypoint, but we also need
 *  to be able to specify the @e orientation (heading) of the robot.  This
 *  is done by specifying an angle and a mode which controls how that
 *  angle is interpreted: Waypoint::angleIsRelative, which can be @c true or @c false.
 *
 *  <table cellspacing=0 cellpadding=0 width="500" class="figures" align="center" border="0"><tr>
 *  <td class="figure"><img src="Waypoint_AngleRel.png"><div style="padding:10px;">
 *  <code>angleIsRelative==</code>@b true: The angle is relative to the path, so that @c 0 will keep
 *  the robot pointed in the direction of travel, even when arcing.  Similarly, @f$\pi/2\ (90^\circ)@f$
 *  would cause the robot to walk sideways.
 *  </div></td><td class="figure"><img src="Waypoint_AngleAbs.png"><div style="padding:10px;">
 *  <code>angleIsRelative==</code>@b false: The angle is relative to the world coordinate system, so a 
 *  constant heading is maintained throughout execution of the path.
 *  </div></td></tr></table>
 *
 *  The final orientation of the robot is simply the heading it was
 *  facing when it reaches the end point.  To turn in place, you can
 *  use a (0,0) egocentric or offset waypoint with an angle parameter.
 *
 *  In order to execute curves, you can supply an arc value:
 *
 *  <table cellspacing=0 cellpadding=0 width="417" class="figures" align="center" border="0"><tr>
 *  <td class="figure"><img src="Waypoint_Arc.png"><div style="padding:10px;">
 *  Here you see the results of 3 different arc values @f$(60^\circ,120^\circ,180^\circ)@f$.  Note how
 *  the arc parameter corresponds to the angle of the circle which is swept.<br>
 *  </div></td></tr></table>
 *
 *  There are two ways to specify arcs.  The @c add*Waypoint functions
 *  use the position arguments to specify the <em>end point</em> of
 *  the arc, and the arc parameter serves to "bow" the path.  The @c
 *  add*Arc functions specify the <em>center of the circle</em> as the
 *  position, and the end point is inferred from the amount of the arc
 *  to sweep.
 *
 *  Beware that arcs greater than @f$180^\circ@f$ are entirely
 *  possible, but will create larger and larger circles which may
 *  cause the robot to initially start moving @e away from the
 *  destination.  This isn't necessarily a bad thing, but may be
 *  unanticipated.  Values approaching @f$2\pi\ (360^\circ)@f$ may
 *  cause numerical instability yielding infinitely large circles.
 *  Values larger than @f$2\pi\ (360^\circ)@f$ will be normalized to
 *  the range @f$[0,2\pi)@f$.
 *
 *  Dead reckoning is very prone to accruing error.  It is highly
 *  recommended that you calibrate the locomotion mechanism carefully
 *  (see WalkCalibration, available under the "Walk Edit" menu with a
 *  run-time help menu) and implement some form of localization to
 *  handle the inevitable drift.
 *
 *  If you have a localization module in place, you can use the
 *  setCurPos() function to update the position of the robot within
 *  the world.  WaypointEngine provides two ways to handle this
 *  ensuing discrepency from the path the robot had been tracing:
 *
 *  <table cellspacing=0 cellpadding=0 width="325" class="figures" align="center" border="0"><tr>
 *  <td class="figure"><img src="Waypoint_Error.png"><div style="padding:10px;">
 *  The effect of the Waypoint::trackPath flag.  When @c true, the robot will attempt to catch up
 *  to its "ideal" location after a perturbation.  When @c false, the robot will ignore the "ideal"
 *  path, and just go straight to the destination from wherever perturbations may push it.
 *  </div></td></tr></table>
 *
 *  trackPath is a per-waypoint setting, setTracking() sets the
 *  default value for any new waypoints which are thereafter created
 *  (the default default is false ;)
 *
 *  Waypoint list files are a fairly straightforward plain text format.
 *  The extension .wyp is suggested.
 *
 *  The waypoint file format is:
 *  - '<tt>\#WyP</tt>' - header to verify file type
 *  - A series of entries:
 *    - '<tt>max_turn_speed </tt><i>num</i>' - sets the maximum error-correction turning speed used for all following waypoints
 *    - '<tt>track_path </tt><i>bool</i>' - sets trackpath mode on or off for all following waypoints (see Waypoint::trackPath)
 *    - '<tt>add_point </tt>{<tt>ego</tt>|<tt>off</tt>|<tt>abs</tt>}<tt> </tt><i>x_val</i><tt> </tt><i>y_val</i><tt> </tt>{<tt>hold</tt>|<tt>follow</tt>}<tt> </tt><i>angle_val</i><tt> </tt><i>speed_val</i><tt> </tt><i>arc_val</i>' - adds the waypoint with the parameters given, see Waypoint, similar to add*Waypoint functions
 *    - '<tt>add_arc </tt>{<tt>ego</tt>|<tt>off</tt>|<tt>abs</tt>}<tt> </tt><i>x_val</i><tt> </tt><i>y_val</i><tt> </tt>{<tt>hold</tt>|<tt>follow</tt>}<tt> </tt><i>angle_val</i><tt> </tt><i>speed_val</i><tt> </tt><i>arc_val</i>' - adds the waypoint with the parameters given, see Waypoint, similar to add*Arc functions
 *  - '<tt>\#END</tt>' - footer to verify ending of file
 */

class WaypointEngine : public LoadSave {
public:

	typedef std::vector<Waypoint> WaypointList_t; //!< convenient shorthand
	typedef std::vector<Waypoint>::iterator WaypointListIter_t; //!< convenient shorthand
	typedef std::vector<Waypoint>::const_iterator WaypointListConstIter_t; //!< convenient shorthand

	//! constructor
	WaypointEngine()
		: LoadSave(), waypoints(), isRunning(false), isLooping(false), isTracking(false),
			curWaypoint(waypoints.end()), waypointTime(0), waypointDistance(0), pathLength(0), arcRadius(0),
			lastUpdateTime(0), Pcorr(.5f), defaultTurnSpeed((float)M_PI/30)
	{init();}
	//! constructor
	WaypointEngine(char * f)
		: LoadSave(), waypoints(), isRunning(false), isLooping(false), isTracking(false),
			curWaypoint(waypoints.end()), waypointTime(0), waypointDistance(0), pathLength(0), arcRadius(0),
			lastUpdateTime(0), Pcorr(.5f), defaultTurnSpeed((float)M_PI/30)
	{init(); loadFile(f); }

	//! returns a rough overestimate of the size needed
	/*! pretends we need to switch max_turn_speed and track_path on
		every point, and the longest options are given for every point */
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;
	virtual unsigned int loadFile(const char * filename) { return LoadSave::loadFile(config->motion.makePath(filename).c_str()); }
	virtual unsigned int saveFile(const char * filename) const { return LoadSave::saveFile(config->motion.makePath(filename).c_str()); }

	virtual void go();      //!< starts walking towards the first waypoint
	virtual void pause();   //!< halts execution of waypoint list
	virtual void unpause(); //!< resumes execution of waypoint list from last paused location

	virtual void setIsLooping(bool isl) { isLooping=isl; } //!< sets #isLooping
	virtual bool getIsLooping() const { return isLooping; } //!< returns #isLooping

	virtual WaypointList_t& getWaypointList() { return waypoints; } //!< returns a reference to #waypoints
	virtual const WaypointList_t& getWaypointList() const { return waypoints; } //!< returns a const reference to #waypoints

	virtual WaypointListIter_t getCurWaypointID() const { return curWaypoint; } //!< returns id value of current waypoint (#curWaypoint)

	virtual float getCurX() const { return curPos[0]; } //!< returns current x position
	virtual float getCurY() const { return curPos[1]; } //!< returns current y position
	virtual float getCurA() const { return curPos[2]; } //!< returns current heading
	//! sets the current position (for instance your localization module has an update)
	virtual void setCurPos(float x, float y, float a) {
		curPos[0]=x; curPos[1]=y; curPos[2]=a;
	}

	virtual void setTracking(bool b) { isTracking=b; } //!< sets the #isTracking flag, only affects future waypoints which are added, not currently listed waypoints (use getWaypointList() to modify existing waypoints)
	virtual bool getTracking() const { return isTracking; } //!< returns #isTracking

	//! call this on each opportunity to check current location and correct velocities
	/*! @return #isRunning for convenience of checking if anything is happening */
	virtual bool cycle(); 

	//!these are for convenience - can also directly edit the waypoint list using access from getWaypointList()
	//!@name Adding Waypoints

	//! adds a waypoint to the end of the list, allows you to specify turtle-style instructions
	/*! <img src="Waypoint_Ego.png">
	 *  @param forward distance forward to move (negative to move backward of course)
	 *  @param left distance to the left to move (negative to move right of course)
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param fwdSpeed is the speed to move at; millimeters per second
         *  @param turnSpeed is the speed to turn; radians per second */
  virtual void addEgocentricWaypoint(float forward, float left, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f);

	//! adds a waypoint to the end of the list, allows you to set locations relative to the location of the previous waypoint (or starting position)
	/*! <img src="Waypoint_Off.png">
	 *  @param x distance delta along x axis of the waypoint
	 *  @param y distance delta along y axis of the waypoint
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param fwdSpeed is the speed to move at; millimeters per second
         *  @param turnSpeed is the speed to turn; radians per second */
  virtual void addOffsetWaypoint(float x, float y, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f) {
    waypoints.push_back(Waypoint(x,y,Waypoint::POSTYPE_OFFSET,angle,angleIsRelative,fwdSpeed,isTracking,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
	}
	//! adds a waypoint to the end of the list, allows you to set locations relative to the world coordinate frame
	/*! <img src="Waypoint_Abs.png">
	 *  @param x position along x axis of the waypoint
	 *  @param y position along y axis of the waypoint
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param fwdSpeed is the speed to move at; milimeters per second
         *  @param turnSpeed is the speed to turn; radians per second */
  virtual void addAbsoluteWaypoint(float x, float y, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f) {
    waypoints.push_back(Waypoint(x,y,Waypoint::POSTYPE_ABSOLUTE,angle,angleIsRelative,fwdSpeed,isTracking,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
	}

	//! adds a waypoint to the end of the list, using an arcing path to get there, allows you to specify turtle-style instructions to specify the focus of the arc
	/*! <img src="Waypoint_Ego.png">
	 *  If you would rather specify the ending point and then "bow" the path, try addEgocentricWaypoint() followed by setting the Waypoint::arc field directly
	 *  @param forward distance in front of the center of the circle of the arc
	 *  @param left distance to the left of the center of the circle of the arc
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param speed is the speed to move at; millimeters per second
	 *  @param arc is the number of radians the arc fills; arcs near 0 (or multiples of 360) may cause numeric instability */
	virtual void addEgocentricArc(float forward, float left, float angle, bool angleIsRelative, float speed, float arc) {
		addEgocentricWaypoint(forward,left,angle,angleIsRelative,speed);
		fixArc(arc);
	}
	//! adds a waypoint to the end of the list, using an arcing path to get there, allows you to specify locations relative to previous waypoint to specify the focus of the arc
	/*! <img src="Waypoint_Off.png">
	 *  If you would rather specify the ending point and then "bow" the path, try addOffsetWaypoint() followed by setting the Waypoint::arc field directly
	 *  @param x distance delta along x of the center of the circle of the arc
	 *  @param y distance delta along y of the center of the circle of the arc
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param speed is the speed to move at; millimeters per second
	 *  @param arc is the number of radians the arc fills; arcs near 0 (or multiples of 360) may cause numeric instability */
	virtual void addOffsetArc(float x, float y, float angle, bool angleIsRelative, float speed, float arc) {
		addOffsetWaypoint(x,y,angle,angleIsRelative,speed);
		fixArc(arc);
	}
	//! adds a waypoint to the end of the list, using an arcing path to get there, allows you to specify absolute locations to specify the focus of the arc
	/*! <img src="Waypoint_Abs.png">
	 *  If you would rather specify the ending point and then "bow" the path, try addAbsoluteWaypoint() followed by setting the Waypoint::arc field directly
	 *  @param x position along x of the center of the circle of the arc
	 *  @param y position along y of the center of the circle of the arc
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param speed is the speed to move at; millimeters per second
	 *  @param arc is the number of radians the arc fills; arcs near 0 (or multiples of 360) may cause numeric instability */
	virtual void addAbsoluteArc(float x, float y, float angle, bool angleIsRelative, float speed, float arc) {
		addAbsoluteWaypoint(x,y,angle,angleIsRelative,speed);
		fixArc(arc);
	}
	
	virtual void appendWaypoints(const WaypointList_t wpl) {
		waypoints.insert(waypoints.end(), wpl.begin(), wpl.end());
	}
	
	virtual void clearWaypointList() {
		waypoints.clear();
	}
	//@}
	
	//! will set the currently active waypoint to another waypoint; correctly calculates location of intermediate waypoints so target location will be the same as if the intervening waypoints had actually been executed
	virtual void setTargetWaypoint(WaypointListIter_t iter);
	
	//!if @a it follows the current waypoint, applies all the waypoints between #curWaypoint and @a it and returns result as an absolute position (angle field stores heading); otherwise calls the other calcAbsoluteCoords(WaypointListIter_t, float, float, float)
	Waypoint calcAbsoluteCoords(WaypointListIter_t it) {
		//find out if 'it' is coming up, or already passed
		bool isAhead=false;
		for(WaypointListIter_t c=curWaypoint; c!=waypoints.end(); c++)
			if(c==it) {
				isAhead=true;
				break;
			}
		if(!isAhead)
			return calcAbsoluteCoords(it,pathStartPos[0],pathStartPos[1],pathStartPos[2]);
		Waypoint cur(targetPos[0],targetPos[1],Waypoint::POSTYPE_ABSOLUTE,targetPos[2],false,0,isTracking,defaultTurnSpeed);
		if(it==curWaypoint)
			return cur;
		for(WaypointListIter_t c=curWaypoint+1; c!=waypoints.end(); c++) {
			cur.apply(*c,eps);
			if(c==it)
				break;
		}
		return cur;
	}

	//!starts at (@a sx, @a sy, heading=@a sa) and then applies all the waypoints up through @a it and returns result as an absolute position (angle field stores heading)
	/*! This is replicated in WaypointList, so any updates should be made there as well */
	Waypoint calcAbsoluteCoords(WaypointListIter_t it,float sx, float sy, float sa) {
		Waypoint cur(sx,sy,Waypoint::POSTYPE_ABSOLUTE,sa,false,0,isTracking,defaultTurnSpeed);
		for(WaypointListIter_t c=waypoints.begin(); c!=waypoints.end(); c++ ) {
			cur.apply(*c,eps);
			if(c==it)
				break;
		}
		return cur;
	}
	


protected:
	void init(); //!< basic memory initialization

	//! assumes the last waypoint is actually center of circle, adjusts it to be the endpoint of following @a arc radians around that circle instead
	void fixArc(float arc);
	
	//! based on current velocity and time since last call, dead reckons current location in #curPos
	/*! doesn't take acceleration into account, but should... :( */
	void computeCurrentPosition(unsigned int t);
	void checkNextWaypoint(unsigned int t);  //!< checks to see if #curPos is within #eps of #targetPos; if so, setTargetWaypoint() to next waypoint
	void computeIdeal(unsigned int t);       //!< computes the ideal location (#idealPos) if we were following the intended path at the intended speed
	void computeNewVelocity(unsigned int t); //!< computes the velocity which should be used given the current position (#curPos) relative to the ideal position (#idealPos)

#ifdef TGT_IS_CREATE
        static float tgtCreateTurnFudgeFactor(float angle);
#endif
        static float fudgedAngle(float originalAngle);

	WaypointList_t waypoints; //!< storage for the waypoints

	bool isRunning;  //!< true if we're currently executing the path
	bool isLooping;  //!< true if we should loop when done
	bool isTracking; //!< new waypoints will use trackPath mode
	WaypointListIter_t curWaypoint;  //!< index of current waypoint
	unsigned int waypointTime; //!< time we started working on current waypoint
	float waypointDistance;    //!< distance from #sourcePos to #targetPos
	float pathLength;          //!< distance to be traveled from #sourcePos to #targetPos (may differ from #waypointDistance due to arcing)
	float arcRadius;           //!< radius of current arc, may be inf or NaN if using a straight line; can also be negative depending on direction!
	unsigned int lastUpdateTime; //!< time we last updated curPos
	float pathStartPos[3]; //!< position when started execution of current path (aka origin offset for relative positions which preceed an absolute waypoint)
	float sourcePos[3]; //!< source position of the robot relative to the origin, aka absolute position of previous waypoint
	float targetPos[3]; //!< target position of the robot relative to the origin, aka absolute position of next waypoint
	float idealPos[4];  //!< ideal position of the robot relative to the origin, (x, y, heading, last element is desired direction of motion)
	float curPos[3];    //!< current position of the robot relative to the origin
	float curVel[3];    //!< current velocity
	float eps[3];       //!< epsilon - "close enough" to register a hit on the waypoint
	float Pcorr;        //!< proportional correction factor for tracking path
	float defaultTurnSpeed;    //!< maximum turning speed for new waypoints
};


/*! @file
 * @brief Defines WaypointEngine, which provides computation and management of a desired path through a series of waypoints
 * @author ejt (Creator)
 */

#endif
