//-*-c++-*-
#ifndef INCLUDED_WaypointList_h_
#define INCLUDED_WaypointList_h_

#include <vector>

//! Holds information about each waypoint, see WaypointEngine for overview
struct Waypoint {
public:

	//! defines different ways to interpret the position values
	enum posType_t {
		POSTYPE_EGOCENTRIC, //!< #x and #y are relative to current heading - so x is forward and y is strafe
		POSTYPE_OFFSET, //!< #x and #y are oriented with the coordinates, but relative to current location (delta x and delta y)
		POSTYPE_ABSOLUTE //!< #x and #y are a specific coordinate location
	};
	
	//! constructor
	Waypoint()
	: x(0), y(0), angle(0), arc(), speed(), turnSpeed(), posType(), angleIsRelative(), trackPath()
	{}
	
	//! constructor
	Waypoint(float xc, float yc, Waypoint::posType_t pos_rel, float ac, bool ang_rel, float spd, bool track, float turn)
	: x(xc), y(yc), angle(ac), arc(0), speed(spd), turnSpeed(turn), posType(pos_rel), angleIsRelative(ang_rel), trackPath(track)
	{}
	
	//!< If @a next is a relative waypoint (offset or egocentric), it is added to this instance's location; otherwise if @a next is absolute, this is set to @a next
	/*! The Waypoint::angle field is used to store the headings */
	void apply(const Waypoint& next, float eps[]);
	
	float x; //!< the displacement along x (millimeters), subject to #posType
	float y; //!< the displacement along y (millimeters), subject to #posType
	float angle; //!< either the angle relative to path to maintain, or the heading to maintain, see #angleIsRelative
	float arc; //!< angle of sector of arc to use to get to waypoint (0 means straight line)
	float speed; //!< speed (in millimeters per second)
	float turnSpeed; //!< maximum speed to correct heading (in radians per second)
	posType_t posType; //!< lets us know how to interpret the #x and #y values
	bool angleIsRelative; //!< if true, #angle is interpreted as relative to the path; otherwise, interpreted as an absolute heading to maintain
	bool trackPath; //!< if true, if off course, will attempt to get back on path at the ideal location; if false, simply heads directly for waypoint from whereever it is
};

class WaypointList : public std::vector<Waypoint> { 
public:
	WaypointList() {}
	
    static const float defaultTurnSpeed;
    
	//!@name Adding Waypoints
	
	//! adds a waypoint to the end of the list, allows you to specify turtle-style instructions
	/*! <img src="Waypoint_Ego.png">
	 *  @param forward distance forward to move (negative to move backward of course)
	 *  @param left distance to the left to move (negative to move right of course)
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param speed is the speed to move at; millimeters per second
	 *  @param turnSpeed is the speed to turn; radians per second */
	void addEgocentricWaypoint(float forward, float left, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f) {
		push_back(Waypoint(forward,left,Waypoint::POSTYPE_EGOCENTRIC,angle,angleIsRelative,fwdSpeed,false,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
	}
	//! adds a waypoint to the end of the list, allows you to set locations relative to the location of the previous waypoint (or starting position)
	/*! <img src="Waypoint_Off.png">
	 *  @param x distance delta along x axis of the waypoint
	 *  @param y distance delta along y axis of the waypoint
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param speed is the speed to move at; millimeters per second
	 *  @param turnSpeed is the speed to turn; radians per second */
	void addOffsetWaypoint(float x, float y, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f) {
		push_back(Waypoint(x,y,Waypoint::POSTYPE_OFFSET,angle,angleIsRelative,fwdSpeed,false,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
	}
	//! adds a waypoint to the end of the list, allows you to set locations relative to the world coordinate frame
	/*! <img src="Waypoint_Abs.png">
	 *  @param x position along x axis of the waypoint
	 *  @param y position along y axis of the waypoint
	 *  @param angle angle of attack to use on the path
	 *  @param angleIsRelative controls interpretation of @a angle; true means angle specifies an offset from the bearing of the target waypoint, false means maintain an absolute heading
	 *  @param fwdSpeed is the speed to move at; millimeters per second
	 *  @param turnSpeed is the speed to turn; radians per second */
	void addAbsoluteWaypoint(float x, float y, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed=-1.f) {
		push_back(Waypoint(x,y,Waypoint::POSTYPE_ABSOLUTE,angle,angleIsRelative,fwdSpeed,false,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
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
	void addEgocentricArc(float forward, float left, float angle, bool angleIsRelative, float speed, float arc) {
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
	void addOffsetArc(float x, float y, float angle, bool angleIsRelative, float speed, float arc) {
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
	void addAbsoluteArc(float x, float y, float angle, bool angleIsRelative, float speed, float arc) {
		addAbsoluteWaypoint(x,y,angle,angleIsRelative,speed);
		fixArc(arc);
	}	

protected:
	//! assumes start at the origin, applies each waypoint through @a it and returns result as an absolute position (angle field stores heading)
	Waypoint calcAbsoluteCoords(const_iterator it);
	
	//! assumes the last waypoint is actually center of circle, adjusts it to be the endpoint of following @a arc radians around that circle instead
	void fixArc(float arc);
};

#endif
