#include "WaypointList.h"
#include "Shared/mathutils.h"

const float WaypointList::defaultTurnSpeed = float(M_PI/30);

void Waypoint::apply(const Waypoint& next, float eps[]) {
	float origx=x;
	float origy=y;
	switch(next.posType) {
		case Waypoint::POSTYPE_EGOCENTRIC: {
			x+=next.x*std::cos(angle)-next.y*std::sin(angle);
			y+=next.x*std::sin(angle)+next.y*std::cos(angle);
			break;
		}
		case Waypoint::POSTYPE_OFFSET:
			x+=next.x;
			y+=next.y;
			break;
		case Waypoint::POSTYPE_ABSOLUTE:
			x=next.x;
			y=next.y;
			break;
	}
	float dx=x-origx;
	float dy=y-origy;
	if(fabs(dx)<eps[0] && fabs(dy)<eps[1]) { //turn in place
		if(next.angleIsRelative)
			angle+=next.angle;
		else
			angle=next.angle;
	} else { //move at heading
		angle=next.angle;
		if(next.angleIsRelative)
			angle+=std::atan2(dy,dx);
	}
	angle+=next.arc/2;
	angle=mathutils::normalizeAngle(angle);
}

/*! This is replicated from WaypointEngine, so any modifications here should be replicated there... */
Waypoint WaypointList::calcAbsoluteCoords(const_iterator it) {
	float eps[] = { 5, 5, 0.0175f };
	Waypoint cur(0,0,Waypoint::POSTYPE_ABSOLUTE,0,false,0,false,defaultTurnSpeed);
	for(const_iterator c=begin(); c!=end(); c++ ) {
		cur.apply(*c,eps);
		if(c==it)
			break;
	}
	return cur;
}

/*! This is adapted from WaypointEngine::fixArc(), so any modifications here should be replicated there... */
void WaypointList::fixArc(float arc) {
	Waypoint& center=back();
	float cdx=center.x; //center delta
	float cdy=center.y; //center delta
	if(center.posType==Waypoint::POSTYPE_ABSOLUTE) {
		//have to find location of waypoint before last one
		if(size()<=1) {
			// no-op
		} else {
			Waypoint start=calcAbsoluteCoords(end()-2);
			cdx-=start.x;
			cdy-=start.y;
		}
	}
	float r=std::sqrt(cdx*cdx+cdy*cdy); //radius of circle
	float ca=std::atan2(cdy,cdx); //angle to center of circle
	center.x-=r*std::cos(ca-arc); //now x is the endpoint
	center.y-=r*std::sin(ca-arc); //now y is the endpoint
	center.arc=arc;
}
