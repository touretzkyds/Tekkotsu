#include "WaypointEngine.h"
#include "Shared/mathutils.h"

void WaypointEngine::go() {
	isRunning=true;
	for(unsigned int i=0; i<3; i++) {
		curVel[i]=0;
		pathStartPos[i]=sourcePos[i]=curPos[i];
	}
	curWaypoint=waypoints.begin();
	if ( waypoints.empty() )
	  return;
	Waypoint target(curPos[0],curPos[1],Waypoint::POSTYPE_ABSOLUTE,curPos[2],false,0,isTracking,defaultTurnSpeed);
	target.apply(waypoints.front(),eps);
	targetPos[0]=target.x;
	targetPos[1]=target.y;
	targetPos[2]=target.angle;
	lastUpdateTime=get_time();
	setTargetWaypoint(curWaypoint);
}

void WaypointEngine::pause() {
	// cout<<"Pausing"<<endl;
	isRunning=false;
}

void WaypointEngine::unpause() {
	if(curWaypoint==waypoints.end())
		go();
	isRunning=true;
	for(unsigned int i=0; i<3; i++)
		curVel[i]=0;
	lastUpdateTime=get_time();
}

bool WaypointEngine::cycle() {
	if(!isRunning)
		return false;
	
	unsigned int curtime=get_time();
	if(curWaypoint!=waypoints.end()) {
		computeCurrentPosition(curtime);
		checkNextWaypoint(curtime);
	}
	if(curWaypoint!=waypoints.end()) {
		computeIdeal(curtime);
		computeNewVelocity(curtime);
	}
	
	return true;
}

#ifdef TGT_IS_CREATE
float WaypointEngine::tgtCreateTurnFudgeFactor(float angle) {
  float fudge;
  float thresholds[4] = {0.0, 2.5, 5.0, 10.0};
  float scales[4] = {0.0, 0.039, 0.051, 0.06};
  float degAngle = fabs(angle * 180.0/M_PI);

  bool found = false;
  for(unsigned int i = 1; i < 4; i++) {
    if(degAngle >= thresholds[i-1] && degAngle < thresholds[i]) {
      if((i-1) != 0)
	fudge = ((scales[i] - scales[i-1])/(thresholds[i]-thresholds[i-1]))*degAngle + (2 * scales[i-1]) - scales[i];
      else
	fudge = ((scales[i] - scales[i-1])/(thresholds[i]-thresholds[i-1]))*degAngle;
      found = true;
    }
  }

  if(!found)
    fudge = scales[3];

  if(angle <= 0)
    fudge *= -1;

  return fudge;
}
#endif

float WaypointEngine::fudgedAngle(float originalAngle) {
#ifdef TGT_IS_CREATE
  return originalAngle + tgtCreateTurnFudgeFactor(originalAngle);
#else
  return originalAngle;
#endif
}

void WaypointEngine::addEgocentricWaypoint(float forward, float left, float angle, bool angleIsRelative, float fwdSpeed, float turnSpeed) {
  waypoints.push_back(Waypoint(forward,left,Waypoint::POSTYPE_EGOCENTRIC,fudgedAngle(angle),angleIsRelative,fwdSpeed,isTracking,turnSpeed>=0?turnSpeed:defaultTurnSpeed));
}


void WaypointEngine::setTargetWaypoint(WaypointListIter_t iter) {
	//cout << "Moving to waypoint " << iter << endl;
	bool isLoop=false;
	if(iter==waypoints.end()) {
		if(isLooping && waypoints.size()>0) { //restart at beginning
			iter=waypoints.begin();
			for(unsigned int i=0; i<3; i++)
				pathStartPos[i]=curPos[i];
			isLoop=true;
		} else { //not looping, halt
			isRunning=false;
			curWaypoint=iter;
			for(unsigned int i=0; i<3; i++) {
				sourcePos[i]=targetPos[i];
				targetPos[i]=curPos[i];
				curVel[i]=0;
			}				
			return;
		}
	}
	if(iter== (iter+1) || isLoop)
		for(unsigned int i=0; i<3; i++)
			sourcePos[i]=targetPos[i];
	else
		for(unsigned int i=0; i<3; i++)
			sourcePos[i]=curPos[i];
	
	Waypoint target;
	if(isLoop)
		target=calcAbsoluteCoords(iter,pathStartPos[0],pathStartPos[1],pathStartPos[2]);
	else
		target=calcAbsoluteCoords(iter);
	targetPos[0]=target.x;
	targetPos[1]=target.y;
	targetPos[2]=target.angle;
	
	float dx=targetPos[0]-sourcePos[0];
	float dy=targetPos[1]-sourcePos[1];
	waypointDistance=std::sqrt(dx*dx+dy*dy);
	waypointTime=get_time();
	curWaypoint=iter;
	float radiusRatio=std::sin(target.arc/2);
	arcRadius = (radiusRatio==0) ? 0 : (waypointDistance/2)/radiusRatio;
	pathLength = arcRadius!=0 ? arcRadius*target.arc : waypointDistance;
	//
	
	// std::cout << "Target is now: ("<<targetPos[0]<<','<<targetPos[1]<<"  hdg" <<targetPos[2]<<")" << std::endl;
}


unsigned int WaypointEngine::getBinSize() const {
	unsigned int numPrecision=9;
	unsigned int wpSize=0;
	unsigned int boilerplateSize=0;
	boilerplateSize+=strlen("#WyP\n");
	boilerplateSize+=strlen("#add_{point|arc} {ego|off|abs} x_val y_val {hold|follow} angle_val speed_val arc_val\n");
	wpSize+=strlen("max_turn_speed ")+numPrecision+1;
	wpSize+=strlen("track_path false\n");
	wpSize+=strlen("add_point ")+4+numPrecision*5+1*5+strlen("follow");
	boilerplateSize+=strlen("#END\n");
	return wpSize*waypoints.size()+boilerplateSize;
}

unsigned int WaypointEngine::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	waypoints.clear();
	if(strncmp("#WyP\n",buf,5)!=0 && strncmp("#WyP\r",buf,5)!=0) {
		return 0;
	}
	
	float turn=defaultTurnSpeed; //init to current mode, in case file doesn't specify
	bool track=isTracking;
	char cmd[40];
	char posType[40];
	float x_val=0;
	float y_val=0;
	char angType[40];
	bool ang_val=0;
	float angle_val=0;
	float speed_val=0;
	float arc_val=0;
	unsigned int linenum=1;
	while(len<=origlen && len>0) {
		//		printf("%d %.9s\n",linenum+1,buf);
		if(buf[0]=='\r') {
			buf++; len--;
			if(buf[0]=='\n') {
				buf++; len--;
			}
			linenum++;
			continue;
		}
		if(buf[0]=='\n') {
			buf++; len--;
			linenum++;
			continue;
		}
		if(buf[0]=='#') {
			if(strncmp("#END\n",buf,5)==0 || strncmp("#END\r",buf,5)==0) {
				return origlen-len+5;
			} else if(strncmp("#END\r\n",buf,6)==0) {
				return origlen-len+6;
			} else {
				while(len>0 && *buf!='\n' && *buf!='\r') {len--;buf++;}
				if(*buf=='\n') { //in case of \r\n
					buf++;
					len--;
				}
				linenum++;
				continue;
			}
		}
		int used=-1U;
		sscanf(buf,"%40s%n",cmd,&used);
		if(!checkInc(used,buf,len,"*** ERROR Waypoint list load corrupted - ran out of room line %d\n",linenum)) return 0;
		if(strncasecmp(cmd,"add_point",9)==0 || strncasecmp(cmd,"add_arc",7)==0) {
			sscanf(buf,"%40s %g %g %40s %g %g %g%n",posType,&x_val,&y_val,angType,&angle_val,&speed_val,&arc_val,&used);
			if(!checkInc(used,buf,len,"*** ERROR Waypoint list load corrupted - bad read on add at line %d\n",linenum)) return 0;
			if(strncasecmp(angType,"hold",4)==0)
				ang_val=false;
			else if(strncasecmp(angType,"follow",6)==0)
				ang_val=true;
			else {
				printf("*** ERROR WaypointEngine: Invalid angle value type %s\n",angType);
				return 0;
			}
			if(strncasecmp(cmd,"add_point",9)==0) {
				if(strncasecmp(posType,"ego",3)==0)
					addEgocentricWaypoint(x_val,y_val,angle_val,ang_val,speed_val);
				else if(strncasecmp(posType,"off",3)==0)
					addOffsetWaypoint(x_val,y_val,angle_val,ang_val,speed_val);
				else if(strncasecmp(posType,"abs",3)==0)
					addAbsoluteWaypoint(x_val,y_val,angle_val,ang_val,speed_val);
				else {
					printf("*** ERROR WaypointEngine: Invalid position type %s\n",posType);
					return 0;
				}						
				waypoints.back().arc=arc_val;
			} else {
				if(strncasecmp(posType,"ego",3)==0)
					addEgocentricArc(x_val,y_val,angle_val,ang_val,speed_val,arc_val);
				else if(strncasecmp(posType,"off",3)==0)
					addOffsetArc(x_val,y_val,angle_val,ang_val,speed_val,arc_val);
				else if(strncasecmp(posType,"abs",3)==0)
					addAbsoluteArc(x_val,y_val,angle_val,ang_val,speed_val,arc_val);
				else {
					printf("*** ERROR WaypointEngine: Invalid position type %s\n",posType);
					return 0;
				}
			}
			waypoints.back().trackPath=track;
			waypoints.back().turnSpeed=turn;
		} else if(strncasecmp(cmd,"track_path",10)==0) {
			int track_tmp;
			sscanf(buf,"%d%n",&track_tmp,&used);
			track=track_tmp;
			if(!checkInc(used,buf,len,"*** ERROR Waypoint load corrupted - bad read on track_path line %d\n",linenum)) return 0;
		} else if(strncasecmp(cmd,"max_turn_speed",14)==0) {
			sscanf(buf,"%g%n",&turn,&used);
			if(!checkInc(used,buf,len,"*** ERROR Waypoint load corrupted - bad read on max_turn_speed line %d\n",linenum)) return 0;
		} else {
			printf("*** ERROR WaypointEngine: Invalid command %s\n",cmd);
			return 0;
		}
		
		linenum++;
	}
	std::cout << "*** WARNING WaypointEngine: load missing #END" << std::endl;
	return origlen-len;
}

unsigned int WaypointEngine::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origLen=len;
	unsigned int used;
	unsigned int cnt=0;
	
	used=snprintf(buf,len,"#WyP\n");
	if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on header\n")) return 0;
	
	used=snprintf(buf,len,"#add_{point|arc} {ego|off|abs} x_val y_val {hold|follow} angle_val speed_val arc_val\n");
	if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on header\n")) return 0;
	
	//set our state variables so we'll be forced to output the first set
	float turn=waypoints.front().turnSpeed-1;
	bool track=!waypoints.front().trackPath;
	
	for(WaypointListConstIter_t it=waypoints.begin(); it!=waypoints.end(); it++) {
		if(it->turnSpeed!=turn) {
			turn=it->turnSpeed;
			used=snprintf(buf,len,"max_turn_speed %g\n",turn);
			if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on waypoint %d turnSpeed\n",cnt)) return 0;
		}
		if(it->trackPath!=track) {
			track=it->trackPath;
			used=snprintf(buf,len,"track_path %d\n",track);
			if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on waypoint %d\n trackPath",cnt)) return 0;
		}
		const char * posType=NULL;
		switch(it->posType) {
			case Waypoint::POSTYPE_EGOCENTRIC:
				posType="EGO"; break;
			case Waypoint::POSTYPE_OFFSET:
				posType="OFF"; break;
			case Waypoint::POSTYPE_ABSOLUTE:
				posType="ABS"; break;
		}
		if(it->arc!=0)
			used=snprintf(buf,len,"add_point %s %g %g %s %g %g %g\n",posType,it->x,it->y,(it->angleIsRelative?"FOLLOW":"HOLD"),it->angle,it->speed,it->arc);
		else //todo - store center of circle
			used=snprintf(buf,len,"add_point %s %g %g %s %g %g %g\n",posType,it->x,it->y,(it->angleIsRelative?"FOLLOW":"HOLD"),it->angle,it->speed,it->arc);
		if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on waypoint %d\n",cnt)) return 0;
		cnt++;
	}
	
	used=snprintf(buf,len,"#END\n");
	if(!checkInc(used,buf,len,"*** ERROR Waypoint list save failed on footer\n")) return 0;
	
	return origLen-len;
}

void WaypointEngine::init() {
	eps[0]=eps[1]= 5.0f; //5 mm
	eps[2]=0.0175f; //1 degree
	for(unsigned int i=0; i<3; i++)
		pathStartPos[i]=targetPos[i]=sourcePos[i]=curPos[i]=curVel[i]=0;
	for(unsigned int i=0; i<4; i++)
		idealPos[i]=0;
}

/*! This is replicated in WaypointList, so any updates should be made there as well */
void WaypointEngine::fixArc(float arc) {
	Waypoint& center=waypoints.back();
	float cdx=center.x; //center delta
	float cdy=center.y; //center delta
	if(center.posType==Waypoint::POSTYPE_ABSOLUTE) {
		//have to find location of waypoint before last one
		if(waypoints.size()<=1) {
			cdx-=pathStartPos[0];
			cdy-=pathStartPos[1];
		} else {
			Waypoint start=calcAbsoluteCoords(waypoints.end()-2);
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

void WaypointEngine::computeCurrentPosition(unsigned int t) {
	float dt=(t-lastUpdateTime)/1000.f;
	float df=dt*curVel[0];
	float ds=dt*curVel[1];
	float da=dt*curVel[2];
	
	float avgAngle=curPos[2]+da/2;
	float ca=std::cos(avgAngle);
	float sa=std::sin(avgAngle);
	
	curPos[0]+=df*ca-ds*sa;
	curPos[1]+=df*sa+ds*ca;
	curPos[2]+=da;
	curPos[2]=mathutils::normalizeAngle(curPos[2]);
	
	lastUpdateTime=t;
}

void WaypointEngine::checkNextWaypoint(unsigned int /*t*/) {
	float rx=targetPos[0]-curPos[0];
	float ry=targetPos[1]-curPos[1];
	float ra=targetPos[2]-curPos[2];
	if(fabs(rx)<eps[0] && fabs(ry)<eps[1] && fabs(ra)<eps[2]) {
		setTargetWaypoint(curWaypoint+1);
	}
}

void WaypointEngine::computeIdeal(unsigned int t) {
	Waypoint& cur=(*curWaypoint);
	if(cur.trackPath) {
		float dx=targetPos[0]-sourcePos[0];
		float dy=targetPos[1]-sourcePos[1];
		//--------------- here
		float dt=(t-waypointTime)/1000.f; //time we've been traveling towards current waypoint
		float ideal_travel=dt*cur.speed; //distance we should have covered
		float p=1; //will be set to percentage of path length to waypoint we have covered
		if(pathLength!=0) {
			p=ideal_travel/pathLength;
			if(p>1)
				p=1;
		}
		if(arcRadius==0) { //radius is "infinite" - straight line solution
			idealPos[0]=sourcePos[0]+dx*p;
			idealPos[1]=sourcePos[1]+dy*p;
			idealPos[2]=targetPos[2];
			idealPos[3]=std::atan2(dy,dx);
		} else {
			// have to find center of arc's circle
			float bearing=std::atan2(dy,dx); //bearing of target from source
			float center_bearing=bearing+((float)M_PI-cur.arc)/2; //bearing of center from source
			float cx=sourcePos[0]+arcRadius*std::cos(center_bearing); //x pos of center
			float cy=sourcePos[1]+arcRadius*std::sin(center_bearing); //y pos of center
			float arc_bearing=center_bearing-(float)M_PI+cur.arc*p; //bearing from center to current ideal location
			//sout->printf("%g %g (%g,%g) %g\n",center_bearing,arcRadius,cx,cy,arc_bearing);
			idealPos[0]=cx+arcRadius*std::cos(arc_bearing);
			idealPos[1]=cy+arcRadius*std::sin(arc_bearing);
			idealPos[3]=arc_bearing+(float)M_PI/2;
			idealPos[2]=cur.angle;
			if(cur.angleIsRelative)
				idealPos[2]+=idealPos[3];
			idealPos[2]=mathutils::normalizeAngle(idealPos[2]);
			idealPos[3]=mathutils::normalizeAngle(idealPos[3]);
		}
	} else {
		idealPos[0]=curPos[0];
		idealPos[1]=curPos[1];
		float rx=targetPos[0]-curPos[0];
		float ry=targetPos[1]-curPos[1];
		if(std::abs(rx)<eps[0] && std::abs(ry)<eps[1]) {
			idealPos[2]=targetPos[2];
		} else {
			idealPos[2]=cur.angle;
			if(cur.angleIsRelative) {
				float dx=targetPos[0]-curPos[0];
				float dy=targetPos[1]-curPos[1];
				idealPos[2]+=std::atan2(dy,dx);
			}
			idealPos[2]=mathutils::normalizeAngle(idealPos[2]);
		}
		idealPos[3]=std::atan2(ry,rx);
		if(arcRadius!=0) {
			//---------------here
			float dt=(t-waypointTime)/1000.f; //time we've been traveling towards current waypoint
			float ideal_travel=dt*cur.speed; //distance we should have covered
			float p=1; //will be set to percentage of path length to waypoint we have covered
			if(pathLength!=0) {
				p=ideal_travel/pathLength;
				if(p>1)
					p=1;
			}
			float arc=cur.arc*(1-p)/2;
			idealPos[2]=mathutils::normalizeAngle(idealPos[2]-arc);
			idealPos[3]=mathutils::normalizeAngle(idealPos[3]-arc);
		}
	}
}
  
void WaypointEngine::computeNewVelocity(unsigned int /*t*/) {
	Waypoint& cur=(*curWaypoint);
	
	//first we'll start with the velocity we would use if we were on path
	//determine distance remaining (only an approximation for arcing paths)
	float dx=targetPos[0]-idealPos[0];
	float dy=targetPos[1]-idealPos[1];
	float spd=std::sqrt(dx*dx+dy*dy)/(FrameTime*NumFrames)*1000.f;
	if(spd>cur.speed) {
		//we're far away - go at full speed
		curVel[0]=cur.speed*std::cos(idealPos[3]-curPos[2]);
		curVel[1]=cur.speed*std::sin(idealPos[3]-curPos[2]);
	} else {
		//we're about to overshoot... just go fast enough to get on target
		curVel[0]=spd*std::cos(idealPos[3]-curPos[2]);
		curVel[1]=spd*std::sin(idealPos[3]-curPos[2]);
	}
	if(arcRadius==0)
		curVel[2]=0;
	else
		curVel[2]=cur.speed/arcRadius;
	
	//sout->printf("ideal vel: %g %g %g\n",curVel[0],curVel[1],curVel[2]);
	
	// then we'll add the errors
	float ex=idealPos[0]-curPos[0];
	float ey=idealPos[1]-curPos[1];
	float ed=std::sqrt(ex*ex+ey*ey);
	float ehead=std::atan2(ey,ex)-curPos[2];
	float ea=mathutils::normalizeAngle(idealPos[2]-curPos[2]);
	float easpd=ea/(FrameTime*NumFrames)*1000.f;
	easpd=mathutils::limitRange(easpd,-cur.turnSpeed,cur.turnSpeed);
	curVel[0]+=Pcorr*ed*std::cos(ehead);
	curVel[1]+=Pcorr*ed*std::sin(ehead);
	curVel[2]+=easpd;
}

/*! @file
 * @brief Defines WaypointEngine, which provides computation and management of a desired path through a series of waypoints
 * @author ejt (Creator)
 */
