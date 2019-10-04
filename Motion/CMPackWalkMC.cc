//This class is ported from Carnegie Mellon's 2001 Robosoccer entry,
//and falls under their license:
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

#include "Shared/RobotInfo.h"
#ifdef TGT_IS_AIBO    // lock out wheeled robots

#define WALKMC_NO_WARN_NOOP
#include "CMPackWalkMC.h"

#include "Shared/WorldState.h"
#include "Events/EventRouter.h"
#include "Events/LocomotionEvent.h"
#include "Wireless/Socket.h"
#include "Shared/Config.h"

#include "Motion/MotionManager.h"

#if !defined(PLATFORM_APERIOS) && defined(TGT_HAS_LEGS)
#  include "Kinematics.h"
#  include "IPC/DriverMessaging.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <cmath>

//#define BOUND_MOTION

#ifdef TGT_HAS_LEGS
const float CMPackWalkMC::MAX_DX   = 180;//225; // mm/sec
const float CMPackWalkMC::MAX_DY   = 140;//170; // mm/sec
const float CMPackWalkMC::MAX_DA   = 1.8f;//2.1; // rad/sec
#else
// Create robot
const float CMPackWalkMC::MAX_DX   = 225; // mm/sec
const float CMPackWalkMC::MAX_DY   = 0  ; // mm/sec
const float CMPackWalkMC::MAX_DA   = 2.1f; // rad/sec
#endif
#ifdef TGT_HAS_WHEELS
const float CMPackWalkMC::OPTIMAL_DA   = 0.9f; // rad/sec for Create: value that seems to give reasonably accurate odometry
#endif
const vector3d CMPackWalkMC::max_accel_xya(MAX_DX*2,MAX_DY*2,MAX_DA*2);


unsigned int checksum(const char *data,int num); //!< computes a file checksum

CMPackWalkMC::CMPackWalkMC(const char* pfile/*=NULL*/)
	: MotionCommand(), isPaused(false), dirty(false), wp(), cp(), body_loc(), body_angle(),
	acc_style(DEFAULT_ACCEL), step_count(-1), step_threshold(0), last_cycle(0),
	pos_delta(0,0,0), angle_delta(0), travelTime(get_time()),
	time(get_time()), TimeStep(FrameTime), slowmo(1.0f),
	CycleOffset(0), TimeOffset(0), NewCycleOffset(false),
	target_disp_xya(0,0,0), vel_xya(0,0,0), target_vel_xya(0,0,0), last_target_vel_xya(0,0,0)
{ init(pfile); }

CMPackWalkMC::CalibrationParam::CalibrationParam() {
	for(unsigned int r=0; r<3; r++) {
		for(unsigned int c=0; c<11; c++)
			f_calibration[r][c]=b_calibration[r][c]=0;
		f_calibration[r][r]=b_calibration[r][r]=1;
	}
	for(unsigned int d=0; d<NUM_DIM; d++)
		max_accel[d]=0;
	max_vel[0]=max_vel[1]=MAX_DX;
	max_vel[2]=MAX_DY;
	max_vel[3]=MAX_DA;
}


void CMPackWalkMC::start() {
	MotionCommand::start();
	LocomotionEvent e(EventBase::locomotionEGID,getID(),EventBase::activateETID,0);
	e.setXYA((float)target_vel_xya.x, (float)target_vel_xya.y, (float)target_vel_xya.z);
	postEvent(e);
	travelTime=get_time();
}

void CMPackWalkMC::stop() {
	zeroVelocities();
	MotionCommand::stop();
}

void CMPackWalkMC::zeroVelocities() {
  unsigned int t=get_time();
  setTargetVelocity(0,0,0);
#ifdef TGT_HAS_WHEELS
  for (unsigned int i = WheelOffset; i < WheelOffset + NumWheels; i++)
    motman->setOutput(this, i, 0.0f);
#endif
  LocomotionEvent e(EventBase::locomotionEGID, getID(), EventBase::deactivateETID, t-travelTime); // velocities default to 0
  travelTime = t;
  postEvent(e);
}



void CMPackWalkMC::init(const char* pfile)
{
#ifdef TGT_HAS_LEGS
	//	RegInit();
	body_loc.init(vector3d(0,0,wp.body_height),vector3d(0,0,0));
	body_angle.init(vector3d(0,wp.body_angle,0),vector3d(0,0,0));
	
	for(unsigned int i=0; i<NumLegs; i++)
		legw[i].air = false;
	
	if(pfile!=NULL)
		loadFile(pfile);
	else
		loadFile(config->motion.walk.c_str());
	
	resetLegPos();
	
	//	cmds[HeadOffset+TiltOffset].set(.3333,1);
#endif
}

int CMPackWalkMC::isDirty()
{
	if(isPaused) return 0;
	if(step_count==0) return 0;
	if((target_vel_xya.x == 0) && (target_vel_xya.y == 0) && (target_vel_xya.z == 0)) {
		// we may stopping, but not stopped yet...
		if((vel_xya.x == 0) && (vel_xya.y == 0) && (vel_xya.z == 0))
			return 0;
	}
#ifdef TGT_HAS_LEGS
	return JointsPerLeg*NumLegs;
#elif defined(TGT_HAS_WHEELS)
	return NumWheels;
#else
	return 0;
#endif
}

unsigned int CMPackWalkMC::getBinSize() const {
	unsigned int used=0;
	used+=creatorSize("WalkMC-2.2");
	used+=sizeof(wp);
	used+=sizeof(cp);
	return used;	
}

unsigned int CMPackWalkMC::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	enum {
		VER1, // original CMPack version
		VER2, // added calibration parameters
		VER3 // added diff drive and sag parameter
	} version;
	
	unsigned int origlen=len;
	if(checkCreatorInc("WalkMC-2.2",buf,len,false)) {
		version=VER3;
	} else if(checkCreatorInc("WalkMC",buf,len,false)) {
			sout->printf("Pre-2.2 release walk parameter file %s\n",filename);
		version=VER2;
	} else {
		// backwards compatability - if there's no creator code, just assume it's a straight WalkParam
		sout->printf("Assuming CMPack format walk parameter file %s\n",filename);
		version=VER1;
	}
	
	// Leg parameters
	for(unsigned int i=0; i<4; ++i) {
		if(!decodeInc(wp.leg[i].neutral.x,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].neutral.y,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].neutral.z,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].lift_vel.x,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].lift_vel.y,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].lift_vel.z,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].down_vel.x,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].down_vel.y,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].down_vel.z,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].lift_time,buf,len)) return 0;
		if(!decodeInc(wp.leg[i].down_time,buf,len)) return 0;
	}
	// Body parameters
	if(!decodeInc(wp.body_height,buf,len)) return 0;
	if(!decodeInc(wp.body_angle,buf,len)) return 0;
	if(!decodeInc(wp.hop,buf,len)) return 0;
	if(!decodeInc(wp.sway,buf,len)) return 0;
	if(!decodeInc(wp.period,buf,len)) return 0;
	if(version<VER3) wp.useDiffDrive=0; else if(!decodeInc(wp.useDiffDrive,buf,len)) return 0;
	if(version<VER3) wp.sag=0; else if(!decodeInc(wp.sag,buf,len)) return 0;
	if(!decodeInc(wp.reserved,buf,len)) return 0;
	// Calibration parameters
	if(version<VER2) {
		cp=CalibrationParam();
	} else {
		for(unsigned int i=0; i<3; ++i)
			for(unsigned int j=0; j<11; ++j)
				if(!decodeInc(cp.f_calibration[i][j],buf,len)) return 0;
		for(unsigned int i=0; i<3; ++i)
			for(unsigned int j=0; j<11; ++j)
				if(!decodeInc(cp.b_calibration[i][j],buf,len)) return 0;
		for(unsigned int i=0; i<CalibrationParam::NUM_DIM; ++i)
			if(!decodeInc(cp.max_accel[i],buf,len)) return 0;
		for(unsigned int i=0; i<CalibrationParam::NUM_DIM; ++i)
			if(!decodeInc(cp.max_vel[i],buf,len)) return 0;
	}
	return origlen-len;	
}

unsigned int CMPackWalkMC::saveBuffer(char buf[], unsigned int len) const {
	unsigned int origlen=len;
	// Leg parameters
	if(!saveCreatorInc("WalkMC-2.2",buf,len)) return 0;
	for(unsigned int i=0; i<4; ++i) {
		if(!encodeInc(wp.leg[i].neutral.x,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].neutral.y,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].neutral.z,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].lift_vel.x,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].lift_vel.y,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].lift_vel.z,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].down_vel.x,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].down_vel.y,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].down_vel.z,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].lift_time,buf,len)) return 0;
		if(!encodeInc(wp.leg[i].down_time,buf,len)) return 0;
	}
	// Body parameters
	if(!encodeInc(wp.body_height,buf,len)) return 0;
	if(!encodeInc(wp.body_angle,buf,len)) return 0;
	if(!encodeInc(wp.hop,buf,len)) return 0;
	if(!encodeInc(wp.sway,buf,len)) return 0;
	if(!encodeInc(wp.period,buf,len)) return 0;
	if(!encodeInc(wp.useDiffDrive,buf,len)) return 0;
	if(!encodeInc(wp.sag,buf,len)) return 0;
	if(!encodeInc(wp.reserved,buf,len)) return 0;
	// Calibration parameters
	for(unsigned int i=0; i<3; ++i)
		for(unsigned int j=0; j<11; ++j)
			if(!encodeInc(cp.f_calibration[i][j],buf,len)) return 0;
	for(unsigned int i=0; i<3; ++i)
		for(unsigned int j=0; j<11; ++j)
			if(!encodeInc(cp.b_calibration[i][j],buf,len)) return 0;
	for(unsigned int i=0; i<CalibrationParam::NUM_DIM; ++i)
		if(!encodeInc(cp.max_accel[i],buf,len)) return 0;
	for(unsigned int i=0; i<CalibrationParam::NUM_DIM; ++i)
		if(!encodeInc(cp.max_vel[i],buf,len)) return 0;
	return origlen-len;	
}

unsigned int CMPackWalkMC::loadFile(const char* filename) {
	return LoadSave::loadFile(config->motion.makePath(filename).c_str());
}
unsigned int CMPackWalkMC::saveFile(const char* filename) const {
	return LoadSave::saveFile(config->motion.makePath(filename).c_str());
}

void CMPackWalkMC::setTargetVelocity(double dx,double dy,double da) {
#ifdef BOUND_MOTION
	da = bound(da, -cp.rotate_max_vel, cp.rotate_max_vel);
	double fa = 1.0 - fabs(da / cp.rotate_max_vel);
	
	vector2d v(dx>0?dx/cp.forward_max_vel:dx/cp.reverse_max_vel,dy/cp.strafe_max_vel);
	double l = v.length();
	if(l > 1) v /= l;
	
	dx = (dx>0?cp.forward_max_vel:cp.reverse_max_vel) * v.x * fa;
	dy = cp.strafe_max_vel * v.y * fa;
#endif
	
#ifdef TGT_HAS_WHEELS
	dy=0; // can't move sideways...
#endif
	
	// static int myCounter = 0;
	// if ( (myCounter++ % 100) == 0 || dx != target_vel_xya.x )
	//   std::cout << "setTargetVelocity " << dx << ' ' << dy << ' ' << da << std::endl;

	// Modifiy the target velocity, but hold off on generating an event
	// until the changes are actually picked up by the motion system.
	target_vel_xya.set(dx,dy,da);
}

void CMPackWalkMC::setTargetVelocity(double vx,double vy,double va,double dur) {
	if(dur<=0) {
		setTargetVelocity(0,0,0);
		return;
	}
	setTargetVelocity(vx,vy,va);
#if TGT_HAS_LEGS
	//compute the point in cycle of the mid-step
	float midsteps[NumLegs];
	for(unsigned int i=0; i<NumLegs; i++)
		midsteps[i] = (float)(wp.leg[i].down_time+wp.leg[i].lift_time)/2;
	//find the number of unique steps per cycle
	unsigned int steps_per_cycle=NumLegs;
	for(unsigned int i=0; int(i)<int(NumLegs)-1; i++) {
		//std::cout << "i=" << i << "\n";
		for(unsigned int j=i+1; j<NumLegs; j++) {
			if(midsteps[i]==midsteps[j]) {
				steps_per_cycle--;
				break;
			}
		}
	}
	step_count = std::max(1,static_cast<int>(dur/(wp.period/1000.0)*steps_per_cycle + 0.5));
#elif TGT_HAS_WHEELS
	target_disp_xya.set(vx*dur,vy*dur,va*dur);
	// std::cout << "setTargetVelocity: target_disp_xya " << vx*dur << " " << vy*dur << " " << va*dur << std::endl;
	step_count=1;
#else
	std::cout << "CMPackWalkMC::setTargetVelocity with duration undefined for this robot architecture.\n";
#endif
}

void CMPackWalkMC::setTargetDisplacement(double dx, double dy, double da, double dur) {
	if(dur<0) {
		setTargetVelocity(0,0,0);
		return;
	}
	const float tx = std::abs((float)dx/getMaxXVel());
	const float ta = std::abs((float)da/getMaxAVel());
#if TGT_HAS_LEGS
	const float ty = std::abs((float)dy/getMaxYVel());
	const double maxTime =  std::max<double>(dur,std::max(tx,std::max(ty,ta)));
	setTargetVelocity(dx/maxTime, dy/maxTime, da/maxTime, maxTime);
#elif TGT_HAS_WHEELS
	(void)dy; // to avoid unused variable warning
	const double maxTime =  std::max<double>(dur,std::max(tx,ta));
	setTargetVelocity(dx/maxTime, 0, da/maxTime);
	target_disp_xya.set(dx,0,da);
	// std::cout << "setTargetDisplacement: target_disp_xya " << dx << " " << 0 << " " << da << std::endl;
	step_count=1;
#else
	(void)tx; // avoid unused variable warnings
	(void)ta;
	std::cout << "CMPackWalkMC::setTargetDisplacement undefined for this robot architecture.\n";
#endif
}

int CMPackWalkMC::updateOutputs() {
	if(!isDirty()) return 0;
	unsigned int curT=get_time();
	if ( fabs(target_vel_xya.z) < 1e-3 )
	  target_vel_xya.z = 0;  // trying to waypointwalk backwards causes tiny oscillations in angular velocity
	// post a locomotion event if we are changing velocity by a nontrivial amount
	if ( last_target_vel_xya != target_vel_xya ) {
		last_target_vel_xya=target_vel_xya;
		LocomotionEvent e(EventBase::locomotionEGID,getID(),EventBase::statusETID,getTravelTime());
		e.setXYA((float)target_vel_xya.x, (float)target_vel_xya.y, (float)target_vel_xya.z);
		postEvent(e);
		travelTime=curT;
	}
	
#if defined(TGT_HAS_LEGS)
	if(vel_xya.sqlength()==0) {
		// we had been stopped - better check someone else didn't move the legs while we weren't using them...
		resetLegPos(); 
	}
	
	float tm = TimeStep * slowmo / 1000;
	
	vector3d cal_target_vel_xya(target_vel_xya);
	if(target_vel_xya.x<0)
		cal_target_vel_xya.x/=cp.max_vel[CalibrationParam::reverse];
	else
		cal_target_vel_xya.x/=cp.max_vel[CalibrationParam::forward];
	cal_target_vel_xya.y/=cp.max_vel[CalibrationParam::strafe];
	cal_target_vel_xya.z/=cp.max_vel[CalibrationParam::rotate];
	if(cal_target_vel_xya.sqlength()>.0025) {
		if(target_vel_xya.x<0)
			applyCalibration(cp.b_calibration,target_vel_xya,cal_target_vel_xya);
		else
			applyCalibration(cp.f_calibration,target_vel_xya,cal_target_vel_xya);
	}
	
	if(step_count<0 && (acc_style==CALIBRATION_ACCEL || (acc_style==DEFAULT_ACCEL && !config->motion.inf_walk_accel))) {
		//software accel:
		vel_xya.x = bound(cal_target_vel_xya.x, vel_xya.x-max_accel_xya.x*tm, vel_xya.x+max_accel_xya.x*tm);
		vel_xya.y = bound(cal_target_vel_xya.y, vel_xya.y-max_accel_xya.y*tm, vel_xya.y+max_accel_xya.y*tm);
		vel_xya.z = bound(cal_target_vel_xya.z, vel_xya.z-max_accel_xya.z*tm, vel_xya.z+max_accel_xya.z*tm);
	} else {
		//no software accel:
		vel_xya=cal_target_vel_xya;
	}
	
	BodyPosition delta;
	delta.loc.set(vel_xya.x*tm,vel_xya.y*tm,0);
	delta.angle.set(0,0,vel_xya.z*tm);
	
	time=(int)(curT*slowmo);
	
	// tss "SmoothWalk" addition follows
	// If necessary, we compute a new TimeOffset here.
	if(NewCycleOffset) {
		TimeOffset = CycleOffset - time % wp.period;
		NewCycleOffset = false;
	}
	
	// Adjusted time--time adjusted for cycle matching
	int AdjustedTime = time + TimeOffset;
	
	// If walking speeds have dwindled down to zero, save our time offset from the beginning of the current walk cycle. Once we start walking again, we start up at the same offset to prevent jerky starts.
	if((vel_xya.x == 0) && (vel_xya.y == 0) && (vel_xya.z == 0)) {
		CycleOffset = AdjustedTime % wp.period;
		NewCycleOffset = true;
	}
	// tss "SmoothWalk" addition ends
	
#ifndef PLATFORM_APERIOS
	bool contactChanged=false;
#endif
	for(unsigned int frame=0; frame<NumFrames; frame++) {
		cal_target_vel_xya.rotate_z(-delta.angle.z);
		
		// incorporate movement delta
		angle_delta += (float)delta.angle.z;
		pos_delta += delta.loc.rotate_z(angle_delta);
		
		//		cout << "setup,,," << flush;
		
		// tss "SmoothWalk" modification follows
		// double cycle = (double)(time % wp.period) / wp.period;
		AdjustedTime = time + TimeOffset + (int)(frame*TimeStep*slowmo);
		double cycle = (double)(AdjustedTime % wp.period) / wp.period;
		
		if(step_count>0) {
			for(unsigned int i=0; i<NumLegs; i++){
				float midstep;
				if(step_threshold<=.5f)
					midstep=(float)(wp.leg[i].lift_time+(wp.leg[i].down_time-wp.leg[i].lift_time)*step_threshold*2);
				else
					midstep=(float)(wp.leg[i].down_time+(1-wp.leg[i].down_time+wp.leg[i].lift_time)*(step_threshold-.5)*2);
				midstep-=floorf(midstep);
				//cout << "leg "<<i<<": " <<AdjustedTime << ' ' << cycle << ' ' << last_cycle << ' ' << midstep << ' ' <<step_count ;
				bool above_last= (last_cycle<midstep);
				bool below_cur = (midstep<=cycle);
				bool wrap      = (cycle<last_cycle);
				//need any 2 of the conditions: above_last && below_cur || wrap && (above_last || below_cur)
				if(above_last+below_cur+wrap>1) { //we just completed a step
					step_count--;
					//cout << " -> " << step_count << endl;
					if(step_count==0) { //we're done, copy out any completed frames
						for(unsigned int f=0; f<frame; f++)
							for(unsigned int joint=LegOffset; joint<LegOffset+NumLegJoints; joint++)
								motman->setOutput(this,joint,cmds[joint][f],f);
						CycleOffset = AdjustedTime % wp.period;
						NewCycleOffset = true;
						last_cycle=cycle;
						target_vel_xya.set(0,0,0);
						last_target_vel_xya=target_vel_xya;
						LocomotionEvent e(EventBase::locomotionEGID,getID(),EventBase::statusETID,getTravelTime());
						e.setXYA((float)target_vel_xya.x, (float)target_vel_xya.y, (float)target_vel_xya.z);
						//cout << e.getDescription(true) << endl;
						postEvent(e);
						postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID,getTravelTime()));
						//		cout << "WalkMC-done" << endl;
						return frame==0?0:NumLegs*JointsPerLeg;
					}
					break; //don't count legs moving in sync as two steps, only ever one step at a time
				}
				//cout << endl;
			}
		}
		
		
		double sway   = wp.sway*cos(2*M_PI*cycle);
		double hop    = wp.hop*sin(4*M_PI*cycle);
		double height = wp.body_height;
		BodyPosition nextpos;
		nextpos.loc.set(0,-sway,height+hop);
		nextpos.angle.set(0,wp.body_angle,0);
		
		//		cout << "loop,,," << flush;
		for(unsigned int i=0; i<NumLegs; i++){
			
			//interpret a down time before a lift time as a request to move the leg in reverse (has its uses)
			float lift_time=(float)wp.leg[i].lift_time;
			float down_time=(float)wp.leg[i].down_time;
			float dir=1;
			if(down_time==lift_time)
				dir=0;
			else if(down_time<lift_time) {
				lift_time=(float)wp.leg[i].down_time;
				down_time=(float)wp.leg[i].lift_time;
				dir=-1;
			}
			
			bool air = (cycle >= lift_time) && (cycle < down_time);
			double air_f = down_time - lift_time;
			double nextlegangles[JointsPerLeg];
			
			if(air != legw[i].air){
#ifndef PLATFORM_APERIOS
				contactChanged=true;
#endif
				if(air){
					/*
					 cout << i << ":   ";
					 cout << legpos[i].x << ' ' << legpos[i].y << ' ' << legpos[i].z << "  ->  ";
					 GetLegAngles(nextlegangles,legpos[i],nextpos,i);
					 for(unsigned int j=0; j<JointsPerLeg; j++)
					 cout << nextlegangles[j] << ' ';
					 cout << endl;
					 */
					tm = wp.period/1000.f * 0.75f; //wtf is the 0.75 based on?  Don't ask me, i just work here! (ejt)
					vector3d vfp;
					double vfa;
					if(step_count<0 && (acc_style==CALIBRATION_ACCEL || (acc_style==DEFAULT_ACCEL && !config->motion.inf_walk_accel))) {
						//software accel:
						vfp.x = bound(cal_target_vel_xya.x, vel_xya.x-max_accel_xya.x*tm, vel_xya.x+max_accel_xya.x*tm);
						vfp.y = bound(cal_target_vel_xya.y, vel_xya.y-max_accel_xya.y*tm, vel_xya.y+max_accel_xya.y*tm);
						vfa   = bound(cal_target_vel_xya.z, vel_xya.z-max_accel_xya.z*tm, vel_xya.z+max_accel_xya.z*tm);
					} else {
						//no software accel:
						vfp.x=cal_target_vel_xya.x;
						vfp.y=cal_target_vel_xya.y;
						vfa=cal_target_vel_xya.z;
					}
					
					vfp.z = 0.0;
					double b = (wp.period/1000.0) * (1.0 - air_f) / 2.0;
					vector3d target;
					if(wp.useDiffDrive) {
						float rot = (float)(vfa/cp.max_vel[CalibrationParam::rotate]);
						if((i&1)==0)
							rot=-rot;
						vfp.x += cp.max_vel[CalibrationParam::forward]*rot;
						target = (wp.leg[i].neutral + vfp*b*dir);
					} else {
						target = (wp.leg[i].neutral + vfp*b*dir).rotate_z(vfa*b);
					}
					target.z+=wp.sag;
					liftPos[i]=legpos[i];
					downPos[i]=target;
					legw[i].airpath.create(legpos[i],target,wp.leg[i].lift_vel,wp.leg[i].down_vel);
				}else{
					legpos[i].z = wp.leg[i].neutral.z;
				}
				legw[i].air = air;
			}
			
			if(air){
				legw[i].cyc = (float)( (cycle - lift_time) / air_f );
				legpos[i] = legw[i].airpath.eval(legw[i].cyc);
				
				// Core tss "SmoothWalk" addition follows
				// KLUDGY MOD. Goal: reduce the height of the
				// AIBO's steps as its velocity nears zero.
				// Since I don't know how most of this code 
				// works, I'll directly alter legpos[i].z in
				// the following code to change the z height
				// with velocity.
				double velfraction_x = fabs(vel_xya.x / MAX_DX);
				double velfraction_y = fabs(vel_xya.y / MAX_DY);
				double velfraction_a = fabs(vel_xya.z / MAX_DA * 2); //rotation seems more sensitive
				
				// Choose the biggest velfraction
				double velfraction =
				(velfraction_x > velfraction_y) ?
				velfraction_x : velfraction_y;
				if(velfraction_a > velfraction)
					velfraction = velfraction_a;
				
				// Modify legpos[i].z with velfraction to
				// shrink it down
				//velfraction = atan(velfraction * M_PI);
				
				velfraction-=1;
				velfraction*=velfraction;
				velfraction*=velfraction;
				
				legpos[i].z *= 1-velfraction;
				// Core tss "SmoothWalk" addition ends
			}else{
				if(dir==0)
					legpos[i]=wp.leg[i].neutral;
				else {
					if(wp.useDiffDrive) {
						tm = wp.period/1000.f * 0.75f; //wtf is the 0.75 based on?  Don't ask me, i just work here! (ejt)
						double vfa;
						if(step_count<0 && (acc_style==CALIBRATION_ACCEL || (acc_style==DEFAULT_ACCEL && !config->motion.inf_walk_accel))) {
							vfa   = bound(cal_target_vel_xya.z, vel_xya.z-max_accel_xya.z*tm, vel_xya.z+max_accel_xya.z*tm);
						} else {
							vfa   = cal_target_vel_xya.z;
						}
						legpos[i] -= delta.loc*dir;
						float rot = (float)( vfa/cp.max_vel[CalibrationParam::rotate]*TimeStep * slowmo / 1000 );
						if((i&1)==0)
							rot=-rot;
						legpos[i].x -= cp.max_vel[CalibrationParam::forward]*rot;
					} else {
						legpos[i] = (legpos[i] - delta.loc*dir).rotate_z(-delta.angle.z);
					}
				}
			}
			
			GetLegAngles(nextlegangles,legpos[i],nextpos,i);
			for(unsigned int j=0; j<JointsPerLeg; j++)
				cmds[LegOffset+i*JointsPerLeg+j][frame].set((float)nextlegangles[j]);
			
		}
		last_cycle=cycle;
	}
	
	for(unsigned int joint=LegOffset; joint<LegOffset+NumLegJoints; joint++)
		motman->setOutput(this,joint,cmds[joint]);
	
#ifndef PLATFORM_APERIOS
	if(contactChanged) {
		DriverMessaging::FixedPoints contactMsg;
		for(unsigned int leg=0; leg<NumLegs; ++leg) {  //need to add all contacts to sync which are current
#ifdef TGT_IS_AIBO
			// not entirely accurate to place contact at (0,0,0):
			// there's a heel around it that will roll forward with step, but we'll just fix the point below the heel
			fmat::Column<3> off = kine->baseToLink(FootFrameOffset+leg).rotation() * fmat::pack(0,0,-BallOfFootRadius);
			if(!legw[leg].air)
				contactMsg.addEntry(FootFrameOffset+leg, off);
#else
			if(!legw[leg].air)
				contactMsg.addEntry(FootFrameOffset+leg, 0,0,0);
#endif
		}
		DriverMessaging::postMessage(contactMsg);
	}
#endif
		
	//		cout << "WalkMC-done" << endl;
	return NumLegs*JointsPerLeg;
	
#elif defined(TGT_HAS_WHEELS)
	// see if we've reached our target displacement
	unsigned int t = getTravelTime();
	double adjustedTravelSeconds = t/1000.0;
	// std::cout << "tvelx= " << target_vel_xya.x << "  travtime=" << t << "  dispx=" << target_disp_xya.x << std::endl;
	if (step_count > 0)  // use nsteps=-1 to move forever
		if ( fabs(target_vel_xya.x)*adjustedTravelSeconds > fabs(target_disp_xya.x) ||
			fabs(target_vel_xya.z)*adjustedTravelSeconds > fabs(target_disp_xya.z) ) {
			target_vel_xya.set(0,0,0);
			last_target_vel_xya=target_vel_xya;
			LocomotionEvent e(EventBase::locomotionEGID,getID(),EventBase::statusETID,t);
			e.setXYA((float)target_vel_xya.x,(float)target_vel_xya.y,(float)target_vel_xya.z);
			postEvent(e);
			postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID,getTravelTime()));
			return 0;
		}
	
	// we're not at target yet, so recompute wheel velocities
	vel_xya=target_vel_xya;
#ifdef TGT_IS_CREATE
	/* this used to be a config item, now hardcoded due to deprecation
	  * of this section (only place it was used, should be extracted from kinematics) */
	const float WHEEL_BASE_WIDTH = 265;
#else
	const float WHEEL_BASE_WIDTH = 140;
#endif
	float turnvel = (float)vel_xya.z * WHEEL_BASE_WIDTH/2;
	if ( fabs(turnvel) < 1e-3)  // eliminate oscillations that generate lots of LocomotionEvents
	  turnvel = 0;
	float left = (float)vel_xya.x - turnvel;
	float right = (float)vel_xya.x + turnvel;
	/* debugging stuff
	static int myCounter = 0;
	if ( (myCounter++ % 25) == 0 || vel_xya.x != target_vel_xya.x )
	  std::cout << "update: " << vel_xya.x << ' ' << vel_xya.y << ' ' << vel_xya.z << " -> " << left << ' ' << right
	            << " last xvel: " << vel_xya.x << " target xvel: " << target_vel_xya.x
	            << ": id=" << getID() << " prio=" << motman->getPriority(getID()) << " cnt=" << myCounter << std::endl;
	*/
	for(unsigned int frame=0; frame<NumFrames; frame++) {
		cmds[LWheelOffset][frame]=left;
		cmds[RWheelOffset][frame]=right;
	}
	motman->setOutput(this, LWheelOffset, cmds[LWheelOffset]);
	motman->setOutput(this, RWheelOffset, cmds[RWheelOffset]);
	return NumWheels;
	
#else
	return 0;
#endif
}

void CMPackWalkMC::resetLegPos() {
	BodyPosition nextpos;
	nextpos.loc.set(0,0,wp.body_height);
	nextpos.angle.set(0,wp.body_angle,0);
#ifdef TGT_HAS_LEGS
	for(unsigned int i=0; i<NumLegs; i++) {
		double tmp[JointsPerLeg];
		for(unsigned int j=0; j<JointsPerLeg; j++)
			tmp[j]=state->outputs[LegOffset+i*JointsPerLeg+j];
		GetLegPosition(legpos[i],tmp,nextpos,i);
		/*
		 for(unsigned int j=0; j<JointsPerLeg; j++)
		 cout << state->outputs[LegOffset+i*JointsPerLeg+j] << ' ';
		 cout << "  ->  " << legpos[i].x << ' ' << legpos[i].y << ' ' << legpos[i].z << endl;
		 */
	}
#endif
	//cout << "----------------------" << endl;
}

unsigned int checksum(const char *data,int num)
{
	unsigned long c;
	int i;
	
	c = 0;
	for(i=0; i<num; i++){
		c = c ^ (data[i]*13 + 37);
		c = (c << 13) | (c >> 19);
	}
	
	return(c);
}

void CMPackWalkMC::applyCalibration(const float mat[3][11], const vector3d& in, vector3d& out) {
	float inmat[11];
	inmat[0]=(float)in.x;
	inmat[1]=(float)in.y;
	inmat[2]=(float)in.z;
	inmat[3]=std::abs((float)in.y);
	inmat[4]=std::abs((float)in.z);
	inmat[5]=std::exp(-.5f*(float)in.z*(float)in.z)*std::sin((float)in.z*2.5f);
	inmat[6]=(float)in.x*(float)in.x+(float)in.y*(float)in.y;
	inmat[7]=(float)in.x*(float)in.z;
	inmat[8]=(float)in.y*(float)in.x;
	inmat[9]=(float)in.z*(float)in.y;
	inmat[10]=1;
	out.set(0,0,0);
	for(unsigned int c=0; c<11; c++)
		out.x+=mat[0][c]*inmat[c];
	for(unsigned int c=0; c<11; c++)
		out.y+=mat[1][c]*inmat[c];
	for(unsigned int c=0; c<11; c++)
		out.z+=mat[2][c]*inmat[c];
}

/*! @file
 * @brief Implements CMPackWalkMC, a MotionCommand for walking around
 * @author CMU RoboSoccer 2001-2002 (Creator)
 * @author ejt (ported)
 *
 * @verbinclude CMPack_license.txt
 */

#endif // lock out wheeled robots
