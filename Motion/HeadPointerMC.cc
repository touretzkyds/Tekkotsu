#include "HeadPointerMC.h"
#include "Kinematics.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "MotionManager.h"
#include "Shared/Config.h"
#include "Wireless/Socket.h"
#include "Shared/ERS7Info.h"
#include "Shared/ERS210Info.h"
#include "Motion/IKSolver.h"
#include "Shared/get_time.h"
#include "Events/EventBase.h"
#include "DualCoding/VRmixin.h"
#include "Crew/MapBuilder.h"
#include "DualCoding/Point.h"

#include <memory>
#include <cmath>

HeadPointerMC::HeadPointerMC()
	: MotionCommand(), dirty(true), hold(true), tolerance(.05f), // if change default tolerance, update documentation in header
		targetReached(true), targetTimestamp(0), timeout(2000)
{
	setWeight(1);
	defaultMaxSpeed();
	takeSnapshot();
}

void HeadPointerMC::freezeMotion() {
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headTargets[i]=headCmds[i].value;
	dirty=false;
#endif
}

void HeadPointerMC::takeSnapshot() {
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headTargets[i]=headCmds[i].value=state->outputs[HeadOffset+i];
	dirty=true;
#endif
}

void HeadPointerMC::defaultMaxSpeed(float x/*=1*/) {
#ifdef TGT_HAS_HEAD
	const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::TiltOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeed[i-HeadOffset]=config->motion.max_head_tilt_speed*FrameTime*x/1000;
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::PanOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeed[i-HeadOffset]=config->motion.max_head_pan_speed*FrameTime*x/1000;
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::NodOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeed[i-HeadOffset]=config->motion.max_head_roll_speed*FrameTime*x/1000;
	n = ERS210Info::outputNames[ERS210Info::HeadOffset+ERS210Info::RollOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		maxSpeed[i-HeadOffset]=config->motion.max_head_roll_speed*FrameTime*x/1000;
#endif
}

void HeadPointerMC::setWeight(float w) {
#ifdef TGT_HAS_HEAD
	for(unsigned int x=0; x<NumHeadJoints; x++)
		headCmds[x].weight=w;
	setDirty();
#endif
}

void HeadPointerMC::setJoints(float pan, float tilt) {
#ifdef TGT_HAS_HEAD
	headTargets[PanOffset]=clipAngularRange(HeadOffset+PanOffset,pan);
	headTargets[TiltOffset]=clipAngularRange(HeadOffset+TiltOffset,tilt);
#endif
}

void HeadPointerMC::setJoints(float tilt1, float pan, float tilt2) {
#ifdef TGT_HAS_HEAD
#ifdef TGT_IS_AIBO
	headTargets[TiltOffset]=clipAngularRange(HeadOffset+TiltOffset,tilt1);
	headTargets[PanOffset]=clipAngularRange(HeadOffset+PanOffset,pan);
	headTargets[NodOffset]=clipAngularRange(HeadOffset+NodOffset,tilt2);
#else
	const char* n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::TiltOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tilt1);
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::PanOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,pan);
	n = ERS7Info::outputNames[ERS7Info::HeadOffset+ERS7Info::NodOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tilt2);
#endif
	setDirty();
#endif
}

void HeadPointerMC::setJoints(float pan, float shoulder, float elbow, float pitch) {
#ifdef TGT_HAS_HEAD
        setMaxSpeed(0, 0.2f);
        setMaxSpeed(1, 0.2f);
        setMaxSpeed(2, 0.2f);
        setMaxSpeed(3, 0.2f);
        setJointValue(0, pan);	
	setJointValue(1, shoulder);	
	setJointValue(2, elbow);	
	setJointValue(3, pitch);	
#endif
}

#include <iostream>
using namespace std;

bool HeadPointerMC::lookAtPoint(float x, float y, float z) {
#if !defined(TGT_HAS_HEAD) || !defined(TGT_HAS_CAMERA)
	return false;
#else
	const KinematicJoint * kinEff = kine->getKinematicJoint(CameraFrameOffset);
	if(kinEff==NULL) {// if that didn't work, give up
		cout<<"HeadPointerMC: Could not find CameraFrameOffset"<<endl;
		return false;
	}
	KinematicJoint * effector = kinEff->cloneBranch(); // make a local copy so we can do thread-safe IK
	std::unique_ptr<KinematicJoint> root(effector->getRoot()); // auto-free the cloned branch when leaving scope
	// kine already updated KinematicJoint positions during getKinematicJoint call, otherwise we'd need this:
	// effector->pullAncestorsQFromArray(state->outputs);
	
	// make the camera into a mobile prismatic joint
	effector->qmax = std::numeric_limits<float>::infinity();
	
	bool conv = effector->getIK().solve(IKSolver::Point(),*effector,IKSolver::Point(x,y,z));
	do {
		if(effector->outputOffset>=HeadOffset && effector->outputOffset<HeadOffset+NumHeadJoints)
			headTargets[effector->outputOffset - HeadOffset] = effector->getQ();
		effector = effector->getParent();
	} while(effector!=NULL);
	setDirty();
	
	return conv;
#endif

}

bool HeadPointerMC::lookAtPoint(const DualCoding::Point &p) {
	DualCoding::Point p2 = p;
	switch ( p2.getRefFrameType() ) {
		case DualCoding::camcentric:
			// technically we could probably do something here based on aligning camera z with ray through the specified point...
			cout << "Error:  HeadPointerMC::lookAtPoint cannot accept point " << p2 << " in camera-centric coordinates\n";
			return false;
		case DualCoding::allocentric: {
			if ( DualCoding::VRmixin::mapBuilder == NULL ) {
				cout << "Error: HeadPointerMC:lookAtPoint can't convert allocentric point " << p2
				<< " to egocentric because MapBuilder is not running\n";
				return false;
			}
			fmat::Column<3> q = DualCoding::VRmixin::mapBuilder->worldToLocalMatrix * p2.getCoords();
			p2.setCoords(q[0], q[1], q[2]);
		}
			// fall through to next case
		case DualCoding::egocentric:
		case DualCoding::unspecified:  // assume egocentric
			return lookAtPoint(p2.coordX(), p2.coordY(), p2.coordZ());
	}
	cerr << "Error: HeadPointerMC::lookAtPoint given DualCoding::Point with invalid reference frame " << p2.getRefFrameType() << endl;
	return false; // only happens if invalid reference frame was set (compiler warning if we leave this off)
}

bool HeadPointerMC::lookAtPoint(const fmat::Column<3> &p) {
  return lookAtPoint(p[0], p[1], p[2]);
}

bool HeadPointerMC::lookAtPoint(float x, float y, float z, float d) {
#if !defined(TGT_HAS_HEAD) || !defined(TGT_HAS_CAMERA)
	return false;
#else
	const KinematicJoint * kinEff = kine->getKinematicJoint(CameraFrameOffset);
	if(kinEff==NULL) {// if that didn't work, give up
		cout<<"HeadPointerMC: Could not find CameraFrameOffset"<<endl;
		return false;
	}
	KinematicJoint * effector = kinEff->cloneBranch(); // make a local copy so we can do thread-safe IK
	std::unique_ptr<KinematicJoint> root(effector->getRoot());
	// kine already updated KinematicJoint positions during getKinematicJoint call, otherwise we'd need this:
	// effector->pullAncestorsQFromArray(state->outputs);
	
	bool conv = effector->getIK().solve(IKSolver::Point(0,0,d),*effector,IKSolver::Point(x,y,z));
	do {
		if(effector->outputOffset>=HeadOffset && effector->outputOffset<HeadOffset+NumHeadJoints)
			headTargets[effector->outputOffset - HeadOffset] = effector->getQ();
		effector = effector->getParent();
	} while(effector!=NULL);
	setDirty();
	
	return conv;
#endif
}
	
bool HeadPointerMC::lookInDirection(float x, float y, float z) {
#if !defined(TGT_HAS_HEAD) || !defined(TGT_HAS_CAMERA)
	return false;
#else
	const KinematicJoint * kinEff = kine->getKinematicJoint(CameraFrameOffset);
	if(kinEff==NULL) {// if that didn't work, give up
		cout<<"HeadPointerMC: Could not find CameraFrameOffset"<<endl;
		return false;
	}
	KinematicJoint * effector = kinEff->cloneBranch(); // make a local copy so we can do thread-safe IK
	std::unique_ptr<KinematicJoint> root(effector->getRoot());
	// kine already updated KinematicJoint positions during getKinematicJoint call, otherwise we'd need this:
	// effector->pullAncestorsQFromArray(state->outputs);
	
	bool conv = effector->getIK().solve(IKSolver::Rotation(),*effector,IKSolver::Parallel(x,y,z));
	do {
		if(effector->outputOffset>=HeadOffset && effector->outputOffset<HeadOffset+NumHeadJoints)
			headTargets[effector->outputOffset - HeadOffset] = effector->getQ();
		effector = effector->getParent();
	} while(effector!=NULL);
	setDirty();
	
	return conv;
#endif
}

bool HeadPointerMC::lookAtJoint(unsigned int j) {
  const fmat::Column<3> jointPos = kine->linkToBase(j).translation();
  return lookAtPoint(jointPos);
}

int HeadPointerMC::updateOutputs() {
	int tmp=isDirty();
	if(tmp || hold) {
		dirty=false;
#ifdef TGT_HAS_HEAD
		for(unsigned int i=0; i<NumHeadJoints; i++) {
			if(maxSpeed[i]<=0) {
				headCmds[i].value=headTargets[i];
				motman->setOutput(this,i+HeadOffset,headCmds[i]);
			} else { // we may be trying to exceeded maxSpeed
				unsigned int f=0;
				while(headTargets[i]>headCmds[i].value+maxSpeed[i] && f<NumFrames) {
					headCmds[i].value+=maxSpeed[i];                                        
					motman->setOutput(this,i+HeadOffset,headCmds[i],f);
					f++;
				}
				while(headTargets[i]<headCmds[i].value-maxSpeed[i] && f<NumFrames) {
					headCmds[i].value-=maxSpeed[i];
                                        motman->setOutput(this,i+HeadOffset,headCmds[i],f);
					f++;
				}
				if(f<NumFrames) { //we reached target value, fill in rest of frames
					headCmds[i].value=headTargets[i];
					for(;f<NumFrames;f++)
						motman->setOutput(this,i+HeadOffset,headCmds[i],f);
				} else // we didn't reach target value, still dirty
					dirty=true;
			}
		}
#endif
		if(!dirty && !targetReached) {
			postEvent(EventBase(EventBase::motmanEGID, getID(),EventBase::statusETID));
			targetReached=true;
			targetTimestamp=get_time();
		}
	}
	return tmp;
}

int HeadPointerMC::isAlive() {
#ifndef TGT_HAS_HEAD
	return false;
#else
	if(dirty || !targetReached)
		return true;
	if(targetReached && (!hold || get_time()-targetTimestamp>timeout)) { //prevents a conflicted HeadPointerMC's from fighting forever
		if(get_time()-targetTimestamp>timeout && getAutoPrune())
			serr->printf("WARNING: HeadPointerMC (mcid %d) timed out - possible joint conflict or out-of-range target\n",getID());
		return false;
	}
	float maxdiff=0;
	for(unsigned int i=0; i<NumHeadJoints; i++) {
		float diff=fabsf(state->outputs[HeadOffset+i]-headTargets[i]);
		if(diff>maxdiff)
			maxdiff=diff;
	}
	return (maxdiff>tolerance);
#endif
}

void HeadPointerMC::setDirty() {
	dirty=true;
	targetReached=false;
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headCmds[i].value=motman->getOutputCmd(HeadOffset+i).value; //not state->outputs[HeadOffset+i]; - see function documentation
#endif
}

bool HeadPointerMC::ensureValidJoint(unsigned int& i) {
#ifndef TGT_HAS_HEAD
	serr->printf("ERROR: HeadPointerMC received a joint index of %d on headless target.\n",i);
#else
	if(i<NumHeadJoints)
		return true;
	if(i>=HeadOffset && i<HeadOffset+NumHeadJoints) {
		i-=HeadOffset;
		serr->printf("WARNING: HeadPointerMC received a joint index of %d (HeadOffset+%d).\n",i+HeadOffset,i);
		serr->printf("         Since all parameters are assumed to be relative to HeadOffset,\n");
		serr->printf("         you should just pass %d directly.\n",i);
		serr->printf("WARNING: Assuming you meant %d...\n",i);
		return true;
	}
	serr->printf("ERROR: HeadPointerMC received a joint index of %d (HeadOffset%+d).\n",i,i-HeadOffset);
	serr->printf("ERROR: This does not appear to be a head joint.  HeadPointerMC only controls\n");
	serr->printf("       head joints, and assumes its arguments are relative to HeadOffset\n");
#endif
	return false;
}

/*! @file
 * @brief Implements HeadPointerMC, a class for various ways to control where the head is looking
 * @author ejt (Creator)
 *
 */
