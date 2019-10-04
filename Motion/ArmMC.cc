#include "ArmMC.h"
#include "Kinematics.h"
#include "IKSolver.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "MotionManager.h"
#include "Shared/Config.h"
#include "Wireless/Socket.h"
#include "Shared/get_time.h"
#include "Events/EventBase.h"

#include <memory>
#include <cmath>
using namespace std;

ArmMC::ArmMC()
	: MotionCommand(), dirty(true), hold(true), tolerance(.05f),
		completionReported(true), targetTimestamp(0), timeout(2000),
		pulseOnPeriod(0), pulseOffPeriod(0), pulseStartTime(0), 
		desiredLoad(0), angleIncrement(0.05), idleCycles(8)
{
	// Initialize armTargets[] by calling takeSnapShot() so we
	// don't have astronomical garbage values in there, which
	// could cause the motion command to fail to complete even if
	// the joint weights are zero.
	takeSnapshot();
	setWeight(0);
	defaultMaxSpeed();
}

void ArmMC::freezeMotion() {
#ifdef TGT_HAS_ARMS
	for(unsigned int i=0; i<NumArmJoints; i++)
		armTargets[i]=armCmds[i].value;
	dirty=false;
#endif
}

void ArmMC::takeSnapshot() {
#ifdef TGT_HAS_ARMS
	for(unsigned int i=0; i<NumArmJoints; i++)
		armTargets[i]=armCmds[i].value=state->outputs[ArmOffset+i];
	dirty=true;
#endif
}

float ArmMC::armJointValue(unsigned int i)  {
#ifdef TGT_HAS_ARMS
	return armTargets[i];
#else
	serr->printf("WARNING: No Arm");
	return -1000.0f;
#endif
}

void ArmMC::defaultMaxSpeed(float x) {
#ifdef TGT_HAS_ARMS
	for(unsigned int i = 0; i<NumArmJoints; i++)
		maxSpeed[i] = 0.3f; // **hack** hard coded for now
#endif
}

void ArmMC::setWeight(float w) {
#ifdef TGT_HAS_ARMS
	for(unsigned int i=0; i<NumArmJoints; i++)
		armCmds[i].weight=w;
	setDirty();
#endif
}

void ArmMC::setWeight(int i, float w) {
#ifdef TGT_HAS_ARMS
	armCmds[i].weight=w;
	setDirty();
#endif
}

void ArmMC::setJoints(float shoulder, float elbow, float wrist){ 	 
#ifdef TGT_HAS_ARMS
	setMaxSpeed(0, 0.4f); 	 
	setMaxSpeed(1, 0.4f); 	 
	setMaxSpeed(2, 0.4f); 	 
	setJointValue(0, shoulder); 	 
	setJointValue(1, elbow); 	 
	setJointValue(2, wrist); 	 
#endif 	 
} 	 
	  	 
void ArmMC::setJoints(float shoulder, float elbow, float yaw, float pitch, float roll, float gripper) { 	 
#ifdef TGT_HAS_ARMS 
	setMaxSpeed(0, 0.4f); 	 
	setMaxSpeed(1, 0.4f); 	 
	setMaxSpeed(2, 0.4f); 	 
	setMaxSpeed(3, 0.4f); 	 
	setMaxSpeed(4, 0.4f); 	 
	setMaxSpeed(5, 0.4f); 	 
	setJointValue(0, shoulder); 	 
	setJointValue(1, elbow); 	 
	setJointValue(2, yaw); 	 
	setJointValue(3, pitch); 	 
	setJointValue(4, roll); 	 
	setJointValue(5, gripper); 	 
#endif 	 
} 	 

void ArmMC::setWrist(float pitch, float roll, float gripper) {
#ifdef TGT_HAS_ARMS
	setMaxSpeed(3, 0.4f); 	 
	setMaxSpeed(4, 0.4f); 	 
	setMaxSpeed(5, 0.4f);	 
	setJointValue(3, pitch); 	 
	setJointValue(4, roll); 	 
	setJointValue(5, gripper);
#endif
}

void ArmMC::setGripperSpeed(float x) {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  setMaxSpeed(GripperOffset-ArmOffset, x);
//Ensures the parameter angleIncrement is set
	setGraspSpeed(x/10.0);
#elif defined(TGT_IS_CALLIOPE5)
  setMaxSpeed(LeftFingerOffset-ArmOffset, x);
  setMaxSpeed(RightFingerOffset-ArmOffset, x);
#else
	std::cerr << "ArmMC::setGripperSpeed doesn't know about this robot type." << std::endl;
#endif
}

void ArmMC::setGripperPulse(unsigned int onPeriod, unsigned int offPeriod) {
	pulseOnPeriod = onPeriod;
	pulseOffPeriod = offPeriod;
	pulseStartTime = get_time();
	setDirty();
}

//--------------------New code----------------------
void ArmMC::requestGripperLoad(int newLoad = -280){
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	desiredLoad = newLoad;
	setDirty();
#else
	cout << "Method \"requestGripperLoad\" is only applicable for CALLIOPE robots" << endl;
#endif
}

int ArmMC::getGripperLoad(){
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	return int(state->pidduties[GripperOffset]*1023);
#else
	cout << "Method \"requestGripperLoad\" is only applicable for CALLIOPE robots" << endl;
  return 0;
#endif
}

void ArmMC::setGraspSpeed( float speed){
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	angleIncrement = speed;
#else
	cout << "Method \"setGraspSpeed\" is only applicable for CALLIOPE robots" << endl;
#endif
}

void ArmMC::setGraspWait(unsigned int cycles){
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	idleCycles = cycles;
#else
	cout << "Method \"setGraspWait\" is only applicable for CALLIOPE robots" << endl;
#endif
}

void ArmMC::incrementGrasp(){
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
	//This is a protected helper method to close the gripper when desiredLoad != 0
	//It will only be executed if the robot is a Calliope-type machine
	static float progressiveIncrement = 0.0;
	static int previousLoad = 0;
	static unsigned int idlePeriods = 0;
	
	int currentLoad = getGripperLoad();
	if(currentLoad < desiredLoad){	//This means our grasp is sufficiently tight, and we don't need to close any more
		return;
	}
	else{	
		//Otherwise, get the current angle of the gripper, and increment it
		float currentGripperAngle = state->outputs[GripperOffset];
		
//		if(state->sensors[GPSXOffset] != 0.0 && currentGripperAngle <= 0.45){
			//If we're in mirage, and we've closed the gripper to a certain extent, exit
			// the method to avoid making the object glitching out
//			return;
//		}
//		else if(currentGripperAngle <= 0.15){
			//If we've managed to close the gripper (almost) completely, we should exit
//			return;
//		}
		
		//This if block ensures that the gripper will get to its desired load
		if(previousLoad == currentLoad && currentLoad != 0){
				if(idlePeriods == idleCycles){
					progressiveIncrement += angleIncrement;
					idlePeriods = 0;
				}
				else { 
					idlePeriods++; 
				}
		} else {
			previousLoad = currentLoad;
			progressiveIncrement = 0;
			idlePeriods = 0;
		}
		
		setJointValue(GripperOffset-ArmOffset, currentGripperAngle - angleIncrement - progressiveIncrement);
	}
#endif
}
//--------------------------------------------------

void ArmMC::clearGripperPulse() {
	pulseOnPeriod = pulseOffPeriod = 0;
#if defined(TGT_HAS_FINGERS)
	motman->setOutput(this, LeftFingerOffset, OutputPID(DefaultPIDs[LeftFingerOffset-PIDJointOffset],1));
	motman->setOutput(this, RightFingerOffset, OutputPID(DefaultPIDs[RightFingerOffset-PIDJointOffset],1));
#elif defined(TGT_IS_MANTIS)
	;
#elif defined(TGT_HAS_GRIPPER)
	motman->setOutput(this, GripperOffset, OutputPID(DefaultPIDs[GripperOffset-PIDJointOffset],1));
#endif
}

bool ArmMC::moveOffsetToPoint(const fmat::Column<3>& offset, const fmat::Column<3>& tgt) {
#ifndef TGT_HAS_ARMS
	return false;
#else
	const KinematicJoint * kinEff=NULL;
	unsigned int effIdx = capabilities.findFrameOffset("GripperFrame");
	if(effIdx!=-1U) // does it have the "pure" reference frame?
		kinEff = kine->getKinematicJoint(effIdx); // yes, try that
	if(kinEff==NULL) // if nothing has worked yet, try the last joint
		kinEff = kine->getKinematicJoint(ArmOffset+NumArmJoints-1);
	if(kinEff==NULL) // if that didn't work, give up
		return false;
	KinematicJoint * effector = kinEff->cloneBranch(); // make a local copy so we can do thread-safe IK
	
	// do the IK
	bool conv = effector->getIK().solve(offset,*effector,IKSolver::Point(tgt));
	
	// copy results to member storage
	while(effector!=NULL) {
		if(effector->outputOffset<NumOutputs)
			setOutputCmd(effector->outputOffset, effector->getQ());
		effector = effector->getParent();
	}
	return conv;
#endif	
}

bool ArmMC::moveOffsetToPointWithOrientation(const fmat::Column<3>& offset, const fmat::Column<3>& tgt, const fmat::Quaternion& ori) {
#ifndef TGT_HAS_ARMS
	return false;
#else
	const KinematicJoint * kinEff=NULL;
	unsigned int effIdx = capabilities.findFrameOffset("GripperFrame");
	if(effIdx!=-1U) // does it have the "pure" reference frame?
		kinEff = kine->getKinematicJoint(effIdx); // yes, try that
	if(kinEff==NULL) // if nothing has worked yet, try the last joint
		kinEff = kine->getKinematicJoint(ArmOffset+NumArmJoints-1);
	if(kinEff==NULL) // if that didn't work, give up
		return false;
	KinematicJoint * effector = kinEff->cloneBranch(); // make a local copy so we can do thread-safe IK
	
	// do the IK
	bool conv = effector->getIK().solve(offset, IKSolver::Rotation(fmat::Quaternion()), *effector, IKSolver::Point(tgt), 1, IKSolver::Rotation(ori), 0);
	
	// copy results to member storage
	while(effector!=NULL) {
		if(effector->outputOffset<NumOutputs)
			setOutputCmd(effector->outputOffset, effector->getQ());
		effector = effector->getParent();
	}
	return conv;
#endif	
}

bool ArmMC::setFingerGap(float dist) {
#ifdef TGT_CALLIOPE5KP
    PostureEngine mypos;
    mypos.setOutputCmd(LeftFingerOffset, 0);
    mypos.setOutputCmd(RightFingerOffset, 0);
    fmat::Column<3> lf = mypos.getPosition(LeftFingerOffset);
    fmat::Column<3> rf = mypos.getPosition(RightFingerOffset);
    float c = (lf-rf).norm();
    fmat::Transform trans = mypos.linkToLink(LeftFingerFrameOffset, LeftFingerOffset);
    AngSignPi alpha = asin(trans(0,0));
    float r = sqrt ( trans(0,3)*trans(0,3) + trans(1,3)*trans(1,3) );
    if ( dist < 0 || dist > 2*r+c ) // assumes fingers can open full 180 degrees
      return false;
    AngSignPi q = asin((dist/2-c/2)/r) - alpha;
    setJointValue(LeftFingerOffset, -q);
    setJointValue(RightFingerOffset, q);
#endif
    return true;
}

bool ArmMC::openGripper(float percentage) {
#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
  float gripperAngle = outputRanges[GripperOffset][MinRange] +
    percentage * (outputRanges[GripperOffset][MaxRange] - outputRanges[GripperOffset][MinRange]);
  setJointValue(GripperOffset-ArmOffset, gripperAngle);
	//Once this method is called, desiredLoad is set to zero so that the updateOutputs method won't keep closing the gripper
	requestGripperLoad(0);
#elif defined(TGT_IS_CALLIOPE5)
  float gripperAngleL = outputRanges[LeftFingerOffset][MaxRange] +
    percentage * (outputRanges[LeftFingerOffset][MinRange] - outputRanges[LeftFingerOffset][MaxRange]);
  float gripperAngleR = outputRanges[RightFingerOffset][MinRange] +
    percentage * (outputRanges[RightFingerOffset][MaxRange] - outputRanges[RightFingerOffset][MinRange]);
  setJointValue(LeftFingerOffset-ArmOffset, gripperAngleL);
  setJointValue(RightFingerOffset-ArmOffset, gripperAngleR);
#endif
  return true;
}

int ArmMC::updateOutputs() {
	int prevDirty = isDirty();
	if (prevDirty || hold) {  // we have joints to command
		dirty = false;
#ifdef TGT_HAS_ARMS
		for(unsigned int i=0; i < NumArmJoints; i++) {
			if(maxSpeed[i]<=0) {
				armCmds[i].value=armTargets[i];
				motman->setOutput(this,i+ArmOffset,armCmds[i]);
			} else { // we may be trying to exceeded maxSpeed
				unsigned int f=0;
				while(armTargets[i]>armCmds[i].value+maxSpeed[i] && f<NumFrames) {
					armCmds[i].value+=maxSpeed[i];
					motman->setOutput(this,i+ArmOffset,armCmds[i],f);
					f++;
				}
				while(armTargets[i]<armCmds[i].value-maxSpeed[i] && f<NumFrames) {
					armCmds[i].value-=maxSpeed[i];
					motman->setOutput(this,i+ArmOffset,armCmds[i],f);
					f++;
				}
				if(f<NumFrames) { //we reached target value, fill in rest of frames
					armCmds[i].value=armTargets[i];
					for(;f<NumFrames;f++)
						motman->setOutput(this,i+ArmOffset,armCmds[i],f);
				} else // we didn't reach target value, still dirty
					dirty=true;
			}
		}
	#if defined(TGT_IS_CALLIOPE2) || defined(TGT_IS_CALLIOPE3)
		if(desiredLoad != 0) incrementGrasp();
	#endif
#endif
		if(!dirty && !completionReported) {
			postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
			completionReported=true;
			targetTimestamp=get_time();
		}
	}

//------------------------------------------------------From here
#if false
	// pulse gripper if requested
	if ( pulseOnPeriod > 0 ) {
		dirty = true;
		unsigned int t = get_time();
		if ( t >= pulseStartTime + pulseOnPeriod + pulseOffPeriod ) { // time to start a new cycle
			pulseStartTime = t;
		}
		// turn power on or off depending on where we are in the cycle
#if defined(TGT_HAS_FINGERS) || defined(TGT_HAS_GRIPPER)
		if (t >= pulseStartTime + pulseOnPeriod) {
			// we're in the "off" part of the cycle
#ifdef TGT_HAS_FINGERS
			motman->setOutput(this, LeftFingerOffset,  OutputPID(0,0,0,1));
			motman->setOutput(this, RightFingerOffset, OutputPID(0,0,0,1));
#else
			motman->setOutput(this, GripperOffset, OutputPID(0,0,0,1));
#endif
		} else {  // we're in the "on" part of the cycle
#ifdef TGT_HAS_FINGERS
			motman->setOutput(this, LeftFingerOffset, OutputPID(DefaultPIDs[LeftFingerOffset-PIDJointOffset],1));
			motman->setOutput(this, RightFingerOffset, OutputPID(DefaultPIDs[RightFingerOffset-PIDJointOffset],1));
#else
			motman->setOutput(this, GripperOffset, OutputPID(DefaultPIDs[GripperOffset-PIDJointOffset],1));
#endif
		}
#endif
	}//End PulseOnPeriod IF STATEMENT
#endif
//------------------------------------------------------To here is becoming obsolete
	return prevDirty;
}

int ArmMC::isAlive() {
#ifdef TGT_HAS_ARMS
	if(dirty || !completionReported)
		return true;

	if(completionReported && (!hold || get_time()-targetTimestamp>timeout)) { //prevents a conflicted ArmPointerMC's from fighting forever
		if(get_time()-targetTimestamp>timeout && getAutoPrune())
			serr->printf("WARNING: ArmPointerMC (mcid %d) timed out - possible joint conflict or out-of-range target\n",getID());
		return false;
	}
	float maxdiff=0;
	for(unsigned int i=0; i<NumArmJoints; i++) {
		float diff=fabsf(state->outputs[ArmOffset+i]-armTargets[i]);
		if(diff>maxdiff)
			maxdiff=diff;
	}
	return (maxdiff>tolerance);
#else
	return false;
#endif
}

void ArmMC::setDirty() {
	dirty=true;
	completionReported=false;
#ifdef TGT_HAS_ARMS
	for(unsigned int i=0; i<NumArmJoints; i++)
		armCmds[i].value=motman->getOutputCmd(ArmOffset+i).value; //not state->outputs[armOffset+i]; - see function documentation
#endif
}

bool ArmMC::ensureValidJoint(unsigned int& i) {
#ifdef TGT_HAS_ARMS
	if(i<NumArmJoints)
		return true;
	if(i>=ArmOffset && i<ArmOffset+NumArmJoints) {
		i-=ArmOffset;
		serr->printf("WARNING: ArmMC received a joint index of %d (ArmOffset+%d).\n",i+ArmOffset,i);
		serr->printf("         Since all parameters are assumed to be relative to ArmOffset,\n");
		serr->printf("         you should just pass %d directly.\n",i);
		serr->printf("WARNING: Assuming you meant %d...\n",i);
		return true;
	}
	serr->printf("ERROR: ArmMC received a joint index of %d (ArmOffset%+d).\n",i,i-ArmOffset);
	serr->printf("ERROR: This does not appear to be a arm joint.  ArmPointerMC only controls\n");
	serr->printf("       arm joints, and assumes its arguments are relative to ArmOffset\n");
#endif
	return false;
}
