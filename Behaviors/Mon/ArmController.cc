#include "Shared/RobotInfo.h"
#include "Behaviors/Controller.h"

#ifndef TGT_HAS_ARMS
class ArmController : public BehaviorBase {
  virtual void doStart() {
    // This will popup a message in the ControllerGUI, if one is connected
    std::vector<std::string> errmsg;
    errmsg.push_back("The selected robot doesn't have an arm...");
    Controller::loadGUI("org.tekkotsu.mon.ControllerErr","",0,errmsg);
    stop();
  }
};
#else

#include "ArmController.h"
#include "Motion/MMAccessor.h"
#include "Motion/ArmMC.h"
#include "Shared/RobotInfo.h"
#include "Motion/IKSolver.h"
#include "Motion/PIDMC.h"

ArmController* ArmController::theOne = NULL;

void ArmController::doStart() {
  // Set up ArmMC, PIDMC and XLargeMotionSequenceMC for controlling and relaxing the arm
  SharedObject<ArmMC> arm;
  arm->setHold(false);
  armMCID = motman->addPersistentMotion(arm);
  SharedObject<PIDMC> pid;
  pidMCID = motman->addPersistentMotion(pid,MotionManager::kHighPriority);
  SharedObject<XLargeMotionSequenceMC> seq;
  seq->setHold(false);
  seqMCID = motman->addPersistentMotion(seq,MotionManager::kHighPriority);
	
  // Identify the initial sequence of pitch and yaw joints using alpha, theta, and jointType
	for (unsigned int i=0; i<NumArmJoints; i++)
		KJjoints[i]=NULL;
  gripperFrameKJ->getRoot()->buildChildMap(KJjoints, ArmOffset, NumArmJoints);
	
  int counter = 0;
  float alphaCum = 0;
  for (unsigned int i = 0; i < NumArmJoints; i++) {
    if (KJjoints[i] == NULL) break;
    if ( counter > 0 // don't worry about any rotation from base frame to first arm joint
         && KJjoints[i]->theta != 0 ) // we're leaving the domain of pure yaw and pitch joints: punt
      break;
		
    alphaCum += KJjoints[i]->alpha;
    if ( std::abs(alphaCum) < 1e-5  && KJjoints[i]->jointType.get() == "revolute" ) {
      numYawJoints++;
      armConfig[counter++] = 'H';
    }
    else if ( std::abs(alphaCum) > 0.99*(M_PI_2) && KJjoints[i]->jointType.get() == "revolute" ) {
      numPitchJoints++;
      armConfig[counter++] = 'V';
    }
  }
  linksToDisplay = numPitchJoints + numYawJoints;
	
#ifdef TGT_LYNXARM6
  horScale = 20;
#endif

  /* Find the dimensions of the arm.  Must work from the gripper
     backward because the kinematic chain may branch going forward (as in
     Calliope2 with its four-bar linkage). */
  fmat::Column<3> armDims;
  fmat::Transform gripperToArm;
  const KinematicJoint* armJ = kine->getKinematicJoint(ArmOffset);
  const KinematicJoint* tmpJ = kine->getKinematicJoint(GripperFrameOffset);
  if ( armJ == NULL || tmpJ == NULL )
    std::cout << "Error in Arm Controller: can't find arm or gripper frame!\n";
  else
    while (tmpJ != armJ && tmpJ != NULL) {
      tmpJ = tmpJ->getParent();
      gripperToArm *= tmpJ->getTo().rigidInverse();
    }
  gripperToArm = gripperToArm.rigidInverse();
  armDims = gripperToArm.translation();
  horScale = verScale = armDims.norm();
  horScale /= 0.85;  // why are we scaling by 1 / 0.85 ?
	
  // determine transformation from first horizontal and vertical joint to base, for IK
  int horIndex = 0, verIndex = 0;
  fmat::Transform horT, verT;
  if (numYawJoints != 0) {
    while (armConfig[horIndex] != 'H')
      horIndex++;
    while (horIndex)
      horT *= KJjoints[horIndex--]->getTo();
  }
  if (numPitchJoints != 0) {
    while (armConfig[verIndex] != 'V')
      verIndex++;
    while (verIndex)
      verT *= KJjoints[verIndex--]->getTo();
  }
  horToBase = horT.translation();
  verToBase = verT.translation();
	
  // Turn on wireless
  theLastOne=theOne;
  theOne=this;
  cmdsock=wireless->socket(Socket::SOCK_STREAM, 2048, 2048);
  wireless->setReceiver(cmdsock->sock, mechacmd_callback);
  wireless->setDaemon(cmdsock,true);	
  wireless->listen(cmdsock->sock, config->main.armControl_port);	
  // Open the ArmGUI on the desktop	
  Controller::loadGUI("org.tekkotsu.mon.ArmGUI","ArmGUI",config->main.armControl_port);
}

void ArmController::doStop() {
  Controller::closeGUI("ArmGUI");
  erouter->removeListener(this);
  wireless->setDaemon(cmdsock,false);
  wireless->close(cmdsock);
  theOne=theLastOne;
  motman->removeMotion(armMCID);
  motman->removeMotion(pidMCID);
  motman->removeMotion(seqMCID);
}

void ArmController::doEvent() {
  if (event->getGeneratorID()==EventBase::timerEGID) {
    for (unsigned int joint = 0; joint < NumArmJoints; joint++) {
      float jointAngle = state->outputs[ArmOffset + joint]; // When using mirage
      // % = (Max - Val) / (Max - Min)
      float normJointVal = (outputRanges[ArmOffset + joint][MaxRange] - jointAngle) /
        (outputRanges[ArmOffset + joint][MaxRange] - outputRanges[ArmOffset + joint][MinRange]);
      cmdsock->printf("Value %d %d %f\n", -1, joint, normJointVal);
    }
		
    computeCoords();
    sendCoords();

    // Reset the red point when the joints are reset to their original values
    if (displayMode == pitchAndYaw || displayMode == yawOnly)
      cmdsock->printf("RedValH %f %f\n", -yawCoords[numYawJoints - 1][0] / horScale, yawCoords[numYawJoints -1][1] / horScale);
    if (displayMode == pitchAndYaw || displayMode == pitchOnly)
      cmdsock->printf("RedValV %f %f\n",  pitchCoords[numPitchJoints - 1][0] / verScale, pitchCoords[numPitchJoints - 1][1] / verScale);
    
  }
}

// The command packet reassembly mechanism
int ArmController::mechacmd_callback(char *buf, int bytes) {
  static char cb_buf[19];
  static int cb_buf_filled;
	
  // If there's an incomplete command in the command buffer, fill
  // up as much of the command buffer as we can and then execute it
  // if possible
  if (cb_buf_filled) {
    while ((cb_buf_filled < 19) && bytes) {
      cb_buf[cb_buf_filled++] = *buf++;	// copy incoming buffer byte
      --bytes;				// decrement remaining byte ct.
    }
    // did we fill it? if so, execute! and mark buffer empty.
    if (cb_buf_filled == 9 && ((unsigned char*)cb_buf)[0] != 'x') {
      if (ArmController::theOne) ArmController::theOne->runCommand((unsigned char*) cb_buf);
      cb_buf_filled = 0;
    }
		
    if (cb_buf_filled == 19) {
      if (ArmController::theOne) ArmController::theOne->runCommand((unsigned char*) cb_buf);
      cb_buf_filled = 0;
    }
  }
	
  // now execute all complete bytes in the incoming buffer
  while (bytes >= 9) {
    if (((unsigned char*)buf)[0] != 'x') {
      if (ArmController::theOne) ArmController::theOne->runCommand((unsigned char *) buf);
      bytes -= 9;
      buf += 9;
    } else if (((unsigned char*)buf)[0] == 'x') {
      if (ArmController::theOne) ArmController::theOne->runCommand((unsigned char *) buf);
      bytes -= 19;
      buf += 19;
    }
  }
	
  // finally, store all remaining bytes in the command buffer
  while (bytes) {
    cb_buf[cb_buf_filled++] = *buf++;
    --bytes;
  }
	
  return 0;
}

void ArmController::runCommand(unsigned char *command) {
  // Extract the command parameter
  float param;
  float param2;
  float param3;
  int cmdno;
  unsigned char *paramp = (unsigned char *) &param;
  unsigned char *paramp2 = (unsigned char *) &param2;
  unsigned char *paramp3 = (unsigned char *) &param3;
  unsigned char *cmdnop = (unsigned char *) &cmdno;
	
#if defined(BYTE_ORDER) && BYTE_ORDER==BIG_ENDIAN
  paramp[0] = command[4];
  paramp[1] = command[3];
  paramp[2] = command[2];
  paramp[3] = command[1];
	
  if (command[0] == 'x') {
    paramp2[0] = command[9];
    paramp2[1] = command[8];
    paramp2[2] = command[7];
    paramp2[3] = command[6];
		
    paramp3[0] = command[14];
    paramp3[1] = command[13];
    paramp3[2] = command[12];
    paramp3[3] = command[11];
		
    cmdnop[0] = command[18];
    cmdnop[1] = command[17];
    cmdnop[2] = command[16];
    cmdnop[3] = command[15];
  }
  else {
    cmdnop[0] = command[8];
    cmdnop[1] = command[7];
    cmdnop[2] = command[6];
    cmdnop[3] = command[5];
  }
#else
  paramp[0] = command[1];
  paramp[1] = command[2];
  paramp[2] = command[3];
  paramp[3] = command[4];
	
  if (command[0] == 'x') {
    paramp2[0] = command[6];
    paramp2[1] = command[7];
    paramp2[2] = command[8];
    paramp2[3] = command[9];
		
    paramp3[0] = command[11];
    paramp3[1] = command[12];
    paramp3[2] = command[13];
    paramp3[3] = command[14];
		
    cmdnop[0] = command[15];
    cmdnop[1] = command[16];
    cmdnop[2] = command[17];
    cmdnop[3] = command[18];
  }
  else {
    cmdnop[0] = command[5];
    cmdnop[1] = command[6];
    cmdnop[2] = command[7];
    cmdnop[3] = command[8];
  }
	
#endif // byte order
	
  // Find out what type of command this is
  std::cout << "ArmController command: "  << int(command[0]) << " = '" << command[0] << "'" << std::endl;
  switch(command[0]) {
  case cmdConnect:	// Connect to ArmGUI -> z command
    connect();
    break;	
  case cmdPoint:		// Point Picker plane #1 & plane#2-> x command
    pointPicked(param, param2, param3, cmdno);
    break;
  case cmdGripper:	// Gripper -> w command
    gripper(param, cmdno);
    break;		
  case cmdSpeed:		// Speed parameter -> y command
    speed = param;
    break;
  case cmdRelax:		// Relax Button -> v command
    relax();
    break;
  case cmdUnrelax:	// Unrelax Button -> u command
    unrelax();
    break;
  case cmdOrientation:	// Orientation -> o command
    orientationIndex = (int)param;
    sendReachablePoints(displayMode);
    break;
  default:
    setJoint(command[0], param);
  }
}

// Setup function
void ArmController::connect() {
  // Code that validates the plane to be used in the interface
  if (numYawJoints > 0 && numPitchJoints > 0)
    displayMode = pitchAndYaw; // create both planes
  else if (numYawJoints > 0 && numPitchJoints == 0)
    displayMode = yawOnly; // create just the horizontal plane
  else
    displayMode = pitchOnly; // create just the vertical plane
	
  /*
   * The following option allows us to tell the ControllerGUI
   * what options we want to enable for the grip orientation.
   * orientationConfig is an integer variable that is divisible
   * by some or all of three prime numbers:
   * 
   * 2 - side grip
   * 3 - overhead grip
   * 5 - unconstrained grip
   * 
   * As more possible grip orientations are created, more
   * prime numbers can be added to this configuration,
   * but modifications will also need to be made java-side.
   * For now, supposing you want only side and overhead,
   * you would set orientationConfig to 2*3 = 6.
   */
  int orientationChoices;
#ifdef TGT_IS_CALLIOPE5
  orientationChoices = 30; // side, overhead, unconstrained
#else
  orientationChoices = 10; // side, unconstrained
#endif
	
  // Send out the number of joints on the arm, display mode, and orientation configuration
  cmdsock->printf("NOJ %d %d %d\n", NumArmJoints, displayMode, orientationChoices);
	
  // Send the joint config
  for (unsigned int i = 0; i < NumArmJoints; i++)
    cmdsock->printf("Config %u %c\n", i, armConfig[i]);
	
  // Send info about how many of each type of joint there are.
  cmdsock->printf("JointTypes %d %d %d\n", numYawJoints, numPitchJoints, FingerJointsPerArm);
	
  // Send Names, Max Ouput Ranges, Min output Ranges, Current Value
  float jointVal, normJointVal;
	
  for (unsigned int joint = 0; joint < NumArmJoints; joint++) {
    jointVal = state->outputs[ArmOffset + joint];
    normJointVal = (outputRanges[ArmOffset + joint][MaxRange] - jointVal) /
      (outputRanges[ArmOffset + joint][MaxRange] - outputRanges[ArmOffset + joint][MinRange]);
    cmdsock->printf("JointParam %d %s %f %f %f\n", joint,
		    outputNames[ArmOffset + joint], 
		    outputRanges[ArmOffset + joint][MaxRange],
		    outputRanges[ArmOffset + joint][MinRange],
		    normJointVal);
  }
	
  // Compute / send
  computeCoords();
  sendCoords();
  if (displayMode == pitchAndYaw || displayMode == yawOnly)
    cmdsock->printf("RedValH %f %f\n", -yawCoords[numYawJoints - 1][0] / horScale,
                    yawCoords[numYawJoints -1][1] / horScale);
  if (displayMode == pitchAndYaw || displayMode == pitchOnly)
    cmdsock->printf("RedValV %f %f\n",  pitchCoords[numPitchJoints - 1][0] / verScale,
                    pitchCoords[numPitchJoints - 1][1] / verScale);
  sendReachablePoints(displayMode);
}

// Command to the Point Picker
void ArmController::pointPicked(float param, float param2, float param3, int cmdno) {
#ifdef TGT_TENTACLE
  //== Setup (x,y,z) so they are relative to 'ARM:5' ==//
#else
  fmat::Column<3> target;
  if (param3 == 1.0) { // horizontal plane
    theta = std::atan2(-param,param2);
    target[0] = param2 * horScale + horToBase[0];
    target[1] = -param * horScale + horToBase[1];
    target[2] = z * verScale;
  }
  else if (param3 == 2.0) { // vertical plane
    z = param2;
    target[0] = (param * verScale + verToBase[0]) * std::cos(theta);
    target[1] = target[0] * std::tan(theta);
    target[2] = param2 * verScale + verToBase[2];
  }
	
  const KinematicJoint* tmpKJ = KJjoints[0];
  // transform to base
  while (tmpKJ->getParent() != NULL) {
    target += tmpKJ->getTo().translation();
    tmpKJ = tmpKJ->getParent();
  }
	
  int oriPri = orientationIndex == 2 ? 0 : 1;
	
  // construct motion sequence
  MMAccessor<XLargeMotionSequenceMC> seq_acc(seqMCID);
	
  // pause & clear current sequence
  seq_acc->pause();
  seq_acc->clear();
	
  // reset to current pose, to prevent jumpiness
  seq_acc->setTime(1);
  PostureEngine curpos(state);
  seq_acc->getPose(curpos);
	
  // calculate difference between current pose and target pose
  fmat::Column<3> from = kine->getPosition(GripperFrameOffset);
  fmat::Column<3> diff = target - from;
	
  // number of frames
  const int numFrames = 10;
	
  // time to get to the point (ms)
  const int time = 8 * diff.norm() / speed;
	
  // time step
  const int timeStep = time / (numFrames+3);
  seq_acc->advanceTime(timeStep*3);
	
  // add subsequent poses
  for (int i = 1; i <= numFrames; i++) {
    fmat::Column<3> step = from + (diff * (float)i / (float)numFrames);
		
    gripperFrameKJ->getIK().solve(fmat::ZERO3,
				  IKSolver::Rotation(),
				  *gripperFrameKJ,
				  IKSolver::Point(step), 1-oriPri,
				  IKSolver::Rotation(orientation[orientationIndex]), oriPri);
		
    for (unsigned int joint = 0; joint < linksToDisplay; joint++) {
      // angle, mapped from 0 - 1, based on joint limits
      float normJointVal;
			
      // set the new angles
      float q = KJjoints[joint]->getQ();
			
      // we can't be outside the limits
      if (q > outputRanges[ArmOffset + joint][MaxRange])
        normJointVal = 0;
      else if (q < outputRanges[ArmOffset + joint][MinRange])
        normJointVal = 1;
			
      // if max and min joint limit are the same, take current world state
      else if (outputRanges[ArmOffset + joint][MaxRange] == outputRanges[ArmOffset + joint][MinRange])
        normJointVal = 0.5f;
			
      // this is the normal case
      else
	// % = (Max - Val) / (Max - Min)
        normJointVal = (outputRanges[ArmOffset + joint][MaxRange] - q) /
          (outputRanges[ArmOffset + joint][MaxRange] - outputRanges[ArmOffset + joint][MinRange]);
			
      seq_acc->setOutputCmd(ArmOffset+joint, q);
			
      if (i == numFrames)
        cmdsock->printf("PP %d %d %f\n", cmdno++, joint, normJointVal);
    }
		
    seq_acc->advanceTime(timeStep);
  }
	
  seq_acc->play();
	
  computeCoords();
  sendCoords();
  // Reset the red point in the other plane (the one the user did not just click on)
  if (param3 == 1.0 && displayMode == pitchAndYaw)
    cmdsock->printf("RedValV %f %f\n",  pitchCoords[numPitchJoints - 1][0] / verScale, pitchCoords[numPitchJoints - 1][1] / verScale);
  else if (param3 == 2.0 && displayMode == pitchAndYaw)
    cmdsock->printf("RedValH %f %f\n", -yawCoords[numYawJoints - 1][0] / horScale, yawCoords[numYawJoints -1][1] / horScale);

  // Allow 2 seconds between each update
  if (reachablePointsDelay.Age().Value() > 0.5) {
    sendReachablePoints(param3 == 1.0f ? pitchOnly : yawOnly);
    reachablePointsDelay.Set();
  }
	
#endif
}

void ArmController::gripper(float param, int cmdno) {
  // Moving the ArmGUI's gripper slider generates a w command, which
  // takes us here.  We decide which joints to move (there may be
  // multiple fingers) and send one or two Gripper commands back to
  // the ArmGUI. It responds to these Gripper commands by moving the
  // individual finger sliders, which in turn send individual joint
  // movement commands back to the ArmController.  These commands have
  // an offset of 30 to mark them as gripper-related so we don't try
  // to set the red and green dots.
#ifdef TGT_HAS_GRIPPER
  float gripperQ = (1.0f - param) / 2.0f;
#  ifdef TGT_HAS_FINGERS
  float right = gripperQ;
  float left = 1.0f - gripperQ;
 	
  cmdsock->printf("Gripper %d %d %f\n", cmdno, RightFingerOffset - ArmOffset, right);
  cmdsock->printf("Gripper %d %d %f\n", cmdno, LeftFingerOffset - ArmOffset, left);
#  else
  cmdsock->printf("Gripper %d %d %f\n", cmdno, GripperOffset - ArmOffset, gripperQ);
#  endif
#endif
}

void ArmController::relax() {
  MMAccessor<PIDMC> relaxer(pidMCID);
  for (unsigned int joint = 0; joint < NumArmJoints; joint++)
    relaxer->setJointPowerLevel(ArmOffset + joint, 0);
  processEvent(EventBase(EventBase::timerEGID,1,EventBase::statusETID,0));
  erouter->addTimer(this,0,500);
}

void ArmController::unrelax() {
  erouter->removeTimer(this);
  MMAccessor<PIDMC> unrelaxer(pidMCID);
  for (unsigned int joint = 0; joint < NumArmJoints; joint++) 
    unrelaxer->setJointPowerLevel(ArmOffset + joint, 1.0);
}

void ArmController::setJoint(unsigned int joint, float angle) {
  unsigned int jointNo = joint < 30 ? joint : (joint - 30);
  if (jointNo >= NumArmJoints) {
    std::cerr << "Illegal command to ArmController " << joint << " " << jointNo << " "  << NumArmJoints << std::endl;
    return;
  }
	
  // enclose the motion sequence so that it gets checked back in before we get to the ArmMC
  {
    // if there's a motion sequence going on, stop it.
    MMAccessor<XLargeMotionSequenceMC> seq_acc(seqMCID);
		
    if (seq_acc->isPlaying()) {
      seq_acc->pause();
      seq_acc->clear();
			
      // set joints to reflect WorldState
      for (unsigned int i = 0; KJjoints[i] != NULL; i++)
	KJjoints[i]->setQ(kine->getKinematicJoint(ArmOffset + i)->getQ());
    }
  }
	
  MMAccessor<ArmMC> arm(armMCID);
  arm->setMaxSpeed(jointNo, speed);
  // Val = %*Min + (1 - %)Max
  float jointVal = angle * outputRanges[ArmOffset + jointNo][MinRange] + (1.0f - angle) * outputRanges[ArmOffset + jointNo][MaxRange];
  arm->setJointValue(jointNo, jointVal); // change the value of the arm physically
	
  if (KJjoints[jointNo] != NULL)
    KJjoints[jointNo]->setQ(jointVal);
	
  computeCoords();
  sendCoords();
	
  if (joint < NumArmJoints) {
    if (displayMode == pitchAndYaw || displayMode == yawOnly)
      cmdsock->printf("RedValH %f %f\n", -yawCoords[numYawJoints - 1][0] / horScale, yawCoords[numYawJoints -1][1] / horScale);
    if (displayMode == pitchAndYaw || displayMode == pitchOnly)
      cmdsock->printf("RedValV %f %f\n",  pitchCoords[numPitchJoints - 1][0] / verScale, pitchCoords[numPitchJoints - 1][1] / verScale);
  }
}

// Validate this function with the projection of joint
void ArmController::computeCoords() {
  // Works for the Calliope, Chiara, Chiara2 and Handeye and almost for the Lynxarm. 
  if ( displayMode != pitchAndYaw ) return;
  fmat::Column<3> baseHorizontal;
  fmat::Column<3> baseVertical;
	
  // Looking for the first horizontal joint 
  for (unsigned int i = 0; i < linksToDisplay; i++) {
    if (armConfig[i] == 'H') {
      baseHorizontal = KJjoints[i]->getWorldPosition();
      break;
    }
  }
  // Looking for the first vertical joint 
  for (unsigned int i = 0; i < linksToDisplay; i++) {
    if (armConfig[i] == 'V') {
      baseVertical = KJjoints[i]->getWorldPosition();
      break;
    }
  }
	
  unsigned int yawCount = 0, pitchCount = 0;
	
  // Calculate the coordinates without projections
  for (unsigned int links = 0; links < linksToDisplay; links++) {
    if (armConfig[links] == 'H') {
      const KinematicJoint* relPosJoint;
      if (yawCount == numYawJoints - 1)
        relPosJoint = gripperFrameKJ;
      else
        relPosJoint = KJjoints[links+1];
      fmat::Column<3> relPos = relPosJoint->getWorldPosition() - baseHorizontal;
      yawCoords[yawCount][0] = relPos[1]; // y-axis 
      yawCoords[yawCount][1] = relPos[0]; // x-axis
      yawCount++;
    }
    else if (armConfig[links] == 'V') {
      const KinematicJoint* relPosJoint;
      if (pitchCount == numPitchJoints - 1)
        relPosJoint = gripperFrameKJ;
      else
        relPosJoint = KJjoints[links+1];
      fmat::Column<3> relPos = relPosJoint->getWorldPosition() - baseVertical;
      pitchCoords[pitchCount][0] = fmat::SubVector<2>(relPos).norm() *
        (yawCoords[yawCount-1][1] * relPos[0] > 0 ? 1 : -1); // x-axis
      pitchCoords[pitchCount][1] = relPos[2]; // z-axis
      pitchCount++;
    }
  }
}

void ArmController::sendCoords() {
  // Send out the coordinates of the links to planes 
  if (displayMode == pitchAndYaw || displayMode == yawOnly) {
    for (unsigned int links = 0; links < numYawJoints; links++)
      cmdsock->printf("CoordH %d %f %f\n", links, -yawCoords[links][0]/horScale, yawCoords[links][1]/horScale);
    cmdsock->printf("GreenValH %f %f\n", -yawCoords[numYawJoints - 1][0] / horScale, yawCoords[numYawJoints - 1][1] / horScale);
  }
  if (displayMode == pitchAndYaw || displayMode == pitchOnly) {
    for (unsigned int links = 0; links < numPitchJoints; links++)
      cmdsock->printf("CoordV %d %f %f\n", links, pitchCoords[links][0] / verScale,
                      pitchCoords[links][1] / verScale);
    cmdsock->printf("GreenValV %f %f\n", pitchCoords[numPitchJoints - 1][0] / verScale,
                    pitchCoords[numPitchJoints - 1][1] / verScale);
  }
}

void ArmController::sendReachablePoints(DisplayMode_t d) {
  //float delta = 0.025f;
  float delta = 0.1f;
	
  if (d == yawOnly || d == pitchAndYaw) {
    // clear the points first
    cmdsock->printf("ClearPtsH\n");
		
    // Send spots of success
    for (float x = -1; x <= 1; x += delta) {
      for (float y = 0; y <= 1; y += delta) {
	fmat::Column<3> target;
	target[0] = y * horScale + horToBase[0];
	target[1] = -x * horScale + horToBase[1];
	target[2] = z * verScale;
				
	const KinematicJoint* tmpKJ = KJjoints[0];
	// transform to base
	while (tmpKJ->getParent() != NULL) {
	  target += tmpKJ->getTo().translation();
	  tmpKJ = tmpKJ->getParent();
	}
				
	int oriPri = orientationIndex == 2 ? 0 : 1;
	// perform IK
	bool success = successJ->getIK().solve(fmat::Column<3>(),
					       IKSolver::Rotation(),
					       *successJ,
					       IKSolver::Point(target), 1-oriPri,
					       IKSolver::Rotation(orientation[orientationIndex]), oriPri);
	if (success)
	  cmdsock->printf("SuccessH %f %f\n", x, y);
      }
    }
    cmdsock->printf("RepaintH\n");

  }
  if (d == pitchOnly || d == pitchAndYaw) {
    // clear the points first
    cmdsock->printf("ClearPtsV\n");
		
    // Send spots of success
    for (float x = 0; x <= 1; x += delta) {
      for (float y = -1; y <= 1; y += delta) {
	fmat::Column<3> target;
	target[0] = x * verScale + verToBase[0] * std::cos(theta);
	target[1] = target[0] * std::tan(theta);
	target[2] = y * verScale + verToBase[2];
				
	const KinematicJoint* tmpKJ = KJjoints[0];
	// transform to base
	while (tmpKJ->getParent() != NULL) {
	  target += tmpKJ->getTo().translation();
	  tmpKJ = tmpKJ->getParent();
	}
				
	int oriPri = orientationIndex == 2 ? 0 : 1;
	// perform IK
	bool success = successJ->getIK().solve(fmat::Column<3>(),
					       IKSolver::Rotation(),
					       *successJ,
					       IKSolver::Point(target), 1-oriPri,
					       IKSolver::Rotation(orientation[orientationIndex]), oriPri);
	if (success)
	  cmdsock->printf("SuccessV %f %f\n", x / std::cos(theta), y);
      }
    }
    cmdsock->printf("RepaintV\n");
  }
}

#endif // TGT_HAS_ARMS

REGISTER_BEHAVIOR_MENU_OPT(ArmController,"TekkotsuMon",BEH_NONEXCLUSIVE);

/*! @file
 * @brief Implements ArmController, listens to control commands coming in from the command port for remotely controlling the arm
 * @author tss (Creator)
 */
