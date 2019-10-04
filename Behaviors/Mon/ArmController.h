//-*-c++-*-
#ifndef INCLUDED_ArmController_h_
#define INCLUDED_ArmController_h_

#include <iostream>
#include "Wireless/Wireless.h"
#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionManager.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Shared/Config.h"
#include "Motion/PIDMC.h"
#include "Motion/MotionSequenceMC.h"
#include "IPC/SharedObject.h"
#include "Events/EventRouter.h"
#include "Motion/KinematicJoint.h"
#include "Shared/TimeET.h"

//! Listens to control commands coming in from the command port for remotely controlling the arm.
/*! The communication protocol is a very simple binary format, shared
 *  with WalkControllerBehavior.  Each command is sent as a 5-byte
 *  group.  The first byte is a command selector, and the following 4
 *  bytes are a floating point argument:
 *
 *  - <@c char: command indicator>
 *  - <@c float: value>
 *  
 *  The valid values for command indicator are given by #CMD_shoulder,
 *  #CMD_elbow, or #CMD_wrist ('s', 'e', or 'w' respectively).
 */  
class ArmController : public BehaviorBase {
	
public:	
	//! Points to the one ArmController object that the input command stream is talking to.
	/*! A kludge. Dunno how you're going to make sure you're not using this uninitialized. */
	static ArmController * theOne;
	static int mechacmd_callback(char *buf, int bytes); //!< called by wireless when there's new data
	
protected:
	//! The ArmMC, for moving individual joints
	MotionManager::MC_ID armMCID;
	
	//! The PIDMC
	MotionManager::MC_ID pidMCID;
	
	//! Used to run motion sequences for smooth paths to points picked in the ArmGUI
	MotionManager::MC_ID seqMCID;
	
	//! Kinematic Joint for the Gripper Frame
	KinematicJoint* gripperFrameKJ;
	
	//! To test for valid points, we use a separate kinematic chain
	KinematicJoint* successJ;
	
	//! All of the joints in the arm
	KinematicJoint* KJjoints[NumArmJoints];
	
	//! Horizontal scaling factor for the Point Picker
	float horScale;
	
	//! Vertical scaling factor for the Point Picker
	float verScale;
	
	//! Number of interpolation steps to take when moving from one point to the next
	unsigned int numSteps;
	
	//! Interpolation steps: every time we receive a joint update, we fire another.
	std::queue<fmat::Column<3> > steps;
	
	//! Controls whether we have one or two Point Pickers, and which kind.
	enum DisplayMode_t {
		pitchAndYaw = 0,
		yawOnly,
		pitchOnly
	} displayMode;
	
	//! @name Command Bytes        
	static const char cmdUnrelax     = 'u';
	static const char cmdRelax       = 'v';
	static const char cmdGripper     = 'w';
	static const char cmdPoint       = 'x';
	static const char cmdSpeed       = 'y';
	static const char cmdConnect     = 'z';
	static const char cmdOrientation = 'o';
	
	//! Angle between base horizontal joint and gripper, as from a bird's eye view
	/*! To be kept constant between point picks. */
	float theta;
	
	//! Height of point picked
	/*! To be kept constant between point picks. */
	float z;
	
	//! First horizontal joint location, relative to base
	fmat::Column<3> horToBase;
	
	//! First vertical joint location, relative to base
	fmat::Column<3> verToBase;
	
	//! Maximum speed for every joint
	float speed;
	
	//! Target arm orientations for IKSolver
	fmat::Quaternion orientation[2];
	
	//! Currently selected orientation (0 - side, 1 - overhead)
	int orientationIndex;
	
	//! Number of yaw joints
	unsigned int numYawJoints;
	
	//! Number of pitch joints
	unsigned int numPitchJoints;
	
	//! Total number of links to display (@a numYawJoints + @a numPitchJoints)
	unsigned int linksToDisplay;
	
	//! Delay between sending reachable points
	TimeET reachablePointsDelay;
	
	//! Whether each joint is horizontal or vertical
	char armConfig[NumArmJoints];
	
	//! Horizontal joint coordinates
	float yawCoords[NumArmJoints][2];
	
	//! Vertical joint coordinates
	float pitchCoords[NumArmJoints][2];
	
	//! The last HPCB object that was theOne.
	/*! So we can restore it to prominence when we die.
	 *  This is a nice gesture, but it doesn't
	 *  really make sense since we're all using the same port. But just
	 *  in case something changes and we don't do that, this mechanism
	 *  is in place. */
	ArmController *theLastOne;
	
	//! The input command stream socket
	Socket *cmdsock;
	
	//! Executes a command. Called by mechacmd_callback.
	void runCommand(unsigned char *command);
	void doEvent();
	
	//! Possible commands from ArmGUI
	void connect();
	void pointPicked(float param, float param2, float param3, int cmdno);
	void gripper(float param, int cmdno);
	void relax();
	void unrelax();
	void setJoint(unsigned int joint, float param);
	
	ArmController(const ArmController&); //!< don't call
	ArmController operator=(const ArmController&); //!< don't call
	
public:
	//! constructor
	ArmController() : BehaviorBase("ArmController"),
	armMCID(MotionManager::invalid_MC_ID),
	pidMCID(MotionManager::invalid_MC_ID),
	seqMCID(MotionManager::invalid_MC_ID),
	gripperFrameKJ(NULL),
	successJ(NULL),
	KJjoints(),
	horScale(0),
	verScale(0),
	numSteps(40),
	steps(),
	displayMode(),
	theta(0),
	z(0),
	horToBase(),
	verToBase(),
	speed(0.4f),
	orientation(),
	orientationIndex(0),
	numYawJoints(0),
	numPitchJoints(0),
	linksToDisplay(0),
	reachablePointsDelay(),
	yawCoords(),
	pitchCoords(),
	theLastOne(theOne), /* Set the default to the robot last joint before the gripper */
	cmdsock(NULL) {
		for(unsigned int i =0; i < (NumArmJoints+1); i++)
			armConfig[i] = '\0';
		gripperFrameKJ = kine->getKinematicJoint(GripperFrameOffset)->cloneBranch();
		successJ = kine->getKinematicJoint(GripperFrameOffset)->cloneBranch();
		
#ifdef TGT_IS_CALLIOPE5
		orientation[0] = fmat::Quaternion::aboutY(M_PI_2);
		orientation[1] = fmat::Quaternion::aboutY(M_PI);
#else
		orientation[0] = orientation[1] = fmat::Quaternion::IDENTITY;
#endif
	}
	
	//! destructor
	virtual ~ArmController() {
		theOne = theLastOne;
		delete gripperFrameKJ;
		gripperFrameKJ = NULL;
	}
	
	//! Setup the scale and the joints quantity to display
	virtual void doStart();
	
	virtual void doStop();
	
	//! Compute the joint positions
	void computeCoords();
	
	//! Send the joint positions to the ArmGUI
	void sendCoords();
	
	//! Compute and send reachable points w/ given orientation
	/*! Only applicable for robots with vertical joints. param = 1.0 for horizontal
	 *  and 2.0 for vertical. */
	void sendReachablePoints(DisplayMode_t d);
	
	static std::string getClassDescription() {
		char tmp[20];
		sprintf(tmp,"%d",*config->main.armControl_port);
		return std::string("Listens to arm control commands coming in from port ")+tmp;
	}
	
	virtual std::string getDescription() const { return getClassDescription(); }
};

/*! @file
 * @brief Describes ArmController, listens to control commands coming in from the command port for remotely controlling the head
 * @author tss (Creator)
 */

#endif 
