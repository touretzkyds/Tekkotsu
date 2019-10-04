//-*-c++-*-
#ifndef INCLUDED_HeadPointerMC_h
#define INCLUDED_HeadPointerMC_h

#include "Shared/RobotInfo.h"
#include "MotionCommand.h"
#include "OutputCmd.h"
#include "Shared/mathutils.h"

namespace DualCoding {
	class Point;
}

//! This class gives some quick and easy functions to point the head at things
class HeadPointerMC : public MotionCommand {
public:
  //! Constructor, defaults to all joints to current value in ::state (i.e. calls takeSnapshot() automatically)
  HeadPointerMC();
	
  //! Destructor
  virtual ~HeadPointerMC() {}
	
  //!Sets #hold - if this is set to false, it will allow a persistent motion to behave the same as a pruned motion, without being pruned
  virtual void setHold(bool h=true) { hold=h; }
  virtual bool getHold() { return hold; } //!< return #hold

  virtual void setTolerance(float t) { tolerance=t; } //!< sets #tolerance
  virtual float getTolerance() { return tolerance; } //!< returns #tolerance
  virtual void setTimeout(unsigned int delay) { timeout=delay; } //!< sets #timeout
  virtual unsigned int getTimeout() { return timeout; } //!< returns #timeout
	
  //! sets the target to last sent commands, and dirty to false; essentially freezes motion in place
  /*! This is very similar to takeSnapshot(), but will do the "right thing" (retain current position) when motion blending is involved.
   *  A status event will be generated if/when the joints reach the currently commanded position.
   *  Probably should use freezeMotion() if you want to stop a motion underway, but takeSnapshot() if you want to reset/intialize to the current joint positions. */
  virtual void freezeMotion();
  //! sets the target joint positions to current sensor values
  /*! Similar to freezeMotion() when a motion is underway, but only if no other MotionCommands are using neck joints.
   *  A status event will @e not be generated unless a motion was already underway.
   *  Probably should use freezeMotion() if you want to stop a motion underway, but takeSnapshot() if you want to reset/intialize to the current joint positions. */
  virtual void takeSnapshot();

  //!@name Speed Control
	
  //! Sets #maxSpeed to 0 (no maximum)
  void noMaxSpeed() { for(unsigned int i=0; i<NumHeadJoints; i++) maxSpeed[i]=0; }
	
  //! Restores #maxSpeed to default settings from Config::Motion_Config
  /*! @param x ratio of the max speed to use; so 0.5 would limit motion to half the recommended upper limit */
  void defaultMaxSpeed(float x=1);
	
  //! Sets #maxSpeed in rad/sec
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @param x maximum radians per second to move */
  void setMaxSpeed(unsigned int i, float x) { maxSpeed[i]=x*FrameTime/1000; }
	
  //! Returns #maxSpeed in rad/sec
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @return the maximum speed of joint @a i in radians per second */
  float getMaxSpeed(unsigned int i) { return maxSpeed[i]*1000/FrameTime; }
	
  //@}
	
  //!@name Joint Accessors
	
  //! Sets the weight values for all the neck joints
  void setWeight(float w);
	
  //! Request a set of neck joint values
  /*! Originally this corresponded directly to the neck joints of the aibo, however
   *  on other platforms it will use capabilties mapping to try to set correspnding joints if available.
   *  (we're not doing kinematics here, trying to set joint values directly.  If your
   *  want a more generic/abstract interface, use lookAtPoint()/lookInDirection().
   *
   *  Note that for a "pan-tilt" camera, you actually want to set the @em last two parameters,
   *  @em not the first two!
   * 
   *  @param tilt value - this is an initial rotation about the camera x axis
   *  @param pan value - this is a rotation about the camera y axis
   *  @param tilt2 value - a second rotation about the camera x axis ("nod")
   *
   *  On ERS-210 and 220, the tilt2 is actually a roll about camera z (ugly, but no one's using those anymore, so moot issue) */
  void setJoints(float pan, float tilt);
  void setJoints(float tilt, float pan, float tilt2);
  void setJoints(float pan, float shoulder, float elbow, float pitch);
	
  //! Directly set a single neck joint value
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @param value the value to be assigned to join @a i, in radians */
  void setJointValue(unsigned int i, float value) {
#ifdef TGT_HAS_HEAD
    if(!ensureValidJoint(i))
      return;
    headTargets[i]=clipAngularRange(HeadOffset+i,value);
    setDirty();
#endif
  }
	
  //! Returns the target value of joint @a i.  Use this if you want to know the current @b commanded joint value; To get the current joint @b position, look in WorldState::outputs
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t) */
  float getJointValue(unsigned int i) const {
    if(ensureValidJoint(i))
      return headTargets[i];
    else
      return 0;
  }
	
  //! Centers the camera on a point in space, attempting to keep the camera as far away from the point as possible
  /*! Point should be relative to the body reference frame (see ::BaseFrameOffset).  Returns true if the target is reachable.
   *  @param x location in millimeters
   *  @param y location in millimeters
   *  @param z location in millimeters
   *
   *  @todo this method is an approximation, could be more precise, and perhaps faster, although this is pretty good. */
  bool lookAtPoint(float x, float y, float z);
	
  //! Centers the camera on a point in space, attempting to keep the camera as far away from the point as possible
  /*! Point should be relative to the body reference frame (see ::BaseFrameOffset).  Returns true if the target is reachable.
   *  @param p point to look at */
  bool lookAtPoint(const DualCoding::Point &p);

  //! Centers the camera on a point in space, attempting to keep the camera as far away from the point as possible
  /*! Point should be relative to the body reference frame (see ::BaseFrameOffset).  Returns true if the target is reachable.
   *  @param p point to look at */
  bool lookAtPoint(const fmat::Column<3> &p);

  //! Centers the camera on a point in space, attempting to move the camera @a d millimeters away from the point
  /*! Point should be relative to the body reference frame (see ::BaseFrameOffset).  Returns true if the target is reachable.
   *  @param x location in millimeters
   *  @param y location in millimeters
   *  @param z location in millimeters
   *  @param d target distance from point in millimeters */
  bool lookAtPoint(float x, float y, float z, float d);
	
  //! Points the camera in a given direction
  /*! Vector should be relative to the body reference frame (see ::BaseFrameOffset).  Returns true if the target is reachable.
   *  @param x component of the direction vector
   *  @param y component of the direction vector
   *  @param z component of the direction vector */
  bool lookInDirection(float x, float y, float z);

  //! Points the camera at a given robot joint; most useful with ::GripperFrameOffset
  bool lookAtJoint(unsigned int j);
	
  //@}
	
public:	
  //!@name Inherited:
  virtual int updateOutputs(); //!< Updates where the head is looking
  virtual int isDirty() { return (dirty || !targetReached)?3:0; } //!< true if a change has been made since the last updateJointCmds() and we're active
  virtual int isAlive(); //!< Alive while target is not reached
  virtual void doStart() { MotionCommand::doStart(); setDirty(); } //!< marks this as dirty each time it is added
  //@}

  //! if targetReached, reassigns headCmds from MotionManager::getOutputCmd(), then sets dirty to true and targetReached to false
  /*! should be called each time a joint value gets modified in case
   *  the head isn't where it's supposed to be, it won't jerk around
   * 
   *  MotionManager::getOutputCmd() is called instead of
   *  WorldState::outputs[] because if this is being called rapidly
   *  (i.e. after every sensor reading) using the sensor values will
   *  cause problems with very slow acceleration due to sensor lag
   *  continually resetting the current position.  Using the last
   *  value sent by the MotionManager fixes this.*/
  void setDirty();

protected:
  //! puts x in the range (-pi,pi)
  static float normalizeAngle(float x) { return x - static_cast<float>( rint(x/(2*M_PI)) * (2*M_PI) ); }
	
  //! if @a x is outside of the range of joint @a i, it is set to either the min or the max, whichever is closer
  static float clipAngularRange(unsigned int i, float x) {
    float min=outputRanges[i][MinRange];
    float max=outputRanges[i][MaxRange];
    if(x<min || x>max) {
      float mn_dist=std::abs(normalizeAngle(min-x));
      float mx_dist=std::abs(normalizeAngle(max-x));
      if(mn_dist<mx_dist)
	return min;
      else
	return max;
    } else
      return x;
  }

  //! Makes sure @a i is in the range (0,NumHeadJoints).  If it is instead in the range (HeadOffset,HeadOffset+NumHeadJoints), output a warning and reset @a i to the obviously intended value.
  /*! @param[in] i joint offset relative to either HeadOffset (i.e. one of TPROffset_t) or 0
   *  @param[out] i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @return true if the intended joint could be ascertained, false otherwise */
  static bool ensureValidJoint(unsigned int& i);

  bool dirty;                          //!< true if a change has been made since last call to updateJointCmds()
  bool  hold;                          //!< if set to true, the posture will be kept active; otherwise joints will be marked unused after each posture is achieved (as if the posture was pruned); set through setHold()
  float tolerance;                     //!< when autopruning, if the maxdiff() of this posture and the robot's current position is below this value, isAlive() will be false, defaults to 0.05 radian (2.86 degree error)
  bool targetReached;                  //!< false if the head is still moving towards its target
  unsigned int targetTimestamp;        //!< time at which the targetReached flag was set
  unsigned int timeout;                //!< number of milliseconds to wait before giving up on a target that should have already been reached, a value of -1U will try forever
  float headTargets[NumHeadJoints];    //!< stores the target value of each joint
  OutputCmd headCmds[NumHeadJoints];   //!< stores the last values we sent from updateOutputs
  float maxSpeed[NumHeadJoints];       //!< initialized from Config::motion_config, but can be overridden by setMaxSpeed(); rad per frame
};

/*! @file
 * @brief Describes HeadPointerMC, a class for various ways to control where the head is looking
 * @author ejt (Creator)
 */

#endif
