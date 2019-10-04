//-*-c++-*-
#ifndef INCLUDED_PostureMC_h
#define INCLUDED_PostureMC_h

#include "MotionCommand.h"
#include "PostureEngine.h"
#include "MotionManager.h"

class WorldState;

//! a MotionCommand shell for PostureEngine
/*! Will autoprune by default once it reaches the target pose.
 * 
 *  If you want to keep it alive so your behavior can repeatedly use
 *  the posture to move around, either call setAutoPrune(@c false), or
 *  (preferably) use MotionManager::addPersistentMotion() when adding
 *  the motion.
 *
 *  This class will throw a status event when the sensors confirm the
 *  robot has reached a specified target posture, within a configurable
 *  range of #tolerance.  The class will keep trying to reach a target
 *  position for #timeout milliseconds before giving up.
 *
 *  By default, this class will adhere to the values of
 *  ::MaxOutputSpeed, except for head joints which use
 *  Config::motion_config values.
 */
class PostureMC : public MotionCommand, public PostureEngine {
public:
	//!timeout will be set to a default value of 2 seconds, tolerance is .035 radians (~2 degrees), hold is true
	//!@name Constructors

	//!constructor
	PostureMC()
		: MotionCommand(),PostureEngine(),dirty(true),hold(true),tolerance(.035f),
			targetReached(false),targetTimestamp(0),timeout(2000)
	{ init(); }
	
	//!constructor, loads from @a filename
	PostureMC(const std::string& filename)
		: MotionCommand(),PostureEngine(),dirty(true),hold(true),tolerance(.035f),
			targetReached(false),targetTimestamp(0),timeout(2000)
	{ init(); loadFile(filename.c_str()); }
	
	//!constructor, initializes joint positions to the current state of the outputs as defined by @a state
	PostureMC(const WorldState* st)
		: MotionCommand(),PostureEngine(st),dirty(true),hold(true),tolerance(.035f),
			targetReached(false),targetTimestamp(0),timeout(2000)
	{ init(); }
	
	//!destructor
	virtual ~PostureMC() {}
	//@}
	
	//!These functions allow you to mediate problems with a physical robot, such as tolerance of joints not going quite where the should, or safe speed of movement
	//!@name Physical Implementation Issues

	//!Call this if you call PostureEngine::setOutputCmd(), or set values through getOutputCmd()'s reference, since neither will know to set the PostureMC dirty flag
	/*! This is the downside of making setOutputCmd() not virtual - if
	 *  you pass this around as just a PostureEngine, calls made to that
	 *  won't be able to update the dirty flag automatically.  However,
	 *  if the object is typed as a PostureMC at the time the call is
	 *  made, then you don't have to worry.\n These are also not virtual
	 *  - otherwise you'd have to still check the motion out and it
	 *  would make this all pointless!
	 *
	 *  MotionManager::getOutputCmd() is called instead of
	 *  WorldState::outputs[] because if this is being called rapidly
	 *  (i.e. after every sensor reading) using the sensor values will
	 *  cause problems with very slow acceleration due to sensor lag
	 *  continually resetting the current position.  Using the last
	 *  value sent by the MotionManager fixes this.
	 *
	 * @return @c *this */
	PostureMC& setDirty(bool d=true);

	//!Sets #hold - if this is set to false, it will allow a persistent motion to behave the same as a pruned motion, without being pruned
	virtual PostureMC& setHold(bool h=true) { hold=h; return *this; }
	virtual bool getHold() { return hold; } //!< return #hold

	virtual PostureMC& setTolerance(float t) { tolerance=t; return *this; } //!< sets #tolerance, returns @c *this
	virtual float getTolerance() { return tolerance; } //!< returns #tolerance
	virtual PostureMC& setTimeout(unsigned int delay) { timeout=delay; return *this; } //!< sets #timeout, returns @c *this
	virtual unsigned int getTimeout() { return timeout; } //!< returns #timeout

	//! Sets #maxSpeeds to 0 (no maximum)
	void noMaxSpeed() { for(unsigned int i=0; i<NumOutputs; i++) maxSpeeds[i]=0; }
	
	//! Sets #maxSpeeds to x% of the default ::MaxOutputSpeed values from the robot info file (e.g. CalliopeInfo.h)
	/*! Neck joints are instead set from the applicable
	 *  Config::motion_config values.
	 *  @param x ratio of the max speed to use; so 0.5 would limit motion to half the recommended upper limit */
	void defaultMaxSpeed(float x=1);
	
	//! Sets #maxSpeeds[i] entry, in rad/sec
	/*! @param i joint offset, see your model's info file (e.g. ERS7Info.h)
	 *  @param x maximum radians per second to move */
	void setMaxSpeed(unsigned int i, float x) { maxSpeeds[i]=x*FrameTime/1000.f; }
	
	//! Returns #maxSpeeds[i] entry, in rad/sec
	/*! @param i joint offset, see your model's info file (e.g. ERS7Info.h)
	 *  @return the maximum speed of joint @a i in radians per second */
	float getMaxSpeed(unsigned int i) { return maxSpeeds[i]*1000/FrameTime; }
	
	//! returns #curPositions[i], which indicates the last commanded position
	/*! This is particularly useful when a maximum speed is set, and you want to know progress towards the target (#cmds) */
	float getCurrentValue(unsigned int i) { return curPositions[i]; }
	
	//@}



	//!@name MotionCommand Stuff
	virtual int updateOutputs();
	virtual int isDirty() { return dirty || !targetReached; }

	//! returns non-zero (true) if PostureEngine::maxdiff() between this and the current position is over #tolerance
	/*! This is handy so you can set to have the robot go to a position and then automatically remove
	 *  the MotionCommand when it gets there - but beware fighting Postures which average out and neither
	 *  succeeds */
	virtual int isAlive();
	virtual void doStart() { MotionCommand::doStart(); setDirty(); } //!< marks this as dirty each time it is added
	//@}

	//!These functions merely set the dirty flag and then call the PostureEngine version of the function
	//!@name PostureEngine Stuff
	inline virtual void takeSnapshot() { setDirty(); PostureEngine::takeSnapshot(); }
	inline virtual void takeSnapshot(const WorldState& st) { setDirty(); PostureEngine::takeSnapshot(st); }
	inline virtual void setWeights(float w) { setWeights(w,0,NumOutputs); }
	inline virtual void setWeights(float w, unsigned int lowjoint, unsigned int highjoint) { setDirty(); PostureEngine::setWeights(w,lowjoint,highjoint); }
	inline virtual void clear() { setDirty(); PostureEngine::clear(); }
	inline virtual PostureEngine& setOverlay(const PostureEngine& pe) { setDirty(); PostureEngine::setOverlay(pe); return *this; }
	inline virtual PostureEngine& setUnderlay(const PostureEngine& pe) { setDirty(); PostureEngine::setUnderlay(pe); return *this; }
	inline virtual PostureEngine& setAverage(const PostureEngine& pe,float w=0.5) { setDirty(); PostureEngine::setAverage(pe,w); return *this; }
	inline virtual PostureEngine& setCombine(const PostureEngine& pe) { setDirty(); PostureEngine::setCombine(pe); return *this; }

	inline PostureEngine& setOutputCmd(unsigned int i, const OutputCmd& c)
	{ dirty=true; targetReached=false; curPositions[i]=motman->getOutputCmd(i).value; PostureEngine::setOutputCmd(i,c); return *this; }

	inline virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL)
		{ setDirty(); return PostureEngine::loadBuffer(buf,len,filename); }

  inline virtual bool solveLinkPosition(const fmat::SubVector<3, const float>& Ptgt, unsigned int link, const fmat::SubVector<3, const float>& Peff)
	{ setDirty(); return PostureEngine::solveLinkPosition(Ptgt,link,Peff); }

	inline virtual bool solveLinkPosition(float Ptgt_x, float Ptgt_y, float Ptgt_z, unsigned int link, float Peff_x, float Peff_y, float Peff_z) { setDirty(); return PostureEngine::solveLinkPosition(Ptgt_x,Ptgt_y,Ptgt_z,link,Peff_x,Peff_y,Peff_z); }

	inline virtual bool solveLinkVector(const fmat::SubVector<3, const float>& Ptgt, unsigned int link, const fmat::SubVector<3, const float>& Peff)
	{ setDirty(); return PostureEngine::solveLinkVector(Ptgt,link,Peff); }

	inline virtual bool solveLinkVector(float Ptgt_x, float Ptgt_y, float Ptgt_z, unsigned int link, float Peff_x, float Peff_y, float Peff_z)
	{ setDirty(); return PostureEngine::solveLinkVector(Ptgt_x,Ptgt_y,Ptgt_z,link,Peff_x,Peff_y,Peff_z); }
	//@}

protected:
	void init(); //!< initialize #curPositions and #maxSpeeds

	bool  dirty;                    //!< true if changes have been made since last updateOutputs()
	bool  hold;                     //!< if set to true, the posture will be kept active; otherwise joints will be marked unused after each posture is achieved (as if the posture was pruned); set through setHold()
	float tolerance;                //!< when autopruning, if the maxdiff() of this posture and the robot's current position is below this value, isAlive() will be false, defaults to 0.01 (5.7 degree error)
	bool  targetReached;            //!< false if any joint is still moving towards its target
	unsigned int targetTimestamp;   //!< time at which the targetReached flag was set
	unsigned int timeout;           //!< number of milliseconds to wait before giving up on a target that should have already been reached, a value of -1U will try forever
	float curPositions[NumOutputs]; //!< stores the last commanded value for each joint
	float maxSpeeds[NumOutputs];    //!< radians per frame. Initialized from Config::motion_config, but can be overridden by setMaxSpeed()
};

/*! @file
 * @brief Describes PostureMC, a MotionCommand shell for PostureEngine
 * @author ejt (Creator)
 */

#endif
