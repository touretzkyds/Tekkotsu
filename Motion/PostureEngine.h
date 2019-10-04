//-*-c++-*-
#ifndef INCLUDED_PostureEngine_h
#define INCLUDED_PostureEngine_h

#include "Motion/OutputCmd.h"
#include "Motion/Kinematics.h"
#include "Shared/RobotInfo.h"
#include "Shared/LoadSave.h"

class WorldState;

//! A class for storing a set of positions and weights for all the outputs
/*! File Format: ([..] indicates an optional parameter)\n
 *  - First line: '<tt>\#POS</tt>'
 *  - Followed by a series of:
 *    - '<tt>specialize </tt><i>[regex]</i>' - only robots whose model name matches the regular
 *      expression will attempt to parse lines between this and the next 'specialize' command.  An empty
 *      (absent) regex matches all robots (equivalent to '<tt>specialize&nbsp;.*</tt>')
 *    - '<tt>condensed </tt><i>model</i>' - specifies model name for which condensed specifications will be
 *      interpreted
 *    - '<tt>verbose</tt>' - switches the load/save format back to "verbose" style, where each output/sensor
 *      value is listed individually
 *    - '<tt>&lt;</tt><i>section</i><tt>&gt;</tt> ... <tt>&lt;/</tt><i>section</i><tt>&gt;</tt>' - specifies a
 *      section for following "verbose" style lines (valid <i>section</i> values listed below)
 *    - '<i>section</i> <i>value1</i> <i>value2</i> <i>value3</i> ...' - specify all values for a section in
 *      one line, "condensed" style (valid <i>section</i> values listed below).  Must have a value for every
 *      item in the section (no extra or missing values)
 *    - '<i>output-name</i> <i>value</i> <i>[weight]</i>' - specifies an output value, with optional weighting
 *      value (if no weight specified, '1' is assumed)
 *  - Last line: '<tt>\#END</tt>'
 *  
 *  Note that '=' can be used to separate tokens as well as any whitespace character.
 *  All angle values should be specified in radians.
 *
 *  Valid section names are:
 *  - <tt>meta-info</tt> - only recognizes two fields: <tt>timestamp</tt> and <tt>framenumber</tt>
 *  - <tt>outputs</tt> - supports all output names (e.g. ERS7Info::outputNames)
 *  - <tt>buttons</tt> - supports all button names (e.g. ERS7Info::buttonNames)
 *  - <tt>sensors</tt> - supports all sensor names (e.g. ERS7Info::sensorNames)
 *  - <tt>pidduties</tt> - supports output names corresponding to PID joints (this is the "duty cycle" for the joint)
 *
 *  Additionally 'weights' can be used as a condensed section name (but not a
 *  verbose tag-style section, since weights are specified on each line).  The
 *  'weights' specification must follow the 'outputs' specification to be effective.
 *
 *  Data following '\#' is ignored as comments.  Be aware if you
 *  load the file and then save it again, these comments will be lost.
 *
 *  Example 1: Specifies only neck joints, looks forward and up a little
<table><tr><td align=left><pre>
\#POS 
NECK:tilt	0
NECK:pan	0
NECK:nod	0.2
\#END
</pre></td></tr></table>
 *  All other, unspecified, joints will be set to weight=0.
 *
 *  Example 2: Logged sensor data from an ERS-7 robot
<table><tr><td align=left><tt>
\#POS<br>
condensed ERS-7<br>
meta-info = 1439041 178803<br>
outputs = -0.131722 0.148077 1.74592 -0.30276 0.341717 2.13361 -1.03935 0.091124 2.18958 -0.804097 0.034171 1.67458 -0.016362 -0.089143 0.125563 0.407243 -0.054399 -0.064704 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0<br>
buttons = 0 0 0 0 0 0 0 0 0 1<br>
sensors = 500 860.139 425 -2.06456 -0.516139 -8.94642 0.99 30.63 2200 8.265 0<br>
pidduties = -0.164062 0.0878906 0.0957031 0.246094 -0.0195312 -0.0546875 -0.164062 -0.0195312 0.164062 0.0273438 0 -0.0683594 -0.00585938 -0.00390625 0.0820312 -0.0390625 0.015625 0.00390625<br>
\#END<br>
</tt></td></tr></table>
 *  Condensed posture files can still be loaded on other models.  For each entry, RobotInfo::Capabilities will be
 *  used to map from the model specified by the <tt>condensed</tt> command to the current host.
 *
 *  @see PostureMC
 *  @see <a href="http://www.cs.cmu.edu/~tekkotsu/Kinematics.html">Tekkotsu's Kinematics page</a>
 *  @see <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/postures.shtml">David Touretzky's "Postures and Motion Sequences" Chapter</a>
 *  @see <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/forwardkin.shtml">David Touretzky's "Forward Kinematics" Chapter</a>
 *  @see <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/postures.pdf">CMU's Cognitive Robotics posture slides</a>
 *  @see <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/kinematics.pdf">CMU's Cognitive Robotics kinematics slides</a>
 */
class PostureEngine : public LoadSave, public Kinematics {
public:

	//!@name Constructors
	
	//!constructor
	PostureEngine() : LoadSave(), Kinematics(*kine), saveFormatCondensed(false), saveSensors(NULL), loadSensors(NULL) {}
	//!constructor, loads a position from a file
	/*! @todo might want to make a library stored in memory of common positions so they don't have to be loaded repeatedly from memstick */
	PostureEngine(const std::string& filename) : LoadSave(), Kinematics(*kine), saveFormatCondensed(false), saveSensors(NULL), loadSensors(NULL) { loadFile(filename.c_str()); }
	//!constructor, initializes joint positions to the current state of the outputs as defined by @a state
	PostureEngine(const WorldState* st) : LoadSave(), Kinematics(*kine), saveFormatCondensed(false), saveSensors(NULL), loadSensors(NULL) { if(st!=NULL) takeSnapshot(*st); }

	//! copy constructor
	PostureEngine(const PostureEngine& pe)
		: LoadSave(pe), Kinematics(pe), saveFormatCondensed(pe.saveFormatCondensed), saveSensors(pe.saveSensors), loadSensors(NULL)
	{
		for(unsigned int i=0; i<NumOutputs; i++)
			cmds[i]=pe.cmds[i];
	}
		
	//! assignment operator
	PostureEngine& operator=(const PostureEngine& pe) {
		LoadSave::operator=(pe);
		Kinematics::operator=(pe);
		saveFormatCondensed=pe.saveFormatCondensed;
		saveSensors=pe.saveSensors;
		for(unsigned int i=0; i<NumOutputs; i++)
			cmds[i]=pe.cmds[i];
		return *this;
	}

	//! destructor
	virtual ~PostureEngine();
	//@}



	//! You should be able to call the non-virtual functions without checking out, just a MotionManager::peekMotion().  Theoretically.
	//!@name Output Value Access/Control
	virtual void takeSnapshot(); //!< sets the values of #cmds to the current state of the outputs (doesn't change the weights)
	virtual void takeSnapshot(const WorldState& st); //!< sets the values of #cmds to the current state of the outputs as defined by @a state (doesn't change the weights)
	virtual void setWeights(float w) { setWeights(w,0,NumOutputs); } //!< set the weights of all #cmds
	virtual void setWeights(float w, unsigned int lowjoint, unsigned int highjoint); //!< the the weights of a range of #cmds
	virtual void clear(); //!< sets all joints to unused
	inline PostureEngine& setOutputCmd(unsigned int i, const OutputCmd& c) { cmds[i]=c; return *this; } //!<sets output @a i to OutputCmd @a c, returns @c *this so you can chain them; also remember that OutputCmd support implicit conversion from floats (so you can just pass a float)
	inline OutputCmd& operator()(unsigned int i) { return cmds[i]; } //!< returns output @a i, returns a reference so you can also set through an assignment to this call, e.g. pose(MouthOffset)=.1; (remember that OutputCmd support implicit conversion from floats)
	inline const OutputCmd& operator()(unsigned int i) const { return cmds[i]; } //!< returns output @a i
	inline OutputCmd& getOutputCmd(unsigned int i) { return cmds[i]; } //!< returns output @a i, returns a reference so you can also set through an assignment
	inline const OutputCmd& getOutputCmd(unsigned int i) const { return cmds[i]; } //!< returns output @a i
	//@}



	//!Uses LoadSave interface so you can load/save to files, uses a human-readable storage format
	//!@name LoadSave
	virtual void setSaveFormat(bool condensed, WorldState* ws); //!< sets #saveFormatCondensed and #loadSaveSensors (pass ::state for @a ws if you want to use current sensor values)
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const;
	virtual unsigned int loadFile(const char filename[]);
	virtual unsigned int saveFile(const char filename[]) const;
	
	//! used for storing sensor data loaded from file
	struct SensorInfo {
		SensorInfo() : buttons(), sensors(), pidduties(), timestamp(-1U), frameNumber(-1U) {}
		std::map<unsigned int,float> buttons; //!< button values
		std::map<unsigned int,float> sensors; //!< sensor values
		std::map<unsigned int,float> pidduties; //!< pid duty cycle values, indices are relative to PIDJointOffset
		unsigned int timestamp; //!< timestamp associated with sensor values, or -1U if none found
		unsigned int frameNumber; //!< sensor frame number associated with sensor values, or -1U if none found
	};
	virtual void setLoadedSensors(SensorInfo* si) { loadSensors=si; } //!< if @a ws is non-NULL, any sensor values in loaded postures will be stored there (otherwise they are ignored)
	virtual SensorInfo* getLoadedSensors() const { return loadSensors; } //!< returns value previously stored by setLoadSensors()
	//@}



	//!@name Kinematics

	//! Performs inverse kinematics to solve for positioning @a Peff on link @a j as close as possible to @a Ptgt (base coordinates in homogenous form); if solution found, stores result in this posture and returns true
	/*! @param Ptgt the target point, in base coordinates
	 *  @param link the output offset of the joint to move
	 *  @param Peff the point (relative to @a link) which you desire to have moved to @a Ptgt (it's a point on the effector, e.g. (0,0,0) if you want to move the effector's origin to the target)
	 *
	 *  The difference between solveLinkPosition() and solveLinkVector() is typically small,
	 *  but critical when you're trying to look at something -- the solution obtained by
	 *  simplying trying to solve for the position may not align the vector with the target --
	 *  solveLinkVector() tries to ensure the vector is aligned with the target, even if that
	 *  isn't the closest solution position-wise.
	 */
	virtual bool solveLinkPosition(const fmat::SubVector<3,const float>& Ptgt, unsigned int link, const fmat::SubVector<3,const float>& Peff);

	//! Performs inverse kinematics to solve for positioning Peff on link @a j as close as possible to @a Ptgt (base coordinates); if solution found, stores result in this posture and returns true
	/*! @param Ptgt_x the target x position (relative to base frame)
	 *  @param Ptgt_y the target y position (relative to base frame)
	 *  @param Ptgt_z the target z position (relative to base frame)
	 *  @param link the output offset of the joint to move
	 *  @param Peff_x the x position (relative to @a link) which you desire to have moved to @a Ptgt (it's a point on the effector)
	 *  @param Peff_y the y position (relative to @a link) which you desire to have moved to @a Ptgt (it's a point on the effector)
	 *  @param Peff_z the z position (relative to @a link) which you desire to have moved to @a Ptgt (it's a point on the effector)
	 *
	 *  The difference between solveLinkPosition() and solveLinkVector() is typically small,
	 *  but critical when you're trying to look at something -- the solution obtained by
	 *  simplying trying to solve for the position may not align the vector with the target --
	 *  solveLinkVector() tries to ensure the vector is aligned with the target, even if that
	 *  isn't the closest solution position-wise.
	 */
	virtual bool solveLinkPosition(float Ptgt_x, float Ptgt_y, float Ptgt_z, unsigned int link, float Peff_x, float Peff_y, float Peff_z)
	{ return solveLinkPosition(fmat::pack(Ptgt_x,Ptgt_y,Ptgt_z),link,fmat::pack(Peff_x,Peff_y,Peff_z)); }

	//! Performs inverse kinematics to solve for aligning the vector through Peff on link @a j and the link's origin to point at @a Ptgt (base coordinates in homogenous form); if solution found, stores result in this posture and returns true
	/*! @param Ptgt the target point, in base coordinates
	 *  @param link the output offset of the joint to move
	 *  @param Peff the point (relative to @a link) which you desire to have moved to @a Ptgt (it's the desired "effector")
	 *
	 *  The difference between solveLinkPosition() and solveLinkVector() is typically small,
	 *  but critical when you're trying to look at something -- the solution obtained by
	 *  simplying trying to solve for the position may not align the vector with the target --
	 *  solveLinkVector() tries to ensure the vector is aligned with the target, even if that
	 *  isn't the closest solution position-wise.
	 */
	virtual bool solveLinkVector(const fmat::SubVector<3,const float>& Ptgt, unsigned int link, const fmat::SubVector<3,const float>& Peff);

	//! Performs inverse kinematics to solve for aligning the vector through Peff on link @a j and the link's origin to point at @a Ptgt (base coordinates); if solution found, stores result in this posture and returns true
	/*! @param Ptgt_x the target x position (relative to base frame)
	 *  @param Ptgt_y the target y position (relative to base frame)
	 *  @param Ptgt_z the target z position (relative to base frame)
	 *  @param link the output offset of the joint to move
	 *  @param Peff_x the x position (relative to @a link) which you desire to have moved to @a Ptgt (it's the desired "effector")
	 *  @param Peff_y the y position (relative to @a link) which you desire to have moved to @a Ptgt (it's the desired "effector")
	 *  @param Peff_z the z position (relative to @a link) which you desire to have moved to @a Ptgt (it's the desired "effector")
	 *
	 *  @todo this method is an approximation, could be more precise, and perhaps faster, although this is pretty good.
	 *
	 *  The difference between solveLinkPosition() and solveLinkVector() is typically small,
	 *  but critical when you're trying to look at something -- the solution obtained by
	 *  simplying trying to solve for the position may not align the vector with the target --
	 *  solveLinkVector() tries to ensure the vector is aligned with the target, even if that
	 *  isn't the closest solution position-wise.
	 */
	virtual bool solveLinkVector(float Ptgt_x, float Ptgt_y, float Ptgt_z, unsigned int link, float Peff_x, float Peff_y, float Peff_z)
	{ return solveLinkVector(pack(Ptgt_x,Ptgt_y,Ptgt_z),link,pack(Peff_x,Peff_y,Peff_z)); }
	
	//! Performs inverse kinematics to solve for positioning @a Peff on link @a j as close as possible to @a Ptgt (base coordinates in homogenous form); if solution found, stores result in this posture and returns true
	/*! @param oriTgt the target orientation, as a quaternion relative to base frame
	 *  @param link the output offset of the joint to move
	 *  @param oriOffset an orientation offset (relative to @a link) which you desire to have moved to @a oriTgt
	 *
	 *  The quaternion functions of fmat are helpful for creating axis-aligned rotations: fmat::quatX(r), fmat::quatY(r), fmat::quatZ(r), and fmat::QUAT_IDENTITY
	 */
	virtual bool solveLinkOrientation(const fmat::Quaternion& oriTgt, unsigned int link, const fmat::Quaternion& oriOffset);
	
	//! Performs inverse kinematics to solve for both a position and orientation, bringing @a posOffset on @a link to @a posTgt, in the orientation specified.
	/*! @param posTgt the target point, in base coordinates
	 *  @param oriTgt the target orientation, as a quaternion relative to base frame
	 *  @param link the output offset of the joint to move
	 *  @param posOffset the point (relative to @a link) which you desire to have moved to @a posTgt (it's the desired "effector")
	 *  @param oriOffset an orientation offset (relative to @a link) which you desire to have moved to @a oriTgt
	 *
	 *  The quaternion functions of fmat are helpful for creating axis-aligned rotations: fmat::quatX(r), fmat::quatY(r), fmat::quatZ(r), and fmat::QUAT_IDENTITY
	 */
	 virtual bool solveLink(const fmat::SubVector<3,const float>& posTgt, const fmat::Quaternion& oriTgt, unsigned int link, const fmat::SubVector<3,const float>& posOffset, const fmat::Quaternion& oriOffset);

	//@}



	//!@name Combining Postures
	
	//! sets joints of this to all joints of @a pe which are not equal to unused (layers @a pe over this) stores into this
	virtual PostureEngine& setOverlay(const PostureEngine& pe);
	//! sets joints of this to all joints of @a pe which are not equal to unused (layers @a pe over this) returns new PostureEngine
	virtual PostureEngine createOverlay(const PostureEngine& pe) const;

	//! sets joints of this which are equal to unused to @a pe, (layers this over @a pe) stores into this
	virtual PostureEngine& setUnderlay(const PostureEngine& pe);
	//! sets joints of this which are equal to unused to @a pe, (layers this over @a pe) returns new PostureEngine
	virtual PostureEngine createUnderlay(const PostureEngine& pe) const;

	//! computes a weighted average of this vs. @a pe, @a w being the weight towards @a pe (so @a w==1 just copies @a pe)
	virtual PostureEngine& setAverage(const PostureEngine& pe,float w=0.5);
	//! computes a weighted average of this vs. @a pe, @a w being the weight towards @a pe (so @a w==1 just copies @a pe)
	virtual PostureEngine createAverage(const PostureEngine& pe,float w=0.5) const;

	//! computes a weighted average of this vs. @a pe, using the weight values of the joints, storing the total weight in the result's weight value
	virtual PostureEngine& setCombine(const PostureEngine& pe);
	//! computes a weighted average of this vs. @a pe, using the weight values of the joints, storing the total weight in the result's weight value
	virtual PostureEngine createCombine(const PostureEngine& pe) const;

	//! returns the sum squared error between this and pe's output values, but only between outputs which are both not unused
	/*! @todo create a version which does weighted summing?  This treats weights as all or nothing */
	virtual float diff(const PostureEngine& pe) const;
	
	//! returns the average sum squared error between this and pe's output values for outputs which are both not unused
	/*! @todo create a version which does weighted summing?  This treats weights as all or nothing */
	virtual float avgdiff(const PostureEngine& pe) const;
	
	//! returns the max error between this and pe's output values for outputs which are both not unused
	/*! @todo create a version which does weighted summing?  This treats weights as all or nothing */
	virtual float maxdiff(const PostureEngine& pe) const;
	
	//@}

protected:
	//! enumeration of the different section types that may be used as section tags in verbose mode
	enum section_t {
		SECTION_METAINFO, //!< includes timestamp and framenumber
		SECTION_OUTPUTS, //!< entries corresponding to NumOutputs
		SECTION_BUTTONS, //!< entries corresponding to NumButtons
		SECTION_SENSORS, //!< entries corresponding to NumSensors
		SECTION_PIDDUTIES //!< entries corresponding to NumPIDJoints
	};
	//! helper function for loadBuffer, called for each individual line
	virtual bool loadLine(unsigned int linenum, const char* filename, const std::map<std::string,section_t>& sectionMap, std::vector<std::string>& words, section_t& curSection, const Capabilities*& caps, bool& filtered);
	//! helper function for loadLine, strips trailing '~'s from output names to provide backward compatability (note pass by reference, operates 'in-place' on @a word)
	/*! (originally, all output names were uniform length and used '~'s as padding... ugh.) */
	void stripTildes(std::string& str) {
		str.erase(str.find_last_not_of('~')+1); // if n is npos, npos is -1U, so becomes 0... should be safe :)
	}

	//! overriding Kinematics::update, make all updates come from this posture engine's own state, not WorldState
	virtual void update() const;

	//!the table of outputs' values and weights, can be accessed through setOutputCmd() and getOutputCmd()
	OutputCmd cmds[NumOutputs];

	bool saveFormatCondensed;      //!< requests a condensed file format, smaller but less readable
	WorldState* saveSensors;   //!< If non-null, saves will include sensor readings from here
	SensorInfo* loadSensors; //!< if non-null, loads will store any included sensor information here
};

/*! @file
 * @brief Describes PostureEngine, a base class for managing the values and weights of all the outputs
 * @author ejt (Creator)
 */

#endif
