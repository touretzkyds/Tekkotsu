//-*-c++-*-
#ifndef INCLUDED_MotionSequenceEngine_h_
#define INCLUDED_MotionSequenceEngine_h_

#include "Shared/LoadSave.h"
#include "IPC/ListMemBuf.h"
#include "PostureEngine.h"
#include "Shared/attributes.h"

//! A handy class for storing a sequence of keyframed movements
/*! Each outputs is handled independently.  It's easy to add keyframes
 *  which modify all of the outputs, but since each output is tracked
 *  individually, OutputCmd's with 0 weight can be used to not affect
 *  other motions.  For instance, if you want to pan the head left to right while
 *  moving the right leg up and down several times, you won't have to
 *  specify the intermediary position of the head in its motion at each of the leg
 *  motion keyframes.
 *
 *  Be aware that the 0 time frame will be replaced on a call to
 *  play() with the current body posture from ::state.  However, this only applies
 *  to outputs which have a non-zero weighted frame defined at some
 *  point.  The weights, of the 0 time frame will remain unchanged.  
 *  These weights are initially set to 0, so that it's
 *  possible to 'fade in' the first frame of the motion sequence from
 *  whereever the body happens to be (or already doing)
 *
 *  To fade out at the end, set a frame with 0 weight for everything.
 *  Otherwise it will simply die suddenly.  When a joint reaches its
 *  last keyframe, if #hold is set (the default) it will hold its last value
 *  until the MotionSequence is reset or stopped.  If #hold is false,
 *  then the joint is treated as 0 weight once it reaches its last frame.
 *
 *  Currently, MotionSequenceEngine is intended mainly for building, 
 *  not editing.  It's easy to add keyframes, but hard/impossible to
 *  delete them.
 *
 *  The MotionSequenceEngine base class is an abstract class so that you can
 *  create memory efficient motion sequences and simply refer to them
 *  by the common base class instead of having to worry about the
 *  actual size allocated in the template, MotionSequenceMC.
 *
 *  @see MotionSequenceEngine::SizeSmall, MotionSequenceEngine::SizeMedium, MotionSequenceEngine::SizeLarge, MotionSequenceEngine::SizeXLarge, 
 *  
 *  The file format used is as follows: ('<' and '>' are not meant literally)
 *  - First line: '<tt>\#MSq</tt>'
 *  - Followed by any series of:\n
 *    - '<tt>advanceTime </tt><i>time-delta</i>' - moves playhead forward, in milliseconds (synonym for <tt>delay</tt>)
 *    - '<tt>delay </tt><i>time-delta</i>' - moves playhead forward, in milliseconds (synonym for <tt>advanceTime</tt>)
 *    - '<tt>setTime </tt><i>time</i>' - sets play time to specified value, in ms
 *    - '<i>outputname</i><tt> </tt><i>value</i><tt> </tt>[<i>weight</i>]' - sets the specified output to the value - assumes 1 for weight; you can view the list of valid joint names in the outputNames array within the RobotInfo extension namespace for your model.  (e.g. ERS210Info::outputNames[])
 *    - '<tt>load </tt><i>filename</i>' - file can be a posture or another motion sequence (if MS, leaves playhead at end of the motion sequence); see setPose() and overlayMotion()
 *    - '<tt>loadExplicit </tt><i>filename</i>' - file must be a posture, sets position for all outputs, including zero-weighted ones; see setExplicitPose()
 *    - '<tt>#</tt><i>comment</i>' - a comment line
 *  - Last line: '<tt>\#END</tt>'
 *  
 *  Lines beginning with '#' are ignored as comments.  Be aware if you
 *  load the file and then save it again, these comments will be lost.
 *
 *  Example 1: This motion sequence will straighten out the head, panning from right to left.\n
<table align="center"><tr><td align=left><pre>
\#MSq 

<i>\# Straighten head</i>
advanceTime  50 
NECK:tilt       0.262
NECK:nod       0 

<i>\# Pan right</i>
advanceTime  850 
NECK:pan~       -0.785 

<i>\# Pan left</i>
advanceTime  900 
NECK:pan~       0.785 
NECK:tilt       0.262 
NECK:nod       0 

\#END
</pre></td></tr></table>
 *  
 *  This graph illustrates the motion of the tilt and pan joints in example 1:
 *  <img src="MotionSequenceGraph1.png">
 *  Notice how the joint will move from wherever it is initally to the first
 *  keyframe <i>for that joint</i>.  Specifying the tilt joint a second time
 *  at the end of the motion forces the tilt joint to be held at that position
 *  throughout the motion, regardless of the #hold setting at the time
 *  the sequence is run.
 *  
 *  Example 2: This example will straighten the head and the tail, pan the
 *  head left to right, and @e then pan the tail left to right.\n
<table align="center"><tr><td align=left><pre>
\#MSq

<i>\# Bring head and tail to neural positions</i>
advanceTime 50
NECK:pan~ 0
NECK:tilt 0
TAIL:pan~ 0
TAIL:tilt 0

<i>\# Pan left</i>
advanceTime 1000
NECK:pan~ 1.571
<i>\# Pan right</i>
advanceTime 1000
NECK:pan~ -1.571

<i>\# Center head</i>
<i>\# Update tail time index</i>
advanceTime 500
NECK:pan~ 0
<i>\# Note this respecification of TAIL:pan~ at 0 -- see graph below</i>
TAIL:pan~ 0

<i>\# Wag left</i>
advanceTime 500
TAIL:pan~ 1.571
<i>\# Wag right</i>
advanceTime 500
TAIL:pan~ -1.571

\#END 
</pre></td></tr></table>
 *  
 *  These graphs illustrate the motion of the pan joint for the head and the tail:
 *  <img src="MotionSequenceGraph2-head.png">
 *  <img src="MotionSequenceGraph2-tail.png">
 *  The head's motion should be straightforward.  The thing to note in the tail's
 *  graph is why the second <code>TAIL:pan~ 0</code> is necessary at time
 *  2550.  If it were not specified, the tail would slowly move to the left over the
 *  course of the head's movement.  We want it to stay still until the head is done,
 *  so it must be respecified at the same position to hold it at that value in the
 *  intervening time.
 *  
 *  After loading a motion sequence, the playtime is left at the end.
 *  This is to make it easy to append/overlay motion sequences.
 *  However, the playhead will be reset to the beginning on the first
 *  call to updateOutputs() if isPlaying() returns true.
 *
 *  You can also create a motion sequence dynamically at run time:
 *  \code
 *  //This code sample will stand up, point the head forward and up 0.1 radians,
 *  //and then autoprune
 *
 *  //First declare the MotionSequence itself:
 *  SharedObject< MotionSequenceMC<MotionSequenceEngine::SizeSmall> > stand;
 *
 *  //Over the course of the first 700 milliseconds, go to standing posture:
 *  standSit->setTime(700);
 *  standSit->setPose(PostureEngine("stand.pos"));
 *  // could also use standSit->loadFile("stand.pos")
 *
 *  //Then take another 700 milliseconds to straighten out the head:
 *  standSit->advanceTime(700);
 *  //We'll set joints individually this time, instead of loading a posture file:
 *  standSit->setOutputCmd(HeadOffset+PanOffset,0);
 *  standSit->setOutputCmd(HeadOffset+RollOffset,0);
 *  standSit->setOutputCmd(HeadOffset+TiltOffset,0.1); //look up .1 radians
 *
 *  //Add to MotionManager:
 *  motman->addPrunableMotion(standSit);
 *  //Playback will now begin automatically, and region deallocated when done
 *  \endcode
 *  
 *  By default, #playing is true.  Thus, when you add a MotionSequenceMC
 *  to the MotionManager, it will begin executing automatically.  If
 *  you do \e not want this behavior, simply call pause() before
 *  adding the sequence.
 *
 *  When the sequence reaches the end, isAlive() will return false.
 *  If the motion was added with MotionManager::addPrunableMotion, the
 *  motion sequence will then autoprune itself from the MotionManager.
 *  However, you can either call MotionManager::addPersistentMotion()
 *  to add it, or call setAutoPrune(false), if you want to retain the
 *  same instantiation between executions.
 *
 *  @see PostureEngine for information on the posture files
 *  @see <a href="http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/postures.shtml">David Touretzky's "Postures and Motion Sequences" Chapter</a>
 *  @see <a href="http://www.cs.cmu.edu/afs/cs/academic/class/15494-s06/www/lectures/postures.pdf">CMU's Cognitive Robotics posture slides</a>
 *  @todo We should also have an insertMotion()
 */
class MotionSequenceEngine : public LoadSave {
public:
	//!constructor, will start playing immediately
	MotionSequenceEngine();
	//!destructor
	virtual ~MotionSequenceEngine() {}

	//! similar to the MotionCommand::updateOutputs, although this isn't a motion command, and doesn't make any calls on MotionManager - merely updates #lasttime, expects subclasses to do the work of sending new commands to the system
	virtual int updateOutputs();
	
	//!@name LoadSave related
	virtual unsigned int getBinSize() const; //!< inherited, returns the size used to save the sequence
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL); //!< inherited, doesn't clear before loading - call clear yourself if you want to reset, otherwise it will overlay.  Leaves playtime at end of load.
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const; //!< inherited, saves the motion sequence - will save a flat file - doesn't remember references to other files which were loaded
	virtual unsigned int loadFile(const char filename[]); //!< inherited, doesn't clear before loading - call clear yourself if you want to reset, otherwise it will overlay.  Leaves playtime at end of load.
	virtual unsigned int saveFile(const char filename[]) const; //!< inherited, saves the motion sequence - will save a flat file - doesn't remember references to other files which were loaded
	//! deprecated, use radians instead; calling this will store angles as degrees on future saves
	void setSaveDegrees() ATTR_deprecated;
	bool isSaveDegrees() const { return loadSaveMode!=1; } //!< returns true if will store angles as degrees on future saves
	void setSaveRadians() { loadSaveMode=1; }              //!< will store angles as radians on future saves
	bool isSaveRadians() const { return loadSaveMode==1; } //!< returns true if will store angles as degrees on future saves
	//@}
	
	//!@name Sequence Construction
	virtual void clear()=0; //!< empties out the sequence (constant time operation - faster than a series of pops)
	void setTime(unsigned int x); //!< set the time for both playback and editing (in milliseconds)
	unsigned int advanceTime(unsigned int x) {setTime(playtime+x); return playtime; } //!< advance the play/edit index by @a x milliseconds, and then returns the new getTime()
	void setOutputCmd(unsigned int i, const OutputCmd& cmd); //!< will insert a keyframe for the given output, or change an existing one
	const OutputCmd& getOutputCmd(unsigned int i); //!< gets the value of output @a i at the playhead
	void setPose(const PostureEngine& pose); //!< calls setOutputCmd for all non-zero weighted OutputCmds in @a pose (if you wish to set from a file, use loadFile)
	void setExplicitPose(const PostureEngine& pose); //!< calls setOutputCmd on each of the OutputCmds in @a pose, even if they are zero-weight (can be used to fade joints in/out with other conflicting motions)
	PostureEngine getPose(); //!< returns the set of OutputCmd's at the current playhead as a PostureEngine
	void getPose(PostureEngine& pose); //!< stores the set of OutputCmd's at the current playhead into the specified PostureEngine
	unsigned int overlayMotion(const std::string& msFile); //!< loads @a msFile from disk and calls overlayMotion(const MotionSequenceEngine&) with it, returns the duration of @a msFile (0 if there was an error)
	void overlayMotion(const MotionSequenceEngine& ms); //!< applies each keyframe of @a ms to this, leaves playhead at the end (in other words, advances playhead to end of @a ms)
	void compress(); //!< compresses the sequence by eliminating sequences of moves which are identical
	virtual unsigned int getMaxFrames() const=0; //!< returns the maximum number of key frames (Move's) which can be stored, determined by the instantiating MotionSequenceMC's template parameter
	virtual unsigned int getUsedFrames() const=0; //!< returns the number of used key frames (Move's) which have been stored by the instantiation MotionSequenceEngine subclass
	void makeSafe(const float vels[NumOutputs], float margin); //!< will insert time into the motion where needed to keep the joint velocities at or below the speeds given in @a vels * @a margin
	//@}

	//!@name Playback Control
	bool isPlaying();                                     //! returns true if currently playing
	void play();                                          //!< restarts playback from beginning
	void pause() { playing=false; }                       //!< pauses playback until another call to play() or resume()
	void resume();                                        //!< begins playback from the current playtime
	unsigned int getTime() const { return playtime; }     //!< returns the current position of the playback (in milliseconds), see setTime()
	unsigned int getEndTime() const { return endtime; }   //!< returns the length of the motion sequence (in milliseconds)
	void setSpeed(float x) { playspeed=x; }               //!< sets the playback speed (e.g. 1=regular, 0.5=half speed, -1=@b backwards)
	float getSpeed() const { return playspeed; }          //!< returns the playback speed
	
	virtual void setHold(bool h=true) { hold=h; } //!< Sets #hold - if this is set to false, it will allow a persistent motion to behave the same as a pruned motion, without being pruned
	virtual bool getHold() { return hold; }       //!< return #hold
	
	//@}

protected:
	// TYPES:
	typedef unsigned short Move_idx_t; //!< type for indexes to move structures in subclass's storage
	static Move_idx_t invalid_move; //!< used to mark the ends of the Move linked lists

	//! This struct holds all the information needed about a frame for a particular output
	struct Move {
		//!constructor
		Move() : cmd(), next(), prev(), starttime(0) {}
		OutputCmd cmd;           //!< the actual command to use
		Move_idx_t next;        //!< the next frame
		Move_idx_t prev;        //!< the previous frame
		unsigned int starttime; //!< the time (relative to first frame) this frame should be expressed at
	};

	// MEMBERS:
	Move_idx_t starts[NumOutputs]; //!< the beginning frame for each output animation
	Move_idx_t prevs[NumOutputs];  //!< the previous frame (the starttime for this frame will always be less than or equal to playtime)
	Move_idx_t nexts[NumOutputs];  //!< the upcoming frame (the starttime for this frame will always be greater than playtime)
	OutputCmd curs[NumOutputs];          //!< merely a cache of current values (if computed, see #curstamps)
	unsigned int curstamps[NumOutputs]; //!< timestamp of corresponding value in #curs
	unsigned int playtime;              //!< the current time of playback, 0 is start of sequence
	unsigned int lasttime;              //!< the time of the last update
	unsigned int endtime;               //!< max of moves's Move::starttime's
	float playspeed;                    //!< multiplies the difference between current time and starttime, negative will cause play backwards
	bool playing;                       //!< true if playing, false if paused
	bool hold;                          //!< if set to true, the posture will be kept active; otherwise joints will be marked unused after each posture is achieved (as if the posture was pruned); set through setHold()
	
	float loadSaveMode;                 //!< 1 to use radians, M_PI/180 for degrees during a save

	virtual Move& getKeyFrame(Move_idx_t x) =0;            //!< returns the Move struct corresponding to @a x in the subclass's actual data structure
	virtual const Move& getKeyFrame(Move_idx_t x) const=0; //!< returns the Move struct corresponding to @a x in the subclass's actual data structure
	virtual Move_idx_t newKeyFrame()=0;                    //!< causes subclass to create a new Move structure, returns its index
	virtual void eraseKeyFrame(Move_idx_t x)=0;            //!< causes subclass to mark the corresponding Move structure as free

	//!Does the actual calculation of position information.  Perhaps replace with a Bezier or spline or something?
	void calcOutput(OutputCmd& ans, unsigned int t, const Move& prev, const Move& next) const {
		float prevweight=(float)(next.starttime-t)/(float)(next.starttime-prev.starttime);
		ans.set(prev.cmd,next.cmd,prevweight);
	}
	
	//!Sets prev and next to the appropriate values for the given time and output index, return true if there was a change
	virtual bool setRange(unsigned int t,Move_idx_t& prev, Move_idx_t& next) const=0;

	//!sets playtime to next time for which any output has a keyframe, -1 if none exists
	unsigned int setNextFrameTime(Move_idx_t p[NumOutputs], Move_idx_t n[NumOutputs]) const;
	
	//!reads a line from a file, parsing it into variables, returns ending position
	static unsigned int readWord(const char buf[], const char * const buflen, char word[], const unsigned int wordlen);

	//!returns the index for the output named in the string or NumOutputs if not found, begins search through RobotInfo::outputName's at index @a i
	static unsigned int getOutputIndex(const char name[], unsigned int i);
};

/*! @file
 * @brief Describes MotionSequenceEngine, abstract code for smoothly transitioning between a sequence of postures
 * @author ejt (Creator)
 */

#endif
