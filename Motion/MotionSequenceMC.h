//-*-c++-*-
#ifndef INCLUDED_MotionSequenceMC_h_
#define INCLUDED_MotionSequenceMC_h_

#include "MotionSequenceEngine.h"
#include "MotionCommand.h"
#include "MotionManager.h"
#include "Events/EventBase.h"

//! Instantiates MotionSequenceEngines - when you want to run a motion sequence, make one of these
/*! Allows a compile-time variable amount of data storage through its template parameter.
 *  @see MotionSequenceEngine for the majority of the usage documentation
 *  @see TinyMotionSequenceMC, SmallMotionSequenceMC, MediumMotionSequenceMC, LargeMotionSequenceMC, XLargeMotionSequenceMC
 *  */
template<unsigned int MAXMOVE>
class MotionSequenceMC : public MotionCommand, public MotionSequenceEngine {
public:
	static const unsigned int CAPACITY=MAXMOVE; //!< allows recovery of capacity in a general way (MAXMOVE may, and probably will, be obscured by a typedef)

	//!constructor
	MotionSequenceMC()
		: MotionCommand(), MotionSequenceEngine(), moves(), dirty(false)
	{
		clear();
	}
	//!constructor, loads from a file and then resets the playtime to beginning and begins to play
	explicit MotionSequenceMC(const std::string& filename)
		: MotionCommand(), MotionSequenceEngine(), moves(), dirty(false)
	{
		clear();
		loadFile(filename.c_str());
		setTime(1);
	}
	//!destructor
	virtual ~MotionSequenceMC() {}

	virtual int isDirty() { return isPlaying(); }
	virtual int isAlive() { return (playspeed>0) ? (playtime<=endtime) : (playtime>0); }
	
	virtual void setDirty() { dirty = true; }

	// I put this here because i want direct access to moves so it'll be faster
	virtual int updateOutputs() {
		MotionSequenceEngine::updateOutputs();
		if(!isPlaying()) {
			if(dirty)
				postEvent(EventBase(EventBase::motmanEGID,getID(),EventBase::statusETID));
			dirty=false;
			for(unsigned int i=0; i<NumOutputs; i++) //just copies getOutputCmd(i) across frames
				motman->setOutput(this,i,getOutputCmd(i));
		} else {
			dirty=true;
			//cout << getTime() << ": ";
			for(unsigned int i=0; i<NumOutputs; i++) { //fill out the buffer of commands for smoother movement
				Move_idx_t prev=prevs[i],next=nexts[i];
				OutputCmd frames[NumFrames];
				frames[0]=getOutputCmd(i);
				for(unsigned int t=playtime+FrameTime,j=1;j<NumFrames;j++,t+=FrameTime) {
					setRange(t,prev,next);
					if(next!=invalid_move)
						calcOutput(frames[j],t,moves[prev],moves[next]);
					else if(hold)
						frames[j]=moves[prev].cmd;
				}
				/*if(i==RFrLegOffset+RotatorOffset)
					for(unsigned int j=0; j<NumFrames; j++)
					cout << '(' << frames[j].value << ',' << frames[j].weight << ") "; */
				motman->setOutput(this,i,frames);
			}
			//cout <<'\n'<< flush;
		}
		return NumOutputs;
		//		if(i<NumLegJointss)
		//			log.push_back(logent(get_time(),playtime,i,frames));
	}
	
	virtual void clear() {
		moves.clear();
		for(unsigned int i=0; i<NumOutputs; i++) {
			prevs[i]=starts[i]=moves.new_back();
			moves.back().cmd.unset();
			moves.back().next=invalid_move;
			moves.back().prev=invalid_move;
			moves.back().starttime=0;
			nexts[i]=invalid_move;
		}
		endtime=0;
		setTime(1);
	}

	virtual unsigned int getMaxFrames() const { return moves.getMaxCapacity(); }
	virtual unsigned int getUsedFrames() const { return moves.size(); }

protected:
	// TYPES:
	typedef ListMemBuf<Move,MAXMOVE,Move_idx_t> list_t; //!< shorthand for the ListMemBuf that stores all of the movement frames

	// MEMBERS:
	list_t moves; //!< stores all of the movement keyframes
	bool dirty; //!< true if last updateOutputs was dirty, so we know when to post status event

	virtual Move& getKeyFrame(Move_idx_t x) { return moves[x]; } //!< returns #moves[@a x]
	virtual const Move& getKeyFrame(Move_idx_t x) const { return moves[x]; } //!< returns #moves[@a x]
	virtual Move_idx_t newKeyFrame() {
		Move_idx_t i=moves.new_front();
		if(i==invalid_move)
			std::cerr << "ERROR: MotionSequenceMC " << getID() << " has run out of memory" << std::endl;
		return i;
	}
	//! marks keyframe @a x unused
	virtual void eraseKeyFrame(Move_idx_t x) { moves.erase(x); }
	//! advances (or rewinds) @a prev and @a next so that @a t falls between them
	bool setRange(unsigned int t,Move_idx_t& prev, Move_idx_t& next) const {
		bool moved=false;
		if(next!=invalid_move && moves[next].starttime<=t) {
			moved=true;
			do {
				prev=next;
				next=moves[prev].next;
			} while(next!=invalid_move && moves[next].starttime<=t);
		} else {
			while(moves[prev].prev!=invalid_move && t<moves[prev].starttime) {
				next=prev;
				prev=moves[next].prev;
				moved=true;
			} 
		}
		return moved;
	}
};

typedef MotionSequenceMC<NumOutputs*2> TinyMotionSequenceMC; //!< Tiny, but enough to handle a transition into a full-body pose
typedef MotionSequenceMC<NumOutputs*3> SmallMotionSequenceMC;//!< Small, but still big enough to handle most of the included MS's (2 full-body frames ~ around 1KB)
typedef MotionSequenceMC<NumOutputs*6> MediumMotionSequenceMC;//!< Medium (5 full body frames ~ est 4KB)
typedef MotionSequenceMC<NumOutputs*11> LargeMotionSequenceMC;//!< Large (10 full body frames ~ est 8KB)
typedef MotionSequenceMC<NumOutputs*26> XLargeMotionSequenceMC;//!< eXtra Large (25 full body frames ~ est 16KB)

/*! @file
 * @brief Describes MotionSequenceEngine and defines MotionSequenceMC, handy little (or not so little) classes for switching between a sequence of postures
 * @author ejt (Creator)
 */

#endif
