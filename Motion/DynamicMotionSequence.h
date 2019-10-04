//-*-c++-*-
#ifndef INCLUDED_DynamicMotionSequence_h_
#define INCLUDED_DynamicMotionSequence_h_

#include "MotionSequenceEngine.h"
#include "MotionCommand.h"
#include "MotionManager.h"
#include "Events/EventBase.h"
#include <vector>

//! Uses STL's vector for dynamic memory allocation - don't use this as a motion command, pointers in shared memory regions can be invalid in other processes
/*! See MotionSequenceEngine for documentation on its members */
class DynamicMotionSequence : public MotionCommand, public MotionSequenceEngine {
public:

	//!constructor
	DynamicMotionSequence() : MotionCommand(), MotionSequenceEngine(), moves(), erased(), dirty(false) { clear(); }

	//!constructor, loads from a file and then resets the playtime to beginning and begins to play
	explicit DynamicMotionSequence(const char* filename) : MotionCommand(), MotionSequenceEngine(), moves(), erased(), dirty(false) {
		clear();
		loadFile(filename);
		setTime(1);
	}

	//!destructor
	virtual ~DynamicMotionSequence() {}

	// I put this here because i want direct access to moves so it'll be faster
/*	void planJointCmds(unsigned int i, JointCmd frames[NumFrames]) {
		Move_idx_t prev=prevs[i],next=nexts[i];
		frames[0]=getJointCmd(i);
		for(unsigned int t=playtime+FrameTime,j=1;j<NumFrames;j++,t+=FrameTime) {
			setRange(t,prev,next);
			if(next!=invalid_move)
				calcJoint(frames[j],t,moves[prev],moves[next]);
			else
				frames[j]=unusedJoint;
		}
	}*/
	

	virtual int isDirty() { return isPlaying(); }
	virtual int isAlive() { return (playspeed>0) ? (playtime<=endtime) : (playtime>0); }
	
	void setDirty() { dirty = true; };

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
			for(unsigned int i=0; i<NumOutputs; i++) { //fill out the buffer of commands for smoother movement
				Move_idx_t prev=prevs[i],next=nexts[i];
				OutputCmd frames[NumFrames];
				frames[0]=getOutputCmd(i);
				for(unsigned int t=playtime+FrameTime,j=1;j<NumFrames;j++,t+=FrameTime) {
					setRange(t,prev,next);
					if(next!=invalid_move)
						calcOutput(frames[j],t,moves[prev],moves[next]);
					else
						frames[j].unset();
				}
				motman->setOutput(this,i,frames);
			}
		}
		return NumOutputs;
		//		if(i<NumLegJointss)
		//			log.push_back(logent(get_time(),playtime,i,frames));
	}
	
	virtual void clear() {
		moves.clear();
		erased.clear();
		for(unsigned int i=0; i<NumOutputs; i++) {
			moves.push_back(Move());
			moves.back().cmd.unset();
			moves.back().next=invalid_move;
			moves.back().prev=invalid_move;
			prevs[i]=starts[i]=moves.size()-1;
			nexts[i]=invalid_move;
		}
		setTime(1);
	}
	virtual unsigned int getMaxFrames() const { return -1U; }
	virtual unsigned int getUsedFrames() const { return moves.size()-erased.size(); }

protected:
	// TYPES:
	typedef std::vector<Move> list_t; //!< shorthand for the ListMemBuf that stores all of the movement frames

	// MEMBERS:
	list_t moves;                     //!< stores all of the movement keyframes
	std::vector<Move_idx_t> erased;   //!< recycles erased keyframes, can't just shift elements in #moves, it would throw off index numbers in Move structures
	bool dirty; //!< true if last updateOutputs was dirty, so we know when to post status event

	virtual Move& getKeyFrame(Move_idx_t x) { return moves[x]; } //!< returns #moves[@a x]
	virtual const Move& getKeyFrame(Move_idx_t x) const { return moves[x]; } //!< returns #moves[@a x]
	virtual Move_idx_t newKeyFrame() {
		if(erased.empty()) {
			moves.push_back(Move());
			return moves.size()-1;
		} else { //recycle from used list
			Move_idx_t x=erased.back();
			erased.pop_back();
			return x;
		}
	}
	//! marks keyframe @a x unused
	virtual void eraseKeyFrame(Move_idx_t x) { erased.push_back(x); }
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

/*! @file
 * @brief Uses STL's vector for dynamic memory allocation - don't use this as a motion command, pointers in shared memory regions can be invalid in other processes
 * @author ejt (Creator)
 */

#endif
