//-*-c++-*-
#ifndef INCLUDED_MotionSequenceNode_h_
#define INCLUDED_MotionSequenceNode_h_

#include "MCNode.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionSequenceMC.h"
#include "Shared/debuget.h"
#include "Shared/MarkScope.h"

//!default name for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defMotionSequenceNodeName[];
//!default description for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defMotionSequenceNodeDesc[];

//! A StateNode for playing a MotionSequence (and looping it if desired)
/*! Eventually, i'd like to just build the looping functionality into
 *  MotionSequence, but in the mean time we have this. */
template<unsigned int SIZE>
class MotionSequenceNode : public  MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>  {
public:
	//!constructor
	MotionSequenceNode()
		: MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>(),
			looping(false), filename() {}

	//!constructor
	MotionSequenceNode(const std::string& name, const std::string& file="", bool loop=false)
		: MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>(name),
			looping(loop), filename(file) {}

	virtual void preStart() {
		MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>::preStart();
		if(const DataEvent<std::string>* ev = dynamic_cast<const DataEvent<std::string>*>(this->event))
			filename = ev->getData();
		update(filename);
	}
	
	virtual void postStart() {
		MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>::postStart();
		MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>::getMC()->play();
	}

	//! sets the file to play
	virtual void setFile(const std::string& file) {
		if ( StateNode::isActive() )
			update(file);
		else
			filename=file;
	}

	//! turns looping on or off
	virtual void setLooping(bool loop) { looping=loop; }

	virtual void doEvent() {
		ASSERTRET(this->event->getGeneratorID()==EventBase::motmanEGID,"Unknown event");
		if ( this->event->getSourceID() == MCNodeBase::getMC_ID() ) {
			if(looping)
				MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>::getMC()->setTime(1);
			StateNode::postStateCompletion();
		}
	}

	//! returns true if currently looping
	virtual bool getLooping() { return looping; }

protected:

	//! resets the motion command and starts it playing
	void update(const std::string& file) {
		if ( file.size() > 0 ) {
			filename=file;
			MMAccessor<MotionSequenceMC<SIZE> > mma = MCNode<MotionSequenceMC<SIZE>,defMotionSequenceNodeName,defMotionSequenceNodeDesc,true>::getMC();
			mma->clear();
			mma->loadFile(filename.c_str());
			mma->setTime(1);
		}
	}

	bool looping; //!< true if we should loop
	std::string filename; //!< filename of current motion sequence
};

typedef MotionSequenceNode<TinyMotionSequenceMC::CAPACITY> TinyMotionSequenceNode; //!< streamlined access to the standard template sizes
typedef MotionSequenceNode<SmallMotionSequenceMC::CAPACITY> SmallMotionSequenceNode; //!< streamlined access to the standard template sizes
typedef MotionSequenceNode<MediumMotionSequenceMC::CAPACITY> MediumMotionSequenceNode; //!< streamlined access to the standard template sizes
typedef MotionSequenceNode<LargeMotionSequenceMC::CAPACITY> LargeMotionSequenceNode; //!< streamlined access to the standard template sizes
typedef MotionSequenceNode<XLargeMotionSequenceMC::CAPACITY> XLargeMotionSequenceNode; //!< streamlined access to the standard template sizes

/*! @file
 * @brief Describes MotionSequenceNode, a StateNode for playing a MotionSequence (and looping it if desired)
 * @author ejt (Creator)
 */

#endif
