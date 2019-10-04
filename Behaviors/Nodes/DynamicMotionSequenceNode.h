//-*-c++-*-
#ifndef _DynamicMotionSequenceNode_h_
#define _DynamicMotionSequenceNode_h_

#include "MCNode.h"
#include "Motion/DynamicMotionSequence.h"
#include "Motion/MotionManager.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"
#include "Shared/MarkScope.h"

//!default name for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defDynamicMotionSequenceNodeName[];
//!default description for PostureNode's (have to instantiate a variable in order to use as a template argument)
/*! instantiation will be placed in MCNode.cc to avoid file bloat */
extern const char defDynamicMotionSequenceNodeDesc[];

//! A StateNode for playing a DynamicMotionSequence (and looping it if desired)

class DynamicMotionSequenceNode : public  MCNode<DynamicMotionSequence,defDynamicMotionSequenceNodeName,defDynamicMotionSequenceNodeDesc,true>  {
public:
	//!constructor
	DynamicMotionSequenceNode()
		: MCNode<DynamicMotionSequence,defDynamicMotionSequenceNodeName,defDynamicMotionSequenceNodeDesc,true>(),
			looping(false), filename() {}

	//!constructor
	DynamicMotionSequenceNode(const std::string& name, const std::string& file="", bool loop=false)
		: MCNode<DynamicMotionSequence,defDynamicMotionSequenceNodeName,defDynamicMotionSequenceNodeDesc,true>(name),
			looping(loop), filename(file) {}

	virtual void preStart() {
		MCNode<DynamicMotionSequence,defDynamicMotionSequenceNodeName,defDynamicMotionSequenceNodeDesc,true>::preStart();
		if(event!=NULL) {
			const DataEvent<std::string>* ev = dynamic_cast<const DataEvent<std::string>*>(event);
			if ( ev != NULL )
				filename = ev->getData();
		}
		update(filename);
	}
	
	virtual void postStart() {
		MCNode<DynamicMotionSequence,defDynamicMotionSequenceNodeName,defDynamicMotionSequenceNodeDesc,true>::postStart();
		getMC()->play();
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
		ASSERTRET(event->getGeneratorID()==EventBase::motmanEGID,"Unknown event");
		if ( event->getSourceID() == MCNodeBase::getMC_ID() ) {
			if(looping)
				getMC()->setTime(1);
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
			MMAccessor<DynamicMotionSequence> dms_acc = getMC();
			dms_acc->clear();
			dms_acc->loadFile(filename.c_str());
			dms_acc->setTime(1);
		}
	}

	bool looping; //!< true if we should loop
	std::string filename; //!< filename of current motion sequence

};

#endif
