//-*-c++-*-
#ifndef _StateMachine_h_
#define _StateMachine_h_

#include "Shared/RobotInfo.h"
#include "Shared/mathutils.h"

using namespace mathutils;

#include "Behaviors/StateNode.h"
#include "Behaviors/Nodes/ArmNode.h"
#include "Behaviors/Nodes/HeadPointerNode.h"
#ifdef TGT_HAS_LEDS
#  include "Behaviors/Nodes/LedNode.h"
#endif
#include "Behaviors/Nodes/LGNode.h"
#include "Behaviors/Nodes/LogNode.h"
#include "Behaviors/Nodes/MCNode.h"
#include "Behaviors/Nodes/MotionSequenceNode.h"
#include "Behaviors/Nodes/DynamicMotionSequenceNode.h"
#include "Behaviors/Nodes/OutputNode.h"
#include "Behaviors/Nodes/PIDNode.h"
#include "Behaviors/Nodes/PostureNode.h"
#include "Behaviors/Nodes/RecordMotionNode.h"
#include "Behaviors/Nodes/SoundNode.h"
#include "Behaviors/Nodes/SpeechNode.h"

#include "Behaviors/Transition.h"
#include "Behaviors/Transitions/CompareTrans.h"
#include "Behaviors/Transitions/CompletionTrans.h"
#include "Behaviors/Transitions/EventTrans.h"
#include "Behaviors/Transitions/NullTrans.h"
#include "Behaviors/Transitions/RandomTrans.h"
#include "Behaviors/Transitions/SignalTrans.h"
#include "Behaviors/Transitions/SmoothCompareTrans.h"
#include "Behaviors/Transitions/TextMsgTrans.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Behaviors/Transitions/VisualTargetTrans.h"

#ifdef TARGET_HAS_IR
#  include "Behaviors/Transitions/VisualTargetCloseTrans.h"
#endif

#include "DualCoding/DualCoding.h"
#include "DualCoding/VisualRoutinesStateNode.h"

#include "Crew/MapBuilderNode.h"
#include "Crew/TrackNode.h"

#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
#  include "Crew/Grasper.h"
#  include "Crew/GrasperNode.h"
#  include "Behaviors/Transitions/GrasperTrans.h"
#endif

#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK
#  include "Behaviors/Nodes/WalkToTargetNode.h"
#  include "Behaviors/Nodes/WalkNode.h"
#  include "Behaviors/Nodes/WaypointWalkNode.h"
#  include "Crew/Pilot.h"
#  include "Crew/PilotNode.h"
#  include "Crew/MotionNodes.h"
#  include "Behaviors/Transitions/PilotTrans.h"
#endif

#endif
