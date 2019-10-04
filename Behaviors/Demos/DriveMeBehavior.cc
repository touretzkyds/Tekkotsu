#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS)

#include "Motion/MotionManager.h"
#include "Motion/WalkMC.h"
#include "Motion/MotionSequenceMC.h"
#include "Motion/PostureEngine.h"
#include "IPC/SharedObject.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"

#include "DriveMeBehavior.h"

#include <iostream>
#include <string>
#include <stdio.h>

/* A very simple behavior that asks the user for WalkMC walking parameters
 * and a walk duration. The AIBO walks accordingly and then stands up, then
 * asks again. And so on and so on. */

// ctor
DriveMeBehavior::DriveMeBehavior()
  : BehaviorBase("DriveMeBehavior"),
    walker_id(MotionManager::invalid_MC_ID),
    stand_id(MotionManager::invalid_MC_ID),
    stand(),
    last_dx(0), last_dy(0), last_da(0), last_time(5000)
{
  // Construct the standing up motion that the aibo does after it's done moving
  stand->setTime(700); // 700 milliseconds to stand up
  stand->setPose(PostureEngine("stand.pos"));
}

void DriveMeBehavior::doStart()
{
  BehaviorBase::doStart();
  // Insert walker into motion manager
  walker_id = motman->addPersistentMotion(SharedObject<WalkMC>());
  // Insert standing pose into motion manager
  stand_id = motman->addPersistentMotion(stand, MotionManager::kStdPriority+1);
  // We listen to timers
  erouter->addListener(this, EventBase::timerEGID);
  // Turn on timer that goes off now to take us immediately to processEvent
  erouter->addTimer(this, 0, 0, false);
}

void DriveMeBehavior::doStop()
{
  BehaviorBase::doStop();
  // We're not listening to timers anymore.
  erouter->removeTimer(this);
  // We're not listening to *anything* anymore! (also kills timers on its own)
  erouter->removeListener(this);
  // remove walker and stander from motion manager
  motman->removeMotion(walker_id);
  motman->removeMotion(stand_id);
}

// The only events we'll ever get are timer events.
void DriveMeBehavior::doEvent()
{
  using namespace std;

  WalkMC *walker;
  SmallMotionSequenceMC *standp;

  // oh, OK, make sure it's a timer event
  if(event->getGeneratorID() != EventBase::timerEGID) return;

  // Check out walker and stop us moving
  walker = (WalkMC*)motman->checkoutMotion(walker_id);
  walker->setTargetVelocity(0,0,0);
  motman->checkinMotion(walker_id);

  // Stand us up right now.
  standp = (SmallMotionSequenceMC*)motman->checkoutMotion(stand_id);
  standp->play();
  motman->checkinMotion(stand_id);

  // get new motions
  string instr;
  cout << "dx? [" << last_dx << "] >\t";
  cout.flush();
  getline(cin, instr);
  sscanf(instr.data(), "%f", &last_dx);

  cout << "dy? [" << last_dy << "] >\t";
  cout.flush();
  getline(cin, instr);
  sscanf(instr.data(), "%f", &last_dy);

  cout << "da? [" << last_da << "] >\t";
  cout.flush();
  getline(cin, instr);
  sscanf(instr.data(), "%f", &last_da);

  cout << "time? [" << last_time << "] >\t";
  cout.flush();
  getline(cin, instr);
  sscanf(instr.data(), "%u", &last_time);

  // Start moving again; start timer; check in walker
  walker = (WalkMC*)motman->checkoutMotion(walker_id);
  walker->setTargetVelocity(last_dx, last_dy, last_da);
  erouter->addTimer(this, 0, last_time, false);
  motman->checkinMotion(walker_id);
}

/*! @file
 * @brief Implements DriveMeBehavior, a very simple behavior that asks the user for WalkMC walking parameters and a walk duration.
 * @author tss (Creator)
 */

#endif
