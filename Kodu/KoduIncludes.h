/// ===================== KODU PRIMITIVES ===================== ///
#ifndef KODU_PRIMITIVE_INCLUDES_H_
#define KODU_PRIMITIVE_INCLUDES_H_

#include "Kodu/KoduConfig.h"
#include "Kodu/KoduDiscover.h"

// actions
#include "Kodu/Primitives/KoduActionDrop.h"
#include "Kodu/Primitives/KoduActionGive.h"
#include "Kodu/Primitives/KoduActionGrab.h"
#include "Kodu/Primitives/KoduActionMotion.h"
#include "Kodu/Primitives/KoduActionPageSwitch.h"
#include "Kodu/Primitives/KoduActionSay.h"

// conditions
#include "Kodu/Primitives/KoduConditionAlways.h"
#include "Kodu/Primitives/KoduConditionBump.h"
#include "Kodu/Primitives/KoduConditionSee.h"
#include "Kodu/Primitives/KoduConditionGot.h"
#include "Kodu/Primitives/KoduConditionGamepad.h"
#include "Kodu/Primitives/KoduConditionHear.h"


#endif // KODU_PRIMITIVE_INCLUDES_H_


/// ===================== PERCEPTUAL TASKS ===================== ///
#ifndef KODU_PERCEPTUAL_TASKS_INCLUDES_H_
#define KODU_PERCEPTUAL_TASKS_INCLUDES_H_

#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"
#include "Kodu/PerceptualTasks/VisualBumpDetectionTask.h"
#include "Kodu/PerceptualTasks/GripperVisualMonitorTask.h"
#include "Kodu/PerceptualTasks/VisualLocalizationTask.h"
#include "Kodu/PerceptualTasks/VisualNavErrMonTask.h"

#endif // KODU_PERCEPTUAL_TASKS_INCLUDES_H_


/// ===================== KODU STATES ===================== ///
#ifndef KODU_STATE_INCLUDES_H_
#define KODU_STATE_INCLUDES_H_

#include "Kodu/KoduAgent.h"
#include "Kodu/KoduWorld.h"
#include "Kodu/PosOrientState.h"

#endif // KODU_STATE_INCLUDES_H_
