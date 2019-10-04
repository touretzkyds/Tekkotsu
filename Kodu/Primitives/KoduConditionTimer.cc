#include "Kodu/KoduWorld.h"
#include "Kodu/Primitives/KoduConditionTimer.h"
#include "Shared/get_time.h"

namespace Kodu {
        
    bool KoduConditionTimer::evaluate(const KoduWorld& kWorldState) {
        // TODO (24/JUL/13) Is needToInitialize necessary?
        if (needToInitialize) {     // do initial setup for timer
            startTime = getTimeInSec();
            timerThreshold = numericGen.getNumericValue();
            needToInitialize = false;
        }
        currentTime = getTimeInSec();
        float timeDifference = currentTime - startTime;
        if (timeDifference >= timerThreshold) {
            timerThreshold = numericGen.getNumericValue();
            startTime = getTimeInSec();
            if (notModifierEnabled) {
                return false;
                
            } else {
                std::cout << "Timer threshold reached [threshold = " << timerThreshold << " secs]\n";
                return true;
            }
        } else if (timeDifference < timerThreshold && notModifierEnabled) {
            std::cout << "Timer threshold NOT reached [threshold = " << timerThreshold << " secs; "
                      << "time elasped = " << timeDifference << " secs]\n";
            return true;
        }
        return false;
    }

    bool KoduConditionTimer::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduConditionTimer*>(kPrimitive) != NULL);
    }

    void KoduConditionTimer::reinitialize() {
        KoduCondition::reinitialize();
        needToInitialize = true;
    }
    
    void KoduConditionTimer::printAttrs() const {
        KoduCondition::printAttrs();
        numericGen.printAttrs();
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        std::cout << "Current time difference: " << (currentTime - startTime) << std::endl;
        std::cout << "Current threshold: " << timerThreshold << std::endl;
    }

    float KoduConditionTimer::getTimeInSec() {
        return (static_cast<float>(get_time()) / 1000.0f);
    }
} // end of Kodu namespace
