#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Keepers/ObjectKeeper.h"
#include "Kodu/Primitives/KoduActionGrab.h"

namespace Kodu {
    
    const DualCoding::ShapeRoot KoduActionGrab::getTargetObject() const {
        if (ObjectKeeper::isValid)
            return ObjectKeeper::tempObject;
        else
            return ObjectKeeper::invalidObject;
    }

    bool KoduActionGrab::isSameTypeAs(const KoduPrimitive* kPrimtive) {
        return (dynamic_cast<const KoduActionGrab*>(kPrimtive) != NULL);
    }

    void KoduActionGrab::reinitialize() {
        KoduAction::reinitialize();
    }

    void KoduActionGrab::printAttrs() const {
        KoduAction::printAttrs();
        PRINT_ATTRS("\"It\" modifier used", usingItModifier);
        std::cout << std::endl;
    }

}
