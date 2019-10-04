#include "Kodu/Keepers/ObjectKeeper.h"
#include "Kodu/Primitives/KoduActionGive.h"

namespace Kodu {
    
    void KoduActionGive::reinitialize() {
        KoduAction::reinitialize();
    }

    void KoduActionGive::printAttrs() const {
        KoduAction::printAttrs();
        std::cout << std::endl;
    }

    const DualCoding::ShapeRoot KoduActionGive::getTargetObject() const {
        if (ObjectKeeper::isValid)
            return ObjectKeeper::tempObject;
        else
            return ObjectKeeper::invalidObject;
    }

}
