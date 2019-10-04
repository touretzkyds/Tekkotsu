#include "Kodu/Keepers/ObjectKeeper.h"
#include "Kodu/Primitives/KoduActionDrop.h"

namespace Kodu {
    
    void KoduActionDrop::reinitialize() {
        KoduAction::reinitialize();
    }

    void KoduActionDrop::printAttrs() const {
        KoduAction::printAttrs();
        std::cout << std::endl;
    }

}