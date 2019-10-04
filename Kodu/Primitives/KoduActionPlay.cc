#include "Kodu/Primitives/KoduActionPlay.h"

namespace Kodu {
    
    const std::string& KoduActionPlay::getSoundFile() {
        return soundFile.getLiteralString();
    }

    bool KoduActionPlay::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduActionPlay*>(kPrimitive) != NULL);
    }

    void KoduActionPlay::reinitialize() {
        KoduAction::reinitialize();
    }
    
    void KoduActionPlay::printAttrs() const {
        KoduAction::printAttrs();
        soundFile.printAttrs();
    }
}
