#include "Kodu/Primitives/KoduActionSay.h"

namespace Kodu {

    const std::string& KoduActionSay::getStringToSpeak() {
        return literalGen.getLiteralString();
    }
    
    bool KoduActionSay::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduActionSay*>(kPrimitive) != NULL);
    }

    void KoduActionSay::reinitialize() {
        KoduAction::reinitialize();
    }

    void KoduActionSay::printAttrs() const {
        KoduAction::printAttrs();
        std::cout << "Speech designator: " << designator << std::endl;
        literalGen.printAttrs();
        std::cout << std::endl;
    }
}
