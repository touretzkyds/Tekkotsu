#include "Kodu/Primitives/KoduActionPageSwitch.h"

namespace Kodu {
    
    int KoduActionPageSwitch::getPageNumber() {
        return (int)(pageNumb.getNumericValue());
    }

    bool KoduActionPageSwitch::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduActionPageSwitch*>(kPrimitive) != NULL);
    }

    void KoduActionPageSwitch::reinitialize() {
        KoduAction::reinitialize();
    }

    void KoduActionPageSwitch::printAttrs() const {
        KoduAction::printAttrs();
        std::cout << "Page requested: "
                  << const_cast<KoduActionPageSwitch*>(this)->pageNumb.getNumericValue() << std::endl;
    }
}
