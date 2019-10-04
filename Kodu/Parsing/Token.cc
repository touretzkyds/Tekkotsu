#include "Kodu/Parsing/Token.h"

namespace Kodu {
        
    bool TokenBase::isKeywordToken() const {
        return (dynamic_cast<const KeywordToken*>(this) != NULL);
    }

    bool TokenBase::isNumericToken() const {
        return (dynamic_cast<const NumericToken*>(this) != NULL);
    } 

    bool TokenBase::isStringToken() const {
        return (dynamic_cast<const StringToken*>(this) != NULL);
    }

    const std::string& TokenBase::getKeywordData() const {
        return dynamic_cast<const KeywordToken*>(this)->getData();
    }

    const float& TokenBase::getNumericData() const{
        return dynamic_cast<const NumericToken*>(this)->getData();
    }

    const std::string& TokenBase::getStringData() const {
        return dynamic_cast<const StringToken*>(this)->getData();
    }
}