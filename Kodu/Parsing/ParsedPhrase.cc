#include "Kodu/Parsing/ParsedPhrase.h"

namespace Kodu {
        
    TokenBase* ParsedPhrase::getPhraseHead() {
        return head;
    }

    std::vector<TokenBase*>& ParsedPhrase::getPhraseModifiers() {
        return modifiers;
    }
    
    bool ParsedPhrase::setPhraseHead(TokenBase* _head) {
        head = _head;
        return (head != NULL);
    }
    
    bool ParsedPhrase::setPhraseModifiers(const std::vector<TokenBase*>& kMods) {
        modifiers = kMods;
        return (!modifiers.empty());
    }
}