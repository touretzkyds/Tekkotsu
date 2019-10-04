#include "Kodu/Parsing/ParsedPage.h"

namespace Kodu {
    
    bool ParsedPage::addRule(ParsedRule* rule) {
        if (rule == NULL)
            return false;
        rules.push_back(rule);
        return true;
    }

    unsigned int ParsedPage::getPageNumber() const {
        return pageNumber;
    }

    unsigned int ParsedPage::getRuleCount() const {
        return rules.size();
    }

    ParsedRule* ParsedPage::getRuleInPos(unsigned int index) {
        if (index >= getRuleCount())
            return NULL;
        return rules[index];
    }
}