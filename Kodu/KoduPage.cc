#include "KoduPage.h"

namespace Kodu {
    
    void KoduPage::addKoduRule(KoduRule* newRule) {
        if (newRule == NULL)
            return;
        rules.push_back(newRule);
    }

    unsigned int KoduPage::getPageNumber() const {
        return pageNumber;
    }
        
    KoduRule* KoduPage::getRule(unsigned int number) {
        return getRuleInPos(number - 1);
    }

    unsigned int KoduPage::getRuleCount() const {
        return rules.size();
    }
    
    KoduRule* KoduPage::getRuleInPos(unsigned int pos) {
        if (pos < getRuleCount())
            return rules[pos];
        else
            return NULL;
    }
/*
    const std::vector<std::string>& KoduPage::getObjectDescriptors() const {
        return objectColors;
    }

    void KoduPage::addObjectDescriptor(const std::string& descriptor) {
        objectColors.push_back(descriptor);
    }
*/
    void KoduPage::setPageRequiresVision(bool bval) {
        pageRequiresVision = bval;
    }
    
    bool KoduPage::requiresVision() const {
        return pageRequiresVision;
        // return (objectColors.size() > 0);
    }
}