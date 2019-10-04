#ifndef PARSED_PAGE_H_
#define PARSED_PAGE_H_

// C++ Library
#include <vector>

// Kodu Library
#include "Kodu/General/GeneralFncs.h"
#include "Kodu/Parsing/ParsedRule.h"

namespace Kodu {

    class ParsedPage {
    private:
        unsigned int pageNumber;
        std::vector<ParsedRule*> rules;

    public:
        ParsedPage(unsigned int pageNumb)
          : pageNumber(pageNumb), rules()
        { }
        
        explicit ParsedPage(const ParsedPage& kPage)
          : pageNumber(kPage.pageNumber), rules(kPage.rules)
        { }

        ~ParsedPage() {
            GeneralFncs::destroyAllPtrsInVector(rules);
        }
        
        ParsedPage& operator=(const ParsedPage& kPage) {
            if (this != &kPage) {
                pageNumber = kPage.pageNumber;
                rules = kPage.rules;
            }
            return *this;
        }
        
        //! Adds a rule to this page
        bool addRule(ParsedRule* rule);

        //! Get the current page's id
        unsigned int getPageNumber() const;

        //! Get the number of rules this page has
        unsigned int getRuleCount() const;

        //! Get the rule in a specified index
        ParsedRule* getRuleInPos(unsigned int index);
    };
}

#endif // PARSED_PAGE_H_