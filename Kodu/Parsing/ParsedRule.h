#ifndef PARSED_RULE_H_
#define PARSED_RULE_H_

#include "Kodu/Parsing/ParsedPhrase.h"

namespace Kodu {
        
    class ParsedRule {
    private:
        unsigned int indentationLevel;
        unsigned int ruleNumber;
        unsigned int parentNumber;
        ParsedPhrase* condition;
        ParsedPhrase* action;

    public:
        //! Constructor
        ParsedRule()
          : indentationLevel(0), ruleNumber(0), parentNumber(0),
            condition(NULL), action(NULL)
        { }
        
        //! Copy Constructor
        explicit ParsedRule(const ParsedRule& kRule)
          : indentationLevel(kRule.indentationLevel), ruleNumber(kRule.ruleNumber),
            parentNumber(kRule.parentNumber), condition(kRule.condition), action(kRule.action)
        { }
        
        //! Destructor
        ~ParsedRule() {
            if (condition != NULL) {
                delete condition;
                condition = NULL;
            }
            if (action != NULL) {
                delete action;
                action = NULL;
            }
        }
        
        //! Assignment operator
        ParsedRule& operator=(const ParsedRule& kRule) {
            if (this != &kRule) {
                indentationLevel = kRule.indentationLevel;
                ruleNumber = kRule.ruleNumber;
                parentNumber = kRule.parentNumber;
                condition = kRule.condition;
                action = kRule.action;
            }
            return *this;
        }
        
        //! Returns the action phrase
        ParsedPhrase* getActionPhrase();
        
        //! Returns the condition phrase
        ParsedPhrase* getConditionPhrase();
        
        //! Returns this rule's indentation level
        unsigned int getIndentationLevel();
        
        //! Returns this rule's parent number (a rule id)
        unsigned int getParentNumber();
        
        //! Returns this rule's id (number)
        unsigned int getRuleNumber();
        
        //! Sets this rule's action phrase
        bool setActionPhrase(ParsedPhrase* actionPhrase);
        
        //! Sets this rule's condition phrase
        bool setConditionPhrase(ParsedPhrase* conditionPhrase);
        
        //! Sets this rule's indentation level
        void setIndentationLevel(unsigned int amount);
        
        //! Sets this rule's parent number (a rule id)
        void setParentNumber(unsigned int parentId);
        
        //! Sets this rule's id (number)
        void setRuleNumber(unsigned int ruleId);
    };
}

#endif // PARSED_RULE_H_