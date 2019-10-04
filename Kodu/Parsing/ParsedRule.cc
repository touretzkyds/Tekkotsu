#include "Kodu/Parsing/ParsedRule.h"

namespace Kodu {
        
    ParsedPhrase* ParsedRule::getActionPhrase() {
        return action;
    }

    ParsedPhrase* ParsedRule::getConditionPhrase() {
        return condition;
    }

    unsigned int ParsedRule::getIndentationLevel() {
        return indentationLevel;
    }

    unsigned int ParsedRule::getParentNumber() {
        return parentNumber;
    }

    unsigned int ParsedRule::getRuleNumber() {
        return ruleNumber;
    }

    bool ParsedRule::setActionPhrase(ParsedPhrase* actionPhrase) {
        if (actionPhrase != NULL) {
            action = actionPhrase;
        }
        return (action != NULL);
    }

    bool ParsedRule::setConditionPhrase(ParsedPhrase* conditionPhrase) {
        if (conditionPhrase != NULL) {
            condition = conditionPhrase;
        }
        return (condition != NULL);
    }

    void ParsedRule::setIndentationLevel(unsigned int amount) {
        indentationLevel = amount;
    }

    void ParsedRule::setParentNumber(unsigned int parentId) {
        parentNumber = parentId;
    }

    void ParsedRule::setRuleNumber(unsigned int ruleId) {
        ruleNumber = ruleId;
    }
}