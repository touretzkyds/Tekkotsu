// Tekkodu Library
#include "Kodu/Parsing/Parser.h"

namespace Kodu {
    
    bool Parser::isValidColor(const std::string& str) {
        return (koduColorKeywords.count(str) > 0);
    }

    bool Parser::isValidComparisonOperator(const std::string& str) {
        return (koduCompKeywords.count(str) > 0);
    }

    bool Parser::isValidScoreLetter(const std::string& str) {
        return (koduScoreLetterKeywords.count(str) > 0);
    }
}