// Tekkodu Library
#include "Kodu/Parsing/Parser.h"

namespace Kodu {

    std::string Parser::koduDefaultDesignator;
    std::string Parser::koduDefaultCompOperator;

    std::set<std::string> Parser::koduColorKeywords;
    std::set<std::string> Parser::koduCompKeywords;
    std::set<std::string> Parser::koduScoreLetterKeywords;

    void Parser::initializeKeywordsAndDefaults() {
        // assign the default values for comparison operators and designators (colors and scoring letters)
        koduDefaultDesignator = "red";
        koduDefaultCompOperator = "equals";

        // initialize the color name set
        koduColorKeywords.insert("blue");
        koduColorKeywords.insert("green");
        koduColorKeywords.insert(koduDefaultDesignator);
        
        // initialize the comparison operator set
        koduCompKeywords.insert("above");
        koduCompKeywords.insert("below");
        koduCompKeywords.insert(koduDefaultCompOperator);
        koduCompKeywords.insert("not_equals");
        koduCompKeywords.insert(">=");
        koduCompKeywords.insert("<=");

        // initialize the score letter set
        std::string scoreName = "score_?";
        for (int i = 65; i <= 90; i++) {
            scoreName[scoreName.size() - 1] = static_cast<char>(i);
            koduScoreLetterKeywords.insert(scoreName);
        }
    }

    void Parser::clearKeywordsAndDefaults() {
        // clear keywords
        koduColorKeywords.clear();
        koduCompKeywords.clear();
        koduScoreLetterKeywords.clear();

        // clear default values
        koduDefaultDesignator.clear();
        koduDefaultCompOperator.clear();
    }
}