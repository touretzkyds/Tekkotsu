#ifndef PARSER_H_
#define PARSER_H_

// C++ Library
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

// Tekkodu Library
// Primitives
// Actions
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/Primitives/KoduActionDoNothing.h"
#include "Kodu/Primitives/KoduActionDrop.h"
#include "Kodu/Primitives/KoduActionGrab.h"
#include "Kodu/Primitives/KoduActionMotion.h"
#include "Kodu/Primitives/KoduActionPageSwitch.h"
#include "Kodu/Primitives/KoduActionPlay.h"
#include "Kodu/Primitives/KoduActionSay.h"
#include "Kodu/Primitives/KoduActionScore.h"
#include "Kodu/Primitives/KoduActionGive.h"
// Conditions
#include "Kodu/Primitives/KoduCondition.h"
#include "Kodu/Primitives/KoduConditionAlways.h"
#include "Kodu/Primitives/KoduConditionBump.h"
#include "Kodu/Primitives/KoduConditionGot.h"
#include "Kodu/Primitives/KoduConditionSee.h"
#include "Kodu/Primitives/KoduConditionScored.h"
#include "Kodu/Primitives/KoduConditionTimer.h"
#include "Kodu/Primitives/KoduConditionGamepad.h"
#include "Kodu/Primitives/KoduConditionHear.h"

#include "Kodu/Primitives/PerceptionSearch.h"

// General Functions
#include "Kodu/General/GeneralFncs.h"
#include "Kodu/General/GeneralMacros.h"

// Generators
#include "Kodu/Generators/KoduGenerators.h"

// Parsing
#include "Kodu/Parsing/ParsedPage.h"
#include "Kodu/Parsing/ParsedPhrase.h"
#include "Kodu/Parsing/ParsedRule.h"
#include "Kodu/Parsing/Token.h"

#include "Kodu/KoduPage.h"
#include "Kodu/KoduRule.h"

namespace Kodu {

    class Parser {
    public:
        #define RED_ERR "\x1b[31m"
        #define YEL_ERR "\x1b[33m"
        #define NON_ERR "\x1b[0m"

        #define ERROR   false

        #define PARSER_ASSERT(CONDITION, ERROR_STATEMENT)                           \
            if ((CONDITION) == false) {                                             \
                std::stringstream errorMessage;                                     \
                ERROR_STATEMENT;                                                    \
                std::cerr << RED_ERR << errorMessage.str() << NON_ERR << std::endl; \
                return NULL;															                          \
            }

        //! Used to create the pages, rules, conditions, and actions of a Kodu program
        class KodeCreator {
        public:
            //! Takes the parsed tokens and creates a Kodu game
            static bool createKode(const std::vector<ParsedPage*>&, std::vector<KoduPage*>&);

        private:
            //! Used to call the appropriate Kode action parser depending on the Phrase head
            static KoduAction* getActionKode(ParsedPhrase*);

            //! Used to call the appropriate Kode condition parser depending on the Phrase head
            static KoduCondition* getConditionKode(ParsedPhrase*);

            //! Checks if a modifier is a comparison operator
            static bool isComparisonOperator(TokenBase*);

            //! Checks if a modifier is a numeric specifier (random modifier or number(s))
            static bool isNumericSpecifier(TokenBase*);

            //! Checks if a modifier is a score designator
            static bool isScoreDesignator(TokenBase*);

            //! Creates a Numeric Generator from numeric specifiers (random modifier and/or number(s))
            static bool numericGenParser(std::vector<TokenBase*>&, NumericGenerator&);
            
            // action parsers
            //! Creates the drop action
            static KoduActionDrop* createDropKode(std::vector<TokenBase*>&);

            //! Creates the give action
            static KoduActionGive* createGiveKode(std::vector<TokenBase*>&);

            //! Creates the grab action
            static KoduActionGrab* createGrabKode(std::vector<TokenBase*>&);

            //! Creates the move action
            static KoduActionMotion* createMoveKode(std::vector<TokenBase*>&);

            //! Creates the play action
            static KoduActionPlay* createPlayKode(std::vector<TokenBase*>&);
            
            //! Creates the say action
            static KoduActionSay* createSayKode(std::vector<TokenBase*>&);

            //! Creates the score action
            static KoduActionScore* createScoreKode(const std::string&, std::vector<TokenBase*>&);

            //! Creates the move action
            static KoduActionMotion* createTurnKode(std::vector<TokenBase*>&);

            // condition parsers
            //! Creates the bump condition
            static KoduConditionBump* createBumpKode(std::vector<TokenBase*>&);

            //! Creates the got condition
            static KoduConditionGot* createGotKode(std::vector<TokenBase*>&);

            //! Creates the see condition
            static KoduConditionSee* createSeeKode(std::vector<TokenBase*>&);

            //! Creates the scored condition
            static KoduConditionScored* createScoredKode(std::vector<TokenBase*>&);
            
            //! Creates the timer condition
            static KoduConditionTimer* createTimerKode(std::vector<TokenBase*>&);
	    
	    //! Create Gamepad Button Condition
	    static KoduConditionGamepad* createGamepadKode(std::vector<TokenBase*>&);
	    
	    static KoduConditionHear* createHearKode(std::vector<TokenBase*>&);
	    

            //! Disallows anyone from creating an instance of this class
            DISALLOW_INSTANTIATION(KodeCreator);
        };

        class TokenParser {
        public:
            //! Parses the retrieved Kodu code into the appropriate data structures
            static bool parseTokens(const std::vector<std::string>&, std::vector<ParsedPage*>&);
        
            //! Reads the Kodu written code from a file
            static bool readText(std::vector<std::string>&);

        private:
            //! Contains all the allowable Kodu "keywords"
            static std::set<std::string> koduKeywords;

            //! Returns the index of the search item
            static int contains(const std::vector<TokenBase*>&, const std::string&);
            
            //! Returns a vector of tokens parsed from a string
            static bool tokenize(const std::string&, std::vector<TokenBase*>&);

            // ==============================
            //! Clear the keyword set (it's no longer needed after parsing)
            static void clearKeywordSet();

            //! Initializes the keyword set
            static void initializeKeywordSet();

            //! Disallows anyone from creating an instance of this class
            DISALLOW_INSTANTIATION(TokenParser);
        };

        //! Initializes all the keyword sets and default string values
        static void initializeKeywordsAndDefaults();

        //! Clears all the keyword sets and default string values
        static void clearKeywordsAndDefaults();

        //! Checks if a specified string is a valid color
        static bool isValidColor(const std::string&);

        //! Checks if a specified string is a valid comparison operator
        static bool isValidComparisonOperator(const std::string&);

        //! Checks if a specified string is a valid score letter
        static bool isValidScoreLetter(const std::string&);

        //! Contains all the allowable color names
        static std::set<std::string> koduColorKeywords;

        //! Contains all the allowable comparison operators
        static std::set<std::string> koduCompKeywords;

        //! Contains all the allowable score letters (such as Score_A, Score_B ... Score_Y, Score_Z)
        static std::set<std::string> koduScoreLetterKeywords;

        static std::string koduDefaultDesignator;
        static std::string koduDefaultCompOperator;

        //! Disallows anyone from creating an instance of this class
        DISALLOW_INSTANTIATION(Parser);

    //public:
        static bool parseAndCreateKoduProgram(std::vector<KoduPage*>& pages) {
            std::vector<std::string> koduStrings;
            std::vector<Kodu::ParsedPage*> tempPages;
            
            initializeKeywordsAndDefaults();

            if (TokenParser::readText(koduStrings) == false) {
                std::cerr << "File reading failed.\n";
                return false;
            }
            
            // parses the code
            if (TokenParser::parseTokens(koduStrings, tempPages) == false) {
                std::cerr << "Code parsing failed.\n";
                return false;
            }

            // create the Kodu pages, rules, conditions, and actions
            if (KodeCreator::createKode(tempPages, pages) == false) {
                std::cerr << "Kode creation failed.\n";
                return false;
            }

            clearKeywordsAndDefaults();
            return true;
        }
    };
}

# endif // PARSER_H_
