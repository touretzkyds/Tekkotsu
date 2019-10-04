#include "Kodu/Parsing/Parser.h"

namespace Kodu {
        
    bool Parser::TokenParser::parseTokens(const std::vector<std::string>& koduStrings,
        std::vector<ParsedPage*>& parsedPages)
    {
        const std::string kIndentationMarker = ":";
        unsigned int currentPageIndex = 0;          // states which page is currently being parsed
        bool isFirstEntryInCode = true;             // used to make sure the first line is a PAGE declaration

        // intialize the set container that contains all the keyowrds for Kodu language (that we are using)
        initializeKeywordSet();

        // preconstruct 12 empty pages
        // std::cout << "Constructing the 12 temp pages...";
        for (unsigned int pageNumb = 1; pageNumb <= 12; pageNumb++)
            parsedPages.push_back(new ParsedPage(pageNumb));
        // std::cout << "done.\n";

        // main parsing loop
        const std::size_t kKoduStringsSize = koduStrings.size();
        // std::cout << "Parsing tokens...\n";
        for (std::size_t index = 0; index < kKoduStringsSize; index++) {
            // create error header (to print on stdout)
            std::stringstream lineNumberTemp;
            lineNumberTemp << "Line " << (index + 1) << ": ";
            std::string lineNumber = lineNumberTemp.str();
            //std::string lineNumber = header.str();

            // tokenize the current line
            std::vector<TokenBase*> ruleTokens;
            
            // convert rule into tokens
            // ASSERTIONS: tokenizing process will be successful
            // std::cout << "Creating tokens from string...\n";
            PARSER_ASSERT((tokenize(koduStrings[index], ruleTokens) == true),
                errorMessage << lineNumber << "There was an error tokenizing this line (see above).");
            // std::cout << "Tokens created.\n";

            // make sure this line has at least two tokens or each move on
            if (ruleTokens.empty())
                continue;

            // ASSERTION: the first token on every line should be a string
            PARSER_ASSERT(ruleTokens[0]->isKeywordToken(),
                errorMessage << lineNumber << "The first word of every line should not be "
                << "a Kodu keyword (except blank/empty lines).");

            // reference the current page
            ParsedPage* currPageParsing = parsedPages[currentPageIndex];
            ParsedRule* tempRule;

// ======================================= Page Identifier Check ====================================== //
            // checks the first char of the first token
            // check if it is a page
            if (ruleTokens[0]->getKeywordData() == "PAGE") {
							// std::cout << "Parsing a page...\n";
                // makes sure the first thing declared is a page
                if (isFirstEntryInCode) {
                    isFirstEntryInCode = false;
                }
                
                // ASSERTION: there are only two tokens on this line
                PARSER_ASSERT((static_cast<int>(ruleTokens.size()) == 2),
                    errorMessage << lineNumber << "New page lines should only "
                    << "have the PAGE identifier and a number from 1 - 12.");

                // ASSERTION: the second token is a numeric token
                PARSER_ASSERT(ruleTokens[1]->isNumericToken(),
                    errorMessage << lineNumber << "The page identifier is not a number.");

                // ASSERTION: the numeric token has a value of 1 - 12
                unsigned int pageId = static_cast<int>(ruleTokens[1]->getNumericData());
                PARSER_ASSERT((1 <= pageId && pageId <= 12),
                    errorMessage << lineNumber << "Page number must be 1 through 12.");

                // ASSERTION: there are no rules on this page
                currentPageIndex = pageId - 1;  // states the page that's currently being manipulated
                PARSER_ASSERT((parsedPages[currentPageIndex]->getRuleCount() == 0),
                    errorMessage << lineNumber << "This page already has rules. "
                    << "You cannot add rules to this page again.");
                
                tempRule = NULL;
                currPageParsing = NULL;
                
                // some bookkeeping
                GeneralFncs::destroyAllPtrsInVector(ruleTokens);

                // std::cout << "Finished parsing page declaration: PAGE " << pageId << std::endl;
                // move onto the next iteration
                continue;
            }
            
// ===================================== Indentation Marker Check ===================================== //
            if (ruleTokens[0]->getKeywordData() == kIndentationMarker) {
							// std::cout << "Parsing an indented rule...\n";
                    // ASSERTION: this is not the first line in the document
                PARSER_ASSERT((!isFirstEntryInCode),
                    errorMessage << lineNumber << "A PAGE declaration must be the "
                    << "first thing in the file.");

                // ASSERTION: this is not the first rule on this page
                PARSER_ASSERT((currPageParsing->getRuleCount() > 0),
                    errorMessage << lineNumber << "This is the first rule on the page. "
                    << "It should not be indented.");

                // count the number of indentations on this line
                // NOTE: starting at 1 because we already know index 0 is an indentation marker
                unsigned int indentLvlCount = 1;
                
                // delete and erase the first instance (since we are starting at one)
                // std::cout << "Erased " << indentLvlCount << " indentation(s).\n";
                GeneralFncs::destroyPtrInVector(ruleTokens, 0);
                
                while (indentLvlCount < ruleTokens.size()) {
                    // ASSERTION: there is at least two more tokens on this line (WHEN and DO)
                    PARSER_ASSERT((ruleTokens.size() >= 2),
                        errorMessage << lineNumber << "This line does not seem to have the "
                        << "WHEN and DO identifiers.");
                    
                    // ASSERTION: the current token is a keyword token
                    PARSER_ASSERT(ruleTokens[0]->isKeywordToken(),
                        errorMessage << lineNumber << "Token #" << (indentLvlCount + 1)
                        << " is not valid. It should be either a '" << kIndentationMarker
                        << "' marker or a WHEN identifier.");
                    
                    // found the WHEN identifier!
                    if (ruleTokens[0]->getKeywordData() == "WHEN") {
											// std::cout << "Found a WHEN keyword!\n";
                        break;
                    }
                    
                    // ASSERTION: the token is an indentation marker
                    PARSER_ASSERT((ruleTokens[0]->getKeywordData() == kIndentationMarker),
                        errorMessage << lineNumber << "Token #" << (indentLvlCount + 1)
                        << " should be a '" << kIndentationMarker << "' or a WHEN.");

                    // ASSERTION: the length of this string is one
                    PARSER_ASSERT((ruleTokens[0]->getKeywordData().size() == 1),
                        errorMessage << lineNumber << "Token #" << (indentLvlCount + 1)
                        << " should only have a length of " << kIndentationMarker.size() << ".");

                    // increase the indentation levels
                    indentLvlCount++;

                    // while searching for the WHEN identifier, delete each indentation marker
                    // std::cout << "Erased " << indentLvlCount << " indentation(s).\n";
                    GeneralFncs::destroyPtrInVector(ruleTokens, 0);
                }

                // get the indentation level of the previous rule
                unsigned int prevRuleIndentCount =
                    currPageParsing->getRuleInPos(currPageParsing->getRuleCount() - 1)->
                    getIndentationLevel();

                // ASSERTION: this line has no more than X + 1 indentation levels
                //            (X = previous line indentation level)
                PARSER_ASSERT((indentLvlCount <= prevRuleIndentCount + 1),
                    errorMessage << lineNumber << "There is(are) too many indentation(s) on this line. "
                    << "Max allowable is either " << prevRuleIndentCount
                    << " or " << (prevRuleIndentCount + 1) << " for this line. "
                    << "This line has " << indentLvlCount << ".");

                // set indentation level 
                tempRule = new ParsedRule();
                tempRule->setIndentationLevel(indentLvlCount);

                // set parent rule
                for (int ruleIndex = (currPageParsing->getRuleCount() - 1); ruleIndex >= 0; ruleIndex--)
                {
                    if (currPageParsing->getRuleInPos(ruleIndex)->getIndentationLevel() == indentLvlCount - 1)
                    {
                        tempRule->setParentNumber(ruleIndex + 1);
                        break;
                    }
                }
                // Proceed to the other case
                // std::cout << "Finished handling indentations.\n";
            }
            
// ======================================= When Identifier Check ====================================== //
            if (ruleTokens[0]->getKeywordData() == "WHEN") {
							// std::cout << "Parsing a rule...\n";
                // ASSERTION: this is not the first line in the document
                PARSER_ASSERT((!isFirstEntryInCode),
                    errorMessage << lineNumber <<  "A PAGE declaration must be the "
                    << "first thing in the document.");

                // if tempRule was created before, create a new instance
                if (tempRule == NULL)
                    tempRule = new ParsedRule();

                // ASSERTION: DO is in this token list
                int doPos = 0;
                PARSER_ASSERT(((doPos = contains(ruleTokens, "DO")) > 0),
                    errorMessage << lineNumber << "Cannot find the DO identifier on this line.");

                // create two vectors: one for the condition and the other for the action
                std::vector<TokenBase*> conditionTokens, actionTokens;
                conditionTokens = GeneralFncs::subVector(ruleTokens, 1, doPos);
                actionTokens = GeneralFncs::subVector(ruleTokens, doPos + 1, ruleTokens.size());

                // setup the Phrases for this rule
                ParsedPhrase* condition = new ParsedPhrase();
                ParsedPhrase* action = new ParsedPhrase();

                // check if the condition was empty (implicit always)
                if (!conditionTokens.empty()) {
                    condition->setPhraseHead(conditionTokens[0]);
                    condition->setPhraseModifiers(GeneralFncs::subVector(conditionTokens, 1,
                        conditionTokens.size()));
                } else {
                    condition->setPhraseHead(new KeywordToken("always"));
                }

                // check if the action was empty (implicit do nothing)
                if (!actionTokens.empty()) {
                    action->setPhraseHead(actionTokens[0]);
                    action->setPhraseModifiers(GeneralFncs::subVector(actionTokens, 1, actionTokens.size()));
                } else {
                    action->setPhraseHead(new KeywordToken("do_nothing"));
                }

                // set rule number, condition, and action
                tempRule->setRuleNumber(currPageParsing->getRuleCount() + 1);
                tempRule->setConditionPhrase(condition);
                tempRule->setActionPhrase(action);

                // add this rule to the current page
                currPageParsing->addRule(tempRule);

                // NULL all pointers used (bookkeeping prevents segmentation faults from out of scope)
                condition = NULL;
                action = NULL;
                tempRule = NULL;
                currPageParsing = NULL;
                
                // some bookkeeping
                GeneralFncs::destroyPtrInVector(ruleTokens, doPos);
                GeneralFncs::destroyPtrInVector(ruleTokens, 0);
                
                // move on to the next iteration
                // std::cout << "Finished parsing rule on line " << (index + 1) << ".\n";
                continue;
            }
            
// =================================== Default response if not any of the others ======================== //
            PARSER_ASSERT(ERROR,
            errorMessage << lineNumber << "(Col 1) Unknown token. "
            << "It does not begin with PAGE, indentation markers '"
            << kIndentationMarker << "', a WHEN identifier, or whitespace (tabs, spaces, etc.).");
        }
        
        // clear the set of Kodu keywords (they are no longer needed)
        clearKeywordSet();
        return true;
    }
    
    bool Parser::TokenParser::readText(std::vector<std::string>& koduStrings) {
        const char* fileName = "my.kode";
        std::cout << "============ Reading Kodu code from " << fileName << " ============\n";
        std::string line;
        std::ifstream kodufile(fileName);
        if (kodufile.is_open()) {
            while (kodufile.good()) {
                getline(kodufile, line);
                koduStrings.push_back(line);
            }
            kodufile.close();
            std::cout << "============ Reading Complete ============\n";
        } else {
            std::cerr << "Error reading the file.\n";
            return false;
        }
        return true;
    }
    
    int Parser::TokenParser::contains(const std::vector<TokenBase*>& tokens, const std::string& searchItem) {
        const int kSize = tokens.size();
        for (int i = 0; i < kSize; i++) {
            if (tokens[i]->isKeywordToken() && tokens[i]->getKeywordData() == searchItem)
                return i;
        }
        return -1;
    }
}
