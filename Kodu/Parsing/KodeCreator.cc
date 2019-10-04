#include "Kodu/Parsing/Parser.h"
#include "Kodu/Primitives/KoduCondition.h"
#include "Kodu/Primitives/KoduAction.h"

namespace Kodu {
        
    bool Parser::KodeCreator::createKode(const std::vector<ParsedPage*>& tempPages,
        std::vector<KoduPage*>& koduPages)
    {
        // main kode parsing loop
        const std::size_t kTempPagesSize = tempPages.size();
        for (std::size_t pgIndex = 0; pgIndex < kTempPagesSize; pgIndex++) {
            // get the current page
            ParsedPage* tempPage = NULL;

            // used to check if and record that a page needs vision
            bool currentPageRequiresVision = false;

            // ASSERTION: the current page should not be NULL
            PARSER_ASSERT(((tempPage = tempPages[pgIndex]) != NULL),
                errorMessage << "Temp page[" << pgIndex << "] is NULL!");

            // create a Kodu page
            KoduPage* koduPage = NULL;

            // ASSERTION: A new Kodu page was created successfully
            PARSER_ASSERT(((koduPage = new KoduPage(tempPage->getPageNumber())) != NULL),
                errorMessage << "An error occurred while trying to create a new Kodu page\n");
            
            // std::cout << "Created a page: PAGE " << koduPage->getPageNumber() << std::endl;
            // create the rules for the current page
            const std::size_t kRuleCount = tempPage->getRuleCount();
            for (std::size_t rIndex = 0; rIndex < kRuleCount; rIndex++) {
                ParsedRule* tempRule = NULL;
                KoduAction* koduAction = NULL;
                KoduCondition* koduCondition = NULL;

                // ASSERTION: the current rule should not be NULL
                PARSER_ASSERT(((tempRule = tempPage->getRuleInPos(rIndex)) != NULL),
                    errorMessage << "Temp page[" << pgIndex << "], Temp Rule[" << rIndex
                    << "] is NULL!");
                
                int ruleNumber = tempRule->getRuleNumber();
                int parentNumber = tempRule->getParentNumber();

                // create the Kodu condition
                // ASSERTION: The condition was successfully created
                PARSER_ASSERT(((koduCondition = getConditionKode(tempRule->getConditionPhrase())) != NULL),
                    errorMessage << "An error occurred while trying to create the condition for "
                    << "rule " << ruleNumber << ". See above.");
                
                // create the Kodu action
                // ASSERTION: The action was successfully created
                PARSER_ASSERT(((koduAction = getActionKode(tempRule->getActionPhrase())) != NULL),
                    errorMessage << "An error occurred while trying to create the action for "
                    << "rule " << ruleNumber << ". See above.");
                
                // create a Kodu rule (object) and add the condition and action to it
                KoduRule* koduRule = NULL;

                // ASSERTION: The Kodu rule was created successfully
                PARSER_ASSERT(((koduRule = new KoduRule(ruleNumber, parentNumber)) != NULL),
                    errorMessage << "An error occurred while trying to create a Kodu rule.");

                koduRule->condition = koduCondition;
                koduRule->action = koduAction;
                // std::cout << "Created Kodu Rule (" << koduRule->getRuleNumber() << ". " 
                //          << koduCondition->getPrimitiveType() << " + " << koduAction->getPrimitiveType() << ").\n";

                // add the newly created rule to the page
                koduPage->addKoduRule(koduRule);
                // std::cout << "Added Kodu Rule " << koduRule->getRuleNumber()
                //           << " to Page " << koduPage->getPageNumber() << ".\n";

                // check if this page requires vision
                if (currentPageRequiresVision == false) {
                    // check if the condition is see
                    if ((koduRule->condition->getConditionType() == KoduCondition::CT_SEE)
                        // check if the condition is bump
                        || (koduRule->condition->getConditionType() == KoduCondition::CT_BUMP)
                        // check if the action is move (and not turn)
                        || (koduRule->action->getActionType() == KoduAction::AT_MOTION
                            && static_cast<KoduActionMotion*>(koduRule->action)->motionTypeIsMove())
                        // check if the action is grab
                        || (koduRule->action->getActionType() == KoduAction::AT_GRAB))
                    {
                        // if any of the above statements is true, then the page requires vision
                        currentPageRequiresVision = true;
                        koduPage->setPageRequiresVision(true);
                    }
                }
                
                // bookkeeping
                tempRule = NULL;
                koduAction = NULL;
                koduCondition = NULL;
                koduRule = NULL;
            }
            // add the newly created Kodu page to the Kodu pages vector
            koduPages.push_back(koduPage);
            // std::cout << "Added Kodu Page " << koduPage->getPageNumber() << " to the Pages vector.\n";

            // bookkeeping
            tempPage = NULL;
            koduPage = NULL;
        }
        return true;
    }

    KoduCondition* Parser::KodeCreator::getConditionKode(ParsedPhrase* tempCondition) {
        // ASSERTION: The Phrase pointer is not NULL
        PARSER_ASSERT((tempCondition != NULL),
            errorMessage << "The variable that (temporarily) holds the parsed action is NULL!");

        // get the head of the phrase (the condition type)
        std::string conditionStr = tempCondition->getPhraseHead()->getKeywordData();

        // the condition that will be returned
        KoduCondition* condition = NULL;

        // Kodu Condition Always
        if (conditionStr == "always") {
            PARSER_ASSERT(((condition = new KoduConditionAlways()) != NULL),
                errorMessage << "An error occurred while trying to create the Kodu Condition Always.\n");
            // return the condition immediately (there is no need to do any else for the condition)
            std::cout << "Created Always condition...\n";
            return condition;
        }
        
        // get the modifiers for a phrase (only called if the condition is not of type Kodu Conditon Always)
        std::vector<TokenBase*> tempModifiers = tempCondition->getPhraseModifiers();

        // Kodu Condition Bump
        if (conditionStr == "bump") {
            // ASSERTION: the Kodu bump condition was successfully created
            PARSER_ASSERT(((condition = createBumpKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Bump condition. "
                << "See above.");
            std::cout << "Created Bump condition...\n";
        }

        // Kodu Condition Got
        else if (conditionStr == "got") {
            // ASSERTION: the Kodu got condition was successfully created
            PARSER_ASSERT(((condition = createGotKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Got condition. "
                << "See above.");
            std::cout << "Created Got condition...\n";
        }

        // Kodu Condition See
        else if (conditionStr == "see") {
            // ASSERTION: the Kodu see condition was succesfully created
            PARSER_ASSERT(((condition = createSeeKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the See condition. "
                << "See above.");
            // std::cout << "Created See condition...\n";
        }
        else  if (conditionStr == "hear") {
	   PARSER_ASSERT(((condition = createHearKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Hear condition. "
                << "See above.");
            std::cout << "Created Hear condition...\n";
	}

        // Kodu Condition Scored
        else if (conditionStr == "scored") {
            // ASSERTION: the Kodu scored condition was successfully created
            PARSER_ASSERT(((condition = createScoredKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Scored condition. "
                << "See above.");
            std::cout << "Created Scored condition...\n";
        }
        // Kodu Timer Condition
        else if (conditionStr == "timer") {
            // ASSERTION: the Kodu timer condition was successfully created
            PARSER_ASSERT(((condition = createTimerKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Timer condition. See above.");
            std::cout << "Created Timer condition...\n";
        } else if (conditionStr == "gamepad") {
					PARSER_ASSERT(((condition = createGamepadKode(tempModifiers)) != NULL),
												errorMessage << "An error occured creating joystick event");
					std::cout << "Created Gamepad condition...\n";
        } 
        // The user did not use a recognized condition
        else {
            PARSER_ASSERT((ERROR),
                errorMessage << "The keyword \"" << conditionStr
                << "\" is not a recognized condition.");

        }
        return condition;
    }

    KoduAction* Parser::KodeCreator::getActionKode(ParsedPhrase* tempAction) {
        // ASSERTION: The Phrase pointer is not NULL
        PARSER_ASSERT((tempAction != NULL),
            errorMessage << "The variable that (temporarily) hold the parsed action is NULL!");

        // get the head of the phrase (the action type)
        std::string actionStr = tempAction->getPhraseHead()->getKeywordData();
        
        // the action that will be returned
        KoduAction* action = NULL;

        // Kodu Action Do Nothing
        if (actionStr == "do_nothing") {
            PARSER_ASSERT(((action = new KoduActionDoNothing()) != NULL),
                errorMessage << "An error occurred while trying to create the Do Nothing action.");
            std::cout << "Created Do Nothing action...\n";
            // return the action immediately (there is no need to do any else for the action)
            return action;
        }

        // get the modifiers for a phrase
        std::vector<TokenBase*> tempModifiers = tempAction->getPhraseModifiers();
        // Kodu Action Give
        if (actionStr == "give"){
             PARSER_ASSERT(((action = createGiveKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Give action. "
                << "See above.");
             std::cout << "Created Give Action...\n";

        }
        // Kodu Action Drop
        else if (actionStr == "drop") {
            // ASSERTION: The Kodu grab action was successfully created
            PARSER_ASSERT(((action = createDropKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Drop action. "
                << "See above.");
            std::cout << "Created Drop action...\n";
        }
        // Kodu Action Grab
        else if (actionStr == "grab") {
            // ASSERTION: The Kodu grab action was successfully created
            PARSER_ASSERT(((action = createGrabKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Grab action. "
                << "See above.");
            std::cout << "Created Grab action...\n";
        }
        // Kodu Action Motion (Move)
        else if (actionStr == "move") {
            // ASSERTION: The Kodu motion (move) action was successfully created
            PARSER_ASSERT(((action = createMoveKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Move (Motion) action. "
                << "See above.");
            // std::cout << "Created Move (Motion) action...\n";
        }
        // Kodu Action Play
        else if (actionStr == "play") {
            // ASSERTION: The Kodu play action was successfully created
            PARSER_ASSERT(((action = createPlayKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Play action. "
                << "See above.");
            std::cout << "Created Play action...\n";
        }
        // Kodu Action Say
        else if (actionStr == "say") {
            // ASSERTION: The Kodu say action was successfully created
            PARSER_ASSERT(((action = createSayKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Say action. "
                << "See above.");
            std::cout << "Created Say action...\n";
        }
        // Kodu Action Score/Set Score/Subtract
        else if (actionStr == "score" || actionStr == "set_score" || actionStr == "subtract") {
            // ASSERTION: The Scoring action was successfully created
            PARSER_ASSERT(((action = createScoreKode(actionStr, tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the "
                << "Score/Set Score/Subtract action. See above.");
            std::cout << "Created Scoring (score | set score | subtract) action...\n";
        }
        // Kodu Action Switch to Page (does not need to be parsed)
        else if (actionStr == "switch_to_page") {
            int pageNumb = static_cast<int>(tempAction->getPhraseModifiers()[0]->getNumericData());
            PARSER_ASSERT(((action = new KoduActionPageSwitch(NumericGenerator(pageNumb, 0))) != NULL),
                errorMessage << "An error occurred while trying to create the Page Switch action.");
            std::cout << "Created Page Switch action...\n";
        }
        // Kodu Action Motion (Turn)
        else if (actionStr == "turn") {
            // ASSERTION: The Kodu motion (turn) action was successfully created
            PARSER_ASSERT(((action = createTurnKode(tempModifiers)) != NULL),
                errorMessage << "An error occurred while trying to create the Turn (Motion) action. "
                << "See above.");
            std::cout << "Created Turn (Motion) action...\n";
        }
        // The user did not use a recognized action
        else {
            // ASSERTION: The user did not use a recognized action
            PARSER_ASSERT((ERROR),
                errorMessage << "The keyword \"" << actionStr << "\" is not a recognized action.");
        }
        return action;
    }

    bool Parser::KodeCreator::isComparisonOperator(TokenBase* mod) {
        return (mod != NULL && mod->isKeywordToken()
            && Parser::isValidComparisonOperator(mod->getKeywordData()));
    }

    bool Parser::KodeCreator::isScoreDesignator(TokenBase* mod) {
        return (mod != NULL && mod->isKeywordToken()
            && (Parser::isValidColor(mod->getKeywordData())
                || Parser::isValidScoreLetter(mod->getKeywordData())));
    }

    bool Parser::KodeCreator::isNumericSpecifier(TokenBase* mod) {
        return (mod != NULL
            && (mod->isNumericToken() || (mod->isKeywordToken() && mod->getKeywordData() == "random")));
    }

    bool Parser::KodeCreator::numericGenParser(std::vector<TokenBase*>& mods, NumericGenerator& numgen) {
        float constOrModuloDivisor = -1.0f;
        float moduloDivisor = -1.0f;
        bool prevTokenWasNumeric = false;
        bool randomKeywordFound = false;
        bool randomTokenWasFoundBeforeNumericToken = false;

        // loop until the current token is not a numeric or keyword token
        while (!mods.empty() && (mods[0]->isNumericToken() || mods[0]->isKeywordToken())) {
            // check to see if the current token is numeric
            if (mods[0]->isNumericToken()) {
                // ASSERTION: The previous token was not a numeric token
                PARSER_ASSERT((!prevTokenWasNumeric),
                    errorMessage << "A numeric token cannot succeed another numeric token. "
                    "Token \"" << mods[0]->getNumericData() << "\" caused the error.");
                // check if this is the first numeric token
                if (constOrModuloDivisor < 0.0f) {
                    // assign the first constant/upper bound found
                    constOrModuloDivisor = mods[0]->getNumericData();
                    // used to check if the next token is numeric
                    prevTokenWasNumeric = true;
                } else if (moduloDivisor < 0.0f) {
                    // set the upper bound
                    moduloDivisor = mods[0]->getNumericData();
                    prevTokenWasNumeric = true;
                }
            }
            // it has to be a keyword token
            else {
                if (mods[0]->getKeywordData() == "random") {
                    // ASSERTION: The random token was not already found
                    PARSER_ASSERT((!randomKeywordFound),
                        errorMessage << "Only one random modifier is allowed in an action or a condition.");
                    // note that the "random" keyword was found
                    randomKeywordFound = true;
                    // note that the previous token is not numeric
                    prevTokenWasNumeric = false;
                    // if the random modifier was found before the numeric token, then do the following
                    if (constOrModuloDivisor < 0.0f) {
                        randomTokenWasFoundBeforeNumericToken = true;
                    }
                } else {
                    // the keyword is something else which SHOULD signal there is nothing for this
                    // parsing function to handle
                    break;
                }
            }
            // bookkeeping and prevent infinite looping
            GeneralFncs::destroyPtrInVector(mods, 0);
        }
        // create the numeric request
        // check if the random keyword was found
        if (randomKeywordFound) {
            // if an upper bound was set along with a constant, do the following (pattern: ## random ##)
            if (moduloDivisor >= 0.0f) {
                numgen.setNumericValues(constOrModuloDivisor, moduloDivisor);
            }

            else if (constOrModuloDivisor >= 0.0f) {
                // the random token was found before the numeric token (pattern: random ##)
                if (randomTokenWasFoundBeforeNumericToken) {
                    numgen.setNumericValues(0.0f, constOrModuloDivisor);
                }
                // the random token was found after the numeric token (pattern: ## random)
                else {
                    numgen.setNumericValues(constOrModuloDivisor, 5.0f);
                }
            }
            // no numeric tokens were found and it is the standalone random modifier (pattern: random)
            else {
                numgen.setNumericValues(0.0f, 5.0f);
            }
        }
        // no random keyword was found, so the number found must only be a numeric constant (pattern: ##)
        else {
            numgen.setNumericValues(constOrModuloDivisor, 0.0f);
        }
        return true;
    }
}
