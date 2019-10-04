#include "Kodu/Parsing/Parser.h"

namespace Kodu {
	KoduActionGive* Parser::KodeCreator::createGiveKode(std::vector<TokenBase*>& mods){
        
        
		return (new KoduActionGive());
	}

	KoduActionDrop* Parser::KodeCreator::createDropKode(std::vector<TokenBase*>& mods) {
		// ASSERTION: There are 1 - 2 modifiers
		PARSER_ASSERT((mods.size() == 0),
									errorMessage << "The Drop action should have 0 tokens.");

		// create the action
		return (new KoduActionDrop());
	}

	KoduActionGrab* Parser::KodeCreator::createGrabKode(std::vector<TokenBase*>& mods) {
		//****** temp fix (need to find out how much are actualy used)
			// ASSERTION: There are 1 - 2 modifiers
			PARSER_ASSERT((0 <= mods.size() && mods.size() <= 2),
										errorMessage << "The Grab action should have 1 - 2 tokens.");
		//******

        // Mandatory modifiers
        bool itModifierWasSpecified = false;

		// Optional modifiers
		bool onceEnabled = false;

		// checkers
		int tokenCount = 1;

		// parsing loop
		while (!mods.empty()) {
			// check if the token is a keyword
			if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				if (keyword == "once") {        KoduState state;

					// ASSERTION: the once modifier was not already parsed
					PARSER_ASSERT((onceEnabled == false),
                        errorMessage << "Found a second once modifier (only one is allowed).");
					onceEnabled = true;
				}

				else if (keyword == "it") {
					// ASSERTION: the "it" modifier was not already parsed
					PARSER_ASSERT((itModifierWasSpecified == false),
                        errorMessage << "Found a second it modifier (only one is allowed).");
					itModifierWasSpecified = true;
				}

				else {
					// ASSERTION: "it" is the keyword
					PARSER_ASSERT((ERROR),
                        errorMessage << "The keyword \"" << keyword
                        << "\" cannot be used for the grab action.");
				}
			}
			// this is a serious error!!!
			else {
				// ASSERTION: There is an illegal token for this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Failed to recognize token " << tokenCount
											<< " for the grab action.");
			}
			// bookkeeping and prevents infinite loop
			GeneralFncs::destroyPtrInVector(mods, 0);
			tokenCount++;
		}
		if(itModifierWasSpecified == false)
			itModifierWasSpecified = true;
		// ASSERTION: the sound file was added
		/*
			PARSER_ASSERT((itModifierWasSpecified == true),
			errorMessage << "This Grab action does not have an it modifier!");
		*/
		// create the action
		return (new KoduActionGrab(itModifierWasSpecified, onceEnabled));
	}

	KoduActionMotion* Parser::KodeCreator::createMoveKode(std::vector<TokenBase*>& mods) {
		// ASSERTION: There are
		PARSER_ASSERT((0 <= mods.size() && mods.size() <= 5),
									errorMessage << "The Move action should only have 0 - 5 tokens.");

		// Optional modifiers
		KoduActionMotion::MotionType_t motionType = KoduActionMotion::MT_MOVE_GAMEPAD;
		KoduActionMotion::MotionRate_t motionRate;
		std::string tempMotionRate;
		unsigned int motionRateCount = 0;

		// checkers
		int tokenCount = 1;
		bool parsedMotionType = false;
		bool parsedMotionRate = false;
		bool parsedDriven = false;
        
		// parsing loop
		while (!mods.empty()) {
			// if it is a keyword
			if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				// check if the keyword is a rate specifier
				if (keyword == "quickly" || keyword == "slowly") {
					if (parsedMotionRate) {
                                            
						motionType = KoduActionMotion::MT_MOVE_WANDER;
						PARSER_ASSERT((motionRateCount <= 3),
													errorMessage << "The move action can only have up to 3 rate specifiers.\n");

						// ASSERTION: The user did not specify a different rate specifier
						PARSER_ASSERT((keyword == tempMotionRate),
													errorMessage << "Different types of rate specifiers cannot be used.\n"
													<< "First type seen: " << tempMotionRate << "\nSecond type seen: "
													<< keyword);
					} else {
						tempMotionRate = keyword;
						parsedMotionRate = true;
					}
					// increment the number of rate specifiers seen
					motionRateCount++;
				}
                
				else if (keyword == "move"){
					motionType = KoduActionMotion::MT_MOVE_GAMEPAD;
					parsedMotionType = true;
					parsedDriven = true;
				}
				// check if it is one of the move action's modifiers
				else if (keyword == "wander" || keyword == "forward" || keyword == "towards") {
					// ASSERTION: A motion type was not previously selected
					PARSER_ASSERT((parsedMotionType == false),
                        errorMessage << "A mowas already selected (There can only "
												<< "be one). Secowanderwandernd move type found: " << keyword << ".");
					PARSER_ASSERT((parsedDriven == false),
                        errorMessage << "A move type for this action cannot be executed");
					if (keyword == "wander")
						motionType = KoduActionMotion::MT_MOVE_WANDER;
					else if (keyword == "forward")
						motionType = KoduActionMotion::MT_MOVE_FORWARD;
					else if (keyword == "towards")
						motionType = KoduActionMotion::MT_MOVE_TOWARDS;
					parsedMotionType = true;
				}
				// user used the wrong keyword
				else {
					// ASSERTION: The user used the wrong keyword for this action
					PARSER_ASSERT((ERROR),
                        errorMessage << "The keyword \"" << keyword
                        << "\" cannot be used with the move action.");
				}
			}
			// user used the wrong token type
			else {
				// ASSERTION: The user used the wrong type of token with this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Token " << tokenCount << " is unrecognized and cannot be used "
											<< "with the move action.");
			}
            
			// bookkeeping
			GeneralFncs::destroyPtrInVector(mods, 0);
		}
		// check if a move type was specified
		if (!parsedMotionType) {
			motionType = KoduActionMotion::MT_MOVE_GAMEPAD;
		}

		// check if a rate specifier was specified
		if (!parsedMotionRate) {
			motionRate = KoduActionMotion::MR_NORMAL;
			motionRateCount = 1;
		} else {
			if (tempMotionRate == "quickly")
				motionRate = KoduActionMotion::MR_QUICKLY;
			else    
				motionRate = KoduActionMotion::MR_SLOWLY;
		}
	
		// create the action
		return (new KoduActionMotion(motionType, motionRate, motionRateCount));
	}

	KoduActionPlay* Parser::KodeCreator::createPlayKode(std::vector<TokenBase*>& mods) {
		// ASSERTION: There are 1 - 2 modifiers
		PARSER_ASSERT((1 <= mods.size() && mods.size() <= 2),
									errorMessage << "The Play action should have 1 - 2 tokens.");

		// Mandatory modifiers
		std::string soundFile;

		// Optional modifiers
		bool onceEnabled = false;

		// checkers
		int tokenCount = 1;

		// parsing loop
		while (!mods.empty()) {
			// if it is a literal string
			if (mods[0]->isStringToken()) {
				// ASSERTION: The sound file was not assigned as yet
				PARSER_ASSERT((soundFile.empty()),
											errorMessage << "Found a second sound file (only one is allowed).");

				// assign the sound file
				soundFile = mods[0]->getStringData();
			}
			// check if the token is a keyword
			else if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				// ASSERTION: "once" is the keyword
				PARSER_ASSERT((keyword == "once"),
											errorMessage << "The keyword \"" << keyword
											<< "\" cannot be used for this action.");
                
				// ASSERTION: the once modifier was not already parsed
				PARSER_ASSERT((onceEnabled == false),
											errorMessage << "Found a second once modifier (only one is allowed).");
				onceEnabled = true;
			}
			// this is a serious error!!!
			else {
				// ASSERTION: There is an illegal token for this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Failed to recognize token " << tokenCount
											<< " for the play action.");
			}
			// bookkeeping and prevents infinite loop
			GeneralFncs::destroyPtrInVector(mods, 0);
			tokenCount++;
		}
		// ASSERTION: the sound file was added
		PARSER_ASSERT((!soundFile.empty()),
									errorMessage << "This Play action does not have a sound file to play!");

		// create the action
		LiteralGenerator playLiteral(soundFile, Kodu::LiteralGenerator::RO_SEQUENTIAL);
		return (new KoduActionPlay(playLiteral, onceEnabled));
	}

	KoduActionSay* Parser::KodeCreator::createSayKode(std::vector<TokenBase*>& mods) {
		// ASSERTION: There are 1 - 10 modifiers
		PARSER_ASSERT((1 <= mods.size() && mods.size() <= 10),
									errorMessage << "The Say action should have 1 - 10 tokens.");

		// Mandatory modifiers
		std::vector<std::string> literalStrings;

		// Optional modifiers
		LiteralGenerator::ReturnOrder_t returnOrder;
		bool randomKeywordWasFound = false;
		bool onceEnabled = false;
		std::string color;

		// checkers
		int tokenCount = 1;
        
		// parsing loop
		while (!mods.empty()) {
			// check if the token is a string
			if (mods[0]->isStringToken()) {
				literalStrings.push_back(mods[0]->getStringData());
			}
			// check if the token is a keyword
			else if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				// if the keyword is the once modifier
				if (keyword == "once") {
					// ASSERTION: The once modifier was not previously found
					PARSER_ASSERT((onceEnabled == false),
                        errorMessage << "Found a second \"once\" modifier (only one is allowed).");
					onceEnabled = true;
				}
				// check if the keyword is the random modifier
				else if (keyword == "random") {
					// ASSERTION: There is only one random keyword in this action phrase
					PARSER_ASSERT((randomKeywordWasFound == false),
                        errorMessage << "The \"random\" can only be used once in this phrase.");
					randomKeywordWasFound = true;
				}
				// check if the keyword is a color
				else if (Parser::isValidColor(keyword) != -1) {
					// ASSERTION: The color was not set
					PARSER_ASSERT((color.empty()),
                        errorMessage << "Found a second color (only one is allowed).");
					color = keyword;
				}
				// this is a serious error!!!
				else {
					// ASSERTION: There is an illegal token for this action
					PARSER_ASSERT((ERROR),
                        errorMessage << "Failed to recognize token " << tokenCount
                        << " for the play action.");
				}
			}
			// this is a serious error!!!
			else {
				// ASSERTION: There is an illegal token for this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Failed to recognize token " << tokenCount
											<< " for the play action.");
			}
			GeneralFncs::destroyPtrInVector(mods, 0);
			tokenCount++;
		}
		// ASSERTION: the literal string was specified
		PARSER_ASSERT((!literalStrings.empty()),
									errorMessage << "This Say action does not have any strings to say!");

		// if the random keyword was not found, then speak the speech in sequential order
		if (randomKeywordWasFound)
			returnOrder = LiteralGenerator::RO_RANDOM;
		else
			returnOrder = LiteralGenerator::RO_SEQUENTIAL;
        
		// if a color was not assigned... assign a default color
		if (color.empty())
			color = Parser::koduDefaultDesignator;

		// create the action
		Kodu::LiteralGenerator sayLiterals(literalStrings, returnOrder);
		return (new KoduActionSay(color, sayLiterals, onceEnabled));
	}

	KoduActionScore* Parser::KodeCreator::createScoreKode(const std::string& scoreType,
																												std::vector<TokenBase*>& mods)
	{
		// ASSERTION: There are 0 - 5 modifiers
		PARSER_ASSERT((0 <= mods.size() && mods.size() <= 5),
									errorMessage << "The Score action should have 0 - 5 tokens.");

		// Optional modifiers
		NumericGenerator tempNumGen(1, 0);
		bool onceEnabled = false;
		std::string scoreDesignator;

		// checkers
		int tokenCount = 1;
		bool designatorWasSet = false;
		bool handledNumerics = false;

		// get the score action type
		KoduActionScore::ScoringType_t scoreOp;
		if (scoreType == "score") {
			scoreOp = KoduActionScore::ST_SCORE;
		} else if (scoreType == "set_score") {
			scoreOp = KoduActionScore::ST_SET_SCORE;
		} else {
			scoreOp = KoduActionScore::ST_SUBTRACT;
		}

		// parsing loop
		while (!mods.empty()) {
			// check if the token is a numeric specifier
			if (isNumericSpecifier(mods[0])) {
				// ASSERTION: Numeric specifiers were not already handled/parsed
				PARSER_ASSERT((handledNumerics == false),
											errorMessage << "Numeric specifiers for this action were already handled. "
											<< "Token " << tokenCount << " is illegal.");
				// get the numeric request
				// ASSERTION: The numeric/random number parser did not fail
				PARSER_ASSERT((numericGenParser(mods, tempNumGen) == true),
											errorMessage << "An error occurred while parsing the numeric specifier token(s) "
											<< "(see above).");
				// note that numbers have been handled
				handledNumerics = true;
			}
			// check if the token is a keyword
			else if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				// if the keyword is the once modifier
				if (keyword == "once") {
					// ASSERTION: The once modifier was not previously found
					PARSER_ASSERT((onceEnabled == false),
                        errorMessage << "Found a second \"once\" modifier (only one is allowed).");
					onceEnabled = true;
				}
				// check if the keyword is a score designator
				else if (isScoreDesignator(mods[0])) {
					// ASSERTION: The score designator was not set
					PARSER_ASSERT((designatorWasSet == false),
                        errorMessage << "Found a second score designator (only one is allowed).");
					scoreDesignator = keyword;
					designatorWasSet = true;
				}
				// this is a serious error!!!
				else {
					// ASSERTION: There is an illegal token for this action
					PARSER_ASSERT((ERROR),
                        errorMessage << "Failed to recognize token " << tokenCount
                        << " for the score/subtract/set score action.");
				}
				// bookkeeping
				GeneralFncs::destroyPtrInVector(mods, 0);
			}
			// this is a serious error!!!
			else {
				// ASSERTION: There is an illegal token for this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Failed to recognize token " << tokenCount
											<< " for the score/subtract/set score action.");
			}
			tokenCount++;
		}
		// assign the default color if no other designator was specified
		if (scoreDesignator.empty())
			scoreDesignator = Parser::koduDefaultDesignator;
		// create the action
		return (new KoduActionScore(scoreOp, tempNumGen, scoreDesignator, onceEnabled));
	}

	KoduActionMotion* Parser::KodeCreator::createTurnKode(std::vector<TokenBase*>& mods) {
		// ASSERTION: There are
		PARSER_ASSERT((0 <= mods.size() && mods.size() <= 5),
									errorMessage << "The Turn action should only have 0 - 5 tokens.");

		// Optional modifiers
		KoduActionMotion::MotionType_t motionType;
		KoduActionMotion::MotionRate_t motionRate;
		Direction_t directionToFace;
		std::string tempMotionRate;
		unsigned int motionRateCount = 0;

		// checkers
		int tokenCount = 1;
		bool parsedMotionType = false;
		bool parsedMotionRate = false;
		bool parsedDirection = false;

		// parsing loop
		while (!mods.empty()) {
			// if it is a keyword
			if (mods[0]->isKeywordToken()) {
				std::string keyword = mods[0]->getKeywordData();
				// check if the keyword is a rate specifier
				if (keyword == "quickly" || keyword == "slowly") {
					if (parsedMotionRate) {
						// ASSERTION: This is not the fourth rate specifier parsed
						PARSER_ASSERT((motionRateCount <= 3),
													errorMessage << "The turn action can only have up to 3 rate specifiers.\n");

						// ASSERTION: The user did not specify a different rate specifier
						PARSER_ASSERT((keyword == tempMotionRate),
													errorMessage << "Different types of rate specifiers cannot be used.\n"
													<< "First type seen: " << tempMotionRate << "\nSecond type seen: "
													<< keyword);
					} else {
						tempMotionRate = keyword;
						parsedMotionRate = true;
					}
					// increment the number of rate specifiers seen
					motionRateCount++;
				}
				// check if it is one of the move action's modifiers
				else if (keyword == "left" || keyword == "right" || keyword == "direction") {
					// ASSERTION: A motion type was not previously selected
					PARSER_ASSERT((parsedMotionType == false),
                        errorMessage << "A turn type for this action was already selected (There can only "
												<< "be one). Second turn type found: " << keyword << ".");
					if (keyword == "left")
						motionType = KoduActionMotion::MT_TURN_LEFT;
					else if (keyword == "right")
						motionType = KoduActionMotion::MT_TURN_RIGHT;
					else
						motionType = KoduActionMotion::MT_TURN_DIRECTION;
					parsedMotionType = true;
				}
				// check if it is one of the directions
				else if (keyword == "east" || keyword == "north" || keyword == "south"
								 || keyword == "west")
					{
						// ASSERTION: the turn type was already parsed
						PARSER_ASSERT((parsedMotionType == true),
													errorMessage << "The keyword \"direction\" must be specified before the "
													<< "direction specifier " << keyword << ".");
						// ASSERTION: the turn type is direction
						PARSER_ASSERT((motionType == KoduActionMotion::MT_TURN_DIRECTION),
													errorMessage << "Direction specifiers such as " << keyword
													<< " can only be used with the turn modifier \"direction\".");
						// ASSERTION: a direction was not parsed already
						PARSER_ASSERT((parsedDirection == false),
													errorMessage << "A direction type was already parsed. (There can only "
													<< "be one). Second direction type found: " << keyword << ".");

						if (keyword == "east")
							directionToFace = DT_EAST;
						else if (keyword == "north")
							directionToFace = DT_NORTH;
						else if (keyword == "south")
							directionToFace = DT_SOUTH;
						else
							directionToFace = DT_WEST;
						parsedDirection = true;
					}
				// user used the wrong keyword
				else {
					// ASSERTION: The user used the wrong keyword for this action
					PARSER_ASSERT((ERROR),
                        errorMessage << "The keyword \"" << keyword
                        << "\" cannot be used with the move action.");
				}
			}
			// user used the wrong token type
			else {
				// ASSERTION: The user used the wrong type of token with this action
				PARSER_ASSERT((ERROR),
											errorMessage << "Token " << tokenCount << " is an unrecognized and cannot be used "
											<< "with the move action.");
			}

			// bookkeeping
			GeneralFncs::destroyPtrInVector(mods, 0);
		}
		// check if a move type was specified
		if (!parsedMotionType) {
			motionType = KoduActionMotion::MT_TURN_LEFT;
		}

		// check if a rate specifier was specified
		if (!parsedMotionRate) {
			motionRate = KoduActionMotion::MR_NORMAL;
			motionRateCount = 1;
		} else {
			if (tempMotionRate == "quickly")
				motionRate = KoduActionMotion::MR_QUICKLY;
			else    
				motionRate = KoduActionMotion::MR_SLOWLY;
		}

		if (motionType == KoduActionMotion::MT_TURN_DIRECTION) {
			PARSER_ASSERT((parsedDirection == true),
										errorMessage << "A direction specifier such as north or east must be specified"
                    << "when using the direction modifier.");

			return (new KoduActionMotion(directionToFace, motionRate, motionRateCount));
		}

		// create the action
		return (new KoduActionMotion(motionType, motionRate, motionRateCount));
	}

} // namespace
