// Tekkodu Library
#include "Kodu/Parsing/Parser.h"

// C++ Library
#include <set>

namespace Kodu {

    std::set<std::string> Parser::TokenParser::koduKeywords;

    void Parser::TokenParser::initializeKeywordSet() {
        // add the color keywords
        koduKeywords.insert(koduColorKeywords.begin(), koduColorKeywords.end());
        
        // add the numeric comparison keywords
        koduKeywords.insert(koduCompKeywords.begin(), koduCompKeywords.end());

        // add the score letter keywords
        koduKeywords.insert(koduScoreLetterKeywords.begin(), koduScoreLetterKeywords.end());

        // add the PAGE, WHEN, and DO identifiers
        koduKeywords.insert("PAGE");
        koduKeywords.insert("WHEN");
        koduKeywords.insert("DO");

        // add the following conditions to the keyword set
        koduKeywords.insert("always");
        koduKeywords.insert("bump");
        koduKeywords.insert("got");
        koduKeywords.insert("scored");
        koduKeywords.insert("see");
        koduKeywords.insert("timer");
	koduKeywords.insert("hear");

        // add the following actions to the keyword set
        koduKeywords.insert("do_nothing");
        koduKeywords.insert("drop");
        koduKeywords.insert("grab");
        koduKeywords.insert("move");
        koduKeywords.insert("switch_to_page");
        koduKeywords.insert("play");
        koduKeywords.insert("say");
        koduKeywords.insert("score");
        koduKeywords.insert("set_score");
        koduKeywords.insert("subtract");
        koduKeywords.insert("turn");					 
	koduKeywords.insert("give");

        // add the following modifiers to the keyword set
        // any modifier
        koduKeywords.insert("any");

        // direction specifiers
        koduKeywords.insert("east");
        koduKeywords.insert("north");
        koduKeywords.insert("south");
        koduKeywords.insert("west");

        // motion modifiers

        koduKeywords.insert("direction");
        koduKeywords.insert("forward");
        koduKeywords.insert("left");
        koduKeywords.insert("right");
	    koduKeywords.insert("move");
        koduKeywords.insert("towards");
        koduKeywords.insert("wander");
        
        // not modifier
        koduKeywords.insert("not");
        
        // object types/names and references
        koduKeywords.insert("it");
        koduKeywords.insert("apple");
        koduKeywords.insert("rock");
        koduKeywords.insert("tree");
	    koduKeywords.insert("octopus");
	    koduKeywords.insert("kodu");
        koduKeywords.insert("robot");
     	koduKeywords.insert("cycle");
	    koduKeywords.insert("turtle");
        // once modifier
        koduKeywords.insert("once");
        
        // random modifier
        koduKeywords.insert("random");

        // rate modifiers
        koduKeywords.insert("quickly");
        koduKeywords.insert("slowly");

        // regional specifiers
        koduKeywords.insert("to_left");
        koduKeywords.insert("to_right");
        koduKeywords.insert("in_front");
        koduKeywords.insert("behind");

        // object distance
        koduKeywords.insert("close_by");
        koduKeywords.insert("far_away");
	
	// Gamepad Keywords
	koduKeywords.insert("gamepad");
	koduKeywords.insert("button");
	koduKeywords.insert("press");
	koduKeywords.insert("release");
	koduKeywords.insert("A");
	koduKeywords.insert("Y");
	koduKeywords.insert("B");
	koduKeywords.insert("X");
	koduKeywords.insert("right");
	koduKeywords.insert("left");
    koduKeywords.insert("L-stick");
	koduKeywords.insert("bumpers");
	koduKeywords.insert("joystick");

    //Comparison
    koduKeywords.insert("equals");
    koduKeywords.insert("not_equals");
    koduKeywords.insert("<=");
    koduKeywords.insert(">=");
    koduKeywords.insert("above");
    koduKeywords.insert("below");
	
	
    }

    void Parser::TokenParser::clearKeywordSet() {
        koduKeywords.clear();
    }
}
