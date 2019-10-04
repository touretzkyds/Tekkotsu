// Tekkodu Library
#include "Kodu/Parsing/Parser.h"

// C++ Library
#include <cctype>

namespace Kodu {

    bool Parser::TokenParser::tokenize(const std::string& kString, std::vector<TokenBase*>& tokens) {
        std::size_t pos = 0;
        const std::size_t kSize = kString.size();
        while (pos < kSize) {
            // get the current character
            char currChar = kString[pos];

            // ASSERTION: the current character is either a double quote, a whitespace, a digit, or a letter
            PARSER_ASSERT((currChar == '"' || currChar == ' ' || isalnum(currChar) || currChar == '<'
                || currChar == '>' || currChar == ':'),
                errorMessage << "The character '" << currChar << "' is unrecognized.");

            // check what the current character is
            // check if current character is a double quote (") identifying a literal string
            if (currChar == '"') {
                // find the next occurence of the double quote (after the first)
                std::size_t nextDqOccurence = kString.find('"', pos + 1);

                // ASSERTION: the second double quote was found
                PARSER_ASSERT((nextDqOccurence != std::string::npos),
                    errorMessage << "Could not find a closing double quote (first occurrence of (\") at Col "
                        << (pos + 1) << ".");

                // capture the literal string without the double quotes
                tokens.push_back(new StringToken(kString.substr(pos + 1, nextDqOccurence - pos - 1)));

                // set the position to the index after the second double quote
                pos = nextDqOccurence + 1;
                continue;
            }

            if (currChar == ':') {
                // ASSERTION: the character after the indentation marker is a whitespaces
                PARSER_ASSERT((pos + 1 < kSize && kString[pos + 1] == ' '),
                    errorMessage << "(Col " << (pos + 2) << ") The character '"
                    << kString[pos + 1] << "' is an invalid character.");

                // add indentation character to the list of tokens
                tokens.push_back(new KeywordToken(":"));

                // increment the position by 2
                pos = pos + 2;
            }

            // check if current character is a letter
            if (isalpha(currChar)) {
                // find the next occurence of whitespace
                std::size_t nextWsOccurence = kString.find(' ', pos + 1);

                // temporary string to hold the keyword
                std::string keyword;

                // test if this was the last token in the string
                if (nextWsOccurence == std::string::npos)
                    keyword = kString.substr(pos);
                else
                    keyword = kString.substr(pos, nextWsOccurence - pos);

                // check if the string is a valid keyword
                PARSER_ASSERT((koduKeywords.count(keyword) == 1),
                    errorMessage << "(Col " << (pos + 1) << ") The token \"" << keyword
                    << "\" is not a recognized Kodu keyword.");

                // add the keyword to the token vector
                tokens.push_back(new KeywordToken(keyword));

                // set the position to the index after the whitespace or the size of the string is nothing is left
                if (nextWsOccurence == std::string::npos)
                    pos = kSize;
                else
                    pos = nextWsOccurence + 1;
                continue;
            }

            // if check if current character is a digit
            if (isdigit(currChar)) {
                // check if the entire "number" string can be converted into an actual floating-point number
                std::size_t tempPos = pos + 1;
                std::size_t stopPos = 0;
                unsigned int dotCount = 0;
                bool hasOnlyZeros = true;

                // find the next occurence of whitespace
                std::size_t nextWsOccurence = kString.find(' ', tempPos);

                // test if this was the last token in the string
                if (nextWsOccurence == std::string::npos)
                    stopPos = kString.size();
                else
                    stopPos = nextWsOccurence;

                // check each character up to, but not including 
                while (tempPos < stopPos) {
                    // ASSERTION: the current character is a digit or a dot
                    PARSER_ASSERT((isdigit(kString[tempPos]) || (kString[tempPos] == '.')),
                        errorMessage << "(Col " << (tempPos + 1) << ") The character '"
                        << kString[tempPos] << "' in token \"" << kString.substr(pos, stopPos - pos)
                        << "\" is not a digit [0-9] or a decimal point (.).");

                    // keep track of the number of dots found (there should only be one!)
                    if (kString[tempPos] == '.') {
                        dotCount++;
                    }

                    // ASSERTION: the number of dots is 0 or 1
                    PARSER_ASSERT((dotCount == 0 || dotCount == 1),
                        errorMessage << "(Col " << (tempPos + 1)
                        << ") There is an additional decimal point (.) in the token \""
                        << kString.substr(pos, stopPos - pos) << "\".");

                    // check if it is a digit other than zero
                    if (isdigit(kString[tempPos]) && kString[tempPos] != '0') {
                        hasOnlyZeros = false;
                    }

                    // increase the position
                    tempPos++;
                }

                // ASSERTION: the decimal point is not the last character in the range
                PARSER_ASSERT((kString[tempPos] != '.'),
                    errorMessage << "(Col " << (tempPos + 1)
                    << ") A decimal point cannot the last character of a number.");

                // ASSERTION: the first character of the number sequence is not a zero (unless followed by a decimal)
                if (kString[pos] == '0') {
									PARSER_ASSERT( ((kString[pos + 1] == '.') || (pos+1 == tempPos)),
                        errorMessage << "(Col " << (pos + 2)
                        << ") A number can only begin with zero if the zero is followed by a decimal point.");
                }

                // convert the digits (and decimal point) to a floating-point number
                float value = (float)strtod(kString.substr(pos, stopPos - pos).c_str(), NULL);

                // I DON'T KNOW IF I HAVE TO DO THIS (TODO 16-JUL-13)
                // make sure if all the chars were not zeros that the return value was greater than zero
                if (!hasOnlyZeros) {
                    // ASSERTION: the value is greater than zero if the while loop found any other digit than zero
                    PARSER_ASSERT((value > 0.0f),
                        errorMessage << "(Col " << (pos + 1)
                        << ") There was an error converting \""
                        << kString.substr(pos, stopPos - pos) << "\" to a floating-point number. "
                        << "Numbers should only contain digits and one dot, if needed.");
                }

                // add the floating-point number to the token vector
                tokens.push_back(new NumericToken(value));

                // set the position to the index after the whitespace 
                pos = stopPos + 1;
                continue;
            }

            // check if current character is an inequality sign
            if (currChar == '<' || currChar == '>') {
                // ASSERTION: there is at least 2 more characters in this string
                PARSER_ASSERT((pos + 2 < kString.size()),
                    errorMessage << "(Col " << (pos + 1)
                    << ") Invalid use of the inequality signs. There must be a number after the signs.");

                // ASSERTION: there is an equal sign in the next character position (pos + 1)
                // and a space at position after that (pos + 2)
                PARSER_ASSERT((kString[pos + 1] == '=' && kString[pos + 2] == ' '),
                    errorMessage << "(Col " << (pos + 1)
                    << ") Invalid use of inequality signs. Correct usage: [number] [space] <= [space] [number]."
                    << "(E.g. 10 <= 13).");

                // add the inequality sign to the token vector
                if (currChar == '<')
                    tokens.push_back(new KeywordToken("<="));
                else
                    tokens.push_back(new KeywordToken(">="));

                // set the position to the index after the whitespace
                pos = pos + 3;
                continue;
            }

            // check if current character is a whitespace (move unto to the next character)
            if (isspace(kString[pos])) {
                pos++;
                continue;
            }
        }
        return true;
    }
}
