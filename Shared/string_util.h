//-*-c++-*-
#ifndef INCLUDED_string_util_h
#define INCLUDED_string_util_h

#include "attributes.h"
#include <string>
#include <vector>

//! some common string processing functions, for std::string
namespace string_util {
	//! uses the standard library's "locale" to convert case of a single character
	char localeToUpper(char c);
	
	//! uses the standard library's "locale" to convert case of a single character
	char localeToLower(char c);
	
	//! returns lower case version of @a s
	std::string makeLower(const std::string& s) ATTR_must_check;

	//! returns upper case version of @a s
	std::string makeUpper(const std::string& s) ATTR_must_check;

	//! returns @a str with @a pre removed - if @a pre is not fully matched, @a str is returned unchanged
	std::string removePrefix(const std::string& str, const std::string& pre) ATTR_must_check;
	
	//! removes whitespace (as defined by isspace()) from the beginning and end of @a str, and returns the result
	std::string trim(const std::string& str) ATTR_must_check;
	
	//! returns true if @a str begins with @a prefix
	bool beginsWith(const std::string& str, const std::string& prefix);
	
	//! returns true if @a str ends with @a suffix
	bool endsWith(const std::string& str, const std::string& suffix);
	
	//! Returns @a tgt if it is an absolute path or @a rel is empty, otherwise drops trailing file (if any) from @a rel and appends @a tgt
	/*! This assumes final path component is a source file, referencing a self-relative path,
	 *  so make sure @a rel is a path to a file, or has a trailing '/' otherwise the final directory
	 *  component will be stripped. (Doesn't actually do any disk operations to test for the file.) */ 
	std::string makePath(const std::string& rel, const std::string& path);
	
	//! convert input string into vector of string tokens
	/*! @param input string to be parsed
	 *  @param delims list of delimiters.
	 *
	 *  Consecutive delimiters will be treated as single delimiter,
	 *  delimiters are @e not included in return data.
	 *  Thanks http://www.rosettacode.org/wiki/Tokenizing_A_String#C.2B.2B */
	std::vector<std::string> tokenize(const std::string & str, const std::string & delims=", \t");
	
	//! parses the input string into an arg list, with corresponding offsets of each arg in the original input
	bool parseArgs(const std::string& input, std::vector<std::string>& args, std::vector<unsigned int>& offsets); 

	//! replaces ~USER prefix with specified user's home directory, or ~ prefix with current HOME environment setting; returns str if no valid expansion is found
	std::string tildeExpansion(const std::string& str) ATTR_must_check;
	
	//! returns true if @a str matches @a re (assumes 'extended' regular expression, not 'basic'), false otherwise and throws std::string message on error
	/*! @param str The string to match
	 *  @param regex The (extended) regular expression which should be parsed and executed
	 *
	 *  This compiles the @a regex and then executes it... for repeated usage of the same
	 *  regular expression, you could be better off compiling it yourself and using the regex library directly. */
	bool reMatch(const std::string& str, const std::string& regex);

	//! returns true if @a str matches @a re (with optional @a flags to control interpretation), false otherwise and throws std::string message on error
	/*! @param str The string to match
	 *  @param regex The regular expression which should be parsed and executed
	 *  @param flags pass flags for regex (e.g. REG_EXTENDED)
	 *
	 *  This compiles the @a regex and then executes it... for repeated usage of the same
	 *  regular expression, you could be better off compiling it yourself and using the regex library directly. */
	bool reMatch(const std::string& str, const std::string& regex, int flags);

	//! Converts a int representation of an IP to a string
	std::string intToStringIP(int ip);
	//! Converts a string representation of an IP to an int
	int stringToIntIP(std::string ip);
	
	//! returns the number of utf8 code points found in the string
	/*! This does not do any verification of the validity of the codepoints. */
	size_t utf8len(const std::string& str);
	
	//! returns demangled version of a symbol name
	std::string demangle(const std::string& symbol);

	//! convert int to string so robot can speak numbers
	std::string int2str(int const n);
};

/*! @file
 * @brief Describes some useful functions for string manipulation in the string_util namespace
 * @author ejt (Creator)
 */

#endif
