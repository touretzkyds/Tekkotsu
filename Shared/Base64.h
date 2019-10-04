#ifndef INCLUDED_Base64_H
#define INCLUDED_Base64_H
/*! @file
 * @brief Describes base64 namespace, which holds some functions for encoding and decoding base64 data
 * C_Base64 - a simple base64 encoder and decoder.
 *
 *     Copyright (c) 1999, Bob Withers - bwit@pobox.com
 *
 * This code may be freely used for any purpose, either personal
 * or commercial, provided the authors copyright notice remains
 * intact.
 *
 * Modified by Ethan Tira-Thompson:
 *     - changed Base64 class to base64 namespace
 *     - modified functions to work on char[] instead of strings
 *     - added doxygen comments to header
 *
 * @author Bob Withers - bwit@pobox.com (Creator)
 */

#include <string>

//! holds some functions for encoding and decoding base64 data
namespace base64
{

	//! returns a string containing the base64 encoding of @a len bytes from @a buf
	std::string encode(char buf[], unsigned int len);

	//! returns a pointer to a newly allocated region contained the binary data decoded from @a data
	/*! If @a data is malformed @c NULL will be returned */
	char* decode(const std::string& data);

	//! returns the number of bytes which will be decoded from @a data
	/*! Does not check data for correctness, just counts the fill
	 *  characters at the end of the string */
	unsigned int decodeSize(const std::string& data);

	//! decodes @a data into @a buf, which you need to provide the size of in @a len
	/*! If @a buf is not large enough or @a data is malformed, @c false
	 *  will be returned; otherwise @c true */
	bool decode(const std::string& data, char buf[], unsigned int len);
}


#endif
