#ifndef __CHARPTRHASH_H
#define __CHARPTRHASH_H

#include <cstddef>

class CharPtrKey;

bool CharPtrKeyEquals(CharPtrKey* k1, CharPtrKey* k2);

size_t hashCharPtrKey(CharPtrKey *k);

class CharPtrKey{
private:
	char* key;
	
	friend bool CharPtrKeyEquals(CharPtrKey* k1, CharPtrKey* k2);
	friend size_t hashCharPtrKey(CharPtrKey *k);
	
public:
	CharPtrKey(const char* Key);
	~CharPtrKey();
private:
	CharPtrKey(const CharPtrKey&); // Do not use
	CharPtrKey& operator=(const CharPtrKey&); // Do not use
};

#endif
