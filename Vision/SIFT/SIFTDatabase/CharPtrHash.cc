#include "CharPtrHash.h"
#include <cstring>

bool CharPtrKeyEquals(CharPtrKey* k1, CharPtrKey* k2){
	return (strcmp(k1->key, k2->key) == 0);
}

size_t hashCharPtrKey(CharPtrKey *k){
	size_t hashVal = 0;
	int i = 0;
	while (k->key[i] != '\0'){
		hashVal += (255 * k->key[i]);
		i++;
	}
	return hashVal;
}

CharPtrKey::CharPtrKey(const char* Key) : key(new char[2048]) {
	strcpy(key, Key);
}

CharPtrKey::~CharPtrKey(){
	delete [] key;
}
