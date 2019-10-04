#ifndef __HOUGHHASH_H
#define __HOUGHHASH_H

#include <cstddef>

class HoughKey {
public:
	class model* M;
	double x, y, scale, orientation;
};

bool HoughKeyEquals(HoughKey* k1, HoughKey* k2);

size_t hashHoughKey(HoughKey *k);

#endif
