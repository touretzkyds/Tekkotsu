#include "HoughHash.h"
#include <cstdlib>

bool HoughKeyEquals(HoughKey* k1, HoughKey* k2){
	return (k1->M           == k2->M
	     && k1->x           == k2->x
	     && k1->y           == k2->y
	     && k1->scale       == k2->scale
	     && k1->orientation == k2->orientation);
}

size_t hashHoughKey(HoughKey *k){
	size_t retval = std::abs((long)((((long)(k->x * k->y) % 1073741824) * ((long)(k->scale * k->orientation * 1024) % 1073741824)) % 1073741824 * 4096));
	//if (retval < 0) retval = 1;
//	cout << k->x << " " << k->y << " " << k->scale << " " << k->orientation << " " << retval << endl;
	return retval;
}
