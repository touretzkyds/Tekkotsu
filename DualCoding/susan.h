//-*-c++-*-
#ifndef _SUSAN_H_
#define _SUSAN_H_

// Code imported from SUSAN Version 2l by Stephen Smith
//
// See the susan.cc file for full copyright information.

namespace DualCoding {

typedef unsigned char uchar;

void susan_thin(int *r, uchar *mid, int x_size, int y_size);

void susan_edges_internal(uchar *in, int *r, uchar *mid, uchar *bp, 
	    int max_no, int x_size, int y_size);

void susan_principle(uchar *in, int *r, uchar **bp, int max_no,
		     int x_size, int y_size);

void edge_draw(uchar *in, uchar *mid, int x_size, int y_size, int drawing_mode);


/* bp gets initialized in here right now! need to have an init function */
void setup_brightness_lut(uchar **bp, int thresh, int form);

} // namespace

#endif /* _SUSAN_H_ */
