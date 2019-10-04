//
// Copyright 2003 Sony Corporation 
//
// Permission to use, copy, modify, and redistribute this software for
// non-commercial use is hereby granted.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//

#ifndef jpeg_mem_dest_h_DEFINED
#define jpeg_mem_dest_h_DEFINED

#ifdef __cplusplus
extern "C" {
#endif
#include <jpeglib.h>
	
	void tk_jpeg_mem_dest(j_compress_ptr cinfo, JOCTET* buf, size_t bufsize);
	int  tk_jpeg_mem_size(j_compress_ptr cinfo);
#ifdef __cplusplus
}
#endif

#endif /* jpeg_mem_dest_h_DEFINED */
