#ifndef jpeg_mem_src_h_DEFINED
#define jpeg_mem_src_h_DEFINED

#include <sys/types.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <jpeglib.h>
void tk_jpeg_mem_src (j_decompress_ptr cinfo, JOCTET * inbuf, size_t maxlen);
#ifdef __cplusplus
}
#endif

#endif /* jpeg_mem_src_h_DEFINED */
