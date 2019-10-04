#ifndef jpeg_istream_src_h_DEFINED
#define jpeg_istream_src_h_DEFINED

#include <istream>

#ifdef __cplusplus
extern "C" {
#endif
#include <jpeglib.h>
void jpeg_istream_src (j_decompress_ptr cinfo, std::istream& inStream);
int jpeg_istream_revert (j_decompress_ptr cinfo);
#ifdef __cplusplus
}
#endif

#endif /* jpeg_istream_dest_h_DEFINED */
