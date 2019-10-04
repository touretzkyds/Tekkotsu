#include "jpeg_istream_src.h"

#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

/* Theoretically, could be used to load several consecutive
 *  JPEGs from memory, but not really tested, some tweaks
 *  would probably be required */
	
//#include <jpeglib.h> 
#include <jerror.h> 

static const unsigned int JPEGStreamSourceBufferSize=1024;
	
typedef struct {
	struct jpeg_source_mgr  pub;
	std::istream * is;
	char c[JPEGStreamSourceBufferSize];
	size_t readCnt;
} JPEGStreamSource;

static void init_sourceFunc(j_decompress_ptr cinfo) {
	JPEGStreamSource  *src = (JPEGStreamSource*)cinfo->src;
	if(!src->is->get(*src->c)) {
		ERREXIT(cinfo, JERR_INPUT_EMPTY);
	}
	src->is->readsome(src->c+1,JPEGStreamSourceBufferSize-1);
	++src->readCnt;
	src->pub.next_input_byte = (JOCTET*)src->c;
	src->pub.bytes_in_buffer = src->is->gcount()+1;
}
static boolean fill_input_bufferFunc(j_decompress_ptr cinfo) {
	JPEGStreamSource  *src = (JPEGStreamSource*)cinfo->src;
	if(!src->is->get(*src->c)) {
		ERREXIT(cinfo, JERR_INPUT_EOF);
		return false;
	}
	src->is->readsome(src->c+1,JPEGStreamSourceBufferSize-1);
	++src->readCnt;
	src->pub.next_input_byte = (JOCTET*)src->c;
	src->pub.bytes_in_buffer = src->is->gcount()+1;
	//std::cout << "jpeg got " << src->pub.bytes_in_buffer << " more bytes" << std::endl;
	return true;
}
static void skip_input_dataFunc(j_decompress_ptr cinfo, long num_bytes) {
	JPEGStreamSource  *src = (JPEGStreamSource*)cinfo->src;
	if (num_bytes > 0) {
		if ((size_t)num_bytes > src->pub.bytes_in_buffer) {
			num_bytes -= src->pub.bytes_in_buffer;
			src->pub.bytes_in_buffer = 0;
			src->is->ignore(num_bytes);
			++src->readCnt;
		} else {
			src->pub.bytes_in_buffer -= num_bytes;
		}
	}
}
static void term_sourceFunc(j_decompress_ptr cinfo) {
	JPEGStreamSource  *src = (JPEGStreamSource*)cinfo->src;
	//std::cout << "putback " << src->pub.bytes_in_buffer << std::endl;
	src->is->seekg(-(std::streamoff)src->pub.bytes_in_buffer, std::ios_base::cur);
	src->pub.bytes_in_buffer=0;
	//std::cout << "putback success " << src->pub.bytes_in_buffer << std::endl;
}

GLOBAL(void) 
jpeg_istream_src (j_decompress_ptr cinfo, std::istream& inStream) {
	JPEGStreamSource *src; 
	
	if (cinfo->src == NULL) /* first time for this JPEG object? */ 
		cinfo->src = (struct jpeg_source_mgr *) (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(JPEGStreamSource)); 
	
	src = (JPEGStreamSource*) cinfo->src;
	src->pub.init_source = init_sourceFunc; 
	src->pub.fill_input_buffer = fill_input_bufferFunc; 
	src->pub.skip_input_data = skip_input_dataFunc; 
	src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */ 
	src->pub.term_source = term_sourceFunc; 
	src->is = &inStream;
	src->readCnt = 0;
	src->pub.next_input_byte = (JOCTET*)src->c;
	src->pub.bytes_in_buffer = 0;
} 

GLOBAL(int) 
jpeg_istream_revert (j_decompress_ptr cinfo) {
	JPEGStreamSource  *src = (JPEGStreamSource*)cinfo->src;
	if(src->readCnt != 1)
		return (src->readCnt==0);
	//std::streamoff numRead = ( (char*)src->pub.next_input_byte - src->c ) + src->pub.bytes_in_buffer;
	src->is->seekg( -(src->is->gcount()+1), std::ios_base::cur);
	return src->is->good();
}


#ifdef __cplusplus
}
#endif
