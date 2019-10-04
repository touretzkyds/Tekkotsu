#include "jpeg_mem_src.h"

#ifdef JPEGMEMSRC_UNUSED 
#elif defined(__GNUC__) && __GNUC__>3 
# define JPEGMEMSRC_UNUSED(x) UNUSED_##x __attribute__((unused)) 
# define JPEGMEMSRC_BODY_UNUSED(x) /*no body necessary*/
#elif defined(__LCLINT__) 
# define JPEGMEMSRC_UNUSED(x) /*@unused@*/ x 
# define JPEGMEMSRC_BODY_UNUSED(x) /*no body necessary*/
#else 
# define JPEGMEMSRC_UNUSED(x) UNUSED_##x 
# define JPEGMEMSRC_BODY_UNUSED(x) (void)UNUSED_##x /* ugly hack to avoid warning */
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* Theoretically, could be used to load several consecutive
 *  JPEGs from memory, but not really tested, some tweaks
 *  would probably be required */
	
//#include <jpeglib.h> 
#include <jerror.h> 

typedef struct {
	struct jpeg_source_mgr  pub;
	JOCTET * buf;
	size_t bufsize;
	size_t lastused; /* on completion, set to number of bytes used from buf */
} JPEGSource;

static void init_sourceFunc(j_decompress_ptr cinfo) {
	JPEGSource  *src = (JPEGSource*)cinfo->src;
	src->pub.next_input_byte = src->buf;
	src->pub.bytes_in_buffer = src->bufsize;
}
static boolean fill_input_bufferFunc(j_decompress_ptr JPEGMEMSRC_UNUSED(cinfo)) {
	JPEGMEMSRC_BODY_UNUSED(cinfo);
	return false;
}
static void skip_input_dataFunc(j_decompress_ptr cinfo, long num_bytes) {
	JPEGSource  *src = (JPEGSource*)cinfo->src;
	if (num_bytes > 0) {
		if ((size_t)num_bytes > src->pub.bytes_in_buffer)
			src->pub.bytes_in_buffer = 0;
		else {
			src->pub.next_input_byte += num_bytes;
			src->pub.bytes_in_buffer -= num_bytes;
		}
	}
}
static void term_sourceFunc(j_decompress_ptr cinfo) {
	JPEGSource* src = (JPEGSource*) cinfo->src;
	if(src->pub.next_input_byte==NULL)
		src->lastused=0;
	else
		src->lastused = src->buf - src->pub.next_input_byte;
}

GLOBAL(void) 
tk_jpeg_mem_src (j_decompress_ptr cinfo, JOCTET * inbuf, size_t maxlen) {
	JPEGSource *src; 
	
	if (cinfo->src == NULL) /* first time for this JPEG object? */ 
		cinfo->src = (struct jpeg_source_mgr *) (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(JPEGSource)); 
	
	src = (JPEGSource*) cinfo->src;
	src->pub.init_source = init_sourceFunc; 
	src->pub.fill_input_buffer = fill_input_bufferFunc; 
	src->pub.skip_input_data = skip_input_dataFunc; 
	src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */ 
	src->pub.term_source = term_sourceFunc; 
	src->pub.next_input_byte = src->buf = inbuf; 
	src->pub.bytes_in_buffer = src->bufsize = maxlen; 
	src->lastused=0;
} 

#ifdef __cplusplus
}
#endif
