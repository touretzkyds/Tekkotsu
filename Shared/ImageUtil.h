//-*-c++-*-
#ifndef INCLUDED_image_util_h_
#define INCLUDED_image_util_h_

#include <string>

struct jpeg_compress_struct;

//! Provides a variety of straightforward calls to compress or decompress images in JPEG or PNG formats
namespace image_util {
	
	//! @name Generic File Utilities
	
	//! loads a file into memory, returns true if successful, storing resulting buffer information in @a buf and @a size
	/*! uses mmap unless LOADFILE_NO_MMAP is defined, in which case it uses 'new' and fread() */
	bool loadFile(const std::string& file, char*& buf, size_t& size);
	//! releases memory from a previous call to loadFile, triggering munmap() or 'delete' as appropriate
	void releaseFile(char* buf, size_t size);
	
	//@}
	
	
	
	//! @name Decode Header Information (from memory buffer)
	
	//! decodes an image header already in memory -- if it looks like a PNG decodePNG() will be called, otherwise decodeJPEG(); returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels*/
	bool decodeImage(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels);
	
	//! decodes a JPEG header already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels */
	bool decodeJPEG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels);

	//! decodes a PNG header already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels */
	bool decodePNG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels);
	
	//@}
	
	
	
	//! @name Decompress From Memory Buffer
	
	//! decodes an image already in memory -- if it looks like a PNG decodePNG() will be called, otherwise decodeJPEG(); returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * (This is a handy way to read the image header only -- pass ((char*)NULL)-1 as @a outbuf and 0 for @a outbufSize,
	 * the function will return false after reading the header and filling in width, height, and channels)
	 * The image will be written in row order, with channels interleaved. */
	bool decodeImage(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//! decodes a JPEG image already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param filename optional parameter, used if warnings are raised
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool decodeJPEG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, const std::string& filename="");

	//! decodes a PNG image already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool decodePNG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//@}
	
	
	
	//! @name Decode Header Information (from stream)
	
	//! decodes an image header already in memory -- if it looks like a PNG decodePNG() will be called, otherwise decodeJPEG(); returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels*/
	bool decodeImage(std::istream& inStream, size_t& width, size_t& height, size_t& channels);
	
	//! decodes a JPEG header already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels */
	bool decodeJPEG(std::istream& inStream, size_t& width, size_t& height, size_t& channels);
	
	//! decodes a PNG header already in memory, returns true if successful
	/*! @param inbuf input memory buffer containing compressed image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels */
	bool decodePNG(std::istream& inStream, size_t& width, size_t& height, size_t& channels);
	
	//@}
	
	
	
	//! @name Decompress From Input Stream
	
	//! decodes am image from a standard library input stream -- if it looks like a PNG decodePNG() will be called, otherwise decodeJPEG(); returns true if successful
	/*! @param inStream input stream from which to read images
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool decodeImage(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//! decodes a JPEG image from a standard library input stream, returns true if successful
	/*! @param inStream input stream providing image
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param filename optional parameter, used if warnings are raised
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool decodeJPEG(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, const std::string& filename="");

	//! decodes a PNG image from a standard library input stream, returns true if successful
	/*! @param inStream input stream from which to read images
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool decodePNG(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//@}
	
	bool decodePNGToDepth(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//! @name Decompress from File
	
	//! decodes an image from file on disk -- if it looks like a PNG decodePNG() will be called, otherwise decodeJPEG(); returns true if successful
	/*! @param file path to file to load
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool loadImage(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//! decodes a JPEG from disk, returns true if successful
	/*! @param file path to file to load
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool loadJPEG(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//! decodes a PNG from disk, returns true if successful
	/*! @param file path to file to load
	 *  @param width the image width
	 *  @param height the image height
	 *  @param channels the number of color channels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *
	 * If @a outbuf is pre-allocated and outbufSize is less than width*height*channels, the function will return false.
	 * The image will be written in row order, with channels interleaved. */
	bool loadPNG(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize);
	
	//@}
	
	
	
	//! @name Compression (In-Memory only)
	
	//! encodes a JPEG from a pixel buffer into another memory buffer, returns number of bytes used, 0 if error
	/*! @param inbuf input memory buffer containing the image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param inbufChannels the number of color channels in the source image; either 1 (grayscale), 3 (YUV), or -3U (RGB)
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param outbufChannels the number of color channels desired in the destination image (only downsample from color to grayscale)
	 *  @param quality how well to reproduce the image, 0-100
	 *
	 *  If @a outbuf is NULL, one of size @f$ width \cdot height \cdot outbufChannels \cdot (quality/2+25)/100+500 @f$ will be allocated for you.
	 *  (just a heuristic size... note that this won't all be used, and can't entirely guarantee it'll be enough!)
	 *
	 *  If @a inbufChannels is 3, @a outbufChannels can be either 3 or 1.  If 1, the first channel of
	 *  @a inbuf is used.  (pre-increment @a inbuf to use a different channel...)  If @a inbufChannels
	 *  is 1, outbufChannels must also be 1. */
	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality);
	
	//! encodes a JPEG from a pixel buffer into another memory buffer, returns number of bytes used, 0 if error
	/*! @param inbuf input memory buffer containing the image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param inbufChannels the number of color channels in the source image; either 1 (grayscale), 3 (YUV), or -3U (RGB)
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param outbufChannels the number of color channels desired in the destination image (only downsample from color to grayscale)
	 *  @param quality how well to reproduce the image, 0-100
	 *  @param cinfo allows you to use a pre-allocated jpeg structure instead of having the function recreate it each time
	 *
	 *  If @a outbuf is NULL, one of size @f$ width \cdot height \cdot outbufChannels \cdot (quality/2+25)/100+500 @f$ will be allocated for you.
	 *  (just a heuristic size... note that this won't all be used, and can't entirely guarantee it'll be enough!)
	 *
	 *  If @a inbufChannels is 3, @a outbufChannels can be either 3 or 1.  If 1, the first channel of
	 *  @a inbuf is used.  (pre-increment @a inbuf to use a different channel...)  If @a inbufChannels
	 *  is 1, outbufChannels must also be 1. */
	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality, jpeg_compress_struct& cinfo);
	
	//! encodes a JPEG from a pixel buffer into another memory buffer, returns number of bytes used, 0 if error
	/*! @param inbuf input memory buffer containing the image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param inbufChannels the number of color channels in the source image; either 1 (grayscale), 3 (YUV), or -3U (RGB)
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param outbufChannels the number of color channels desired in the destination image (only downsample from color to grayscale)
	 *  @param quality how well to reproduce the image, 0-100
	 *  @param yskip increment for the y channel
	 *  @param uvskip increment for the u and v channels
	 *  @param cinfo allows you to use a pre-allocated jpeg structure instead of having the function recreate it each time
	 *
	 *  If @a outbuf is NULL, one of size @f$ width \cdot height \cdot outbufChannels \cdot (quality/2+25)/100+500 @f$ will be allocated for you.
	 *  (just a heuristic size... note that this won't all be used, and can't entirely guarantee it'll be enough!)
	 *
	 *  If @a inbufChannels is 3, @a outbufChannels can be either 3 or 1.  If 1, the first channel of
	 *  @a inbuf is used.  (pre-increment @a inbuf to use a different channel...)  If @a inbufChannels
	 *  is 1, outbufChannels must also be 1. */
	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality, unsigned int yskip, unsigned int uvskip, jpeg_compress_struct& cinfo);

	//! encodes a PNG from a pixel buffer into another memory buffer, returns number of bytes used, 0 if error
	/*! @param inbuf input memory buffer containing the image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param inbufChannels the number of color channels in the source image; either 1 (grayscale), or 3 (color); must match @a outbufChannels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param outbufChannels the number of color channels desired in the destination image; must match @a inbufChannels
	 *
	 *  If @a outbuf is NULL, one of size @f$ width \cdot height \cdot outbufChannels + 500 @f$ will be allocated for you.
	 *  (just a heuristic size... note that this won't all be used, and can't entirely guarantee it'll be enough!)
	 *
	 *  Uses the default compression level, which gets most of the compression available without encurring excessive
	 *  computation.  According to libpng documentation, is equivalent to passing '6' as compressionLevel to the other version of the function.
	 *
	 *  Currently doesn't support changes in channels, so @a inbufChannels must match @a outbufChannels */
	size_t encodePNG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels);
	
	//! encodes a PNG from a pixel buffer into another memory buffer, returns number of bytes used, 0 if error
	/*! @param inbuf input memory buffer containing the image
	 *  @param inbufSize the size of @a inbuf allocation
	 *  @param width the image width
	 *  @param height the image height
	 *  @param inbufChannels the number of color channels in the source image; either 1 (grayscale), or 3 (color); must match @a outbufChannels
	 *  @param outbuf on input, a buffer to use for decompression; if NULL, a new buffer will be allocated and assigned
	 *  @param outbufSize if @a outbuf is non-NULL, this should indicate the size of @a outbuf
	 *  @param outbufChannels the number of color channels desired in the destination image; must match @a inbufChannels
	 *  @param compressionLevel a value 0 (none, grows slightly) through 9 (slow, but smallest) to pass to zlib for size vs. speed.  Compression is always lossless, so this does not affect image quality.
	 *
	 *  If @a outbuf is NULL, one of size @f$ width \cdot height \cdot outbufChannels + 500 @f$ will be allocated for you.
	 *  (just a heuristic size... note that this won't all be used, and can't entirely guarantee it'll be enough!)
	 *
	 *  Currently doesn't support changes in channels, so @a inbufChannels must match @a outbufChannels */
	size_t encodePNG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int compressionLevel);
	
	//@}
	
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
