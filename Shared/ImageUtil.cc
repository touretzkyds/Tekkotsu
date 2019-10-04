#include "ImageUtil.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <iostream>
#include "Shared/jpeg-6b/jpeg_mem_dest.h"
#include "Shared/jpeg-6b/jpeg_mem_src.h"
#include "Shared/jpeg-6b/jpeg_istream_src.h"
#ifndef NO_TEKKOTSU_CONFIG
#  include "Shared/Config.h"
#endif
#ifndef LOADFILE_NO_MMAP
#  include <fcntl.h>
#  include <errno.h>
#  include <sys/types.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#endif

#include <png.h>
#include <zlib.h>
extern "C" {
#include <jpeglib.h>
}
using namespace std; 

namespace image_util {
	
	bool loadFile(const std::string& file, char*& buf, size_t& size) {
		struct stat statbuf;
		if(stat(file.c_str(),&statbuf)!=0) {
			perror("image_util::loadFile");
			return false;
		}
#ifdef LOADFILE_NO_MMAP
		FILE * infile= fopen(file.c_str(), "rb");
		if (infile==NULL) {
			int err=errno;
			cerr << "image_util::loadFile(): Could not open '" << file << "' (fopen:" << strerror(err) << ")" << endl;
			return false;
		}
		char* inbuf=new char[statbuf.st_size];
		if(!fread(inbuf,statbuf.st_size,1,infile)) {
			int err=errno;
			cerr << "image_util::loadFile(): Could not load '" << file << "' (fread:" << strerror(err) << ")" << endl;
			if(fclose(infile))
				perror("Warning: image_util::loadFile(), on fclose");
			delete [] inbuf;
			return false;
		}
		if(fclose(infile))
			perror("Warning: image_util::loadFile(), on fclose");
#else /*use mmap to load file into memory*/
		int infd=open(file.c_str(),O_RDONLY,0666);
		if(infd<0) {
			int err=errno;
			cerr << "image_util::loadFile(): Could not open '" << file << "' (open:" << strerror(err) << ")" << endl;
			return false;
		}
		void* inbuf=mmap(NULL,(size_t)statbuf.st_size,PROT_READ,MAP_FILE|MAP_PRIVATE,infd,0);
		if (inbuf == MAP_FAILED) { /* MAP_FAILED generally defined as ((void*)-1) */
			int err=errno;
			cerr << "image_util::loadFile(): Could not load '" << file << "' (mmap:" << strerror(err) << ")" << endl;
			if(close(infd)<0)
				perror("Warning: image_util::loadFile(), on closing temporary file descriptor from open");
			return false;
		}
		if(close(infd)<0)
			perror("Warning: image_util::loadFile(), on closing temporary file descriptor from open");
#endif
		buf=static_cast<char*>(inbuf);
		size=(size_t)statbuf.st_size;
		return true;
	}

#ifdef LOADFILE_NO_MMAP
	void releaseFile(char* buf, size_t /*size*/) {
		delete [] buf;
	}
#else
	void releaseFile(char* buf, size_t size) {
		if(munmap(buf,size))
			perror("Warning: image_util::releaseFile(), on munmap");
	}
#endif

	/// @cond INTERNAL
	
	//! provides error handling in the case of a jpeg error, inspired by libpng's handling
	struct my_error_mgr {
		//! fields used for jpeg error handling (this MUST BE FIRST MEMBER so libjpeg can treat my_error_mgr as a jpeg_error_mgr struct — C-style "inheritance")
		struct jpeg_error_mgr pub;
		char buffer[JMSG_LENGTH_MAX]; //!< message indicating reason for error
		jmp_buf setjmp_buffer;	//!< a jump buffer to trigger on error, similar to exception except not as safe (use a C library, this is what you get...)
	};
	//! called on error from libjpeg, @a cinfo should be an instance of my_error_mgr
	static void my_error_exit (j_common_ptr cinfo)
	{
		/* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
		my_error_mgr * myerr = (my_error_mgr *) cinfo->err;
		(*cinfo->err->format_message) (cinfo, myerr->buffer);
		/* Return control to the setjmp point */
		longjmp(myerr->setjmp_buffer, 1);
	}
	
	typedef int (*cinfo_cleanup_f)(jpeg_decompress_struct* cinfo);

	//! common implementation for variants of decodeJPEG() to call after they've set up the decompression structure
	bool decodeJPEG(jpeg_decompress_struct& cinfo, my_error_mgr& jerr, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, const std::string& filename, cinfo_cleanup_f cinfo_cleanup=NULL) {
		jpeg_read_header(&cinfo, true);
		cinfo.out_color_space=JCS_YCbCr;
		jpeg_calc_output_dimensions(&cinfo);
		width=cinfo.output_width;
		height=cinfo.output_height;
		channels=cinfo.output_components;
		if(cinfo.output_width==0 || cinfo.output_height==0 || cinfo.output_components==0) {
			jpeg_destroy_decompress(&cinfo);
			return true;
		}
		if(outbuf==NULL) {
			outbufSize=width*height*channels;
			outbuf=new char[outbufSize];
		} else if(width*height*channels>outbufSize) {
			// not enough output space
			if(cinfo_cleanup!=NULL)
				(*cinfo_cleanup)(&cinfo);
			jpeg_destroy_decompress(&cinfo);
			return false;
		}
		
		// setup image destination
		unsigned int remain=cinfo.output_height;
		unsigned int row_stride = cinfo.output_width * cinfo.output_components;
		JSAMPROW rows[remain];
		rows[0]=(JSAMPLE*)outbuf;
		for(unsigned int i=1; i<remain; i++)
			rows[i]=rows[i-1]+row_stride;
		JSAMPROW* curpos=rows;
		//JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
		
		// read image
		jpeg_start_decompress(&cinfo);
		while (remain>0) {
			unsigned int used=jpeg_read_scanlines(&cinfo, curpos, remain);
			curpos+=used;
			remain-=used;
		}
		jpeg_finish_decompress(&cinfo);
		if(jerr.pub.num_warnings>0) {
			if(filename.size()>0)
				cerr << "Warning: image_util JPEG decompression of '" << filename << "' had warnings" << endl;
			else
				cerr << "Warning: image_util JPEG decompression had warnings" << endl;
			jerr.pub.num_warnings=0;
		}
		
		// teardown
		jpeg_destroy_decompress(&cinfo);
		return true;
	}
	
	/// @endcond
	
	bool decodeJPEG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels) {
		jpeg_decompress_struct cinfo; //!< used to interface with libjpeg - holds compression parameters and state
		my_error_mgr jerr;          //!< used to interface with libjpeg - gives us access to error information
		cinfo.err = jpeg_std_error(&jerr.pub);
		jerr.pub.error_exit = my_error_exit;
		/* Establish the setjmp return context for my_error_exit to use. */
		if (setjmp(jerr.setjmp_buffer)) {
			/* If we get here, the JPEG code has signaled an error.
			 * We need to clean up the JPEG object, close the input file, and return.
			 */
			jpeg_destroy_decompress(&cinfo);
			cerr << "JPEG header check failed: " << jerr.buffer << endl;
			return false;
		}
		
		jpeg_create_decompress(&cinfo);
		
		// get image header info
		tk_jpeg_mem_src(&cinfo, (JOCTET*)inbuf, inbufSize);
		jpeg_read_header(&cinfo, true);
		cinfo.out_color_space=JCS_YCbCr;
		jpeg_calc_output_dimensions(&cinfo);
		width=cinfo.output_width;
		height=cinfo.output_height;
		channels=cinfo.output_components;
		jpeg_destroy_decompress(&cinfo);
		return true;
	}
	
	bool decodeJPEG(std::istream& inStream, size_t& width, size_t& height, size_t& channels) {
		jpeg_decompress_struct cinfo; //!< used to interface with libjpeg - holds compression parameters and state
		my_error_mgr jerr;          //!< used to interface with libjpeg - gives us access to error information
		cinfo.err = jpeg_std_error(&jerr.pub);
		jerr.pub.error_exit = my_error_exit;
		/* Establish the setjmp return context for my_error_exit to use. */
		if (setjmp(jerr.setjmp_buffer)) {
			/* If we get here, the JPEG code has signaled an error.
			 * We need to clean up the JPEG object, close the input file, and return.
			 */
			jpeg_istream_revert(&cinfo);
			jpeg_destroy_decompress(&cinfo);
			cerr << "JPEG stream header check failed: " << jerr.buffer << endl;
			return false;
		}
		
		jpeg_create_decompress(&cinfo);
		
		// get image header info
		jpeg_istream_src(&cinfo, inStream);
		jpeg_read_header(&cinfo, true);
		cinfo.out_color_space=JCS_YCbCr;
		jpeg_calc_output_dimensions(&cinfo);
		width=cinfo.output_width;
		height=cinfo.output_height;
		channels=cinfo.output_components;
		bool reverted = jpeg_istream_revert(&cinfo);
		jpeg_destroy_decompress(&cinfo);
		return reverted;
	}
	
	bool decodeJPEG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, const std::string& filename/*=""*/) {
		jpeg_decompress_struct cinfo; //!< used to interface with libjpeg - holds compression parameters and state
		my_error_mgr jerr;          //!< used to interface with libjpeg - gives us access to error information
		cinfo.err = jpeg_std_error(&jerr.pub);
		jerr.pub.error_exit = my_error_exit;
		/* Establish the setjmp return context for my_error_exit to use. */
		if (setjmp(jerr.setjmp_buffer)) {
			/* If we get here, the JPEG code has signaled an error.
			* We need to clean up the JPEG object, close the input file, and return.
			*/
			jpeg_destroy_decompress(&cinfo);
			cerr << "JPEG decompression failed: " << jerr.buffer << endl;
			return false;
		}

		jpeg_create_decompress(&cinfo);
		
		// get image header info
		tk_jpeg_mem_src(&cinfo, (JOCTET*)inbuf, inbufSize);
		return decodeJPEG(cinfo,jerr,width,height,channels,outbuf,outbufSize,filename);
	}

	bool decodeJPEG(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, const std::string& filename) {
		jpeg_decompress_struct cinfo; //!< used to interface with libjpeg - holds compression parameters and state
		my_error_mgr jerr;          //!< used to interface with libjpeg - gives us access to error information
		cinfo.err = jpeg_std_error(&jerr.pub);
		jerr.pub.error_exit = my_error_exit;
		/* Establish the setjmp return context for my_error_exit to use. */
		if (setjmp(jerr.setjmp_buffer)) {
			/* If we get here, the JPEG code has signaled an error.
			* We need to clean up the JPEG object, close the input file, and return.
			*/
			jpeg_istream_revert(&cinfo);
			jpeg_destroy_decompress(&cinfo);
			cerr << "JPEG stream decompression failed: " << jerr.buffer << endl;
			return false;
		}

		jpeg_create_decompress(&cinfo);
		
		// get image header info
		jpeg_istream_src(&cinfo, inStream);
		return decodeJPEG(cinfo,jerr,width,height,channels,outbuf,outbufSize,filename,&jpeg_istream_revert);
	}


	/// @cond INTERNAL
	
	//! stores progress of user_read_png_data() between calls
	struct png_read_mem_status {
		png_bytep buf; //!< pointer to the beginning of the read buffer, the current position within the buffer is indicated by #offset
		size_t bufsize; //!< size of the read buffer pointed to by #buf
		size_t offset; //!< current read position within #buf
	};
	//! user callback function for reading a png from user parameters stored in @a png_ptr into @a data
	static void user_read_png_data(png_structp png_ptr, png_bytep data, png_size_t length) {
		png_read_mem_status* status=(png_read_mem_status*)(png_get_io_ptr(png_ptr));
		size_t endoff=status->offset+length;
		if(endoff<=status->bufsize) {
			memcpy(data,status->buf+status->offset,length);
			status->offset=endoff;
		} else {
			png_error(png_ptr, "Read Error - ran out of file");
		}
	}
	
	struct png_read_stream_status {
		std::istream* is;
		std::streamoff n;
	};
	
	//! user callback function for reading a png from user parameters stored in @a png_ptr into @a data
	static void user_read_png_istream(png_structp png_ptr, png_bytep data, png_size_t length) {
		png_read_stream_status* status=(png_read_stream_status*)(png_get_io_ptr(png_ptr));
		if(!status->is->read((char*)data,length))
			png_error(png_ptr, "Read Error - stream closed early");
		status->n+=length;
	}
	
	//! stores progress of user_write_png_data() between calls
	struct png_write_mem_status {
		png_bytep buf;  //!< beginning of buffer being writen into (doesn't move with progress)
		size_t bufsize; //!< total size of buffer
		size_t offset;  //!< position within buffer, i.e. amount of buffer written so far
	};
	//! user callback function for writing a png at @a data into user parameters stored in @a png_ptr
	void user_write_png_data(png_structp png_ptr, png_bytep data, png_size_t length) {
		png_write_mem_status* status=(png_write_mem_status*)(png_get_io_ptr(png_ptr));
		size_t endoff=status->offset+length;
		if(endoff<=status->bufsize) {
			memcpy(status->buf+status->offset,data,length);
			status->offset=endoff;
		} else {
			png_error(png_ptr, "Write Error - ran out of file");
		}
	}
	//! user callback function for flushing results of user_write_png_data() (this is a no-op)
	void user_flush_png_data(png_structp /*png_ptr*/) {}
	
	typedef int (*png_cleanup_f)(png_structp png_ptr);
	int png_istream_revert(png_structp png_ptr) {
		png_read_stream_status* status=(png_read_stream_status*)(png_get_io_ptr(png_ptr));
		status->is->seekg(-status->n,ios_base::cur);
		status->n=0;
		return status->is->good();
	}

	//! Common implementation for decodePNG() implementations
	bool decodePNG(png_structp png_ptr, png_infop info_ptr, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, png_cleanup_f png_cleanup=NULL) {
		// get image header info
		png_read_info(png_ptr, info_ptr);
		width=png_get_image_width(png_ptr, info_ptr);
		height=png_get_image_height(png_ptr, info_ptr);
		channels=3;
		if(width==0 || height==0)
			return true;
		if(outbuf==NULL) {
			outbufSize=width*height*channels;
			outbuf=new char[outbufSize];
		} else if(width*height*channels>outbufSize) {
			if(png_cleanup)
				(*png_cleanup)(png_ptr);
			png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
			return false;
		}
		
		// setup image destination
		size_t bit_depth=png_get_bit_depth(png_ptr, info_ptr);
		if(bit_depth == 16)
			png_set_strip_16(png_ptr);
		if (bit_depth < 8)
			png_set_packing(png_ptr);
		
		size_t color_type=png_get_color_type(png_ptr, info_ptr);
		if (color_type & PNG_COLOR_MASK_ALPHA)
			png_set_strip_alpha(png_ptr);
		if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
			png_set_gray_to_rgb(png_ptr);
		
		png_color_16 my_background;
		my_background.index=0;
		my_background.red=1<<15;
		my_background.green=1<<15;
		my_background.blue=1<<15;
		my_background.gray=1<<15;
		png_color_16p image_background=NULL;
		if (png_get_bKGD(png_ptr, info_ptr, &image_background))
			png_set_background(png_ptr, image_background, PNG_BACKGROUND_GAMMA_FILE, 1, 1.0);
		else
			png_set_background(png_ptr, &my_background, PNG_BACKGROUND_GAMMA_SCREEN, 0, 1.0);
		png_read_update_info(png_ptr, info_ptr);
		
		// read image
		size_t rowbytes=png_get_rowbytes(png_ptr, info_ptr);
		/*png_bytep row_pointers[height];
		row_pointers[0]=reinterpret_cast<png_bytep>(data->Base()+FULL_HEADER_SIZE);
		for(unsigned int h=1; h<height; ++h)
			row_pointers[h]=row_pointers[h-1]+rowbytes;
		png_read_image(png_ptr, row_pointers);*/
		png_bytep row=reinterpret_cast<png_bytep>(outbuf);
		for(unsigned int h=0; h<height; ++h) {
			if(row+rowbytes>reinterpret_cast<png_bytep>(outbuf+outbufSize)) {
				cerr << "image_util::decodePNG ran out of PNG buffer space" << endl;
				break;
			}

			png_read_row(png_ptr, row, NULL);
			row+=rowbytes;
		}
		png_read_end(png_ptr, NULL);
		
		// teardown
		png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
		return true;
	}

	/// @endcond
	
	bool decodePNG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels) {
		png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
		if (!png_ptr)
			return false;
		png_infop info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			return false;
		}
		if (setjmp(png_jmpbuf(png_ptr))) {
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			return false;
		}
		
		png_read_mem_status read_status;
		read_status.buf=(png_bytep)inbuf;
		read_status.bufsize=inbufSize;
		read_status.offset=0;
		png_set_read_fn(png_ptr, &read_status, user_read_png_data);
		
		// get image header info
		png_read_info(png_ptr, info_ptr);
		width=png_get_image_width(png_ptr, info_ptr);
		height=png_get_image_height(png_ptr, info_ptr);
		channels=3;
		png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
		return true;
	}
	
	bool decodePNG(std::istream& inStream, size_t& width, size_t& height, size_t& channels) {
		png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
		if (!png_ptr)
			return false;
		png_infop info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			return false;
		}
		if (setjmp(png_jmpbuf(png_ptr))) {
			png_istream_revert(png_ptr);
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			return false;
		}
		
		png_read_stream_status status;
		status.is = &inStream;
		status.n = 0;
		png_set_read_fn(png_ptr, &status, user_read_png_istream);
		
		// get image header info
		png_read_info(png_ptr, info_ptr);
		width=png_get_image_width(png_ptr, info_ptr);
		height=png_get_image_height(png_ptr, info_ptr);
		channels=3;
		bool reverted = png_istream_revert(png_ptr);
		png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
		return reverted;
	}
	
	bool decodePNG(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
		if (!png_ptr)
			return false;
		png_infop info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			return false;
		}
		if (setjmp(png_jmpbuf(png_ptr))) {
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			return false;
		}
		
		png_read_mem_status read_status;
		read_status.buf=(png_bytep)inbuf;
		read_status.bufsize=inbufSize;
		read_status.offset=0;
		png_set_read_fn(png_ptr, &read_status, user_read_png_data);
		return decodePNG(png_ptr, info_ptr, width, height, channels, outbuf, outbufSize);
	}

	bool decodePNGToDepth(png_structp png_ptr, png_infop info_ptr, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize, png_cleanup_f png_cleanup=NULL) {
		// get image header info
		png_read_info(png_ptr, info_ptr);
		width=png_get_image_width(png_ptr, info_ptr);
		height=png_get_image_height(png_ptr, info_ptr);
		channels=2;
		if(width==0 || height==0)
			return true;
		if(outbuf==NULL) {
			outbufSize=width*height*channels;
			outbuf=new char[outbufSize];
		} else if(width*height*channels>outbufSize) {
			if(png_cleanup)
				(*png_cleanup)(png_ptr);
			png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
			return false;
		}
		
		// setup image destination
		size_t bit_depth=png_get_bit_depth(png_ptr, info_ptr);
		if(bit_depth == 16)
			png_set_strip_16(png_ptr);
		if (bit_depth < 8)
			png_set_packing(png_ptr);
		
		size_t color_type=png_get_color_type(png_ptr, info_ptr);
		if (color_type & PNG_COLOR_MASK_ALPHA)
			png_set_strip_alpha(png_ptr);
		if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
			png_set_gray_to_rgb(png_ptr);
		
		png_color_16 my_background;
		my_background.index=0;
		my_background.red=1<<15;
		my_background.green=1<<15;
		my_background.blue=1<<15;
		my_background.gray=1<<15;
		png_color_16p image_background=NULL;
		if (png_get_bKGD(png_ptr, info_ptr, &image_background))
			png_set_background(png_ptr, image_background, PNG_BACKGROUND_GAMMA_FILE, 1, 1.0);
		else
			png_set_background(png_ptr, &my_background, PNG_BACKGROUND_GAMMA_SCREEN, 0, 1.0);
		png_read_update_info(png_ptr, info_ptr);

		// read image row by row
		size_t rowbytes=png_get_rowbytes(png_ptr, info_ptr);

		png_bytep row = (png_bytep) (new char*[rowbytes]);

		for (unsigned int i = 0; i < rowbytes; ++i) {
			row[i] = 0;
		}

		// Cast the output buffer pointer to an unsigned short to do some pointer
		// arithmetic
		unsigned short* outputBuf = (unsigned short*) outbuf;

		for(unsigned int h=0; h<height; ++h) {
			// Read in a row from the image
			png_read_row(png_ptr, row, NULL);
			// compute the depth from the pixels
			for (unsigned int w = 0; w < width; ++w) {
				// Just need the intensity channel
				int y = row[0 + 3 * w];
				//int u = row[1 + 3 * w];
				//int v = row[2 + 3 * w];

				// Assumed bounds on y-channel are 16-220
				double d = (y - 16) / 220.0 * 10000;
                                unsigned short depth = d;
                                if (depth > 10000) {
				  depth = 10000;
				}
                                if (depth < 800) {
                                  depth = 0;
                                }
				*outputBuf++ = depth;
			}
		}
                //cout << "(" << min_y << ", " << max_y << ")" << endl;

		png_read_end(png_ptr, NULL);
		
		// teardown
		delete(row);
		png_destroy_read_struct(&png_ptr, &info_ptr,(png_infopp)NULL);
		return true;
	}	

	bool decodePNGToDepth(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
		if (!png_ptr)
			return false;
		png_infop info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			return false;
		}
		if (setjmp(png_jmpbuf(png_ptr))) {
			png_istream_revert(png_ptr);
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			return false;
		}
		
		png_read_stream_status status;
		status.is = &inStream;
		status.n = 0;
		png_set_read_fn(png_ptr, &status, user_read_png_istream);
		return decodePNGToDepth(png_ptr, info_ptr, width, height, channels, outbuf, outbufSize, png_istream_revert);
	}

	bool decodePNG(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
		if (!png_ptr)
			return false;
		png_infop info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			return false;
		}
		if (setjmp(png_jmpbuf(png_ptr))) {
			png_istream_revert(png_ptr);
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			return false;
		}
		
		png_read_stream_status status;
		status.is = &inStream;
		status.n = 0;
		png_set_read_fn(png_ptr, &status, user_read_png_istream);
		return decodePNG(png_ptr, info_ptr, width, height, channels, outbuf, outbufSize, png_istream_revert);
	}

	
	
	const unsigned int TEST_HEADER_LEN=8;
	bool decodeImage(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels) {
		if(!png_sig_cmp((png_byte*)inbuf, 0, TEST_HEADER_LEN)) {
			return decodePNG(inbuf,inbufSize,width,height,channels);
		} else {
			return decodeJPEG(inbuf,inbufSize,width,height,channels);
		}
	}
	bool decodeImage(char* inbuf, size_t inbufSize, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		if(!png_sig_cmp((png_byte*)inbuf, 0, TEST_HEADER_LEN)) {
			return decodePNG(inbuf,inbufSize,width,height,channels,outbuf,outbufSize);
		} else {
			return decodeJPEG(inbuf,inbufSize,width,height,channels,outbuf,outbufSize);
		}
	}
	bool decodeImage(std::istream& inStream, size_t& width, size_t& height, size_t& channels) {
		char header[TEST_HEADER_LEN];
		inStream.read(header,TEST_HEADER_LEN);
		size_t cnt = (size_t)inStream.gcount();
		if(cnt==0)
			return false;
		if(!inStream.seekg(-cnt,ios_base::cur))
			return false;
		bool png = !png_sig_cmp((png_byte*)header, 0, cnt);
		if(png)
			return false; //return decodePNG(inStream, width, height, channels);
		else
			return decodeJPEG(inStream, width, height, channels);
	}
	bool decodeImage(std::istream& inStream, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		char header[TEST_HEADER_LEN];
		inStream.read(header,TEST_HEADER_LEN);
		size_t cnt = (size_t)inStream.gcount();
		if(cnt==0)
			return false;
		if(!inStream.seekg(-cnt,ios_base::cur))
			return false;
		bool png = !png_sig_cmp((png_byte*)header, 0, cnt);
		if(png)
			return decodePNG(inStream, width, height, channels, outbuf, outbufSize);
		else
			return decodeJPEG(inStream, width, height, channels, outbuf, outbufSize);
	}



	bool loadJPEG(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		char* inbuf;
		size_t inbufSize;
		if(!loadFile(file, inbuf, inbufSize))
			return false;
		bool res=decodeJPEG(inbuf,inbufSize,width,height,channels,outbuf,outbufSize);
		releaseFile(inbuf,inbufSize);
		return res;
	}
	bool loadPNG(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		char* inbuf;
		size_t inbufSize;
		if(!loadFile(file, inbuf, inbufSize))
			return false;
		bool res=decodePNG(inbuf,inbufSize,width,height,channels,outbuf,outbufSize);
		releaseFile(inbuf,inbufSize);
		return res;
	}
	bool loadImage(const std::string& file, size_t& width, size_t& height, size_t& channels, char*& outbuf, size_t& outbufSize) {
		char* inbuf;
		size_t inbufSize;
		if(!loadFile(file, inbuf, inbufSize))
			return false;
		bool res=decodeImage(inbuf,inbufSize,width,height,channels,outbuf,outbufSize);
		releaseFile(inbuf,inbufSize);
		return res;
	}



	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality) {
		jpeg_compress_struct cinfo;
		jpeg_error_mgr jerr;
		// We set the err object before we create the compress...  the idea
		// is if the creation fails, we can still get the error as to why it failed.
		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);
		size_t ans=encodeJPEG(inbuf,inbufSize,width,height,inbufChannels,outbuf,outbufSize,outbufChannels,quality,cinfo);
		jpeg_destroy_compress(&cinfo);
		return ans;
	}
	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality, jpeg_compress_struct& cinfo) {
		return encodeJPEG(inbuf,inbufSize,width,height,inbufChannels,outbuf,outbufSize,outbufChannels,quality,1,1,cinfo);
	}
	size_t encodeJPEG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int quality, unsigned int yskip, unsigned int uvskip, jpeg_compress_struct& cinfo) {
		try {
			if(quality>100)
				quality=100;
			if(quality<0)
				quality=0;
			// check destination buffer
			if(outbuf==NULL) {
				outbufSize=width*height*outbufChannels*(quality+50)/200+768;
				outbuf=new char[outbufSize];
			}
			
			//pass the destination buffer and buffer size here
			tk_jpeg_mem_dest(&cinfo, reinterpret_cast<JOCTET*>(outbuf), outbufSize);
			
			// mode setup
			cinfo.image_width = width;
			cinfo.image_height = height;
			cinfo.input_components = inbufChannels;
			if(inbufChannels==1) {
				cinfo.in_color_space = JCS_GRAYSCALE;
			} else if(inbufChannels==3) {
				cinfo.in_color_space = JCS_YCbCr;
			} else if(inbufChannels==-3U) {
				// ugly hack to request internal conversion from RGB
				cinfo.input_components = inbufChannels = 3;
				cinfo.in_color_space = JCS_RGB;
			} else {
				cerr << "image_util JPEG compression failed - don't know how to compress into " << outbufChannels << " channels" << endl;
				return 0;
			}
			
			// parameter setup
			jpeg_set_defaults(&cinfo);
			jpeg_set_quality(&cinfo, quality, false); /* limit to baseline-JPEG values */
#ifndef NO_TEKKOTSU_CONFIG
			cinfo.dct_method=config->vision.jpeg_dct_method;
#endif
			if(cinfo.in_color_space==JCS_GRAYSCALE && inbufChannels!=1) {
				//special case, need to remove interleaved channels as we compress (single channel, grayscale)
				jpeg_start_compress(&cinfo, true);
				unsigned int row_stride = width*inbufChannels;	/* JSAMPLEs per row in image_buffer */
				JSAMPROW row_pointer[1] = { new JSAMPLE[width] };
				while (cinfo.next_scanline < cinfo.image_height) {
					if(inbufSize<row_stride) {
						cerr << "image_util ran out of input buffer during JPEG grayscale compression" << endl;
						break;
					}
					for(unsigned int x=0; x<width; x++) // copy over a row into temporary space
						row_pointer[0][x] = inbuf[x*inbufChannels];
					jpeg_write_scanlines(&cinfo, row_pointer, 1); // pass that row to libjpeg
					inbuf+=row_stride;
					inbufSize-=row_stride;
				}
				jpeg_finish_compress(&cinfo);
				delete [] row_pointer[0];
				
			} else	{
				if(cinfo.in_color_space==JCS_YCbCr) {
					unsigned int ysamp=1;
					unsigned int uvsamp=1;
					const unsigned int maxsamp=2;  // according to jpeg docs, this should be able to go up to 4, but I get error: "Sampling factors too large for interleaved scan"
					if(yskip>uvskip) {
						uvsamp=yskip-uvskip+1;
						if(uvsamp>maxsamp)
							uvsamp=maxsamp;
					} else {
						ysamp=uvskip-yskip+1;
						if(ysamp>maxsamp)
							ysamp=maxsamp;
					}
					cinfo.comp_info[0].h_samp_factor=ysamp;
					cinfo.comp_info[0].v_samp_factor=ysamp;
					cinfo.comp_info[1].h_samp_factor=uvsamp;
					cinfo.comp_info[1].v_samp_factor=uvsamp;
					cinfo.comp_info[2].h_samp_factor=uvsamp;
					cinfo.comp_info[2].v_samp_factor=uvsamp;
				}
				
				// compression
				jpeg_start_compress(&cinfo, true);
				unsigned int row_stride = width*inbufChannels;	/* JSAMPLEs per row in image_buffer */
				JSAMPROW row_pointer[1] = { const_cast<JSAMPROW>(reinterpret_cast<const JSAMPLE*>(inbuf)) };
				while (cinfo.next_scanline < cinfo.image_height) {
					if(inbufSize<row_stride) {
						cerr << "image_util ran out of input buffer during JPEG compression" << endl;
						break;
					}
					jpeg_write_scanlines(&cinfo, row_pointer, 1);
					row_pointer[0]+=row_stride;
					inbufSize-=row_stride;
				}
				jpeg_finish_compress(&cinfo);
			}
			
			// results
			return tk_jpeg_mem_size(&cinfo);
		} catch(const std::exception& ex) {
			std::cerr << "image_util Exception while compressing JPEG: " << ex.what() << std::endl; //really, can only be bad_alloc
			jpeg_finish_compress(&cinfo);
		}
		return 0;
	}

	size_t encodePNG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels) {
		return encodePNG(inbuf,inbufSize,width,height,inbufChannels,outbuf,outbufSize,outbufChannels,Z_DEFAULT_COMPRESSION);
	}
	
	size_t encodePNG(const char* inbuf, size_t inbufSize, size_t width, size_t height, size_t inbufChannels, char*& outbuf, size_t& outbufSize, size_t outbufChannels, int compressionLevel) {
		if(compressionLevel!=Z_DEFAULT_COMPRESSION) {
			if(compressionLevel<Z_NO_COMPRESSION)
				compressionLevel=Z_NO_COMPRESSION;
			if(compressionLevel>Z_BEST_COMPRESSION)
				compressionLevel=Z_BEST_COMPRESSION;
		}
		
		// check destination buffer
		if(outbuf==NULL) {
			outbufSize=width*height*outbufChannels + 256;
			if(compressionLevel==Z_NO_COMPRESSION)
				outbufSize=static_cast<size_t>(outbufSize*1.005+50); // with no compression, size will grow (plus constant header)
			else
				outbufSize=static_cast<size_t>(outbufSize*2.0/3.0+1);
			outbuf=new char[outbufSize];
		}
		
		// setup state structures
		png_structp  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
		if (!png_ptr) {
			cerr << "image_util::encodePNG(): png_create_write_struct failed" << endl;
			return 0;
		}
		png_infop  info_ptr = png_create_info_struct(png_ptr);
		if (!info_ptr) {
			png_destroy_write_struct(&png_ptr, NULL);
			cerr << "image_util::encodePNG(): png_create_info_struct failed" << endl;
			return 0;
		}
		
		png_write_mem_status write_status;
		write_status.buf=reinterpret_cast<png_byte*>(outbuf);
		write_status.bufsize=outbufSize;
		write_status.offset=0;
		png_set_write_fn(png_ptr, &write_status, user_write_png_data, user_flush_png_data);
		
		if(setjmp(png_jmpbuf(png_ptr))) {
			cerr << "An error occurred during PNG compression" << endl;
			png_destroy_write_struct(&png_ptr, &info_ptr);
			return 0;
		}
		
		// configure compression
		int bit_depth=8;
		int color_type;
		if(outbufChannels==3)
			color_type=PNG_COLOR_TYPE_RGB;
		else if(outbufChannels==1)
			color_type=PNG_COLOR_TYPE_GRAY;
		else {
			cerr << "image_util PNG compression failed - don't know how to compress into " << outbufChannels << " channels" << endl;
			return 0;
		}
		png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
		png_set_compression_level(png_ptr,compressionLevel);
		png_write_info(png_ptr, info_ptr);
		png_byte* row=const_cast<png_byte*>(reinterpret_cast<const png_byte*>(inbuf));
		const unsigned int inc=inbufChannels;
#ifdef DEBUG
		if( (color_type==PNG_COLOR_TYPE_RGB && inc!=3) || (color_type==PNG_COLOR_TYPE_GRAY && inc!=1) ) {
			cerr << "image_util::encodePNG() only supports color mode from sources with interleaving of 3 samples (increment==3), or grayscale from \"pure\" sources (increment==1)" << endl;
			png_write_end(png_ptr, NULL);
			return 0;
		}
#endif
		
		// do compression
		unsigned int row_stride = width*inc;
		const png_byte* endp=reinterpret_cast<const png_byte*>(row+inbufSize);
		for(unsigned int h=0; h<height; ++h) {
			if(row+row_stride>endp) {
				cerr << "image_util ran out of src image -- bad height?" << endl;
				break;
			}
			png_write_row(png_ptr, row);
			row+=row_stride;
		}
		png_write_end(png_ptr, NULL);
		png_destroy_write_struct(&png_ptr, &info_ptr);
		
		// return results
		return write_status.offset;
	}

} // image_util namespace

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
