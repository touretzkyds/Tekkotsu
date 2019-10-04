#include "FileSystemImageSource.h"
#include "Shared/ImageUtil.h"
#include "Shared/debuget.h"
#include <sys/types.h>
#include <sys/mman.h>

//better to put this here instead of the header
using namespace std; 

void FileSystemImageSource::ImageInfo::prepare() {
	if(prepared)
		return;
	if(data==NULL) {
		// load the file from disk
		FileInfo::prepare();
		if(data==NULL)
			return;
		
		// get the image info
		width=height=components=0;
		size_t totalSize=0;
		char * imgbuf = ((char*)NULL)-1;
		
		image_util::decodeImage(data,size,width,height,components,imgbuf,totalSize); // this call fails (no allocation), but sets image info

		totalSize = width*height*components;
		// using mmap to allocate space so we can still use mlock/munlock on it later
		imgbuf = static_cast<char*>(mmap(NULL,totalSize,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANON,-1,0));
		if(imgbuf==MAP_FAILED) {
			std::string err="FileSystemImageSource::ImageInfo::prepare() unable to mmap allocation for image decompression of ";
			err+=filename;
			perror(err.c_str());
			FileInfo::release();
			return;
		}
		
		// decompress the image for real this time:
		if(!image_util::decodeImage(data,size,width,height,components,imgbuf,totalSize)) {
			cerr << "Image decompression failed for " << filename << endl;
			munmap(imgbuf,totalSize); // error, give up and clear our memory usage
			FileInfo::release();
			return; // don't fall through to the prepare() below!
		}
		
		// replace the raw compressed data with the uncompressed image
		FileInfo::release();
		setData(imgbuf,totalSize);
	}
	FileInfo::prepare();
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
