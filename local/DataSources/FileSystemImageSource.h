//-*-c++-*-
#ifndef INCLUDED_FileSystemImageSource_h_
#define INCLUDED_FileSystemImageSource_h_

#include "FileSystemDataSource.h"

//! Extends FileSystemDataSource to decompress image data
class FileSystemImageSource : public FileSystemDataSource {
public:
	//! constructor
	FileSystemImageSource(LoggedDataDriver& p, const std::string& filter)
		: FileSystemDataSource(p,filter), layer(0), frameNumber(0)
	{
		addEntry("Layer",layer,"Controls the resolution layer at which the image should be processed.\n0 indicates \"automatic\" mode (picks layer closest to image's resolution), positive numbers indicate the resolution layer directly.\nNegative values are relative to the number of layers marked available by the vision setup, so that typically -1 would correspond to the \"double\" layer, and -2 would correspond to the \"full\" layer.");
	}

	//! Controls the resolution layer at which the image should be processed
	/*! 0 indicates "automatic" mode (picks layer closest to image's resolution), positive
	 *  numbers indicate the resolution layer directly.
	 *  
	 *  Negative values are relative to the number of layers marked available by the
	 *  vision setup, so that typically -1 would correspond to the "double" layer, and -2
	 *  would correspond to the "full" layer. */
	plist::Primitive<int> layer;

protected:
	
	//! extends FileInfo to provide image decompression and some additional fields for image meta-data
	class ImageInfo : public FileInfo {
	public:
		//! constructor
		ImageInfo(const FileSystemImageSource& ds, const std::string& name, double time) : FileInfo(name,time), dataSource(ds), width(), height(), components() {}
		
		//! uses FileInfo's prepare to load file into memory, and then replaces with a decompressed version
		virtual void prepare();
		
		//! reference back to the containing data source so we can access FileSystemImageSource::layer
		const FileSystemImageSource& dataSource;
		size_t width; //!< width of image
		size_t height; //!< height of image
		size_t components; //!< number of color channels
	};
	
	bool sendData() {
		if((*curfile)->getData()==NULL)
			return false;
		const ImageInfo& curimg = dynamic_cast<const ImageInfo&>(**curfile);
		setImage(ImageHeader(0, layer, curimg.width, curimg.height, curimg.components, ++frameNumber, static_cast<unsigned int>(nextTime), curimg.filename), curimg.getData());
		return true;
	}
	
	virtual void enqueueFile(const std::string& name, double lifetime) { files.push_back(new ImageInfo(*this,name,lifetime)); } 
	
	unsigned int frameNumber;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
