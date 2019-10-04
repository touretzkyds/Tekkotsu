#include "FileSystemCommPort.h"
#include <fcntl.h>

const std::string FileSystemCommPort::autoRegisterFileSystemCommPort = CommPort::getRegistry().registerType<FileSystemCommPort>("FileSystemCommPort");

bool FileSystemCommPort::open() {
	if(openedCnt++>0)
		return true;
	path.addPrimitiveListener(this);
	mode.addPrimitiveListener(this);
	curloc=path;
	curmode=static_cast<std::ios_base::openmode>((int)mode);
	bool ans=true;
	if(curmode & std::ios_base::out) {
		int wrmode = O_WRONLY | O_CREAT | O_NOCTTY;
		if(curmode & (std::ios_base::app | std::ios_base::ate))
			wrmode |= O_APPEND;
		if(curmode & std::ios_base::trunc)
			wrmode |= O_TRUNC;
		int fd = ::open(path.c_str(),wrmode,0x777);
		//std::cout << "open write " << fd << std::endl;
		if(fd<0) {
			connectionError("Could not open '"+path+"' for writing",false,strerror(errno));
			ans=false;
		} else {
			wbuf.adoptFD(fd);
		}
	}
	if(curmode & std::ios_base::in) {
		int fd = ::open(path.c_str(),O_RDONLY | O_NOCTTY);
		//std::cout << "open read " << fd << std::endl;
		if(fd<0) {
			connectionError("Could not open '"+path+"' for reading",false,strerror(errno));
			ans=false;
		} else {
			rbuf.adoptFD(fd);
		}
	}
	opened();
	return ans;
}

bool FileSystemCommPort::close() {
	if(openedCnt==0)
		std::cerr << "Warning: FileSystemCommPort close() without open()" << std::endl;
	if(--openedCnt>0)
		return false;
	closing();
	path.removePrimitiveListener(this);
	mode.removePrimitiveListener(this);
	rbuf.close();
	wbuf.close();
	return true;
}

void FileSystemCommPort::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&path) {
		if(path!=curloc) {
			unsigned int tmp=openedCnt;
			openedCnt=1; // fake out close() and open() to make sure they trigger the action
			close();
			open();
			openedCnt=tmp; // reset original reference count
		}
	} else if(&pl==&mode) {
		if(mode!=curmode) {
			std::cerr << "Cannot change access mode while file is open" << std::endl;
		}
	} else {
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
