#include "FileSystemDataSource.h"
#include "local/DeviceDrivers/LoggedDataDriver.h"
#include "Shared/get_time.h"
#include "Shared/RobotInfo.h"
#include "Shared/string_util.h"
#include "Shared/MarkScope.h"
#include <set>
#include <fstream>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <regex.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <cmath>
#include <errno.h>

using namespace std; 

FileSystemDataSource::~FileSystemDataSource() {
	MarkScope autolock(lock);
	if(timeScale!=NULL)
		leavingRealtime(false);
	clearFiles();
}

unsigned int FileSystemDataSource::nextTimestamp() {
	return (curfile!=files.end()) ? static_cast<unsigned int>(nextTime) : -1U;
}

const std::string& FileSystemDataSource::nextName() {
	if(curfile!=files.end())
		return (*curfile)->filename;
	else {
		static const std::string noneStr="(none)";
		return noneStr;
	}
}

bool FileSystemDataSource::advance() {
	MarkScope autolock(lock);
	Thread::testCurrentCancel();
	if(curfile==files.end())
		return false;
	const unsigned int timestamp = get_time();
	if(!frozen) {
		if(nextTime>timestamp)
			return false;
		if(nextTime+(*curfile)->lifetime<=timestamp) {
			double looptime=getLoopTime(true);
			if(looptime>0) {
				while(nextTime+looptime<=timestamp)
					nextTime+=looptime;
			}
			while(nextTime+(*curfile)->lifetime<=timestamp)
				nextFrame(0); // too late, drop frames
		}
		if(curfile==files.end())
			return false;
	}
	
	preprepare(curfile);

	bool sentData=false;
	if((*curfile)->getData()!=NULL) {
		if(verbose>=2) {
			std::cout << "Applying '" << (*curfile)->filename << "' at " << timestamp;
			if(timestamp!=nextTime)
				std::cout << " (should've been " << nextTime << ")";
			std::cout << std::endl;
		}
		sentData = sendData();
		(*curfile)->done();
	}
	
	double origNext=nextTime;
	nextFrame();
	if(frozen)
		nextTime=origNext; // hold nextTime if frozen
	
	return sentData;
}

void FileSystemDataSource::registerSource() {
	ASSERT(!registered,"Already registered?");
	nextTime=get_time();
	path.addPrimitiveListener(this);
	filenameFilter.addPrimitiveListener(this);
	loop.addPrimitiveListener(this);
	framerate.addPrimitiveListener(this);
	plistValueChanged(path);
	registered=true;
}

void FileSystemDataSource::deregisterSource() {
	ASSERT(registered,"Already deregistered?");
	registered=false;
	nextTime=get_time();
	path.removePrimitiveListener(this);
	filenameFilter.removePrimitiveListener(this);
	loop.removePrimitiveListener(this);
	framerate.removePrimitiveListener(this);
	if(timeScale!=NULL)
		leavingRealtime(false);
}

void FileSystemDataSource::enteringRealtime(const plist::Primitive<double>& simTimeScale) {
	timeScale = &simTimeScale;
	resetPoller();
}
void FileSystemDataSource::leavingRealtime(bool /*isFullSpeed*/) {
	if(poller.isStarted()) {
		timeScale->removePrimitiveListener(this);
		poller.stop().join();
	}
	timeScale = NULL;
}

void FileSystemDataSource::doFreeze() {
	freezeTime=get_time();
	resetPoller();
}
void FileSystemDataSource::doUnfreeze() {
	nextTime+=get_time()-freezeTime;
	resetPoller();
}

void FileSystemDataSource::resetPoller() {
	MarkScope autolock(lock);
	MarkScope sl(poller.getStartLock());
	if(getTimeScale()>0 && nextTimestamp()!=-1U && registered && timeScale!=NULL && !frozen) {
		// should be running...
		if(!poller.isStarted()) { // start if it isn't running
			timeScale->addPrimitiveListener(this);
			poller.start();
		}
	} else if(poller.isStarted()) { // should not be running... stop if it is
		timeScale->removePrimitiveListener(this);
		poller.stop().join();
	}
}

const std::string& FileSystemDataSource::getUsedPath() const { return (path.size()==0) ? parent.path : path; }

bool FileSystemDataSource::loadFileList(bool clearCurrent/*=true*/, bool reportMissing/*=true*/) {
	MarkScope autolock(lock);
	nextTime=freezeTime=get_time();
	struct stat sb;
	if(clearCurrent)
		clearFiles();
	else if(files.size()==0)
		clearCurrent=true; // was empty, pretend we just cleared it, have to do the same re-initialization
	if(getUsedPath().size()==0)
		return true; //empty path means disabled
	if(stat(getUsedPath().c_str(),&sb)) {
		if(reportMissing)
			std::cerr << "FileSystemDataSource could not access path '" << getUsedPath() << "'" << std::endl;
		return false;
	}
	if(sb.st_mode&S_IFDIR) {
		loadFileListFromDirectory();
	} else {
		//Test to see if the file matches the filter
		try {
			if(string_util::reMatch(getUsedPath(),filenameFilter))
				loadSingleFile(getUsedPath().c_str());
			else { //if it doesn't match the image RE, assume it's an index file
				if(!loadFileListFromIndex())
					std::cerr << "Source '" << getUsedPath() << "' does not match the filename filter '" << filenameFilter << "' and is not an index list." << std::endl;
			}
		} catch(const std::string& err) {
			std::cerr << err << std::endl;
		}
	}
	if(clearCurrent) {
		files_t::iterator it=curfile=files.begin();
		for(unsigned int numPreload=2; numPreload>0 && it!=files.end(); numPreload--)
			preprepare(it++);
	}
	actualLoopTime=naturalLoopTime=calcLoopTime();
	resetPoller();
	return true;
}

void* FileSystemDataSource::DataThread::run() {
	for(;;) {
		ASSERTRETVAL(getTimeScale()>0,"FileSystemDataSource::runloop in non-realtime mode",NULL);
		unsigned int next = parent.nextTimestamp();
		if(next==-1U)
			return NULL; // no more frames
		unsigned int cur = get_time();
		if(cur < next) {
			while(usleep(static_cast<useconds_t>((next-cur)*1000/getTimeScale()))) {
				if(errno!=EINTR) {
					perror("FileSystemDataSource::runloop(): nanosleep");
					break;
				}
				// may have been interrupted to recompute sleep time for change in time scale
				testCancel();
				cur = get_time();
				next = parent.nextTimestamp();
			}
		}
		testCancel();
		parent.advance();
	}
}

void FileSystemDataSource::setFrame(unsigned int f, unsigned int numPreload/*=2*/) {
	MarkScope autolock(lock);
	for(;curfile!=files.end() && (*curfile)->isPrepared(); ++curfile) {
		if(files.size()>MAX_LOAD)
			(*curfile)->release();
		else
			(*curfile)->done();
	}
	nextTime=freezeTime=get_time();
	curfile=files.begin();
	std::advance(curfile,f);
	files_t::iterator it=curfile;
	for(; numPreload>0 && it!=files.end(); numPreload--) {
		preprepare(it);
		if(++it==files.end() && loop)
			it=files.begin();
	}
}

void FileSystemDataSource::nextFrame(unsigned int numPreload/*=2*/) {
	MarkScope autolock(lock);
	if(numPreload==0 && verbose>=1)
		cout << "Dropping " << (*curfile)->filename << " scheduled " << nextTime <<  " for duration " << (*curfile)->lifetime << endl;
	if(files.size()>MAX_LOAD)
		(*curfile)->release();
	else
		(*curfile)->done();
	nextTime+=(*curfile)->lifetime;
	if(++curfile==files.end()) {
		if(!loop) {
			if(verbose>=3)
				cout << "Reached end of logged data source \"" << parent.getName() << '"' << endl;
		} else {
			nextTime+=initialDelay;
			curfile=files.begin();
			if(verbose>=3)
				cout << "Looping file system data source at " << nextTime << " to " << (*curfile)->filename << " (loop time=" << getLoopTime() << ")" << endl;
		}
	}
	files_t::iterator it=curfile;
	for(; numPreload>0 && it!=files.end(); numPreload--) {
		preprepare(it);
		if(++it==files.end() && loop)
			it=files.begin();
	}
}

double FileSystemDataSource::calcLoopTime() const {
	if(files.size()==0)
		return 0;
	double t=initialDelay;
	for(files_t::const_iterator it=files.begin(); it!=files.end(); ++it)
		t+=(*it)->lifetime;
	return t;
}
void FileSystemDataSource::setLoopTime(double t) {
	if(files.size()==0)
		return;
	double remain = t - getLoopTime(true);
	if(remain + files.back()->lifetime < 0) {
		std::cerr << "FileSystemDataSource::setLoopTime(" << t << ") would result in a negative frame lifetime" << std::endl;
		return;
	}
	files.back()->lifetime+=remain;
	actualLoopTime=t;
}

void FileSystemDataSource::clearFiles() {
	MarkScope autolock(lock);
	// FileInfo destructor should take care of deleting data buffers...
	for(files_t::iterator it=files.begin(); it!=files.end(); ++it)
		delete *it;
	files.clear();
	curfile=files.begin();
	initialDelay=0;
	actualLoopTime=naturalLoopTime=0;
}

void FileSystemDataSource::plistValueTouched(const plist::PrimitiveBase& pl) {
	if(&pl==&path) {
		// reassigning the same value still triggers reloading
		loadFileList();
	}
}
void FileSystemDataSource::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&path || &pl==&	filenameFilter) {
		loadFileList();
	} else if(&pl==&loop) {
		if(loop && nextTimestamp()==-1U)
			curfile=files.begin();
		resetPoller();
	} else if(&pl==&framerate) {
		if(!usingIndexFile()) {
			const double dt=1000.f/framerate;
			bool first = (curfile==files.begin());
			if(!first)
				nextTime-=(*--curfile)->lifetime;
			for(files_t::const_iterator it=files.begin(); it!=files.end(); ++it)
				(*it)->lifetime=dt;
			if(!first)
				nextTime+=(*curfile++)->lifetime;
			parent.plistValueChanged(path); // to reset the loop time if sharing a source
		}
	} else if(&pl==timeScale) {
		if(poller.isRunning())
			poller.interrupt();
	} else {
		cerr << "FileSystemDataSource didn't handle call to plistValueChanged for " << pl.get() << endl;
	}
}

void FileSystemDataSource::loadXML(xmlNode* node) {
	bool wasListener;
	if((wasListener=path.isPrimitiveListener(this))) // intentional assignment
		 path.removePrimitiveListener(this);
	plist::Dictionary::loadXML(node);
	if(wasListener) {
		path.addPrimitiveListener(this);
		plistValueChanged(path);
	}
}

void FileSystemDataSource::loadSingleFile(const std::string& file) {
	MarkScope autolock(lock);
	indexed=false;
	enqueueFile(file,1000.f/framerate);
}

void FileSystemDataSource::loadFileListFromDirectory() {
	regex_t re;
	if(int err=regcomp(&re,filenameFilter.c_str(),REG_EXTENDED | REG_NOSUB)) {
		char msg[128];
		regerror(err,&re,msg,128);
		std::cerr << "Bad filter '" << filenameFilter << "': " << msg << std::endl;
		regfree(&re);
		return;
	}
	DIR * d=opendir(getUsedPath().c_str());
	if(d==NULL) {
		std::cerr << "Could not open directory " << getUsedPath() << std::endl;
		regfree(&re);
		return;
	}
	struct dirent* res;
	
#ifdef HAVE_READDIR_R
	struct dirent cur;
	if(readdir_r(d,&cur,&res)) {
		std::cerr << "Error reading files from " << getUsedPath() << std::endl;
		closedir(d);
		regfree(&re);
		return;
	}
#else
	res=readdir(d);
#endif

	std::set<std::string> dirfiles;
	while(res!=NULL) {
		int match=regexec(&re,res->d_name,0,NULL,0);
		if(match==0) {
			dirfiles.insert(res->d_name);
		} else if(match!=REG_NOMATCH) {
			char msg[128];
			regerror(match,&re,msg,128);
			std::cerr << "Regex error on '" << res->d_name << "': " << msg << std::endl;
		} // else std::cout << "Skipping " << res->d_name << std::endl;
#ifdef HAVE_READDIR_R
		if(readdir_r(d,&cur,&res)) {
			std::cerr << "Error reading files from " << getUsedPath() << std::endl;
			closedir(d);
			regfree(&re);
			return;
		}
#else
		res=readdir(d);
#endif
	}
	closedir(d);
	regfree(&re);
	
	MarkScope autolock(lock);
	//std::cout << "Processing " << getUsedPath() << std::endl;
	double tinc=1000.f/framerate;
	for(std::set<std::string>::const_iterator it=dirfiles.begin(); it!=dirfiles.end(); ++it) {
		//std::cout << "Enqueuing " << *it << std::endl;
		enqueueFile((getUsedPath()+"/")+(*it),tinc);
	}
	indexed=false;
}

bool FileSystemDataSource::loadFileListFromIndex() {
	MarkScope autolock(lock);
	indexed=(indexed || files.size()==0);
	regex_t re;
	if(int err=regcomp(&re,filenameFilter.c_str(),REG_EXTENDED | REG_NOSUB)) {
		char msg[128];
		regerror(err,&re,msg,128);
		std::cerr << "Bad filter '" << filenameFilter << "': " << msg << std::endl;
		regfree(&re);
		return false;
	}
	
	ifstream in(getUsedPath().c_str());
	string cur;
	getline(in,cur);
	if(cur.find("First frame ")==0) //skip the header line from the GUI, e.g. 'First frame 42898 timestamp: 1439018'
		getline(in,cur);
	
	double tinc=1000.f/framerate;
	double lasttime=-tinc;
	while(in) {
		string fn = cur.substr(0,cur.find('\t'));
		int match=regexec(&re,fn.c_str(),0,NULL,0);
		if(match==0) {
			double curtime=lasttime+tinc;
			if(fn.size()!=cur.size()) {
				const char * timep=cur.c_str()+cur.rfind('\t');
				char * endp=NULL;
				curtime=strtof(timep,&endp);
				if(timep==endp) {
					std::cerr << "ERROR: '" << getUsedPath() << "' does not seem to be a valid index file." << std::endl;
					std::cerr << "       Use output from VisionGUI, or use format 'filename <tab> time'" << std::endl;
					std::cerr << "       Where 'time' is the time in milliseconds at which the file should be processed, relative" << std::endl;
					std::cerr << "       to the time at which the index file is loaded." << std::endl;
					regfree(&re);
					return false;
				}
				if(lasttime>=0) {
					files.back()->lifetime=curtime-lasttime;
					//std::cout << "(previous frame lifetime " << files.back()->lifetime << ") ";
				} else if(files.size()>0) {
					files.back()->lifetime+=curtime;
					//std::cout << "(previous frame increased lifetime to " << files.back()->lifetime << ") ";
				} else {
					initialDelay=curtime;
					nextTime=get_time()+curtime;
					//std::cout << "nextTime set to " << nextTime << " ";
				}
			}
			if(fn[0]!='/') { // if not absolute path, tack on path to index file (*do this after previous check*!)
				string::size_type srcdir=getUsedPath().rfind('/');
				if(srcdir!=string::npos)
					fn=getUsedPath().substr(0,srcdir+1)+fn;
			}
			//std::cout << "Enqueuing " << fn << " at " << curtime << endl;
			enqueueFile(fn,tinc);
			lasttime=curtime;
		} else if(match!=REG_NOMATCH) {
			char msg[128];
			regerror(match,&re,msg,128);
			std::cerr << "Regex error on '" << fn << "': " << msg << std::endl;
		} // else std::cout << "Skipping " << res->d_name << std::endl;
		getline(in,cur);
	}
	regfree(&re);
	return true;
}

void FileSystemDataSource::FileInfo::prepare() {
	if(prepared)
		return;
	if(data==NULL) {
		struct stat statbuf;
		if(stat(filename.c_str(),&statbuf)!=0) {
			std::string err="FileSystemDataSource::FileInfo::prepare() failed to stat file ";
			err+=filename;
			perror(err.c_str());
			return;
		}
		int fd=open(filename.c_str(),O_RDONLY);
		if(fd<0) {
			std::string err="FileSystemDataSource::FileInfo::prepare() unable to open file ";
			err+=filename;
			perror(err.c_str());
			return;
		}
		refcount=new unsigned int;
		*refcount=1;
		size=static_cast<size_t>(statbuf.st_size);
		data=static_cast<char*>(mmap(NULL,size,PROT_READ,MAP_PRIVATE|MAP_FILE,fd,0));
		if(data==MAP_FAILED) {
			data=NULL;
			size=0;
			std::string err="FileSystemDataSource::FileInfo::prepare() unable to mmap file ";
			err+=filename;
			perror(err.c_str());
			return;
		}
		if(close(fd)!=0) {
			std::string err="FileSystemDataSource::FileInfo::prepare() unable to close file ";
			err+=filename;
			perror(err.c_str());
			return;
		}
	}
	if(mlock(data,size)!=0) {
		if(errno==ENOMEM) {
			static bool firsterr=true; // give a warning just the first time if mlock fails because RLIMIT_MEMLOCK is too low
			if(firsterr) {
				firsterr=false;
				rlimit rl;
#ifndef __CYGWIN__
				getrlimit(RLIMIT_MEMLOCK,&rl);
				cerr << "Notice: mlock() failed because RLIMIT_MEMLOCK is too low, limited to " << (rl.rlim_cur/1024) << "KB\n"
				     << "Increasing this limit can smooth logged data I/O in low memory situations. (see ulimit/limit commands)" << endl;
#endif
			}
		} else {
			std::string err="FileSystemDataSource::FileInfo::prepare() unable to mlock file ";
			err+=filename;
			perror(err.c_str());
		}
		return;
	}
	prepared=true;
}

void FileSystemDataSource::FileInfo::done() {
	if(data==NULL || !prepared)
		return;
	prepared=false;
	if(munlock(data,size)!=0) {
		std::string err="FileSystemDataSource::FileInfo::done() unable to munlock file ";
		err+=filename;
		perror(err.c_str());
	}
}

void FileSystemDataSource::FileInfo::release() {
	if(data==NULL)
		return;
	done();
	if(--(*refcount)==0) {
		if(munmap(data,size)!=0) {
			std::string err="FileSystemDataSource::FileInfo::release() unable to munmap file ";
			err+=filename;
			perror(err.c_str());
		}
		delete refcount;
	}
	data=NULL;
	size=0;
	refcount=NULL;
}

void FileSystemDataSource::preprepare(const FileSystemDataSource::files_t::iterator& fi) {
	if(verbose>=4 && (*fi)->getData()==NULL) {
		if(fi==curfile) 
			std::cout << "Loading '" << (*fi)->filename << "' at " << get_time() << std::endl;
		else {
			std::cout << "Preloading '" << (*fi)->filename << "' at " << get_time();
			files_t::iterator tmp = curfile;
			double tgtTime=nextTime; 
			{
				tgtTime+=(*tmp)->lifetime;
				++tmp;
				if(tmp==files.end()) {
					tmp=files.begin();
					tgtTime+=initialDelay;
				}
			} while(tmp!=curfile && tmp!=fi);
			std::cout << ", scheduled for " << static_cast<unsigned int>(tgtTime) << std::endl;
		}
		
	}
	(*fi)->prepare();
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
