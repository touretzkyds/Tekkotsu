//-*-c++-*-
#ifndef INCLUDED_FileSystemDataSource_h_
#define INCLUDED_FileSystemDataSource_h_

#include "local/DataSource.h"
#include "Shared/plist.h"
#include "Shared/get_time.h"
#include "IPC/Thread.h"
#include <list>
#include <iostream>

class LoggedDataDriver;

//! Manages the loading of a series of files from disk
/*! Can handle an index file listing other data files and their timestamps,
 *  a directory holding a set of files, or a single explicit file. */
class FileSystemDataSource : public virtual DataSource, public virtual plist::Dictionary, public virtual plist::PrimitiveListener {
public:
	//! constructor
	FileSystemDataSource(LoggedDataDriver& p, const std::string& filter)
		: DataSource(), plist::Dictionary(), PrimitiveListener(), path(), filenameFilter(filter), loop(true), framerate(1000.f/(FrameTime*NumFrames)), verbose(0),
		poller(*this), files(), curfile(files.begin()),
		initialDelay(0), nextTime(0), indexed(true), freezeTime(), timeScale(NULL), parent(p), actualLoopTime(0), naturalLoopTime(0), lock(), registered(false)
	{
		setLoadSavePolicy(FIXED,SYNC);
		addEntry("Path",path,"If set, overrides the common LoggedData driver path.  In addition to directory or index file, can also be set to a single input data file.");
		addEntry("FileFilter",filenameFilter,"If Source is a directory or index file, only files matching the filter will be loaded from it.");
		addEntry("Loop",loop,"If true, restart file list at the beginning when the end is reached; otherwise just stop loading data");
		addEntry("Framerate",framerate,"The sensors updates per second which should be loaded.\nNote this is limited by the global Sensors.Framerate setting");
		addEntry("Verbose",verbose,"Controls how much feedback to give on the console regarding progress\n  0 - none\n  1 - report when messages are dropped\n  2 - also report when a message is sent\n  3 - also report when loop occurs\n  4 - also report when each message is preloaded");
	}

	//! destructor
	~FileSystemDataSource();
	
	virtual unsigned int nextTimestamp();
	virtual const std::string& nextName();
	virtual bool advance();
	virtual void registerSource();
	virtual bool isRegistered() const { return registered; } //!< returns #registered
	virtual void deregisterSource();
	virtual void enteringRealtime(const plist::Primitive<double>& simTimeScale);
	virtual void leavingRealtime(bool /*isFullSpeed*/);
	
	//! sets the next frame to be sent (e.g. pass 0 to reset to the first frame)
	/*! prepares (pre-loads) @a numPreload frames from the new #curfile onward */
	virtual void setFrame(unsigned int f, unsigned int numPreload=2);
	
	//! increments #curfile to the next frame, preparing (pre-loads) @a numPreload frames from the new #curfile onward
	virtual void nextFrame(unsigned int numPreload=2);
	
	//! returns the total time taken to loop over the files
	/*! @param actual if false, will return the "natural" time of a loop, ignoring setLoopTime() effects */
	virtual double getLoopTime(bool actual=true) const { return actual ? actualLoopTime : naturalLoopTime; }
	virtual void setLoopTime(double t); //!< adds time to the final frame to increase total sequence time to @a t milliseconds
	
	virtual bool usingIndexFile() const { return indexed; } //!< returns #indexed
	
	//! empties #files
	void clearFiles();
	
	virtual void plistValueTouched(const plist::PrimitiveBase& pl);
	virtual void plistValueChanged(const plist::PrimitiveBase& pl);
	
	virtual void loadXML(xmlNode* node);
	
	//! returns the target path, either #path, or parent.path if #path is empty
	virtual const std::string& getUsedPath() const;
	
	//! call this to (re)load the list of available file names from disk
	/*! If @a clearCurrent is set, then the current file list will be cleared;
	 *  otherwise, the loaded files will be appended to the current queue */
	virtual bool loadFileList(bool clearCurrent=true, bool reportMissing=true);
	
	//! The directory, data file, or index file from which to load via call to loadFileListFromDirectory(), loadSingleFile(), or loadFileListFromIndex()
	/*! A single file can be either a single data file (e.g. sensor or camera image), or an index file as output by VisionGUI, or in the format 'filename \<tab\> time', where 'filename' is an absolute path or relative to the directory containing the index file, and 'time' is in milliseconds, relative to the time at which the index file is loaded.\n In the future, this could also be network addresses for teleoperation and remote processing. */
	plist::Primitive<std::string> path;
	
	//! a regular expression (POSIX.2 extended format) to select which files to load from #path, if #path is a directory or index file
	plist::Primitive<std::string> filenameFilter;
	
	//! controls whether to restart #curfile at the beginning of #files when it reaches the end
	plist::Primitive<bool> loop;
	
	//! The sensors updates per second which should be loaded.  Note this is limited by the global Sensors.Framerate setting
	plist::Primitive<float> framerate;
	
	//! Controls how much feedback to give on the console regarding progress
	/*! 0 - none\n
	 *  1 - report when messages are dropped\n
	 *  2 - also report when a message is sent\n
	 *  3 - also report when loop or end-of-log occurs\n
	 *  4 - also report when each message is preloaded */
	plist::Primitive<int> verbose;
	
protected:
	static const unsigned int MAX_LOAD=4000; //!< maximum number of data elements to try to keep 'active'.  If there's more than this in #files, we'll only load one at time, and immediately release it afterward
	struct FileInfo;
	typedef std::list<FileInfo*> files_t; //!< type of #files, the list of files to load
	
	struct DataThread : public Thread {
		DataThread(FileSystemDataSource& p) : parent(p) {}
		virtual void* run();
		FileSystemDataSource& parent;
	} poller;
	
	//! Gives the data referenced by curfile to the simulator
	virtual bool sendData()=0;
	
	virtual void doFreeze();
	virtual void doUnfreeze();
	virtual void resetPoller();
	
	//! load a single file
	virtual void loadSingleFile(const std::string& file);
	//! load a list of files from a directory specified by #path
	virtual void loadFileListFromDirectory();
	//! load a list of files from an index file specified by #path
	/*! This supports either the format produced by VisionGUI, or a simplier '<code>filename [\<tab\> time]\\n</code>' format,
	 *  where if @c time is unspecified, the frame's time is incremented by the #framerate from the previously listed file.
	 *  Filenames should either be either absolute paths or relative to the directory which contains the index file.*/
	virtual bool loadFileListFromIndex();
	
	//! adds up the lifetime of all #files, plus #initialDelay
	virtual double calcLoopTime() const;
	
	//! stores basic information regarding each file in the queue, including routines for loading to and from disk
	struct FileInfo {
		//! default constructor
		FileInfo() : filename(), lifetime(0), data(NULL), size(0), refcount(NULL), prepared(false) { }
		//! basic constructor, provide name and lifetime, data and size are left empty until later prepare() is called
		FileInfo(const std::string& name, double time) : filename(name), lifetime(time), data(NULL), size(0), refcount(NULL), prepared(false) { }
		//! copy constructor, adds reference to the source's data (shallow copy)
		FileInfo(const FileInfo& o) : filename(o.filename), lifetime(o.lifetime), data(o.data), size(o.size), refcount(o.refcount), prepared(false) { if(refcount!=NULL) ++(*refcount); }
		//! assignment operator, adds reference to the source's data (shallow copy)
		FileInfo& operator=(const FileInfo& o) { 
			release();
			filename=o.filename; lifetime=o.lifetime; data=o.data; size=o.size; refcount=o.refcount;
			if(refcount!=NULL)
				++(*refcount); 
			return *this;
		}
		//! destructor
		virtual ~FileInfo() { release(); }
		
		virtual void prepare(); //!< make sure data is in physical memory for imminent usage
		virtual void done(); //!< we can let the data out of memory if needed
		virtual void release(); //!< if data is in an allocated region, free it
		
		const char* getData() const { return data; }
		size_t getSize() const { return size; }
		void setData(char* d, size_t s) { release(); data=d; size=s; refcount=new unsigned int; *refcount=1; }
		bool isPrepared() const { return prepared; }
		
		std::string filename; //!< the path to the file to load
		double lifetime; //!< duration for which this frame is "current", before next frame should be sent
	protected:
		char* data; //!< loaded/parsed contents of #filename, all ready to send, just need to copy into message
		size_t size; //!< size of buffer pointed to by #data (may be decompressed/processed, different than file size...)
		unsigned int* refcount; //!< maintain a reference count of the #data (just across FileInfo instances...)
		bool prepared; //!< true if prepare() has been called
	};
	
	//! calls prepare on @a fi, dumping diagnostic info if verbose is high enough
	virtual void preprepare(const FileSystemDataSource::files_t::iterator& fi);
	
	//! creates a new entry on #files, virtual to allow subclasses to use a FileInfo subclass with more meta data (e.g. see FileSystemImageSource::ImageInfo)
	virtual void enqueueFile(const std::string& name, double lifetime) { files.push_back(new FileInfo(name,lifetime)); } 
	
	files_t files; // collection of FileInfo entries
	files_t::iterator curfile; //!< an iterator referencing #files -- indicates next file to send
	double initialDelay; //!< milliseconds to wait before sending first frame
	double nextTime; //!< timestamp that #curfile should be sent
	bool indexed; //!< true if the file list was specified by an index file
	unsigned int freezeTime; //!< time at which doFreeze was called
	const plist::Primitive<double>* timeScale; //!< simulator's timeScale parameter, so we can register for updates when it changes
	LoggedDataDriver& parent; //!< device driver this is a member of
	double actualLoopTime; //!< time it would take to run through all of the loaded frames
	double naturalLoopTime; //!< time it would take to run through all of the frames as set immediately after load (i.e. no setLoopTime())
	Thread::Lock lock; //!< don't be in advance() while modifying file list
	bool registered; //!< synced to registerSource/deregisterSource so we don't start the thread if not actually in use
	
private:
	FileSystemDataSource(const FileSystemDataSource&); //!< don't call
	FileSystemDataSource& operator=(const FileSystemDataSource&); //!< don't call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
