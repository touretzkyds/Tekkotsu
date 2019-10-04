#ifndef PLATFORM_APERIOS
#include "RCRegion.h"
#include "Shared/MarkScope.h"
#include "Shared/debuget.h"
#include "Shared/plist.h"
#include "Thread.h"
#include <unistd.h>
#include <sstream>
#include <sys/stat.h>
#include <errno.h>

#if TEKKOTSU_SHM_STYLE!=SYSV_SHM && TEKKOTSU_SHM_STYLE!=POSIX_SHM && TEKKOTSU_SHM_STYLE!=NO_SHM
#  error Unknown TEKKOTSU_SHM_STYLE setting
#endif

#if TEKKOTSU_SHM_STYLE==SYSV_SHM
#  include <sys/ipc.h>
#  include <sys/shm.h>
#elif TEKKOTSU_SHM_STYLE==POSIX_SHM
#  include <sys/mman.h>
#  include <sys/fcntl.h>
#  ifdef USE_UNBACKED_SHM
plist::Primitive<bool> RCRegion::useUniqueMemoryRegions(false);
#  else
plist::Primitive<std::string> RCRegion::shmRoot("/tmp/tekkotsu_sim/");
plist::Primitive<bool> RCRegion::useUniqueMemoryRegions(true);
#  endif
pid_t RCRegion::rootPID(::getpid());
#endif

using namespace std;

#if TEKKOTSU_SHM_STYLE==SYSV_SHM
key_t RCRegion::nextKey=1024;
#elif TEKKOTSU_SHM_STYLE==POSIX_SHM || TEKKOTSU_SHM_STYLE==NO_SHM
key_t RCRegion::nextKey=0;
#endif

RCRegion::attachedRegions_t RCRegion::attachedRegions;
bool RCRegion::isFaultShutdown=false;
bool RCRegion::multiprocess=true;
Thread::Lock* RCRegion::staticLock=NULL;


#if TEKKOTSU_SHM_STYLE==SYSV_SHM
//under SYSV shared memory, the keys are just numbers, and it's just as likely that the conflicted
//region belongs to an unrelated process as it is that the region is from a previous run -- so we
//take the safe route and rename our own keys
RCRegion::ConflictResolutionStrategy RCRegion::conflictStrategy=RCRegion::RENAME;

#elif TEKKOTSU_SHM_STYLE==POSIX_SHM
//under POSIX shared memory, the keys are names, so we can have some confidence that a region
//with the same name is ours from a previous run, so we replace it to avoid leaking (although
//it's still possible we're conflicting with another application, but good name choices should
//mitigate this)
RCRegion::ConflictResolutionStrategy RCRegion::conflictStrategy=RCRegion::REPLACE;

#elif TEKKOTSU_SHM_STYLE==NO_SHM
//with shared memory disabled, a conflict indicates we're reusing the same name... this is probably
//a bug, so we should fail-fast
RCRegion::ConflictResolutionStrategy RCRegion::conflictStrategy=RCRegion::EXIT;

#endif

RCRegion * RCRegion::attach(const Identifier& rid) {
	MarkScope l(getStaticLock());
	attachedRegions_t::iterator it=attachedRegions.find(rid.key);
	if(it==attachedRegions.end())
		return new RCRegion(rid); // the constructor will add entry to attachedRegions
	else {
		ASSERTRETVAL((*it).second!=NULL,"ERROR: attached region is NULL!",NULL);
		(*it).second->AddReference();
		return (*it).second;
	}
}

void RCRegion::AddReference() {
	MarkScope l(getStaticLock());
	//cout << "AddReference " << id.shmid << ' ' << ProcessID::getID();
	references[ProcessID::getID()]++;
	references[ProcessID::NumProcesses]++;
	//cout << " counts are now:";
	//for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
	//	cout << ' ' << references[i];
	//cout << endl;
}

void RCRegion::RemoveReference() {
	//cout << "RemoveReference " << id.key << ' ' << ProcessID::getID();
	Thread::Lock * old=NULL;
	{
		MarkScope l(getStaticLock());
		if(references[ProcessID::getID()] == 0) {
			cerr << "Warning: RCRegion reference count underflow on " << id.key << " by " << ProcessID::getID() << "!  ";
			for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
				cerr << ' ' << references[i];
			cerr << endl;
			return;
		}
		bool wasLastProcRef=(--references[ProcessID::getID()] == 0);
		bool wasLastAnyRef=(--references[ProcessID::NumProcesses] == 0);
		ASSERT(wasLastProcRef || !wasLastAnyRef,"global reference decremented beyond process reference");
#if TEKKOTSU_SHM_STYLE==NO_SHM
		wasLastProcRef=wasLastAnyRef;
#else
		if(!multiprocess)
			wasLastProcRef=wasLastAnyRef;
#endif
		/*if(isFaultShutdown) {
			cerr << "Process " << ProcessID::getID() << " dereferenced " << id.key << ".  Counts are now:";
			for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
				cerr << ' ' << references[i];
			cerr << endl;
		}*/
		if(wasLastProcRef) {
			//cout << " detach";
#if TEKKOTSU_SHM_STYLE==SYSV_SHM
			if(shmdt(base)<0)
				perror("Warning: Region detach");
			base=NULL;
			references=NULL;
			if(wasLastAnyRef) {
				//cout << " delete" << endl;
				if(shmctl(id.shmid,IPC_RMID,NULL)<0)
					perror("Warning: Region delete");
			}
#elif TEKKOTSU_SHM_STYLE==POSIX_SHM
			if(munmap(base,calcRealSize(id.size))<0) {
				perror("Warning: Shared memory unmap (munmap)");
			}
			base=NULL;
			references=NULL;
			if(wasLastAnyRef) {
				//cout << " delete" << endl;
				if(!unlinkRegion()) {
					int err=errno;
					if(isFaultShutdown && (err==EINVAL || err==ENOENT))
						//On a fault shutdown, we initially try to unlink everything right away,
						// so an error now is just confirmation that it worked
						cerr << "Region " << id.key << " appears to have been successfully unlinked" << endl;
					else {
						cerr << "Warning: Shared memory unlink of region " << id.key << " returned " << strerror(err);
						if(err==EINVAL || err==ENOENT)
							cerr << "\n         May have already been unlinked by a dying process.";
						cerr << endl;
					}
				} else if(isFaultShutdown)
					//That shouldn't have succeeded on a faultShutdown...
					cerr << "Region " << id.key << " appears to have been successfully unlinked (nonstandard)" << endl;
			}
#elif TEKKOTSU_SHM_STYLE==NO_SHM
			delete base;
			base=NULL;
			references=NULL;
#else
#  error "Unknown TEKKOTSU_SHM_STYLE setting"
#endif
			delete this;
			if(attachedRegions.size()==0 && !isFaultShutdown) {
				//was last attached region, clean up lock for good measure
				old=staticLock;
				staticLock=NULL;
			}
		}
	}
	delete old;
	//cout << endl;
}

void RCRegion::AddSharedReference() {
	MarkScope l(getStaticLock());
	//cout << "AddSharedReference " << id.shmid << ' ' << ProcessID::getID();
	references[ProcessID::NumProcesses]++;
	//cout << " counts are now:";
	//for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
	//	cout << ' ' << references[i];
	//cout << endl;
}

void RCRegion::RemoveSharedReference() {
	MarkScope l(getStaticLock());
	//cout << "RemoveSharedReference " << id.shmid << ' ' << ProcessID::getID();
	if(references[ProcessID::NumProcesses]==0) {
		cerr << "Warning: RCRegion shared reference count underflow on " << id.key << " by " << ProcessID::getID() << "!  ";
		for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
			cerr << ' ' << references[i];
		cerr << endl;
		return;
	}
	references[ProcessID::NumProcesses]--;
	ASSERT(references[ProcessID::NumProcesses]>0,"removal of shared reference was last reference -- should have local reference as well");
	//cout << " counts are now:";
	//for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
	//	cout << ' ' << references[i];
	//cout << endl;
}


void RCRegion::aboutToFork(ProcessID::ProcessID_t newID) {
	//cout << "RCRegion aboutToFork to " << newID << endl;
	Thread::Lock* old;
	{
		MarkScope l(getStaticLock());
		attachedRegions_t::const_iterator it=attachedRegions.begin();
		for(; it!=attachedRegions.end(); ++it) {
			//cout << "Duplicating attachments for " << (*it).first;
			(*it).second->references[newID]=(*it).second->references[ProcessID::getID()];
			(*it).second->references[ProcessID::NumProcesses]+=(*it).second->references[newID];
			//cout << " counts are now:";
			//for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
			//	cout << ' ' << (*it).second->references[i];
			//cout << endl;
		}
		old=staticLock;
		staticLock=NULL;
	}
	delete old;
}

void RCRegion::faultShutdown() {
	MarkScope l(getStaticLock());
	if(isFaultShutdown) {
		cerr << "WARNING: RCRegion::faultShutdown() called again... ignoring" << endl;
		return;
	}
	isFaultShutdown=true;
	if(attachedRegions.size()==0) {
		cerr << "WARNING: RCRegion::faultShutdown() called without any attached regions (may be a good thing?)" << endl;
		return;
	}
#if TEKKOTSU_SHM_STYLE==POSIX_SHM
	//this may not really work, but it's worth a last-ditch attempt
	//in case the reference counts are screwed up.
	attachedRegions_t::const_iterator it=attachedRegions.begin();
	for(; it!=attachedRegions.end(); ++it) {
		cerr << "RCRegion::faultShutdown(): Process " << ProcessID::getID() << " unlinking " << (*it).second->id.key << endl;
#ifdef USE_UNBACKED_SHM
		shm_unlink(getQualifiedName((*it).second->id.key).c_str());
#else
		unlink(getQualifiedName((*it).second->id.key).c_str());
#endif
	}
#endif
	for(unsigned int i=0; i<100; i++) {
		unsigned int attempts=ProcessID::NumProcesses;
		unsigned int lastSize=attachedRegions.size();
		while(attachedRegions.size()==lastSize && attempts-->0)
			(*attachedRegions.begin()).second->RemoveReference();
		if(attempts==-1U) {
			cout << "Warning: could not dereference " << attachedRegions.begin()->second->id.key << endl;
			attachedRegions.erase(attachedRegions.begin());
		}
		if(attachedRegions.size()==0)
			break;
	}
}

RCRegion::attachedRegions_t::const_iterator RCRegion::attachedBegin(bool threadSafe) {
	if(threadSafe) {
		MarkScope l(getStaticLock());
		attachedRegions.begin()->second->AddReference();
		return attachedRegions.begin();
	} else
		return attachedRegions.begin();
}
RCRegion::attachedRegions_t::const_iterator RCRegion::attachedEnd() {
	return attachedRegions.end();
}
void RCRegion::attachedAdvance(RCRegion::attachedRegions_t::const_iterator& it, int x/*=1*/) {
	MarkScope l(getStaticLock());
	if(it!=attachedRegions.end())
		it->second->RemoveReference();
	std::advance(it,x);
	if(it!=attachedRegions.end())
		it->second->AddReference();
}

RCRegion::~RCRegion() {
	MarkScope l(getStaticLock());
	attachedRegions.erase(id.key);
	ASSERT(base==NULL,"destructed with attachment!");
	ASSERT(references==NULL,"destructed with local references!");
	//cout << "~RCRegion " << id.shmid << ' ' << ProcessID::getID() << endl;
}
	
unsigned int RCRegion::calcRealSize(unsigned int size) {
	size=((size+align-1)/align)*align; //round up for field alignment
	size+=extra; //add room for the reference count
	unsigned int pagesize=::getpagesize();
	unsigned int pages=(size+pagesize-1)/pagesize;
	return pages*pagesize; //round up to the nearest page
}


Thread::Lock& RCRegion::getStaticLock() {
	if(staticLock==NULL)
		staticLock=new Thread::Lock;
	return *staticLock;
}

#if TEKKOTSU_SHM_STYLE==SYSV_SHM

void RCRegion::init(size_t sz, key_t sug_key, bool create) {
	MarkScope l(getStaticLock());
	id.size=sz;
	sz=calcRealSize(sz);
	if(create) {
		int flags = 0666 | IPC_CREAT | IPC_EXCL;
		if(sug_key==IPC_PRIVATE) {
			if((id.shmid=shmget(sug_key, sz, flags)) < 0) {
				int err=errno;
				if(err != EEXIST) {
					cerr << "ERROR: Getting new private region " << key << " of size " << sz ": " << strerror(err) << " (shmget)" << endl;
					exit(EXIT_FAILURE);
				}
			}
			id.key=sug_key;
		} else {
			nextKey=sug_key;
			switch(conflictStrategy) {
				case RENAME:
					while((id.shmid=shmget(id.key=nextKey++, sz, flags)) < 0) {
						int err=errno;
						if(err != EEXIST) {
							cerr << "ERROR: Getting new region " << key << " of size " << sz ": " << strerror(err) << " (shmget)" << endl;
							exit(EXIT_FAILURE);
						}
					}
					break;
				case REPLACE:
					if((id.shmid=shmget(id.key=nextKey, sz, flags)) >= 0)
						break;
					int err=errno;
					if(err != EEXIST) {
						cerr << "ERROR: Getting new region " << key << " of size " << sz ": " << strerror(err) << " (shmget)" << endl;
						exit(EXIT_FAILURE);
					}
#ifdef DEBUG
					cerr << "Warning: conflicted key " << key << ", attempting to replace\n"
						 << "         (may have been leftover from a previous crash)" << endl;
#endif
					if(shmctl(id.shmid,IPC_RMID,NULL)<0)
						perror("Warning: Region delete from conflict - is another simulator running?");
					//note fall-through from REPLACE into EXIT - only try delete once, and then recreate and exit if it fails again
				case EXIT: 
					if((id.shmid=shmget(id.key=nextKey, sz, flags)) < 0) {
						int err=errno;
						cerr << "ERROR: Getting new region " << key << " of size " << sz ": " << strerror(err) << " (shmget)" << endl;
						exit(EXIT_FAILURE);
					}
			}
		}
	} else {
		int flags = 0666;
		if((id.shmid=shmget(sug_key, sz, flags)) < 0) {
			int err=errno;
			cerr << "ERROR: Getting existing region " << key << " of size " << sz ": " << strerror(err) << " (shmget)" << endl;
			exit(EXIT_FAILURE);
		}
		id.key=sug_key;
	}
	//cout << "ATTACHING " << id.shmid << " NOW" << endl;
	base=static_cast<char*>(shmat(id.shmid, NULL, SHM_RND));
	int err=errno;
	//cout << "Base is " << (void*)base << endl;
	if (base == reinterpret_cast<char*>(-1)) {
		cerr << "ERROR: Attaching region " << key << " of size " << sz << ": " << strerror(err) << " (shmat)" << endl;
		if(shmctl(id.shmid,IPC_RMID,NULL)<0)
			perror("Region delete");
		exit(EXIT_FAILURE);
	}
	references=reinterpret_cast<unsigned int*>(base+sz-extra);
	if(create) {
		for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
			references[i]=0;
	}
	AddReference();
	attachedRegions[id.key]=this;
}

#elif TEKKOTSU_SHM_STYLE==POSIX_SHM
	
std::string RCRegion::getQualifiedName(const std::string& key) {
#ifdef USE_UNBACKED_SHM
	string idval="/";
#else
	string idval=shmRoot;
#endif
	if(useUniqueMemoryRegions) {
		char pidstr[10];
		snprintf(pidstr,10,"%d-",rootPID);
		idval+=pidstr;
	}
	idval+=key;
	return idval;
}
int RCRegion::openRegion(int mode) const {
#ifdef USE_UNBACKED_SHM
	return shm_open(getQualifiedName().c_str(),mode,0666);
#else
	return open(getQualifiedName().c_str(),mode,0666);
#endif
}
bool RCRegion::unlinkRegion() const {
#ifdef USE_UNBACKED_SHM
	return shm_unlink(getQualifiedName().c_str())==0;
#else
	return unlink(getQualifiedName().c_str())==0;
#endif
}
void RCRegion::init(size_t sz, const std::string& name, bool create) {
	MarkScope l(getStaticLock());
	id.size=sz; //size of requested region
	sz=calcRealSize(sz); //add some additional space for region lock and reference counts
#ifndef USE_UNBACKED_SHM
	struct stat statbuf;
	//cout << "Checking " << shmRoot.substr(0,shmRoot.rfind('/')) << endl;
	if(stat(shmRoot.substr(0,shmRoot.rfind('/')).c_str(),&statbuf)) {
		for(string::size_type c=shmRoot.find('/',1); c!=string::npos; c=shmRoot.find('/',c+1)) {
			//cout << "Checking " << shmRoot.substr(0,c) << endl;
			if(stat(shmRoot.substr(0,c).c_str(),&statbuf)) {
				mkdir(shmRoot.substr(0,c).c_str(),0777);
			} else if(!(statbuf.st_mode&S_IFDIR)) {
				cerr << "*** ERROR " << shmRoot.substr(0,c) << " exists and is not a directory" << endl;
				cerr << "           Cannot create file-backed shared memory regions in " << shmRoot << endl;
				exit(EXIT_FAILURE);
			}
		}
		cout << "Created '" << shmRoot.substr(0,shmRoot.rfind('/')) << "' for file-backed shared memory storage" << endl;
	} else if(!(statbuf.st_mode&S_IFDIR)) {
		cerr << "*** ERROR " << shmRoot.substr(0,shmRoot.rfind('/')) << " exists and is not a directory" << endl;
		cerr << "           Cannot create file-backed shared memory regions with prefix " << shmRoot << endl;
		exit(EXIT_FAILURE);
	}
#endif
	int fd=-1;
	if(name.size()>=MAX_NAME_LEN)
		cerr << "*** WARNING RCRegion named " << name << " will be clipped to " << name.substr(0,MAX_NAME_LEN-1) << endl;
	strncpy(id.key,name.c_str(),MAX_NAME_LEN-1);
	id.key[MAX_NAME_LEN-1]='\0';
	if(create) {
		static unsigned int renameSN=0;
		switch(conflictStrategy) {
			case RENAME: {
				char origName[MAX_NAME_LEN];
				strncpy(origName,id.key,MAX_NAME_LEN);
				if((fd=openRegion(O_RDWR|O_CREAT|O_EXCL))>=0)
					break;
				do {
					int err=errno;
					if(err!=EEXIST) {
						cerr << "ERROR: Opening new region " << id.key << ": " << strerror(err) << " (shm_open)" << endl;
						exit(EXIT_FAILURE);
					}
					unsigned int p=snprintf(id.key,MAX_NAME_LEN,"%s-%d",origName,++renameSN);
					if(p>=MAX_NAME_LEN) {
						cerr << "ERROR: conflicted key " << origName << ", attempting to rename, but generated name is too long" << endl;
						exit(EXIT_FAILURE);
					}
					//id.key[MAX_NAME_LEN-1]='\0';
#ifdef DEBUG
					cerr << "Warning: conflicted key " << origName << ", attempting to rename as " << id.key << "\n"
						<< "         (may have been leftover from a previous crash)" << endl;
#endif
				} while((fd=openRegion(O_RDWR|O_CREAT|O_EXCL))<0);
				break;
			}
			case REPLACE: {
				if((fd=openRegion(O_RDWR|O_CREAT|O_EXCL))>=0)
					break;
				int err=errno;
				if(err!=EEXIST) {
					cerr << "ERROR: Opening new region " << id.key << ": " << strerror(err) << " (shm_open)" << endl;
					exit(EXIT_FAILURE);
				}
#ifdef DEBUG
				cerr << "Warning: conflicted key " << id.key << ", attempting to replace\n"
				     << "         (may have been leftover from a previous crash)" << endl;
#endif
				if(!unlinkRegion())
					perror("Warning: Shared memory unlink");
			}
			//note fall-through from REPLACE into EXIT - only try delete once, and then recreate and exit if it fails again
			case EXIT: {
				if((fd=openRegion(O_RDWR|O_CREAT|O_EXCL))<0) {
					int err=errno;
					cerr << "ERROR: Opening new region " << id.key << ": " << strerror(err) << " (shm_open)" << endl;
					if(err==EEXIST)
						cerr << "This error suggests a leaked memory region, perhaps from a bad crash on a previous run.\n"
#ifdef USE_UNBACKED_SHM
							<< "You may either be able to use shm_unlink to remove the region, or reboot.\n"
#endif
							<< "Also make sure that no other copies of the simulator are already running." << endl;
					exit(EXIT_FAILURE);
				}
			}
		}
		if (ftruncate(fd,sz)<0) {
			int err=errno;
			cerr << "ERROR: Sizing region " << id.key << " to " << sz << ": " << strerror(err) << " (ftruncate)" << endl;
			if(close(fd)<0)
				perror("Warning: Closing temporary file descriptor from shm_open");
			if(!unlinkRegion())
				perror("Warning: Shared memory unlink");
			exit(EXIT_FAILURE);
		}
	} else {
		if((fd=openRegion(O_RDWR))<0) {
			int err=errno;
			cerr << "ERROR: Opening existing region " << id.key << ": " << strerror(err) << " (shm_open)" << endl;
			exit(EXIT_FAILURE);
		}
	}
	base=static_cast<char*>(mmap(NULL,sz,PROT_READ|PROT_WRITE,MAP_SHARED,fd,(off_t)0));	
	int err=errno;
	if (base == MAP_FAILED) { /* MAP_FAILED generally defined as ((void*)-1) */
		cerr << "ERROR: Attaching region " << id.key << " of size " << sz << ": " << strerror(err) << " (mmap)" << endl;
		if(close(fd)<0)
			perror("Warning: Closing temporary file descriptor from shm_open");
		if(!unlinkRegion())
			perror("Warning: Shared memory unlink");
		exit(EXIT_FAILURE);
	}
	if(close(fd)<0) {
		perror("Warning: Closing temporary file descriptor from shm_open");
	}
	references=reinterpret_cast<unsigned int*>(base+sz-extra);
	if(create) {
		for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
			references[i]=0;
	}
	AddReference();
	attachedRegions[id.key]=this;
}

#elif TEKKOTSU_SHM_STYLE==NO_SHM

void RCRegion::init(size_t sz, const std::string& name, bool create) {
	MarkScope l(getStaticLock());
	id.size=sz; //size of requested region
	sz=calcRealSize(sz); //add some additional space for region lock and reference counts
	if(name.size()>=MAX_NAME_LEN)
		cerr << "*** WARNING RCRegion named " << name << " will be clipped to " << name.substr(0,MAX_NAME_LEN-1) << endl;
	strncpy(id.key,name.c_str(),MAX_NAME_LEN-1);
	id.key[MAX_NAME_LEN-1]='\0';
	if(create) {
		static unsigned int renameSN=0;
		switch(conflictStrategy) {
			case RENAME: {
				if(attachedRegions.find(id.key)==attachedRegions.end())
					break;
				char origName[MAX_NAME_LEN];
				strncpy(origName,id.key,MAX_NAME_LEN);
				do {
					int err=errno;
					if(err!=EEXIST) {
						cerr << "ERROR: Opening new region " << id.key << ": " << strerror(err) << " (shm_open)" << endl;
						exit(EXIT_FAILURE);
					}
					unsigned int p=snprintf(id.key,MAX_NAME_LEN,"%s-%d",origName,++renameSN);
					if(p>=MAX_NAME_LEN) {
						cerr << "ERROR: conflicted key " << origName << ", attempting to rename, but generated name is too long" << endl;
						exit(EXIT_FAILURE);
					}
					//id.key[MAX_NAME_LEN-1]='\0';
#ifdef DEBUG
					cerr << "Warning: conflicted key " << origName << ", attempting to rename as " << id.key << "\n"
						<< "         (may have been leftover from a previous crash)" << endl;
#endif
				} while(attachedRegions.find(id.key)!=attachedRegions.end());
				break;
			}
			case REPLACE: {
				if(attachedRegions.find(id.key)==attachedRegions.end())
					break;
#ifdef DEBUG
				cerr << "Warning: conflicted key " << id.key << ", attempting to replace" << endl;
#endif
			}
				//note fall-through from REPLACE into EXIT - only try delete once, and then recreate and exit if it fails again
			case EXIT: {
				if(attachedRegions.find(id.key)!=attachedRegions.end()) {
					cerr << "ERROR: Opening new region " << id.key << ": conflicted with existing region." << endl;
					exit(EXIT_FAILURE);
				}
			}
		}
		base=new char[sz];
	} else {
		attachedRegions_t::const_iterator it=attachedRegions.find(id.key);
		ASSERT(it==attachedRegions.end(),"attachment not found with disabled shared mem (TEKKOTSU_SHM_STYLE==NO_SHM)");
		if(it==attachedRegions.end()) {
			base=new char[sz];
		} else {
			base=it->second->base;
		}
	}
	references=reinterpret_cast<unsigned int*>(base+sz-extra);
	if(create) {
		for(unsigned int i=0; i<ProcessID::NumProcesses+1; i++)
			references[i]=0;
	}
	AddReference();
	attachedRegions[id.key]=this;
}


#else
#  error "Unknown TEKKOTSU_SHM_STYLE setting"
#endif

/*
 class syserr : public std::exception {
public:
#if TEKKOTSU_SHM_STYLE==SYSV_SHM
	 syserr(int errnum, const key_t& key, const std::string& msg) throw() : info()
#elif TEKKOTSU_SHM_STYLE==POSIX_SHM
	 syserr(int errnum, const std::string& key, const std::string& msg) throw() : info()
#endif
 {
		 stringstream tmp;
		 tmp << "Exception regarding region " << key << ": " << msg << '(' << strerror(errnum) << ')';
		 info=tmp.c_str();
 }
	 virtual ~syserr() throw() {}
	 virtual const char * what() const throw() { return info.c_str(); }
protected:
			std::string info;
 };
 */

/*! @file
* @brief Implements RCRegion, which provides compatability with the OPEN-R type of the same name
* @author ejt (Creator)
*/

#endif
