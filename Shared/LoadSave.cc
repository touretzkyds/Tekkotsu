#include "LoadSave.h"
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

LoadSave::~LoadSave() {}

unsigned int LoadSave::checkCreator(const char* creator, const char buf[], unsigned int len, bool isLoading) const throw() {
	const char* type=buf+getSerializedSize<unsigned int>();
	if(strcmp(type,creator)!=0) {
		if(isLoading)
			std::cout << "*** WARNING " << creator << "::loadBuffer - corruption detected, got '" << std::string(type) << "'" << std::endl;
		return 0;
	}
	unsigned int sz=0;
	decode(sz,buf,len);
	return sz+stringpad;
}
bool LoadSave::checkCreatorInc(const char* creator, const char*& buf, unsigned int& len, bool isLoading) const throw() {
	if(unsigned int used=checkCreator(creator,buf,len,isLoading)) { buf+=used; len-=used; return true; }
	else return false;
}
void LoadSave::checkCreatorIncT(const char* creator, const char*& buf, unsigned int& len, bool isLoading) const throw(std::runtime_error) {
	if(checkCreatorInc(creator,buf,len,isLoading))
		return;
	std::string err="incorrect creator code: ";
	err+=creator;
	err+=" vs ";
	err+=std::string(buf,strlen(creator));
	throw std::runtime_error(err);
}

unsigned int LoadSave::checkCreator(const char* creator, FILE* f, bool isLoading) const throw() {
	unsigned int sz=0,last;
	unsigned int origpos=ftell(f);
	char* type=NULL;
	if(!(last=decode(type,f))) {
		fseek(f,origpos,SEEK_SET);
		return 0;
	} else
		sz+=last;
	if(strncmp(type,creator,strlen(creator))!=0) {
		if(isLoading)
			std::cout << "*** WARNING " << creator << "::loadBuffer - corruption detected" << std::endl;
		fseek(f,origpos,SEEK_SET);
		delete [] type;	
		return 0;
	}
	delete [] type;	
	return sz;
}

unsigned int LoadSave::saveCreator(const char* creator, char buf[], unsigned int len) const throw() {
	return encode(std::string(creator),buf,len);
}
bool LoadSave::saveCreatorInc(const char* creator, char*& buf, unsigned int& len) const throw() {
	if(unsigned int used=saveCreator(creator,buf,len)) { buf+=used; len-=used; return true; }
	else return false;
}
void LoadSave::saveCreatorIncT(const char* creator, char*& buf, unsigned int& len) const throw(std::runtime_error) {
	if(saveCreatorInc(creator,buf,len))
		return;
	std::string err="unable to save creator code: ";
	err+=creator;
	err+=" (ran out of space?)";
	throw std::runtime_error(err);
}
unsigned int LoadSave::saveCreator(const char* creator, FILE* f) const throw() {
	return encode(creator,f);
}

unsigned int LoadSave::loadFile(const char* file) {
	int err;
	std::cout << "Loading: " << file << std::endl;
	FILE* f = fopen(file,"r");
	if(f==NULL) {
		std::cout << "*** WARNING could not open file for loading \"" << file << "\"" << std::endl;
		return 0;
	}
	unsigned int sz = loadFileStream(f,file);
	if(sz==0)
		std::cout << "*** WARNING loading of " << file << " failed " << std::endl;
	err=fclose(f);
	if(err!=0) {
		std::cout << "*** WARNING error " << err << " while closing " << file << std::endl;
		return 0;
	}
	return sz;
}

unsigned int LoadSave::saveFile(const char* file) const {
	int err;
	std::cout << "Saving: " << file << std::endl;
	FILE* f = fopen(file,"w");
	if(f==NULL) {
		std::cout << "*** WARNING could not open file for saving \"" << file << "\"" << std::endl;
		return 0;
	}
	unsigned int sz = saveFileStream(f);
	if(sz==0)
		std::cout << "*** WARNING saving of " << file << " failed " << std::endl;
	err=fclose(f);
	if(err!=0) {
		std::cout << "*** WARNING error " << err << " while closing " << file << std::endl;
		return 0;
	}
	return sz;
}

unsigned int LoadSave::loadFileStream(FILE* f, const char* filename) {
	struct stat sb;
	int err=fstat(fileno(f),&sb);
	if(err!=0) {
		if (filename)
			std::cerr << "File: " << filename << std::endl;
		perror("fstat failed in LoadSave::loadFileStream()");
		return 0;
	}
	long origpos=ftell(f);
	if(origpos<0) {
		if (filename)
			std::cerr << "File: " << filename << std::endl;
		perror("ftell failed in LoadSave::loadFileStream()");
		return 0;
	}
	size_t cap=static_cast<size_t>(sb.st_size-origpos);
	char * buf = NULL;
	try { buf=new char[cap]; } catch(...) {}
	if(buf==NULL) {
		if (filename)
			std::cerr << "File: " << filename << std::endl;
		std::cout << "*** WARNING could not allocate " << cap << "+ bytes for loadFile";
		return 0;
	}
	unsigned int read=fread(buf,1,cap,f);
	unsigned int pos=0;
	while(cap!=pos+read) {
		pos+=read;
		read=fread(&buf[pos],1,cap-pos,f);
	}
	unsigned int resp=loadBuffer(buf,cap);
	delete [] buf;
	if(resp!=cap)
		fseek(f,origpos+resp,SEEK_SET);
	return resp;
}
unsigned int LoadSave::saveFileStream(FILE* f) const {
	unsigned int sz=getBinSize();
	char * buf = new char[sz];
	if(buf==NULL) {
		std::cout << "*** ERROR could not allocate " << sz << " bytes for loadFile";
		return 0;
	}
	unsigned int resp=saveBuffer(buf,sz);
	if(resp==0) {
		std::cout << "*** WARNING saveBuffer (from saveFileStream) didn't write any data (possibly due to overflow or other error)" << std::endl;
	} else {
		size_t wrote = fwrite(buf,1,resp,f);
		if(wrote!=(size_t)resp)
			std::cout << "*** ERROR short write (wrote " << wrote << ", expected " << resp << ")" << std::endl;
	}
	delete [] buf;
	return resp;
}
unsigned int LoadSave::saveStream(std::ostream& os) const {
	unsigned int sz=getBinSize();
	char * buf = new char[sz];
	if(buf==NULL) {
		std::cout << "*** ERROR could not allocate " << sz << " bytes for loadFile";
		return 0;
	}
	unsigned int resp=saveBuffer(buf,sz);
	if(resp==0) {
		std::cout << "*** WARNING saveBuffer (from saveStream) didn't write any data (possibly due to overflow or other error)" << std::endl;
	} else {
		os.write(buf,resp);
		if(!os)
			std::cout << "*** ERROR saveStream's output stream reported bad write" << std::endl;
		resp=0;
	}
	delete [] buf;
	return resp;
}


unsigned int LoadSave::LoadFile(const char* filename) { return loadFile(filename); }
unsigned int LoadSave::SaveFile(const char* filename) const { return saveFile(filename); }
bool LoadSave::chkAdvance(int res, const char** buf, unsigned int* len, const char* msg, ...) {
	va_list al;
	va_start(al,msg);
	//todo
	bool ans=checkInc(res,*buf,*len,"%s",msg);
	va_end(al);
	return ans;
}

/*! @file
 * @brief Implements LoadSave, inherit from this to use a standard interface for loading and saving
 * @author ejt (Creator)
 */

