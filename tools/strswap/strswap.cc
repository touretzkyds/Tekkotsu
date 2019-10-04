#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

typedef char byte;

void write(ostream& o, const byte* b, unsigned int n);

int main(int argc, const char* argv[]) {
	if(argc<4) {
		cerr << "Usage: "<<argv[0]<<" [--inplace] filename oldstr newstr" <<endl;
		cerr << "       This will replace all occurances of oldstr by newstr in the file." << endl;
		cerr << "       Results will go to stdout - you should probably pipe them to a file." << endl;
		return 1;
	}
	unsigned int argn=1;
	bool inplace=(strcmp(argv[argn],"--inplace")==0);
	if(inplace)
		argn++;
	fstream f(argv[argn],std::ios::in|std::ios::binary);
	if(!f) {
		cerr << "Unable to open " << argv[argn] << " for reading" << endl;
		return 1;
	}
	if(inplace) {
		f.close();
		f.open(argv[argn],std::ios::in|std::ios::out|std::ios::binary);
	}
	if(!f) {
		cerr << "Unable to open " << argv[argn] << " for writing inplace" << endl;
		return 1;
	}
	argn++;
	const byte * srchstr=static_cast<const byte*>(argv[argn++]);
	const byte * replstr=static_cast<const byte*>(argv[argn++]);
	for(;argn<(unsigned int)argc;argn++)
		cerr << "Warning: ignoring extra argument `"<<argv[argn]<<"'" << endl;
	unsigned int srchlen=strlen(static_cast<const char*>(srchstr));
	unsigned int repllen=strlen(static_cast<const char*>(replstr));
	if(inplace && repllen!=srchlen) {
		cerr << "oldstr and newstr must be the same length for inplace" << endl;
		return 1;
	}
	
	unsigned int bufsize=srchlen*4096;
	byte * buf = new byte[bufsize];
	f.read(buf,srchlen-1);
	unsigned int cur=f.gcount();
	
	while(f) {
		f.get(buf[cur++]);
		if(memcmp(&buf[cur-srchlen],srchstr,srchlen)==0) {
			if(inplace) {
				f.seekp(-repllen,std::ios::cur);
				f.write(replstr,repllen);
				f.seekp(repllen,std::ios::cur);
			} else {
				cout.write(buf,cur-srchlen);
				cout.write(replstr,repllen);
			}
			f.read(buf,srchlen-1);
			cur=f.gcount();
		}
		if(cur>=bufsize) {
			if(cur>bufsize)
				cout << "Overflow!" << endl;
			if(!inplace)
				cout.write(buf,cur-srchlen+1);
			memcpy(buf,&buf[cur-srchlen+1],srchlen-1);
			cur=srchlen-1;
		}
	}
	if(!inplace)
		write(cout,buf,cur-1);

	return 0;
}

void write(ostream& o, const byte* b, unsigned int n) {
	while(n--!=0)
		o.put(*b++);
}
