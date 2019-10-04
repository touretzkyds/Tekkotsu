#include "local/minisim.h"

#include "Shared/Base64.h"

#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

using namespace std;
using namespace minisim;

int main(int argc, char** argv) {
  initialize();

  if(argc<3 || ( strcmp(argv[1],"encode") && strcmp(argv[1],"decode") )) {
    cerr << "Usage: " << argv[0] << " (encode|decode) filename" << endl;
    exit(2);
  }

  bool isEncode=!strcmp(argv[1],"encode");
  string file=argv[2];
  
  struct stat sb;
  if(stat(file.c_str(),&sb)) {
    perror("stat");
    exit(1);
  }
  char * buf = new char[sb.st_size];
  int fd=open(file.c_str(),O_RDONLY,0);
  if(fd==-1) {
    perror("open");
    exit(1);
  }
  unsigned int r=0;
  while(r<(unsigned int)sb.st_size) {
    unsigned int n=read(fd,buf,sb.st_size-r);
    if(n<=0) {
      cerr << "file load error" << endl;
      close(fd);
      exit(1);
    }
    r+=n;
  }
  close(fd);

  if(isEncode) {
    cout << base64::encode(buf,sb.st_size);
  } else {
    string enc(buf,sb.st_size);
    char * dat = base64::decode(enc);
    if(dat==NULL) {
      cerr << "decoding error" << endl;
      exit(1);
    }
    unsigned int len=base64::decodeSize(enc);
    ssize_t n = write(STDOUT_FILENO,dat,len);
    if(static_cast<unsigned int>(n)!=len)
      cerr << "Warning: short write from decode" << endl;
    delete [] dat;
  }
  delete [] buf;

  destruct();
  return 0;
}

