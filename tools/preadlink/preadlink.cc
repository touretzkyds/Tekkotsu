#include <iostream>
#include <stdlib.h>
#include <limits.h>

using namespace std;

int main(int argc, char* argv[]) {
  char *argpath = argv[1];
  char abspath[PATH_MAX+1];
  char *ref;

  ref = realpath(argpath, abspath);
  cout << ref << endl;
  return 0;
}
