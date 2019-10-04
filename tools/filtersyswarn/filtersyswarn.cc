#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <regex.h>
#include <sys/types.h>

using namespace std;

//you can pass a replacement path to override OPENRSDK_ROOT

int main(int argc, const char** argv) {
	regex_t instbegin, inststat, warnbegin, warnstat, inclbegin, inclstat;
	
	string OPENR="^";
	if(argc>1) {
		OPENR+=argv[1];
	} else {
		char* OPENRSDK_ROOT=getenv("OPENRSDK_ROOT");
		OPENR+=(OPENRSDK_ROOT!=NULL)?OPENRSDK_ROOT:"/usr/local/OPEN_R_SDK/";
	}
	OPENR="("+OPENR+"|Shared/newmat/)";
	string clip="  [...]";
	
	regcomp(&inclbegin,"^In file included from",REG_EXTENDED|REG_NOSUB);
	regcomp(&inclstat, "^                 from",REG_EXTENDED|REG_NOSUB);

	// string instbeginstr=".*: (At global scope:|In instantiation of |In method |In member function|In destructor |In constructor |In function |In static )";
	// problems with line wrapping on long paths... but does this filter too much now?
	string instbeginstr=".*: (At|In)";
	string inststatstr=instbeginstr+"|   (instantiated|required) from |^   ";
	regcomp(&instbegin,instbeginstr.c_str(),REG_EXTENDED|REG_NOSUB);
	regcomp(&inststat,inststatstr.c_str(),REG_EXTENDED|REG_NOSUB);

	string pliststr_no_vdtor=".*/plist[a-zA-Z]*\\.h:[0-9]*: warning: base class ";
	string xmlloadsave_no_vdtor=".*/XMLLoadSave\\.cc:[0-9]*: warning: base class .*_xmlParserCtxt";
	string dualcoding_no_vdtor=".*DualCoding/(PolygonData\\.cc|ShapeFuns\\.h):[0-9]*: warning: base class ";
	string fmat_no_vdtor=".*:[0-9]*: warning: base class .*(class|struct) fmat::";
	string std_no_vdtor=".*:[0-9]*: warning: base class .*(class|struct) std::";
	string dyna_no_vdtor=".*Dynamixel.*:[0-9]*: warning: base class ";
    string factory_incomplete=".*Factories\\.h:[0-9]*:[0-9]*: warning: (invalid use of incomplete type|declaration of) ";
	string escape_range=".*: warning: escape sequence out of range";
	regcomp(&warnbegin,(OPENR+".*: warning: |"+escape_range+"|"+pliststr_no_vdtor+
                        "|"+xmlloadsave_no_vdtor+"|"+dualcoding_no_vdtor+"|"+fmat_no_vdtor+"|"+std_no_vdtor+"|"+dyna_no_vdtor+
                        "|"+factory_incomplete+"|.*local/terk/(TeRKPeerCommon|peer/MRPLPeer|SerialIO).h:.*: warning:").c_str(),REG_EXTENDED|REG_NOSUB);
	regcomp(&warnstat,"^   |^ +(template<|class |struct |\\^)",REG_EXTENDED|REG_NOSUB);

	string file, instant;

	string s;
	getline(cin,s);
	bool outfile=false,outinstant=false;
	bool fileclipped=false,instantclipped=false,warnclipped=false;
	while(cin) {
		if(regexec(&inclbegin,s.c_str(),0,NULL,0)==0) {
			file="";
			do {
				file+=s+'\n';
				getline(cin,s);
			} while(regexec(&inclstat,s.c_str(),0,NULL,0)==0);
			outfile=false;
			fileclipped=warnclipped;
			instantclipped=warnclipped=false;
		}
		if(regexec(&instbegin,s.c_str(),0,NULL,0)==0) {
			instant="";
			do {
				instant+=s+'\n';
				getline(cin,s);
			} while(regexec(&inststat,s.c_str(),0,NULL,0)==0);
			outinstant=false;
			instantclipped=warnclipped;
			warnclipped=false;
		}
		if(regexec(&warnbegin,s.c_str(),0,NULL,0)==0) {
			instant="";
			do {
				getline(cin,s);
			} while(regexec(&warnstat,s.c_str(),0,NULL,0)==0);
			warnclipped=true;
			continue;
		}
		if(!outfile) {
			if(fileclipped) {
				cout << clip << endl;
				fileclipped=false;
			}
			cout << file;
			outfile=true;
		}
		if(!outinstant) {
			if(instantclipped) {
				cout << clip << endl;
				instantclipped=false;
			}
			cout << instant;
			outinstant=true;
		}
		if(warnclipped) {
			cout << clip << endl;
			warnclipped=false;
		}
		cout << s << endl;
		getline(cin,s);
	}
	if(warnclipped)
		cout << clip << endl;
}
