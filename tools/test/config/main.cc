#include "Shared/Config.h"
#include <iostream>

using namespace std;

int main (int argc, char * const argv[]) {
#ifndef TGT_ERS7
	// we have different default settings for different models
	// so the 'ideal' file is only going to match ERS7
	std::cout << "Skipping config load test on non-ERS7" << std::endl;
#else
	std::cout << "Loading old format" << std::endl;
	config=new Config("tekkotsu.cfg");
	std::cout << "Saving 1" << std::endl;
	config->saveFile("tekkotsu.plist");
	std::cout << "Reloading 1" << std::endl;
	Config config2("tekkotsu.plist");
	//config2.clearParseTree();
	std::cout << "Saving 2" << std::endl;
	config2.saveFile("tekkotsu2.plist");
	std::cout << "Loading current" << std::endl;
	Config config3("tekkotsu.xml");
	std::cout << "Saving 3" << std::endl;
	config3.saveFile("tekkotsu3.plist");
	delete config;
#endif
	
	char v0[] = "1";
	char * v1 = v0;
	const char * v2 = "2";
	const char * const v3 = "3";
	char v4[] = "1";
	const char v5[] = "2";
	std::string v6 = "3";
	
	//const size_t outbufSize=1000;
	//char outbuf[outbufSize];
	
	plist::Array test1;
	test1.addValue("4"); test1.addValue(v1); test1.addValue(v2); test1.addValue(v3); test1.addValue(v4); test1.addValue(v5); test1.addValue(v6);
	test1.saveFile("test1.plist");
	/*size_t used=test1.saveBuffer(outbuf,outbufSize);
	if(used==0)
		return 1;
	outbuf[used]='\0';
	cout << outbuf << endl;*/
	
	plist::ArrayOf<plist::Primitive<int> > test2;
	test2.addValue("4"); test2.addValue(v1); test2.addValue(v2); test2.addValue(v3); test2.addValue(v4); test2.addValue(v5); test2.addValue(v6);
	test2.saveFile("test2.plist");
	
	plist::ArrayOf<plist::Primitive<std::string> > test3;
	test3.addValue("4"); test3.addValue(v1); test3.addValue(v2); test3.addValue(v3); test3.addValue(v4); test3.addValue(v5); test3.addValue(v6);
	test3.saveFile("test3.plist");
	
	plist::ArrayOf<plist::Collection> col;
	col.addEntry(test1,"Array of anything (but only adding strings)");
	col.addEntry(test2,"Array of integers");
	col.addEntry(test3,"Array of strings");
	/*size_t used=col.saveBuffer(outbuf,outbufSize);
	if(used==0)
		return 1;
	outbuf[used]='\0';
	cout << outbuf << endl;*/
	col.saveFile("col.plist");
	
	return 0;
}
