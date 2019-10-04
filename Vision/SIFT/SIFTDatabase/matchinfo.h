#ifndef __MATCHINFO_H
#define __MATCHINFO_H

#include <vector>

class model;
class object;

class matchInfo {
public:
	
	model* M;
	model* matchedM;
	object* O;
	object* matchedO;
	double s, theta, tx, ty;
	std::vector<double> initialVals;
	std::vector<double> newVals;
	std::vector<bool>   hasMatch;
	std::vector<double> matchedVals;
	
	matchInfo()
		: M(NULL), matchedM(NULL), O(NULL), matchedO(NULL), 
		s(), theta(), tx(), ty(), initialVals(), newVals(), hasMatch(), matchedVals()
	{}
	
private:
	matchInfo(const matchInfo&); // Do not use
	matchInfo& operator=(const matchInfo&); // Do not use
	
};

#endif
