#include <iostream>
#include <cstdlib>
#include "Motion/KinematicJoint.h"

void printStructure(const KinematicJoint& kj, unsigned int depth=0) {
	/*for(unsigned int i=0; i<depth; ++i)
		std::cout << '\t';
	std::cout << kj.outputOffset.get() << std::endl;*/
	std::cout << kj.getPath() << std::endl;
	for(KinematicJoint::branch_iterator it=kj.getBranches().begin(); it!=kj.getBranches().end(); ++it)
		printStructure(**it,depth+1);
}

int main(int argc, const char* argv[]) {
	KinematicJoint kj;
	if(argc>1)
		kj.loadFile(argv[1]);
	
	printStructure(kj);
	
	kj.clearParseTree();
	if(argc>2) {
		if(strcmp(argv[2],"-")==0)
			kj.saveStream(std::cout);
		else
			kj.saveFile(argv[2]);
	}
	return EXIT_SUCCESS;
}