#ifndef __MODEL_H
#define __MODEL_H

#include <vector>
#include <string>
#include <fstream>

class keygroup;
class object;

class model{
public:
	model(int ID);
	
	int id;
	unsigned int generation;
	std::vector<keygroup*> keygroups;
	object* O;
	std::string name;
  	
	void setName(const std::string& Name);
	const std::string& getName() const;
  	
	int getID() const;
  	
	void writeToFile(std::ofstream& outfile);
  	
private:
	model(const model&);
	model& operator=(const model&);
};

#endif
