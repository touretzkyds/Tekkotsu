#ifndef __OBJECT_H
#define __OBJECT_H

#include <vector>
#include <string>
#include <fstream>

class model;

class object{
public:
	object(int ID);
	int id;
	std::vector<model*> models;
	std::string name;
	
	void setName(const std::string& Name);
	const std::string& getName() const;
	
	model* modelExists(int modelID);
	
	int getID() const;
	
	void writeToFile(std::ofstream& outfile);
	
};

#endif
