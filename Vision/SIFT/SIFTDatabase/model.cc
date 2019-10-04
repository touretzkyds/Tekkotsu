#include "model.h"
#include "keygroup.h"
#include "object.h"

model::model(int ID) : id(ID), generation(0), keygroups(), O(NULL), name("model_unnamed") {}

void model::setName(const std::string& Name){ name = Name; }

const std::string& model::getName() const { return name; }

int model::getID() const { return id; }

void model::writeToFile(std::ofstream& outfile){
	outfile << id << std::endl;
	outfile << O->getID() << std::endl;
	outfile << generation << std::endl;
	outfile << name << std::endl;
	outfile << keygroups.size() << std::endl;
	for (size_t i = 0; i < keygroups.size(); i++){
		outfile << keygroups[i]->getID() << "\t";
	}
	outfile << std::endl;
}

