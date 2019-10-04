#include "object.h"
#include "model.h"

object::object(int ID) : id(ID), models(), name("object_unnamed") {}

void object::setName(const std::string& Name) { name = Name; }

const std::string& object::getName() const { return name; }

model* object::modelExists(int modelID) {
	// Check if the model exists
	for (unsigned int i = 0; i < models.size(); i++){
		if (models[i]->getID() == modelID){
			return models[i];
		}
	}
	return NULL;
}

int object::getID() const { return id; }

void object::writeToFile(std::ofstream& outfile){
	outfile << id << std::endl;
	outfile << name << std::endl;
	outfile << models.size() << std::endl;
	for (unsigned int i = 0; i < models.size(); i++){
		outfile << models[i]->getID() << "\t";
	}
	outfile << std::endl;
}

