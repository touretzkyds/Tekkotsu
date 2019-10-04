#ifndef __KNOWLEDGEBASE_H
#define __KNOWLEDGEBASE_H

#include <vector>
#include "Hashtable.h"
#include "CharPtrHash.h"

class model;
class object;
class keypoint;
class keypointPair;
class keygroup;
class matchInfo;

class KDTree;

namespace NEWMAT {
	class Matrix;
}

// void getBestHoughTransforms(vector<NEWMAT::Matrix>* transforms, vector<keypoint*> keys, vector<keypoint*> matches, vector<double>* error, vector<vector<keypointPair*>*>* inliersToReturn);

class KnowledgeBase{
private:
	
	class modelMatchingInfo{
	public:
		modelMatchingInfo();
		~modelMatchingInfo();
		
		model* matchedModel;
		NEWMAT::Matrix* solution;
		std::vector<keypointPair*>* inliers;
		double error, probOfMatch;
	private:
		modelMatchingInfo(const modelMatchingInfo&); // Do not call
		modelMatchingInfo& operator=(const modelMatchingInfo&); // Do not call
	};
	
	
	size_t maxNumModels;
	std::vector<keypoint*> keys;
	std::vector<keygroup*> keygroups;
	std::vector<model*>    models;
	std::vector<object*>  objects;
	KDTree* myTree;
	Hashtable<double, CharPtrKey, hashCharPtrKey, CharPtrKeyEquals> paramHash;
  	
	void cleanUpMemory();
  	
	void keypointMatching(std::vector<keypoint*>* newKeys, std::vector< std::vector< std::vector<keypoint*> > >& matches, std::vector< std::vector< std::vector<keypoint*> > >& imageKey, bool objectSpecified, int wantedObjectID);
	void modelMatching(size_t numNewKeys, std::vector< std::vector< std::vector<keypoint*> > >& matches, std::vector< std::vector< std::vector<keypoint*> > >& imageKey, std::vector<modelMatchingInfo*>& mminfo/*, vector<model*>& acceptableModels, vector<double>& modelConfidence, vector<double>& modelError, vector<int>& solutionIndex, vector< vector< vector<NEWMAT::Matrix*> > >& solutions, vector< vector< vector< vector<keypointPair*>* > > >& inliers*/);
	
	void rebuildKDTree();
	
	object* learn_newObject(std::vector<keypoint*>* newKeys);
	object* unlearn_Object(std::vector <keypoint*>* newKeys, int oID);
	void    unlearn_fromModel(std::vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform, double& s, double& theta, double& tx, double& ty);		
	model*  learn_newModel( std::vector<keypoint*>* newKeys, object* O, std::vector<modelMatchingInfo*> mminfo);
	model*  unlearn_Model( std::vector<keypoint*>* newKeys, object* O, std::vector<modelMatchingInfo*> mminfo, int mID);
	void    learn_toModel(  std::vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform, double& s, double& theta, double& tx, double& ty);
	void    learn_toModel(  std::vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform);
	void    unlearn_fromModel(  std::vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform);
	object* objectExists(int objectID);
	
	friend class SiftTekkotsu;
	
public:
	KnowledgeBase();
	~KnowledgeBase();
	
	// note: if haveKeys, keypoint* in K are either integrated into KDTree, or are freed by learn
	// either way, caller function (no?) need to free key explicitly
	// 		void learn(string filename, vector<keypoint*>& K, bool haveKeys, bool toIntegrate);
	
	// If mInfo != NULL, a copy of inliers will be made into mInfo
	// caller function must eventually free these inliers
	void learn(std::vector<keypoint*>& K, matchInfo& mInfo, bool toIntegrate);
	
	// 		void learnFromFiles(vector<string> filenames, bool toIntegrate);
	// 		void learnFromFile(string filename, bool toIntegrate);
	
	
	void   setParameter(const char* paramName, double paramVal);
	double getParameter(const char* paramName);
	
	void saveToFile(std::ofstream& outfile);
	void readFromFile(std::ifstream& infile);
	
private:
	KnowledgeBase(const KnowledgeBase&); // Do not call
	KnowledgeBase operator=(const KnowledgeBase&); // Do not call
	
};

int getNumParams();
const char* getParamList(int i);


#endif
