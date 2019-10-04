#include "KnowledgeBase.h"

#include "keypoint.h"
#include "keygroup.h"
#include "keypointpair.h"
#include "model.h"
#include "object.h"
#include "matchinfo.h"

#include "HoughHash.h"
#include "KDTree.h"
#include "Shared/newmat/newmat.h"

#include <limits>
#include "Shared/mathutils.h" // for isnan fix

using namespace std;

const int numParams = 2;
const char* paramList[numParams] = {"probOfMatch", "errorThreshold"};

int getNumParams(){
	return numParams;
}

const char* getParamList(int i){
	return paramList[i];
}

double choose(int n, int r){
	if (n < 1 || r < 0) return -0.0;
	if (r < n-r) r = n-r;
	
	double numerator = 1;
	double denominator = 1;
	
	for (int i = r+1; i <= n; i++){
		numerator *= (double)i;
		//		cout << "numerator " << numerator << endl;
	}
	for (int i = 1; i <= n-r; i++){
		denominator *= (double)i;
		//		cout << "denominator " << denominator << endl;
	}
	
	if (numerator == std::numeric_limits<double>::infinity()) return numerator;
	
	return numerator/denominator;
}

NEWMAT::Matrix leastSquareSolution(std::vector<keypointPair*> keyPairs, double *error){
	// make assumption that #keys = #matches
	int numKeys = (int)keyPairs.size();
	//	cout << "numKeys = " << numKeys << endl;
	
	// Create A, b
	int Arows = 2 * numKeys;
	int Acols = 4;
	//	double Avals[Arows * Acols];
	//	int indexA = 0;
	
	int brows = 2 * numKeys;
	int bcols = 1;
	//	double bvals[brows * bcols];
	//	int indexb = 0;
	
	NEWMAT::Matrix m_A(Arows, Acols);
	NEWMAT::Matrix m_b(brows, bcols);
	
	for (int i = 0; i < Arows; i+=2){
		m_A(i+1, 1) = static_cast<NEWMAT::Real>( keyPairs[i/2]->getKey2()->modelX );
		m_A(i+1, 2) = static_cast<NEWMAT::Real>( -(keyPairs[i/2]->getKey2()->modelY) );
		m_A(i+1, 3) = static_cast<NEWMAT::Real>( 1 );
		m_A(i+1, 4) = static_cast<NEWMAT::Real>( 0 );
		m_A(i+2, 1) = static_cast<NEWMAT::Real>( keyPairs[i/2]->getKey2()->modelY );
		m_A(i+2, 2) = static_cast<NEWMAT::Real>( keyPairs[i/2]->getKey2()->modelX );
		m_A(i+2, 3) = static_cast<NEWMAT::Real>( 0 );
		m_A(i+2, 4) = static_cast<NEWMAT::Real>( 1 );
		
		m_b(i+1, 1) = static_cast<NEWMAT::Real>( keyPairs[i/2]->getKey1()->imageX );
		m_b(i+2, 1) = static_cast<NEWMAT::Real>( keyPairs[i/2]->getKey1()->imageY );
	}
	
	NEWMAT::Matrix m_ATran = m_A.t();
	
	NEWMAT::Matrix m_ATranA = m_ATran * m_A;
	
	NEWMAT::Matrix m_x = (m_ATranA.i() * m_ATran) * m_b;
	
	NEWMAT::Matrix m_Axb = (m_A * m_x) - m_b;
	
	*error = 0.0;
	
	for (int i = 1; i <= brows; i++){
		*error += (m_Axb(i,1) * m_Axb(i,1));
	}
	*error *= 2.0;
	*error /= (Arows - 4);
	*error /= (Arows);
	*error = std::sqrt(*error);
	
	return m_x;
}


NEWMAT::Matrix getBestTransform2(std::vector<keypointPair*> allPairs, std::vector<keypointPair*> seedPairs, double *error, std::vector<keypointPair*>** inliersToReturn){
	int maxIterations = 3;
	
	const double maxXYError = 0.25 * MAXIMAGEDIM;
	const double maxScaleError = 4.0;
	const double maxRotationError = 30.0/180.0 * M_PI;
	
	std::vector<keypointPair*>* inliers = new std::vector<keypointPair*>;
	
	// Everything belongs to inliers for now
	for (int i = 0; i < (int)seedPairs.size(); i++){
		inliers->push_back(seedPairs[i]);
	}
	
	
	double xyError = maxXYError;
	double scaleError = maxScaleError;
	double rotationError = maxRotationError;
	for (int i = 0; i < maxIterations; i++){
		double oldSolution[4] = {0,0,0,0};
		for (int ii = 0; ii < 100; ii++){
			//		cout << "\tIteration " << i << " with " << inliers->size() << " inliers" << endl;
			NEWMAT::Matrix solution = leastSquareSolution(*inliers, error);
			bool hasConverged = true;
			for (int j = 0; j < 4; j++){
				hasConverged &= (solution(j+1,1) == oldSolution[j]);
				oldSolution[j] = solution(j+1,1);
			}
			if (hasConverged) break;
			//		cout << "\tIteration " << i << " with " << inliers->size() << " inliers" << endl;
			double scale = sqrt(solution(1,1) * solution(1,1) + solution(2,1) * solution(2,1));
			double tempX = (double)(solution(1,1)) / scale;
			/// This part is weird.. for some reason, sometimes tempX is outside of range ///
			if (tempX < -1.0) tempX = -1.0;
			if (tempX >  1.0) tempX =  1.0;
			double theta = acos(tempX);
			//		cout << "\t\tin range? " << (tempX >= -1.0) << (tempX <= 1.0) << (tempX == 1.0) <<  " " << tempX << endl;
			if (solution(2,1) / scale < 0.0) theta = -theta;
			delete inliers;
			inliers = new std::vector<keypointPair*>;
			//		cout << "\t\t(" << solution(1,1) << "," << solution(2,1) << "," << solution(3,1) << "," << solution(4,1) << ")'" << endl;
			//		cout << "\t\tscale = " <<  scale << endl;
			//		cout.precision(10);
			//		cout << "\t\t" << (solution(1,1) / scale) << endl;
			//		cout << "\t\ttheta = " <<  theta << endl;
			//		cout << "\t\ttx = " <<  solution(3,1) << endl;
			//		cout << "\t\tty = " <<  solution(4,1) << endl;
			for (int j = 0; j < (int)allPairs.size(); j++){
				double xTranslate, yTranslate, zoom, rotation;
				allPairs[j]->getBackwardTransform(&xTranslate, &yTranslate, &zoom, &rotation);
				
				double imageX = allPairs[j]->getKey1()->imageX;
				double imageY = allPairs[j]->getKey1()->imageY;
				double modelX = allPairs[j]->getKey2()->modelX;
				double modelY = allPairs[j]->getKey2()->modelY;
				double expectedX = modelX * solution(1,1) - modelY * solution(2,1) + solution(3,1);
				double expectedY = modelX * solution(2,1) + modelY * solution(1,1) + solution(4,1);
				double xDist = imageX - expectedX;
				double yDist = imageY - expectedY;
				double xyDist = sqrt(xDist * xDist + yDist * yDist);
				
				double scaleDist = scale / zoom;
				if (scaleDist < 1.0) scaleDist = 1.0 / scaleDist;
				//			double scaleDist = scale - zoom;
				//			if (scaleDist < 0.0) scaleDist = -scaleDist;
				
				double rotationDist = theta - rotation;
				while (rotationDist > M_PI) rotationDist -= (2 * M_PI);
				while (rotationDist <= -M_PI) rotationDist += (2 * M_PI);
				
				// 		cout << "image: (" << imageX << "," << imageY << ") model: (" << modelX << "," << modelY << ") expected: (" << expectedX << "," << expectedY << ")\n";
				// 		cout << "zoom: " << zoom << " rotation: " << rotation << " rotationError: " << rotationError << " rotationDist: " << rotationDist;
				// 		cout << "\t" << (xyError       >= xyDist) << (scaleError    >= scaleDist) << (rotationError >= rotationDist) << endl;
				
				if (xyError       >= xyDist
					&& scaleError    >= scaleDist
					&& rotationError >= rotationDist
					){
					inliers->push_back(allPairs[j]);
					allPairs[j]->getKey1()->isInlier = true;      // NOTE: isInlier works as an indicator only because each keypoint is called by a getBestTransform ONCE
				}else{
					allPairs[j]->getKey1()->isInlier = false;     // NOTE: isInlier works as an indicator only because each keypoint is called by a getBestTransform ONCE
				}
			}
			
			if (inliers->size() < 4) break;
			
		}
		
		if (inliers->size() < 4) break;
		
		//		cout << inliers->size() << " inliers left, previous error: " << *error << "\n";
		
		xyError = maxXYError / (i+2);
		scaleError = maxScaleError / (i+2);
		rotationError = maxRotationError / (i+2);
		
		// 		xyError /= 2;
		// 		scaleError /= 2;
		// 		rotationError /= 2;
	}
	
	NEWMAT::Matrix finalSolution;
	
	if (inliers->size() >= 4)
		finalSolution = leastSquareSolution(*inliers, error);
	else{
		// double array[4] = {0.0, 0.0, 0.0, 0.0};
		// finalSolution = m_Matrix(4, 1, array);
		finalSolution = NEWMAT::Matrix(4, 1);
		finalSolution(1,1) = 0;
		finalSolution(2,1) = 0;
		finalSolution(3,1) = 0;
		finalSolution(4,1) = 0;
		*error = -1;
	}
	
	//	cout << "inliers->size() = " << inliers->size() << endl;
	
	*inliersToReturn = inliers;
	
	for (int i = 0; i < (int)inliers->size(); i++){
		(*inliers)[i]->getKey1()->modelMatches.push_back((*inliers)[i]->getKey2());
		(*inliers)[i]->getKey1()->modelMatchErrors.push_back(*error);
	}
	
	return finalSolution;
	
}

void getBestHoughTransforms(std::vector<NEWMAT::Matrix>* transforms, std::vector<keypoint*> keys, std::vector<keypoint*> matches, std::vector<double>* error, std::vector<std::vector<keypointPair*>*>* inliersToReturn){
	
	// 	cout << "getBestHoughTransforms\n";
	
	Hashtable<vector<keypointPair*>, HoughKey, hashHoughKey, HoughKeyEquals> HoughHash;
	vector<HoughKey*> allKeys;
	vector< vector<keypointPair*>* > allHashPairs;
	
	vector<keypointPair*> allPairs;
	
	transforms->clear();
	error->clear();
	inliersToReturn->clear();
	
	// Each match must also vote for all matches in the same cluster
	for (int i = 0; i < (int)keys.size(); i++){
		keypointPair* newpair = new keypointPair(keys[i], matches[i]);
		allPairs.push_back(newpair);
		
		double xTranslate, yTranslate, zoom, rotation;
		newpair->getBackwardTransform(&xTranslate, &yTranslate, &zoom, &rotation);
		
		// Compute best 2 bins for each dimension
		double bestX[2], bestY[2], bestZoom[2], bestRotation[2];
		
		if (zoom  < 1.0){
			double tempZoom = 1.0;
			while (tempZoom > zoom) tempZoom /= 2.0;
			bestZoom[0] = tempZoom;
			bestZoom[1] = tempZoom * 2.0;
		}else{
			double tempZoom = 1.0;
			while (tempZoom < zoom) tempZoom *= 2.0;
			bestZoom[0] = tempZoom / 2.0;
			bestZoom[1] = tempZoom;
		}
		
		double degrees30 = 30.0 / 180.0 * M_PI;
		int rotationBin = (int)(rotation / degrees30);
		bestRotation[0] = (double)rotationBin * degrees30;
		bestRotation[1] = bestRotation[0] + degrees30;
		
		for (int ii = 0; ii <= 1; ii++){
			double maxImageDim = bestZoom[ii] * MAXIMAGEDIM;
			double partitionImageDim = maxImageDim / 4.0;
			int xBin = (int)(xTranslate / partitionImageDim);
			bestX[0] = partitionImageDim * xBin;
			bestX[1] = bestX[0] + partitionImageDim;
			int yBin = (int)(yTranslate / partitionImageDim);
			bestY[0] = partitionImageDim * yBin;
			bestY[1] = bestY[0] + partitionImageDim;
			for (int jj = 0; jj <= 1; jj++){
				for (int xk = 0; xk <= 1; xk++){
					for (int yk = 0; yk <= 1; yk++){
						HoughKey* hKey = new HoughKey();
						allKeys.push_back(hKey);
						hKey->M = newpair->getKey2()->G->M;
						hKey->x = bestX[xk];
						hKey->y = bestY[yk];
						hKey->scale = bestZoom[ii];
						hKey->orientation = bestRotation[jj];
						
						vector<keypointPair*>* hashPairs = HoughHash.retrieve(hKey);
						if (hashPairs == NULL){
							hashPairs = new vector<keypointPair*>();
							allHashPairs.push_back(hashPairs);
							hashPairs->push_back(newpair);
							HoughHash.insert(hashPairs, hKey);
						}else{
							hashPairs->push_back(newpair);
						}
						
						//						cout << "Adding to hash: ("
						//						     << hashPairs->size() << ",\t"
						//						     << bestX[xk] << ",\t"
						//						     << bestY[yk] << ",\t"
						//						     << bestZoom[ii] << ",\t"
						//						     << bestRotation[jj] << ")\n";
					}
				}
			}
		}
	}
	
	// Iterate through results from hash
	vector<vector<keypointPair*>*> hashData;
	HoughHash.retrieveAllData(&hashData);
	
	for (int i = 0; i < (int)hashData.size(); i++){
		if (hashData[i]->size() < 3) continue;
		double err;
		vector<keypointPair*>* in;
		//		cout << "# in hash bin: " << hashData[i]->size() << endl << flush;
		NEWMAT::Matrix solution = getBestTransform2(allPairs, *hashData[i], &err, &in);
		
		// Check that a solution is found
		size_t numInliers = in->size();
		//		cout << numInliers << endl;
		if (numInliers >= 4){
			// Check for duplicity
			bool isDuplicate = false;
			for (int j = 0; j < (int)(transforms->size()); j++){
				if ((*transforms)[j] == solution){
					isDuplicate = true;
					break;
				}
			}
			
			if (isDuplicate){
				delete in;
				continue;
			}
			
			// Find location to do insertion (sort)
			vector< vector<keypointPair*>* >::iterator location = (*inliersToReturn).begin();
			vector< NEWMAT::Matrix >::iterator transformsLocation = (*transforms).begin();
			vector< double >::iterator errorLocation = (*error).begin();
			for (vector< vector<keypointPair*>* >::iterator j = (*inliersToReturn).begin(); j < (*inliersToReturn).end(); j++){
				if (in->size() >= (*j)->size()){
					break;
				}
				location++;
				transformsLocation++;
				errorLocation++;
			}
			// 			cout << "# inliers " << in->size() << ", error: " << err << endl;
			
			transforms->insert(transformsLocation, solution);
			error->insert(errorLocation, err);
			// Make copy of in
			vector<keypointPair*>* in2;
			in2 = new vector<keypointPair*>();
			for (int j = 0; j < (int)in->size(); j++){
				keypointPair* newPair = new keypointPair((*in)[j]->getKey1(), (*in)[j]->getKey2());
				in2->push_back(newPair);
			}
			inliersToReturn->insert(location, in2);
			delete in;
			//			double scale = sqrt(solution(1) * solution(1) + solution(2) * solution(2));
			//			double theta = acos(solution(1) / scale);
			//			cout << "(" << (*hashData[i])[0]->getKey2()->G->M->O 
			//					<< "," << (*hashData[i])[0]->getKey2()->G->M->id << ")"
			//					<< "  " << hashData[i]->size() << " \t"
			//					<< error << " \t" << scale << " \t" << theta << " \t" << solution(3) << " \t" << solution(4) << endl;
		}else{
			delete in;
		}
	}
	
	//	cout << endl << endl << endl;
	
	for (int i = 0; i < (int)(*inliersToReturn).size(); i++){
		// 		cout << "# inliers " << (*inliersToReturn)[i]->size() << ", error: " << (*error)[i] << endl;
	}
	
	for (int i = 0; i < (int)allPairs.size(); i++){
		delete allPairs[i];
	}
	for (int i = 0; i < (int)allHashPairs.size(); i++){
		delete allHashPairs[i];
	}
	for (int i = 0; i < (int)allKeys.size(); i++){
		delete allKeys[i];
	}
}

KnowledgeBase::modelMatchingInfo::modelMatchingInfo()
: matchedModel(NULL), solution(NULL), inliers(NULL), 
error(numeric_limits<double>::infinity()),
probOfMatch(numeric_limits<double>::infinity()) {}

// KnowledgeBase::modelMatchingInfo::modelMatchingInfo(const KnowledgeBase::modelMatchingInfo& other) : //copy constructor for modelMatchingInfo
//   matchedModel(other.matchedModel), solution(other.solution), inliers(other.inliers), error(other.error) {}

// KnowledgeBase::modelMatchingInfo::operator=(const KnowledgeBase::modelMatchingInfo& other) //Operator overloading for modelMatchingInfo
// {
//   matchedModel = other.matchedModel;
//   solution = other.solution;
//   inliers = other.inliers;
//   error = other.error;
//   return *this;
//     }

KnowledgeBase::modelMatchingInfo::~modelMatchingInfo(){
	if (solution != NULL) delete solution;
	if (inliers != NULL){
		for (int i = 0; i < (int)inliers->size(); i++)
			delete (*inliers)[i];
		delete inliers;
	}
}

KnowledgeBase::KnowledgeBase()
: maxNumModels(0), keys(), keygroups(), models(), objects(), myTree(new KDTree(keys)), paramHash() {
	setParameter("probOfMatch", 0.9);
	setParameter("errorThreshold", 0.05*MAXIMAGEDIM);
}

KnowledgeBase::~KnowledgeBase(){
	cleanUpMemory();
}

void KnowledgeBase::cleanUpMemory(){
	maxNumModels = 0;
	
	if (myTree){
		delete myTree;
		myTree = NULL;
	}
	
	for (int i = 0; i < (int)keys.size(); i++){
		delete keys[i];
	}
	keys.clear();
	
	for (int i = 0; i < (int)keygroups.size(); i++){
		delete keygroups[i];
	}
	keygroups.clear();
	
	for (int i = 0; i < (int)models.size(); i++){
		delete models[i];
	}
	models.clear();
	
	for (int i = 0; i < (int)objects.size(); i++){
		delete objects[i];
	}
	objects.clear();
	
	vector<double*> paramVals;
	vector<CharPtrKey*> paramNames;
	paramHash.retrieveAllData(&paramVals);
	paramHash.retrieveAllKeys(&paramNames);
	for (int i = 0; i < (int)paramNames.size(); i++){
		CharPtrKey* tempKey;
		paramHash.deleteData(paramNames[i], &tempKey);
		delete paramNames[i];
	}
	for (int i = 0; i < (int)paramVals.size(); i++){
		free(paramVals[i]);
	}
}

void KnowledgeBase::keypointMatching(vector<keypoint*>* newKeys, vector< vector< vector<keypoint*> > >& matches, vector< vector< vector<keypoint*> > >& imageKey, bool objectSpecified, int wantedObjectID){
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		// 		vector<keypoint*> bestKeypoint;
		int maxSearches = 20;
		int n = 10;
		myTree->getBestNKeypointMatch(*(*newKeys)[i], maxSearches, n);
		if ((*newKeys)[i]->bestMatch[0] == NULL){
			// 			cout << i << "!\n";
			continue;
		}
		double dist0 = ((*newKeys)[i]->bestMatch[0] == NULL) ? numeric_limits<double>::infinity() : sqrt((*newKeys)[i]->sqDist(*((*newKeys)[i]->bestMatch[0])));
		double dist1 = numeric_limits<double>::infinity(); //((*newKeys)[i]->bestMatch[1] == NULL) ? numeric_limits<double>::infinity() : sqrt((*newKeys)[i]->sqDist(*((*newKeys)[i]->bestMatch[1])));
		// Find next nearest feature from non-neighbor group
		for (int j = 1; j < n; j++){
			if ((*newKeys)[i]->bestMatch[j] == NULL) break;
			if (!((*newKeys)[i]->bestMatch[0]->G->isNeighbor((*newKeys)[i]->bestMatch[j]->G))){
				// 				cout << "Found at bestMatch[" << j << "]\n";
				dist1 = sqrt((*newKeys)[i]->bestDist[j]);
				break;
			}
		}
		// 	cout << i << endl;
		if (dist0 <= 0.85 * dist1){
			int objectID = (*newKeys)[i]->bestMatch[0]->G->M->O->getID();
			if (objectSpecified && objectID != wantedObjectID) continue;
			int modelID  = (*newKeys)[i]->bestMatch[0]->G->M->getID();
			// 			cout << (*newKeys)[i]->bestMatch[0] << " " << objectID << " " << modelID << endl;
			// 			cout << "(" << (*newKeys)[i]->x <<               "," << (*newKeys)[i]->y << ") - "
			// 			     << "(" << (*newKeys)[i]->bestMatch[0]->x << "," << (*newKeys)[i]->bestMatch[0]->y << ")"
			// 			     << "(" << (*newKeys)[i]->x - (*newKeys)[i]->bestMatch[0]->x << "," << (*newKeys)[i]->y - (*newKeys)[i]->bestMatch[0]->y << ")\n";
			matches[objectID][modelID].push_back((*newKeys)[i]->bestMatch[0]);
			imageKey[objectID][modelID].push_back((*newKeys)[i]);
			// Also add best points in neighbor groups
			for (int j = 0; j < (int)((*newKeys)[i]->bestMatch[0]->G->neighbors.size()); j++){
				double e;
				keypoint* bestMatchInGroup = (*newKeys)[i]->bestMatch[0]->G->neighbors[j]->bestMatchInGroup((*newKeys)[i], &e);
				// Make assumption that a group contains at least one keypoint
				// Make assumption that groups' neighbors must be from same object
				modelID = bestMatchInGroup->G->M->getID();
				matches[objectID][modelID].push_back(bestMatchInGroup);
				imageKey[objectID][modelID].push_back((*newKeys)[i]);
			}
		}
	}
	
	// 	cout << "Finished matching keypoints\n";
}

void KnowledgeBase::modelMatching(size_t numNewKeys, vector< vector< vector<keypoint*> > >& matches, vector< vector< vector<keypoint*> > >& imageKey, vector<modelMatchingInfo*>& mminfo/*, vector<model*>& acceptableModels, vector<double>& modelConfidence, vector<double>& modelError, vector<int>& solutionIndex, vector< vector< vector<NEWMAT::Matrix*> > >& solutions, vector< vector< vector< vector<keypointPair*>* > > >& inliers*/){
	// 		cout << "checkpoint1 " << objects.size() << " " << models.size() << endl;
	
	double probOfMatch = getParameter("probOfMatch");
	double threshold = getParameter("errorThreshold");
	
	for (int i = 0; i < (int)objects.size(); i++){
		int objectID = objects[i]->getID();
		// 		cout << objectID << endl;
		for (int j = 0; j < (int)objects[i]->models.size(); j++){
			int modelID = objects[i]->models[j]->getID();
			// 			cout << "\t" << modelID << endl;
			if (imageKey[objectID][modelID].size() > 4){
				double error;
				// 				cout << "(" << objectID << "," << modelID << ") [" << imageKey[objectID][modelID].size() <<  "]\t";
				
				vector<vector<keypointPair*>*> in;
				vector<double> errors;
				vector<NEWMAT::Matrix> transforms;
				getBestHoughTransforms(&transforms, imageKey[objectID][modelID], matches[objectID][modelID], &errors, &in);
				// 				cout << "#transforms=" << transforms.size() << "\n";
				if (transforms.size() == 0){
					continue;
				}
				/*
				 int bestTransformIndex = -1;
				 int maxNumInliers = 0;
				 for (int k = 0; k < (int)transforms.size(); k++){
				 // 					cout << (int)(in[k])->size() << endl;
				 if (maxNumInliers < (int)(in[k])->size()){
				 bestTransformIndex = k;
				 maxNumInliers = (int)(in[k])->size();
				 }
				 }
				 cout << maxNumInliers << endl;
				 solutions[objectID][modelID] = new NEWMAT::Matrix(transforms[bestTransformIndex]);
				 inliers[objectID][modelID] = in[bestTransformIndex];
				 error = errors[bestTransformIndex];
				 for (int k = 0; k < bestTransformIndex; k++){
				 for (int kk = 0; kk < (int)(in[k]->size()); kk++)
				 delete (*(in[k]))[kk];
				 delete in[k];
				 }
				 for (int k = bestTransformIndex+1; k < (int)transforms.size(); k++){
				 for (int kk = 0; kk < (int)(in[k]->size()); kk++)
				 delete (*(in[k]))[kk];
				 delete in[k];
				 }
				 int numInliers = (inliers[objectID][modelID])->size();
				 // 				if (numInliers < 4){
				 //						cout << endl;
				 //						continue;
				 // 				}
				 // 				double scale = sqrt((*(solutions[objectID][modelID]))(1,1) * (*(solutions[objectID][modelID]))(1,1) + (*(solutions[objectID][modelID]))(2,1) * (*(solutions[objectID][modelID]))(2,1));
				 // 				double theta = acos((*(solutions[objectID][modelID]))(1,1) / scale);
				 */
				
				for (int t = 0; t < (int)transforms.size(); t++){
					// 					solutions[objectID][modelID].push_back(new NEWMAT::Matrix(transforms[t]));
					// 					inliers[objectID][modelID].push_back(in[t]);
					
					// 					double scale = sqrt((*(solutions[objectID][modelID][t]))(1,1) * (*(solutions[objectID][modelID][t]))(1,1) + (*(solutions[objectID][modelID][t]))(2,1) * (*(solutions[objectID][modelID][t]))(2,1));
					// 					double theta = acos((*(solutions[objectID][modelID][t]))(1,1) / scale);
					error = errors[t];
					
					size_t numInliers = in[t]->size();
					
					size_t numKeypointsInModel = 0;
					for (int jj = 0; jj < (int)objects[i]->models[j]->keygroups.size(); jj++){
						numKeypointsInModel += objects[i]->models[j]->keygroups[jj]->keypts.size();
					}
					
					double d = (double)numKeypointsInModel / (double)(keys.size());
					double l = 0.125 * 0.125;
					double p = d*l;
					double q = (1.0-p);
					
					double PM = 0.01;
					double nCjj = 1.0; // n C n = 1.0
					double Pf_notM = (numInliers < numNewKeys) ? nCjj * std::pow(p, (int)numNewKeys) : 0.0;
					cout.precision(6);
					for (int jj = (int)numNewKeys - 1; jj >= (int)numInliers; jj--){
						// 					nCjj = nCjj  * (double)(jj+1) / (double)((*newKeys).size() - jj);
						nCjj = choose((int)numNewKeys, jj);
						if (nCjj == numeric_limits<double>::infinity()) continue;	
						Pf_notM += nCjj * std::pow(p, jj) * std::pow(q, (int)numNewKeys - jj);//prob;
						// 					cout << Pf_notM << " " << nCjj << " " << pow(p, jj) << " " << pow(q, (*newKeys).size() - jj) << "\n";
					}
					
					double PM_f = PM / (PM + Pf_notM);
					
					// 				cout << PM_f << " " << scientific << PM << " " << Pf_notM << " " << p << fixed << "\t"
					// 						<< numInliers << "/" << numKeypointsInModel << "=" << (double)numInliers / (double)numKeypointsInModel
					// 						<< "\t" << error << " \t" << scale << " \t" << theta << " \t" << (*(solutions[objectID][modelID]))(3,1) << " \t" << (*(solutions[objectID][modelID]))(4,1) << endl;
					
					if (PM_f >= probOfMatch && numInliers >= 4 && error <= threshold){
						// 						acceptableModels.push_back(objects[i]->models[j]);
						// 						modelConfidence.push_back(PM_f);
						// 						modelError.push_back(error);
						// 						solutionIndex.push_back(solutions[objectID][modelID].size()-1);
						// 						cout << "[qq] " << numInliers << "/" << numKeypointsInModel << ", error: " << error << 
						// 								", scale: " << scale << ", theta: " << theta << 
						// 								endl;
						modelMatchingInfo *newmminfo = new modelMatchingInfo();
						newmminfo->matchedModel = objects[i]->models[j];
						newmminfo->solution = new NEWMAT::Matrix(transforms[t]);
						newmminfo->inliers = in[t];
						newmminfo->error = error;
						newmminfo->probOfMatch = PM_f;
						mminfo.push_back(newmminfo);
					}else{
						// Clean up in[t]
						for (int tt = 0; tt < (int)in[t]->size(); tt++){
							delete (*(in[t]))[tt];
						}
						delete in[t];
					}
				}
				// 				if (mminfo.size() == 0) cout << endl;
				// 			}else{
				// 				solutions[objectID][modelID] = NULL;
			}
		}
	}
	
	// 	cout << "checkpoint2\n";
	
	// Sort models by lowest error
	// Bubble sort
	// NOTE: This is really unncessary if we only want top 3 models! Simply do insertion sort for 3 elements
	bool isDone = false;
	for (int i = (int)mminfo.size()-1; !isDone && i > 0; i--){
		isDone = true;
		for (int j = 0; j < i; j++){
			if (mminfo[j]->error > mminfo[j+1]->error){
				// 				model* tempModel = acceptableModels[j];
				// 				acceptableModels[j] = acceptableModels[j+1];
				// 				acceptableModels[j+1] = tempModel;
				
				// 				double tempDouble = modelConfidence[j];
				// 				modelConfidence[j] = modelConfidence[j+1];
				// 				modelConfidence[j+1] = tempDouble;
				
				// 				tempDouble = modelError[j];
				// 				modelError[j] = modelError[j+1];
				// 				modelError[j+1] = tempDouble;
				
				// 				int tempInt = solutionIndex[j];
				// 				solutionIndex[j] = solutionIndex[j+1];
				// 				solutionIndex[j+1] = tempInt;
				
				modelMatchingInfo* tempmminfo = mminfo[j];
				mminfo[j] = mminfo[j+1];
				mminfo[j+1] = tempmminfo;
				
				isDone = false;
				
			}
		}
	}
	// 	cout << "Model matching complete\n";
}

void KnowledgeBase::rebuildKDTree(){
	if (myTree) delete myTree;
	cout << "Rebuilding with " << keys.size() << " keys..\n";
	myTree = new KDTree(keys);
	cout << "Rebuilding complete\n";
}
object* KnowledgeBase::unlearn_Object(vector<keypoint*>* newKeys, int oID){
	object *O = new object((int)objects.size());
	model  *M = new model(0);
	// 	M->filename.clear();
	// 	M->filename.append(filename);
	M->O = O;
	O->models.push_back(M);
	if (maxNumModels < O->models.size()) maxNumModels = O->models.size();
	//models.push_back(M);
	//objects.push_back(O);
	for(int a = 0; objects.size(); a++) {
		if(objects.at(a)->id == oID) objects.erase(objects.begin()+a);
	}
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		
		keygroup *G = new keygroup();
		G->M = M;
		M->keygroups.push_back(G);
		keygroups.push_back(G);
		
		(*newKeys)[i]->G = G;
		(*newKeys)[i]->generation = M->generation;
		/*
		 int spot = 0;		
		 for(int j = 0; j > (int)(G->keypts.size()); j++) {
		 if(G->keypts.at(j) == (*newKeys)[i]) spot = j;
		 }
		 G->keypts.erase(G->keypts.begin()+spot-1, G->keypts.begin()+spot);
		 */		
		/*for(int j = 0; j < (int)(keygroups.size()); j++) {
		 //if(keygroups.at(j) == G) spot = j;
		 //found = matchingGroup->keypts.at(j) == (*newKeys)[i];
		 if((*keygroups.at(j)).compareTo(G) == 0) {
		 keygroups.erase(keygroups.begin()+j);
		 //cout << "Keygroups erased... \n";
		 }		
		 }*/
		for(int j = 0; j > (int)(keys.size()); j++) {
			if(keys.at(j) == (*newKeys)[i]) keys.erase(keys.begin()+j);
		}		
		
		// 		((*newKeys)[i])->id = getNewKeypointID();
	}
	
	M->generation--;
	rebuildKDTree();
	return O;
}
object* KnowledgeBase::learn_newObject(vector<keypoint*>* newKeys){
	object *O = new object((int)objects.size());
	objects.push_back(O);
	model  *M = new model(0);
	// 	M->filename.clear();
	// 	M->filename.append(filename);
	M->O = O;
	O->models.push_back(M);
	if (maxNumModels < O->models.size()) maxNumModels = O->models.size();
	models.push_back(M);
	
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		
		keygroup *G = new keygroup();
		G->M = M;
		M->keygroups.push_back(G);
		keygroups.push_back(G);
		
		(*newKeys)[i]->G = G;
		(*newKeys)[i]->generation = M->generation;
		G->keypts.push_back((*newKeys)[i]);
		keys.push_back((*newKeys)[i]);
		// 		((*newKeys)[i])->id = getNewKeypointID();
	}
	
	M->generation++;
	
	rebuildKDTree();
	
	return O;
}
model* KnowledgeBase::unlearn_Model( vector<keypoint*>* newKeys, object* O, vector<modelMatchingInfo*> mminfo, int mID){
	model* M = new model((int)O->models.size());
	M->O = O;
	// 	M->filename.clear();
	// 	M->filename.append(filename);
	//O->models.push_back(M);
	for(int j = 0; j < (int)(models.size()); j++) {
		cout << "checking..." << models.at(j)->name << " " << mID;
		if(models.at(j)->id == mID) {
			models.erase(models.begin()+j);
			cout << "Found and removed from models.";
		}
	}
	for(int j = 0; j < (int)(O->models.size()); j++) {
		cout << "checking..." << O->models.at(j)->name << " " << mID;
		if(O->models.at(j)->id == mID) {
			O->models.erase(O->models.begin()+j);
			cout << "Found and removed from O's models.";
		}
	}	
	if (maxNumModels < O->models.size()) maxNumModels = O->models.size();
	//models.push_back(M);
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		// Create keygroup for each keypoint
		keygroup *G = new keygroup();
		//keygroups.push_back(G);
		
		G->M = M;
		M->keygroups.push_back(G);
		
		G->keypts.push_back((*newKeys)[i]);
		(*newKeys)[i]->G = G;
		(*newKeys)[i]->generation = M->generation;
		// int spot = 0;
		// bool found = false;
		
		// Link to best matches
		int groupCount = 0;
		for (int j = 0; j < (int)(*newKeys)[i]->modelMatches.size() && groupCount < 3; j++){
			keygroup *grp = (*newKeys)[i]->modelMatches[j]->G;
			model *modelMatch = grp->M;
			if ((mminfo.size() > 0 && modelMatch == mminfo[0]->matchedModel) ||
				(mminfo.size() > 1 && modelMatch == mminfo[1]->matchedModel) ||
				(mminfo.size() > 2 && modelMatch == mminfo[2]->matchedModel))
			{
				grp->neighbors.push_back(G);
				G->neighbors.push_back(grp);
				groupCount++;
			}
		}
		for(int j = 0; j < (int)(keygroups.size()); j++) {
			//if(keygroups.at(j) == G) spot = j;
			//found = matchingGroup->keypts.at(j) == (*newKeys)[i];
			if((*keygroups.at(j)).compareTo(G) == 0) {
				keygroups.erase(keygroups.begin()+j);
				//cout << "Keygroups erased... \n";
			}		
		}
		//keys.push_back((*newKeys)[i]);
		// 		((*newKeys)[i])->id = getNewKeypointID();
	}
	
	M->generation++;
	
	rebuildKDTree();
	
	return M;
}
model* KnowledgeBase::learn_newModel( vector<keypoint*>* newKeys, object* O, vector<modelMatchingInfo*> mminfo){
	model* M = new model((int)O->models.size());
	M->O = O;
	// 	M->filename.clear();
	// 	M->filename.append(filename);
	O->models.push_back(M);
	if (maxNumModels < O->models.size()) maxNumModels = O->models.size();
	models.push_back(M);
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		// Create keygroup for each keypoint
		keygroup *G = new keygroup();
		keygroups.push_back(G);
		
		G->M = M;
		M->keygroups.push_back(G);
		
		G->keypts.push_back((*newKeys)[i]);
		(*newKeys)[i]->G = G;
		(*newKeys)[i]->generation = M->generation;
		
		// Link to best matches
		int groupCount = 0;
		for (int j = 0; j < (int)(*newKeys)[i]->modelMatches.size() && groupCount < 3; j++){
			keygroup *grp = (*newKeys)[i]->modelMatches[j]->G;
			model *modelMatch = grp->M;
			if ((mminfo.size() > 0 && modelMatch == mminfo[0]->matchedModel) ||
				(mminfo.size() > 1 && modelMatch == mminfo[1]->matchedModel) ||
				(mminfo.size() > 2 && modelMatch == mminfo[2]->matchedModel))
			{
				grp->neighbors.push_back(G);
				G->neighbors.push_back(grp);
				groupCount++;
			}
		}
		
		keys.push_back((*newKeys)[i]);
		// 		((*newKeys)[i])->id = getNewKeypointID();
	}
	
	M->generation++;
	
	rebuildKDTree();
	
	return M;
}
void KnowledgeBase::unlearn_fromModel(vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform, double& s, double& theta, double& tx, double& ty){
	cout << "Locating data in database... \n";
	s = sqrt((*transform)(1,1) * (*transform)(1,1) + (*transform)(2,1) * (*transform)(2,1));
	double temp = (*transform)(1,1) / s;
	if (temp > 1.0) temp = 1.0;
	if (temp < -1.0) temp = -1.0;
	theta = acos(temp);
	double m  = (*transform)(1,1);
	double n  = (*transform)(2,1);
	tx = (*transform)(3,1);
	ty = (*transform)(4,1);
	
	if (isnan(theta)) theta = 0.0;
	if ((*transform)(2,1) / s < 0.0) theta = -theta;
	
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		int spot = 0;
		for(int j = 0; j > (int)(keys.size()); j++) {
			if(keys.at(j) == (*newKeys)[i]) spot = j;
		}		
		keys.erase(keys.begin()+spot-1, keys.begin()+spot);		
		
		double u = (*newKeys)[i]->imageX;
		double v = (*newKeys)[i]->imageY;
		double newy = (m*(v-ty)-n*(u-tx)) / (m*m + n*n);
		double newx = (m*(u-tx)+n*(v-ty)) / (m*m + n*n);
		double newscale = (*newKeys)[i]->imageScale / s;
		double neworientation = (*newKeys)[i]->imageOrientation - theta;
		while (neworientation > M_PI)   neworientation -= (2 * M_PI);
		while (neworientation <= -M_PI) neworientation += (2 * M_PI);
		
		// 				cout << "(" << u << "," << v << "," << (*newKeys)[i]->orientation << "," << (*newKeys)[i]->scale << ") --> ("
		// 						<< "(" << newx << "," << newy << "," << neworientation << "," << newscale << ")\n";
		
		(*newKeys)[i]->modelX           = newx;
		(*newKeys)[i]->modelY           = newy;
		(*newKeys)[i]->modelScale       = newscale;
		(*newKeys)[i]->modelOrientation = neworientation;
		(*newKeys)[i]->generation       = bestModel->generation;
		
		bool hasMatchingGroup = false;
		keygroup* matchingGroup = NULL;
		keypoint* modelMatch = NULL;
		modelMatch = (keypoint*)matchingGroup; // hack to suppress compiler warning due to commented out code below
		
		if ((*newKeys)[i]->bestMatch[0] != NULL){
			for (int j = 0; j < (int)((*newKeys)[i]->modelMatches.size()); j++){
				if ((*newKeys)[i]->modelMatches[j]->G->M == bestModel){
					hasMatchingGroup = true;
					modelMatch = (*newKeys)[i]->modelMatches[j];
					(*newKeys)[i]->modelMatches[j] = NULL;
					matchingGroup = modelMatch->G;
					break;
				}
			}
		}
		 if (hasMatchingGroup){
		/*					
		 // For now, add indiscriminately
		 // According to Lowe's paper, should only link if keypoint adds information
		 cout << "Found matching group! \n";
		 spot = 0;
		 bool found = false;
		 for(int j = 0; j < (int)(matchingGroup->keypts.size()); j++) {
		 if(matchingGroup->keypts.at(j) == (*newKeys)[i]) spot = j;
		 found = matchingGroup->keypts.at(j) == (*newKeys)[i];
		 cout << "CompareTo says:" << matchingGroup->compareTo(newKeys) << "\n";
		 }
		 cout << "Found matching keypts, at " << found << " " << spot << " of " << (int) matchingGroup->keypts.size() << " \n";		
		 matchingGroup->keypts.erase(keys.begin()+spot);
		 cout << "Erased all the stuff? \n";			
		 //(*newKeys)[i]->G = matchingGroup;
		 }else{
		 // Create new cluster
		 cout << "Not Found? \n";
		 keygroup *G = new keygroup();
		 keygroups.push_back(G);
		 
		 G->M = bestModel;
		 spot = 0;
		 for(int j = 0; j > (int)(bestModel->keygroups.size()); j++) {
		 if(bestModel->keygroups.at(j) == G) spot = j;
		 }		
		 bestModel->keygroups.erase((bestModel->keygroups).begin()+spot-1, (bestModel->keygroups).begin()+spot);
		 
		 G->keypts.push_back((*newKeys)[i]);
		 (*newKeys)[i]->G = G;
		 
		 */	
		 }
	}
	cout << "Done \n";
	bestModel->generation--;
	
	rebuildKDTree();
	
	
}
void KnowledgeBase::learn_toModel(vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform, double& s, double& theta, double& tx, double& ty){
	s = sqrt((*transform)(1,1) * (*transform)(1,1) + (*transform)(2,1) * (*transform)(2,1));
	double temp = (*transform)(1,1) / s;
	if (temp > 1.0) temp = 1.0;
	if (temp < -1.0) temp = -1.0;
	theta = acos(temp);
	double m  = (*transform)(1,1);
	double n  = (*transform)(2,1);
	tx = (*transform)(3,1);
	ty = (*transform)(4,1);
	
	if (isnan(theta)) theta = 0.0;
	if ((*transform)(2,1) / s < 0.0) theta = -theta;
	
	for (int i = 0; i < (int)(*newKeys).size(); i++){
		keys.push_back((*newKeys)[i]);
		
		double u = (*newKeys)[i]->imageX;
		double v = (*newKeys)[i]->imageY;
		double newy = (m*(v-ty)-n*(u-tx)) / (m*m + n*n);
		double newx = (m*(u-tx)+n*(v-ty)) / (m*m + n*n);
		double newscale = (*newKeys)[i]->imageScale / s;
		double neworientation = (*newKeys)[i]->imageOrientation - theta;
		while (neworientation > M_PI)   neworientation -= (2 * M_PI);
		while (neworientation <= -M_PI) neworientation += (2 * M_PI);
		
		// 				cout << "(" << u << "," << v << "," << (*newKeys)[i]->orientation << "," << (*newKeys)[i]->scale << ") --> ("
		// 						<< "(" << newx << "," << newy << "," << neworientation << "," << newscale << ")\n";
		
		(*newKeys)[i]->modelX           = newx;
		(*newKeys)[i]->modelY           = newy;
		(*newKeys)[i]->modelScale       = newscale;
		(*newKeys)[i]->modelOrientation = neworientation;
		(*newKeys)[i]->generation       = bestModel->generation;
		
		bool hasMatchingGroup = false;
		keygroup* matchingGroup = NULL;
		keypoint* modelMatch = NULL;
		
		if ((*newKeys)[i]->bestMatch[0] != NULL){
			for (int j = 0; j < (int)((*newKeys)[i]->modelMatches.size()); j++){
				if ((*newKeys)[i]->modelMatches[j]->G->M == bestModel){
					hasMatchingGroup = true;
					modelMatch = (*newKeys)[i]->modelMatches[j];
					matchingGroup = modelMatch->G;
					break;
				}
			}
		}
		
		if (hasMatchingGroup){
			// For now, add indiscriminately
			// According to Lowe's paper, should only link if keypoint adds information
			matchingGroup->keypts.push_back((*newKeys)[i]);
			(*newKeys)[i]->G = matchingGroup;
		}else{
			// Create new cluster
			keygroup *G = new keygroup();
			keygroups.push_back(G);
			
			G->M = bestModel;
			bestModel->keygroups.push_back(G);
			
			G->keypts.push_back((*newKeys)[i]);
			(*newKeys)[i]->G = G;
			
		}
	}
	
	bestModel->generation++;
	
	rebuildKDTree();
	
}

void KnowledgeBase::learn_toModel(vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform){
	double s, theta, tx, ty;
	learn_toModel(newKeys, bestModel, transform, s, theta, tx, ty);
}
void KnowledgeBase::unlearn_fromModel(vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform){
	double s, theta, tx, ty;
	unlearn_fromModel(newKeys, bestModel, transform, s, theta, tx, ty);
}


void KnowledgeBase::learn(vector<keypoint*>& K, matchInfo& mInfo, bool toIntegrate){
	
	/// Read keypoints
	vector<keypoint*>* newKeys;
	
	newKeys = &K;
	// Assign IDs
	// 	for (int i = 0; i < (int)K.size(); i++){
	// 		(*newKeys)[i]->id = getNewKeypointID();
	// 	}
	
	/// Match keypoints
    size_t objectsSize = objects.size();
    size_t maxNumModels1 = maxNumModels+1;
    // 	vector<keypoint*> matches[objectsSize][maxNumModels1];
    vector< vector< vector<keypoint*> > > matches;
    matches.resize(objectsSize);
    for (size_t i = 0; i < objectsSize; i++){
		matches[i].resize(maxNumModels1);
    }
    // 	vector<keypoint*> imageKey[objectsSize][maxNumModels1];
    vector< vector< vector<keypoint*> > > imageKey;
    imageKey.resize(objectsSize);
    for (size_t i = 0; i < objectsSize; i++){
		imageKey[i].resize(maxNumModels1);
    }
    keypointMatching(newKeys, matches, imageKey, false, -1);
	
    /// Match model
	vector<modelMatchingInfo*> mminfo;
	// 	vector<model*> acceptableModels;
	// 	vector<double> modelConfidence;
	// 	vector<double> modelError;
	// 	vector<int> solutionIndex;
	// 	NEWMAT::Matrix* solutions[objects.size()][maxNumModels+1];
	// 	vector<keypointPair*>* inliers[objects.size()][maxNumModels+1];
	// 	for (int i = 0; i < (int)objects.size(); i++){
	// 		for (int j = 0; j < maxNumModels+1; j++){
	// 			solutions[i][j] = NULL;
	// 			inliers[i][j] = NULL;
	// 		}
	// 	}
	// 	vector< vector< vector<NEWMAT::Matrix*> > > solutions;
	// 	vector< vector< vector< vector<keypointPair*>* > > > inliers;
	// 	solutions.resize(objectsSize);
	// 	inliers.resize(objectsSize);
	// 	for (int i = 0; i < (int)objects.size(); i++){
	// 		solutions[i].resize(maxNumModels1);
	// 		inliers[i].resize(maxNumModels1);
	// 	}
	
	modelMatching((*newKeys).size(), matches, imageKey, mminfo/*, acceptableModels, modelConfidence, modelError, solutionIndex, solutions, inliers*/);
	
	// 	int oldNumOfObjects = (int)objects.size();
	// 	int oldMaxNumModels = maxNumModels;
	
	/// Integrate
	model* bestModel;
	double bestError;
	if (mminfo.size() > 0){
		bestModel = mminfo[0]->matchedModel;
		bestError = mminfo[0]->error;
	}else{
		bestModel = NULL;
		bestError = numeric_limits<double>::infinity();
	}
	
	double threshold = getParameter("errorThreshold"); // 0.001 * MAXIMAGEDIM;
	if (bestModel == NULL){
		// No matching objects!
		// Create new object
		// 		cout << "No matches; creating new object\n";
		// 		cout << "[qq] 0/" << endl;
		if (toIntegrate){
			object *O = learn_newObject(newKeys);
			model* M = O->models[0];
			
			mInfo.M = M;
			mInfo.matchedM = NULL;
			mInfo.O = O;
			mInfo.matchedO = NULL;
			mInfo.s = mInfo.theta = mInfo.tx = mInfo.ty = 0;
		}else{
			mInfo.M = mInfo.matchedM = NULL;
			mInfo.O = mInfo.matchedO = NULL;
			mInfo.s = mInfo.theta = mInfo.tx = mInfo.ty = 0;
		}
		for (int i = 0; i < (int)(*newKeys).size(); i++){
			// Set values for mInfo
			mInfo.initialVals.push_back((*newKeys)[i]->getID());
			mInfo.initialVals.push_back((*newKeys)[i]->imageX);
			mInfo.initialVals.push_back((*newKeys)[i]->imageY);
			mInfo.initialVals.push_back((*newKeys)[i]->imageScale);
			mInfo.initialVals.push_back((*newKeys)[i]->imageOrientation);
			mInfo.newVals.push_back((*newKeys)[i]->getID());
			mInfo.newVals.push_back((*newKeys)[i]->modelX);
			mInfo.newVals.push_back((*newKeys)[i]->modelY);
			mInfo.newVals.push_back((*newKeys)[i]->modelScale);
			mInfo.newVals.push_back((*newKeys)[i]->modelOrientation);
			mInfo.hasMatch.push_back(false);
			mInfo.matchedVals.push_back(-1);
			mInfo.matchedVals.push_back(-1);
			mInfo.matchedVals.push_back(-1);
			mInfo.matchedVals.push_back(-1);
			mInfo.matchedVals.push_back(-1);
		}
	}else{
		
		// Copy relevant information to mInfo
		mInfo.O = mInfo.matchedO = bestModel->O;
		mInfo.matchedM = bestModel;
		
		if (bestError > threshold){
			// Over threshold; create new model
			// 			cout << "Matched to object " << bestModel->O->id << ", but not to model" << endl;
			if (toIntegrate){
				mInfo.M = learn_newModel(newKeys, bestModel->O, mminfo);
			}else{
				mInfo.M = NULL;
			}
			mInfo.s = mInfo.theta = mInfo.tx = mInfo.ty = 0;
			for (int i = 0; i < (int)(*newKeys).size(); i++){
				// Set values for mInfo
				mInfo.initialVals.push_back((*newKeys)[i]->getID());
				mInfo.initialVals.push_back((*newKeys)[i]->imageX);
				mInfo.initialVals.push_back((*newKeys)[i]->imageY);
				mInfo.initialVals.push_back((*newKeys)[i]->imageScale);
				mInfo.initialVals.push_back((*newKeys)[i]->imageOrientation);
				mInfo.newVals.push_back((*newKeys)[i]->getID());
				mInfo.newVals.push_back((*newKeys)[i]->modelX);
				mInfo.newVals.push_back((*newKeys)[i]->modelY);
				mInfo.newVals.push_back((*newKeys)[i]->modelScale);
				mInfo.newVals.push_back((*newKeys)[i]->modelOrientation);
				mInfo.hasMatch.push_back(false);
				mInfo.matchedVals.push_back(-1);
				mInfo.matchedVals.push_back(-1);
				mInfo.matchedVals.push_back(-1);
				mInfo.matchedVals.push_back(-1);
				mInfo.matchedVals.push_back(-1);
			}
		}else{
			// Add to old model view
			
			if (!toIntegrate){
				// 			cout << "Matched to object " << bestModel->O->id << ", model " << bestModel->id << " (" << bestModel->filename << ") (error=" << bestError << "<=" << threshold << "=threshold)" << endl;
				mInfo.M = bestModel;
				
				// 		for (int i = 0; i < (int)(*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]])).size(); i++){
				// 			(*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]]))[i]->getKey1()->finalMatch = (*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]]))[i]->getKey2();
				// 		}
				for (int i = 0; i < (int)mminfo[0]->inliers->size(); i++){
					(*(mminfo[0]->inliers))[i]->getKey1()->finalMatch = (*(mminfo[0]->inliers))[i]->getKey2();
				}
				
				// 				model *oldModel = bestModel;
				NEWMAT::Matrix* transform = mminfo[0]->solution;
				double s = sqrt((*transform)(1,1) * (*transform)(1,1) + (*transform)(2,1) * (*transform)(2,1));
				double temp = (*transform)(1,1) / s;
				if (temp > 1.0) temp = 1.0;
				if (temp < -1.0) temp = -1.0;
				double theta = acos(temp);
				double m  = (*transform)(1,1);
				double n  = (*transform)(2,1);
				double tx = (*transform)(3,1);
				double ty = (*transform)(4,1);
				
				if (isnan(theta)) theta = 0.0;
				if ((*transform)(2,1) / s < 0.0) theta = -theta;
				
				mInfo.s = s;
				mInfo.theta = theta;
				mInfo.tx = tx;
				mInfo.ty = ty;
				
				for (int i = 0; i < (int)(*newKeys).size(); i++){
					// 					((*newKeys)[i])->id = getNewKeypointID();
					
					// Set values for mInfo
					mInfo.initialVals.push_back((*newKeys)[i]->getID());
					mInfo.initialVals.push_back((*newKeys)[i]->imageX);
					mInfo.initialVals.push_back((*newKeys)[i]->imageY);
					mInfo.initialVals.push_back((*newKeys)[i]->imageScale);
					mInfo.initialVals.push_back((*newKeys)[i]->imageOrientation);
					
					double u = (*newKeys)[i]->imageX;
					double v = (*newKeys)[i]->imageY;
					double newy = (m*(v-ty)-n*(u-tx)) / (m*m + n*n);
					double newx = (m*(u-tx)+n*(v-ty)) / (m*m + n*n);
					double newscale = (*newKeys)[i]->imageScale / s;
					double neworientation = (*newKeys)[i]->imageOrientation - theta;
					while (neworientation > M_PI)   neworientation -= (2 * M_PI);
					while (neworientation <= -M_PI) neworientation += (2 * M_PI);
					
					// 				cout << "(" << u << "," << v << "," << (*newKeys)[i]->orientation << "," << (*newKeys)[i]->scale << ") --> ("
					// 						<< "(" << newx << "," << newy << "," << neworientation << "," << newscale << ")\n";
					
					(*newKeys)[i]->modelX           = newx;
					(*newKeys)[i]->modelY           = newy;
					(*newKeys)[i]->modelScale       = newscale;
					(*newKeys)[i]->modelOrientation = neworientation;
					
					mInfo.newVals.push_back((*newKeys)[i]->getID());
					mInfo.newVals.push_back((*newKeys)[i]->modelX);
					mInfo.newVals.push_back((*newKeys)[i]->modelY);
					mInfo.newVals.push_back((*newKeys)[i]->modelScale);
					mInfo.newVals.push_back((*newKeys)[i]->modelOrientation);
					
					keypoint* k = (*newKeys)[i]->finalMatch;
					if (k != NULL){
						mInfo.hasMatch.push_back(true);
						mInfo.matchedVals.push_back(k->getID());
						mInfo.matchedVals.push_back(k->modelX);
						mInfo.matchedVals.push_back(k->modelY);
						mInfo.matchedVals.push_back(k->modelScale);
						mInfo.matchedVals.push_back(k->modelOrientation);
					}else{
						mInfo.hasMatch.push_back(false);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
					}
				}
			}else{
				// 				cout << "Matched to object " << bestModel->O->id << ", model " << bestModel->id << " (" << bestModel->filename << ") (error=" << bestError << "<=" << threshold << "=threshold)" << endl;
				mInfo.M = bestModel;
				
				// 				for (int i = 0; i < (int)(*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]])).size(); i++){
				// 					(*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]]))[i]->getKey1()->finalMatch = (*(inliers[bestModel->O->id][bestModel->id][solutionIndex[0]]))[i]->getKey2();
				// 				}
				for (int i = 0; i < (int)mminfo[0]->inliers->size(); i++){
					(*(mminfo[0]->inliers))[i]->getKey1()->finalMatch = (*(mminfo[0]->inliers))[i]->getKey2();
				}
				
				for (int i = 0; i < (int)(*newKeys).size(); i++){
					// Set values for mInfo
					mInfo.initialVals.push_back((*newKeys)[i]->getID());
					mInfo.initialVals.push_back((*newKeys)[i]->imageX);
					mInfo.initialVals.push_back((*newKeys)[i]->imageY);
					mInfo.initialVals.push_back((*newKeys)[i]->imageScale);
					mInfo.initialVals.push_back((*newKeys)[i]->imageOrientation);
				}
				
				// For now, add all keypoints
				// 				model *oldModel = bestModel;
				NEWMAT::Matrix* transform = mminfo[0]->solution;
				
				double s, theta, tx, ty;
				
				learn_toModel(newKeys, bestModel,transform, s, theta, tx, ty);
				
				
				mInfo.s = s;
				mInfo.theta = theta;
				mInfo.tx = tx;
				mInfo.ty = ty;
				
				for (int i = 0; i < (int)(*newKeys).size(); i++){
					mInfo.newVals.push_back((*newKeys)[i]->getID());
					mInfo.newVals.push_back((*newKeys)[i]->modelX);
					mInfo.newVals.push_back((*newKeys)[i]->modelY);
					mInfo.newVals.push_back((*newKeys)[i]->modelScale);
					mInfo.newVals.push_back((*newKeys)[i]->modelOrientation);
					
					keypoint* k = (*newKeys)[i]->finalMatch;
					if (k != NULL){
						mInfo.hasMatch.push_back(true);
						mInfo.matchedVals.push_back(k->getID());
						mInfo.matchedVals.push_back(k->modelX);
						mInfo.matchedVals.push_back(k->modelY);
						mInfo.matchedVals.push_back(k->modelScale);
						mInfo.matchedVals.push_back(k->modelOrientation);
					}else{
						mInfo.hasMatch.push_back(false);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
						mInfo.matchedVals.push_back(-1);
					}
				}
				
			}
		}
	}
	
	if (toIntegrate){
		/// Rebuild KDTree
		// KDTree rebuild should be done in the functions learn_*
		rebuildKDTree();
	}else{
		for (int i = 0; i < (int)((*newKeys).size()); i++){
			delete (*newKeys)[i];
		}
	}
	
	/// Clean up
	// 	for (int i = 0; i < oldNumOfObjects; i++){
	// 		for (int j = 0; j < oldMaxNumModels+1; j++){
	// 			for (int t = 0; t < (int)solutions[i][j].size(); t++){
	// // 			if (solutions[i][j]) 
	// 				delete solutions[i][j][t];
	// // 			if (inliers[i][j]){
	// 				for (int k = 0; k < (int)(inliers[i][j][t]->size()); k++){
	// 					delete (*(inliers[i][j][t]))[k];
	// 				}
	// 				delete inliers[i][j][t];
	// // 			}
	// 			}
	// 		}
	// 	}
	for (int i = 0; i < (int)mminfo.size(); i++){
	    delete mminfo[i];
	}
}

object* KnowledgeBase::objectExists(int objectID){
	// Check if the object exists in kb
	for (unsigned int i = 0; i < objects.size(); i++){
		if (objects[i]->getID() == objectID){
			return objects[i];
		}
	}
	return NULL;
}

void KnowledgeBase::setParameter(const char* paramName, double paramVal){
	CharPtrKey* key = new CharPtrKey(paramName);
	double* paramPtr = paramHash.retrieve(key);
	if (paramPtr == NULL){
		paramPtr = (double*)malloc(sizeof(double));
		*paramPtr = paramVal;
		paramHash.insert(paramPtr, key);
	}else{
		*paramPtr = paramVal;
		delete key;
	}
}

double KnowledgeBase::getParameter(const char* paramName){
	CharPtrKey* key = new CharPtrKey(paramName);
	double* paramPtr = paramHash.retrieve(key);
	delete key;
	if (paramPtr == NULL){
		return 0.0;
	}else{
		return *paramPtr;
	}
}

void KnowledgeBase::saveToFile(ofstream& outfile){
	// Write out keypointID
	outfile << keypoint::getKeypointID() << endl;
	
	// Write out params
	outfile << numParams << endl;
	for (int i = 0; i < numParams; i++){
		outfile << paramList[i] << endl;
		outfile << getParameter(paramList[i]) << endl;
	}
	
	// Write out objects
	outfile << objects.size() << endl;
	for (unsigned int i = 0; i < objects.size(); i++){
		objects[i]->writeToFile(outfile);
	}
	
	// Write out models
	outfile << models.size() << endl;
	for (unsigned int i = 0; i < models.size(); i++){
		models[i]->writeToFile(outfile);
	}
	
	// Write out keygroups
	outfile << keygroups.size() << endl;
	for (unsigned int i = 0; i < keygroups.size(); i++){
		keygroups[i]->writeToFile(outfile);
	}
	
	// Write out keypoints
	outfile << keys.size() << endl;
	for (unsigned int i = 0; i < keys.size(); i++){
		keys[i]->writeToFile(outfile);
	}
	
	outfile << maxNumModels << endl;
}

void KnowledgeBase::readFromFile(ifstream& infile){
	// Start from clean state
	cleanUpMemory();
	
	// Set keypointID
	int keypointID;
	infile >> keypointID;
	keypoint::setKeypointID(keypointID);
	
	// Read and store params
	int nParams;
	infile >> nParams;
	// 	cout << nParams << endl;
	for (int i = 0; i < nParams; i++){
		char paramName[2048];
		double paramVal;
		infile.getline(paramName, 2048);
		infile.getline(paramName, 2048);
		infile >> paramVal;
		setParameter(paramName, paramVal);
		// 		cout << paramName << ": " << paramVal << endl;
	}
	
	// Read objects
	unsigned int numObjects;
	infile >> numObjects;
	// 	cout << numObjects << endl;
	vector< vector<int> > objectModelID;
	for (unsigned int i = 0; i < numObjects; i++){
		int objectID;
		unsigned int numModels;
		char objectName[2048];
		infile >> objectID;
		cout << objectID << endl;
		infile.getline(objectName, 2048);
		infile.getline(objectName, 2048);
		string objectNameStr(objectName);
		cout << objectName << endl;
		infile >> numModels;
		// 		cout << numModels << endl;
		vector<int> modelIDs;
		for (unsigned int j = 0; j < numModels; j++){
			int id;
			infile >> id;
			modelIDs.push_back(id);
			// 			cout << id << "\t";
		}
		// 		cout << endl;
		objectModelID.push_back(modelIDs);
		//Create object
		object* O = new object(objectID);
		O->setName(objectNameStr);
		objects.push_back(O);
	}
	
	// Read models
	unsigned int numModels;
	infile >> numModels;
	// 	cout << "#models: " << numModels << endl;
	vector< vector<int> > modelKeygroupID;
	for (unsigned int i = 0; i < numModels; i++){
		int objectID, modelID, generation;
		unsigned int numKeygroups;
		char modelName[2048];
		infile >> modelID >> objectID >> generation;
		// 		cout << "ID: " << objectID << "->" << modelID << endl;
		infile.getline(modelName, 2048);
		infile.getline(modelName, 2048);
		string modelNameStr(modelName);
		// 		cout << "Name: " << modelNameStr << endl;
		infile >> numKeygroups;
		// 		cout << "#keygroups: " << numKeygroups << endl;
		vector<int> keygroupIDs;
		for (unsigned int j = 0; j < numKeygroups; j++){
			int id;
			infile >> id;
			keygroupIDs.push_back(id);
			// 			cout << id << "\t";
		}
		// 		cout << endl;
		modelKeygroupID.push_back(keygroupIDs);
		// Create model
		model* M = new model(modelID);
		M->generation = generation;
		M->setName(modelNameStr);
		// 		bool found = false;
		for (unsigned int j = 0; j < objects.size(); j++){
			if (objects[j]->getID() == objectID){
				M->O = objects[j];
				objects[j]->models.push_back(M);
				models.push_back(M);
				// 				found = true;
				break;
			}
		}
		// 		if (!found) cout << "!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n";
	}
	
	// Read keygroups
	unsigned int numKeygroups;
	infile >> numKeygroups;
	// 	cout << "#keygroups: " << numKeygroups << endl;
	vector< vector<int> > keygroupKeyID;
	vector< vector<int> > keygroupNeighborObjectID, keygroupNeighborModelID, keygroupNeighborID;
	for (unsigned int i = 0; i < numKeygroups; i++){
		int objectID, modelID, keygroupID;
		unsigned int numKeys, numNeighbors;
		infile >> keygroupID >> objectID >> modelID;
		// 		cout << "ID: " << objectID << "->" << modelID << "->" << keygroupID << endl;
		infile >> numNeighbors;
		// 		cout << "#neighbors: " << numNeighbors << endl;
		vector<int> neighborObjectIDs, neighborModelIDs, neighborIDs;
		for (unsigned int j = 0; j < numNeighbors; j++){
			int Oid, Mid, id;
			infile >> Oid >> Mid >> id;
			neighborObjectIDs.push_back(Oid);
			neighborModelIDs.push_back(Mid);
			neighborIDs.push_back(id);
			// 			cout << Oid << "->" << Mid << "->" << id << "\t";
		}
		// 		cout << endl;
		keygroupNeighborObjectID.push_back(neighborObjectIDs);
		keygroupNeighborModelID.push_back(neighborModelIDs);
		keygroupNeighborID.push_back(neighborIDs);
		infile >> numKeys;
		// 		cout << "#keys: " << numKeys << endl;
		vector<int> keyIDs;
		for (unsigned int j = 0; j < numKeys; j++){
			int id;
			infile >> id;
			keyIDs.push_back(id);
			// 			cout << id << "\t";
		}
		// 		cout << endl;
		keygroupKeyID.push_back(keyIDs);
		// Create keygroup
		keygroup* G = new keygroup(keygroupID);
		// 		bool found = false;
		for (unsigned int j = 0; j < objects.size(); j++){
			if (objects[j]->getID() == objectID){
				for (unsigned int k = 0; k < objects[j]->models.size(); k++){
					if (objects[j]->models[k]->getID() == modelID){
						G->M = objects[j]->models[k];
						objects[j]->models[k]->keygroups.push_back(G);
						keygroups.push_back(G);
						// 						found = true;
						break;
					}
				}
				break;
			}
		}
		// 		if (!found) cout << "!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n";
	}
	// Connect the keygroups' neighbors
	for (unsigned int i = 0; i < keygroupNeighborID.size(); i++){
		for (unsigned int j = 0; j < keygroupNeighborID[i].size(); j++){
			// 			bool found = false;
			for (unsigned int oi = 0; oi < objects.size(); oi++){
				if (objects[oi]->getID() == keygroupNeighborObjectID[i][j]){
					for (unsigned int mi = 0; mi < objects[oi]->models.size(); mi++){
						if (objects[oi]->models[mi]->getID() == keygroupNeighborModelID[i][j]){
							for (unsigned gi = 0; gi < objects[oi]->models[mi]->keygroups.size(); gi++){
								if (objects[oi]->models[mi]->keygroups[gi]->getID() == keygroupNeighborID[i][j]){
									// 									found = true;
									keygroups[i]->neighbors.push_back(objects[oi]->models[mi]->keygroups[gi]);
									break;
								}
							}
							break;
						}
					}
					break;
				}
			}
			// 			if (!found) cout << "!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n!\n";
		}
	}
	
	// Read keypoints
	unsigned int numKeypoints;
	vector<int> finalMatches;
	infile >> numKeypoints;
	// 	vector< vector<int>
	// 	cout << "#keys: " << numKeypoints << endl;
	for (unsigned int i = 0; i < numKeypoints; i++){
		int keypointID1, objectID, modelID, keygroupID;
		// 		int finalMatchObjectID, finalMatchModelID, finalMatchKeygroupID, finalMatchID;
		infile >> keypointID1 >> objectID >> modelID >> keygroupID;
		// 		cout << objectID << "->" << modelID << "->" << keygroupID << "->" << keypointID1 << endl;
		// Create keypoint and link to keygroup
		keypoint* K = new keypoint(keypointID1);
		for (unsigned int oi = 0; oi < objects.size(); oi++){
			if (objects[oi]->getID() == objectID){
				for (unsigned int mi = 0; mi < objects[oi]->models.size(); mi++){
					if (objects[oi]->models[mi]->getID() == modelID){
						for (unsigned int gi = 0; gi < objects[oi]->models[mi]->keygroups.size(); gi++){
							if (objects[oi]->models[mi]->keygroups[gi]->getID() == keygroupID){
								K->G = objects[oi]->models[mi]->keygroups[gi];
								objects[oi]->models[mi]->keygroups[gi]->keypts.push_back(K);
								keys.push_back(K);
								break;
							}
						}
						break;
					}
				}
				break;
			}
		}
		infile
		// 				>> K->valid
		>> K->imageX
		>> K->imageY
		>> K->imageScale
		>> K->imageOrientation
		>> K->imageIntScale
		>> K->imageIntOctave
		>> K->modelX
		>> K->modelY
		>> K->modelScale
		>> K->modelOrientation
		>> K->generation
		;
		bool finalMatchValid;
		infile >> finalMatchValid;
		if (finalMatchValid){
			int finalMatchID;
			infile >> finalMatchID;
			finalMatches.push_back(finalMatchID);
		}else{
			K->finalMatch = NULL;
			finalMatches.push_back(-1);
		}
		// Discard matches as we reconstruct from finalMatches
		int numMatches;
		infile >> numMatches;
		for (int m = 0; m < numMatches; m++){
			int matchID;
			infile >> matchID;
		}
		// 		cout << "valid:" << K->valid << endl;
		unsigned int descSize;
		infile >> descSize;
		// 		cout << "desc size = " << descSize << endl;
		for (unsigned int di = 0; di < descSize; di++){
			double d;
			infile >> d;
			K->desc.push_back(d);
		}
		// 		unsigned int bestDistSize;
		// 		infile >> bestDistSize;
		// 		cout << "bestDist size = " << bestDistSize << endl;
		// 		for (unsigned int di = 0; di < bestDistSize; di++){
		// 			double d;
		// 			infile >> d;
		// 			K->bestDist.push_back(d);
		// 		}
		// 		bool finalMatchValid;
		// 		infile >> finalMatchValid;
		// 		if (finalMatchValid){
		// 			infile >> finalMatchObjectID >> finalMatchModelID >> finalMatchKeygroupID >> finalMatchID;
		// 		}
		// 		return;
	}
	
	for (unsigned int i = 0; i < keys.size(); i++){
		if (finalMatches[i] == -1) continue;
		bool foundFinalMatch = false;
		for (unsigned int j = 0; j < keys.size(); j++){
			if (keys[j]->getID() == finalMatches[i]){
				foundFinalMatch = true;
				keys[i]->finalMatch = keys[j];
				keys[j]->matches.push_back(keys[i]);
			}
		}
		if (!foundFinalMatch) cout << "Error!\n";
	}
	
	keypoint::setKeypointID(keypointID);
	
	infile >> maxNumModels;
	
	rebuildKDTree();
	
}

