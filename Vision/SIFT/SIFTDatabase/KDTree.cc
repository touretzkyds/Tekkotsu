#include "KDTree.h"
#include "keypoint.h"
#include "PQueue.h"
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;

void KDTree::computeMeans(std::vector< std::vector <double> >& vals){
	size_t i, j;
	size_t numDimensions = keypts[0]->desc.size();
	size_t numKeypoints = keypts.size();
	
	mean.clear();
	
	for (i = 0; i < numDimensions; i++){
		vector<double> valsElem;
		double sum = 0.0f;
		for(j = 0; j < numKeypoints; j++){
			valsElem.push_back(keypts[j]->desc[i]);
			sum += valsElem[j];
		}
		vals.push_back(valsElem);
		mean.push_back((double)sum / (double)numKeypoints);
	}
	
}

// This should only be called after computeMeans has been executed
int KDTree::computeVariances(){
	
	size_t i, j;
	size_t numDimensions = keypts[0]->desc.size();
	size_t numKeypoints = keypts.size();
	
	double maxVariance = -1.0f;
	size_t    maxVarAxis  = -1u;
	
	variance.clear();
	
	for (i = 0; i < numDimensions; i++){
		double sumSqDiff = 0.0f;
		for(j = 0; j < numKeypoints; j++){
			double diff = keypts[j]->desc[i] - mean[i];
			sumSqDiff += diff * diff;
		}
		double v = (double)sumSqDiff / (double)numKeypoints;
		if (v > maxVariance){
			maxVarAxis = i;
			maxVariance = v;
		}
		variance.push_back(v);
	}
	
	return (int)maxVarAxis;
}

KDTree::KDTree(const std::vector<keypoint*>& C)
  : isLeaf(), keypts(), axis(0), variance(), mean(), median(), leftChild(NULL), rightChild(NULL), height(0)
{
	// Make copy of C
	keypts.insert(keypts.end(), C.begin(), C.end());
	
/*
	cout << "# of clusters: " << clusters.size() << endl;
	for (int i = 0; i < (int)clusters.size(); i++){
		for (int j = 0; j < KEYPOINTDIM; j++){
			cout << clusters[i]->aveDesc[j] << "\t";
		}
		cout << endl << endl;
	}
*/
	
	if (keypts.size() <= 1){
		// we're done
		makeLeaf();
	} else {
		isLeaf = false;
		vector< vector<double> > vals;
		computeMeans(vals);
		axis = computeVariances();
		median = computePositionK(vals[axis], vals[axis].size() / 2);
		
		// Partition
		vector<keypoint*> left, right;
		size_t size = vals[axis].size();
		for (size_t i = 0; i < size; i++){
			if (vals[axis][i] < median){
				left.push_back(keypts[i]);
			}else{
				right.push_back(keypts[i]);
			}
		}
		
		if (left.size() == 0 || right.size() == 0){
			// We do NOT want this to happen,
			// as we end up with all the elements piling to the same side everytime,
			// so we create an infinitely deep branch
			// Look for next best variance to use
			vector<axisVarPair> pairs;
			for (int i = 0; i < KEYPOINTDIM; i++){
				axisVarPair pair;
				pair.axis = i;
				pair.variance = variance[i];
				pairs.push_back(pair);
			}
			sort(pairs.begin(), pairs.end());
			// Skip the first(best) variance, since it doesn't work
			for (int j = 1; j < KEYPOINTDIM; j++){
				axis = pairs[j].axis;
				median = computePositionK(vals[axis], vals[axis].size() / 2);
				left.clear();
				right.clear();
				for (size_t i = 0; i < size; i++){
					if (vals[axis][i] < median){
						left.push_back(keypts[i]);
					}else{
						right.push_back(keypts[i]);
					}
				}
				if (left.size() != 0 && right.size() != 0) break;
			}
		}
		
		if (left.size() == 0 || right.size() == 0){
			// Cannot partition on ANY axis's median
			makeLeaf();
/*
			cout << "Unable to partition " << clusters.size() << "!\n";
			for (int i = 0; i < (int)clusters.size(); i++){
				for (int j = 0; j < KEYPOINTDIM; j++){
					cout << clusters[i]->aveDesc[j] << "\t";
				}
				cout << endl << endl;
			}
			cout << endl;
*/
		}else{
			// Build subtrees
			leftChild = new KDTree(left);
			rightChild = new KDTree(right);
			
			height = 1 + ((leftChild->height > rightChild->height) ? leftChild->height : rightChild->height);
		}

	}
}

KDTree::~KDTree(){
	if (leftChild)  delete leftChild;
	if (rightChild) delete rightChild;
}

void KDTree::makeLeaf(){
	if (leftChild)  delete leftChild;
	if (rightChild) delete rightChild;
	leftChild  = NULL;
	rightChild = NULL;
	isLeaf     = true;
	height     = 0;
}

bool KDTree::isleaf(){
	return isLeaf;
}


const std::vector<keypoint*>& KDTree::getKeypoints(){
	return keypts;
}


int KDTree::getAxis(){
	return axis;
}


double KDTree::getVariance(int x){
	return variance[x];
}


double KDTree::getMean(int x){
	return mean[x];
}


double KDTree::getMedian(){
	return median;
}


KDTree* KDTree::getLeftChild(){
	return leftChild;
}


KDTree* KDTree::getRightChild(){
	return rightChild;
}


int KDTree::getHeight(){
	return height;
}

void KDTree::getBestNKeypointMatch(keypoint& key, int maxSearches, int n){
	PQueue<nodeSqDistPair> pq;
	int count = 0;
	bool countStart = false;
	
	// Want to find at least one leaf node
	// maxSearches += height;
	
	// Initialize pq with single node, no active axes
	nodeSqDistPair pThis;
	pThis.node = this;
	pThis.sqDist = 0.0;
	pq.push(pThis);
	
	// Initialize bestDist, bestCluster
//	vector<double> bestDist;
//	vector<void*>  key.bestMatch;
	key.bestMatch.clear();
	key.bestDist.clear();
	for (int i = 0; i < n; i++){
		key.bestDist.push_back(numeric_limits<double>::infinity());
		key.bestMatch.push_back(NULL);
	}
	
	while (!countStart || (count < maxSearches && !pq.isEmpty())){
	
		// Pop out
		nodeSqDistPair pair;
		pair = pq.pop();
		if (pair.sqDist > key.bestDist[n-1]) break;
		
		KDTree *iter = pair.node;
		while (!iter->isLeaf){
			
			KDTree *nextNode, *otherNode;
			
			if (key.desc[iter->axis] < iter->median){
				// left
				nextNode = iter->leftChild;
				otherNode = iter->rightChild;
			}else{
				// right
				nextNode = iter->rightChild;
				otherNode = iter->leftChild;
			}
			// iter maintains the same activeAxes
			nodeSqDistPair pairFar = nodeSqDistPair(otherNode, pair);
			// Recompute distance for pairFar
			double oldAxisDist = pairFar.activeDist[iter->axis];
			double newAxisDist = std::abs(key.desc[iter->axis] - iter->median);
			pairFar.sqDist -= (oldAxisDist * oldAxisDist);
			pairFar.sqDist += (newAxisDist * newAxisDist);
			pairFar.activeAxes[pair.node->axis] = true;
			pairFar.activeDist[pair.node->axis] = newAxisDist;
			pq.push(pairFar);
			
			iter = nextNode;
		}
		
		if (countStart) count++;
		countStart = true;
		//delete pair;
		
// Deal with leaf node now
		for (int k = 0; k < (int)iter->keypts.size(); k++){
			double sqDist = key.sqDist(*iter->keypts[k]);//0.0;
/*
			for (int j = 0; j < KEYPOINTDIM; j++){
				double diff = iter->keypts[k]->desc[j] - key.desc[j];
				sqDist += (diff * diff);
//					cout << "key.desc[j] = " << key.desc[j]
//					     << "\titer->keypts[k]->desc[j] = " << iter->keypts[k]->desc[j]
//					     << "\tsqDist = " << sqDist
//					     << "\tbestDist = " << bestDist << endl;
			}
*/
//				cout << count << "----------\n";
			for (int ii = 0; ii < n; ii++){
				if (sqDist < key.bestDist[ii]){
					key.bestDist.insert(key.bestDist.begin() + ii, sqDist);
					key.bestMatch.insert(key.bestMatch.begin() + ii, iter->keypts[k]);
//						cout << "Replacing at iteration " << count << endl;
					key.bestDist.pop_back();
					key.bestMatch.pop_back();
//						for (int jj = 0; jj < (int)bestDist.size(); jj++){
//							cout << bestDist[jj] << "\t";
//						}
//						cout << endl;
					break;
//				}else{
//					cout << sqDist << " " << (sqDist < bestDist[ii]) << endl;
				}
			}
		}
		
	}
//	cout << (!countStart) << (count < maxSearches) << (!pq.isEmpty()) << (!countStart || (count < maxSearches && !pq.isEmpty())) << "Done\n";
		
}

bool operator<(const axisVarPair& p1, const axisVarPair& p2){
	return p1.variance < p2.variance;
}



nodeSqDistPair::nodeSqDistPair() :
  node(NULL),sqDist(numeric_limits<double>::infinity()),
  activeAxes(KEYPOINTDIM, false), activeDist(KEYPOINTDIM, 0.0) {}

nodeSqDistPair::nodeSqDistPair(KDTree* N, nodeSqDistPair& pair)
  : node(N), sqDist(pair.sqDist), 
    activeAxes(pair.activeAxes), activeDist(pair.activeDist) {}

nodeSqDistPair::nodeSqDistPair(const nodeSqDistPair& other) :
  node(other.node), sqDist(other.sqDist), activeAxes(other.activeAxes), activeDist(other.activeDist) {}

nodeSqDistPair& nodeSqDistPair::operator=(const nodeSqDistPair& other) {
  node = other.node;
  sqDist = other.sqDist;
  activeAxes = other.activeAxes;
  activeDist = other.activeDist;
  return *this;
}

nodeSqDistPair::~nodeSqDistPair(){
//	if (activeAxes) delete activeAxes;
//	if (activeDist) delete activeDist;
}

bool nodeSqDistPair::operator<(const nodeSqDistPair& p){
	return this->sqDist > p.sqDist;
}




double computePositionK(std::vector<double> V, size_t posK){
//	cout << "V:      \t"; for (int ii = 0; ii < (int)V.size(); ii++) cout << V[ii] << ", "; cout << endl;
//	cout << "Looking for position " << posK << " in vector of length " << V.size() << "\n";
	size_t i;
	size_t size = V.size();
	double p;
	if (size <= posK){
		cout << "!size " << size << " >= posK " << posK << endl;
	}
	if (size <= 5){
		sort(V.begin(), V.end());
		return V[posK];
	}else{
		vector<double> medians;
		// Step 1: get medians of every set of 5
		size_t numOfSets = size /5;
//		cout << "\tnumOfSets = " << numOfSets << endl;
		for (i = 0; i < numOfSets; i++){
			vector<double> temp;
			temp.insert(temp.end(), V.begin() + (5*i), V.begin() + (5*(i+1)));
//			cout << "\ttemp.size() = " << temp.size() << endl;
			medians.push_back(computePositionK(temp, 2));
		}
		if ((size % 5) != 0){
			vector<double> temp2;
			temp2.insert(temp2.end(), V.begin() + (5*numOfSets), V.end());
//			cout << "\ttemp2.size() = " << temp2.size() << endl;
			medians.push_back(computePositionK(temp2, temp2.size() / 2));
		}
//		cout << "\t\t"; for (int ii = 0; ii < (int)medians.size(); ii++) cout << medians[ii] << ", "; cout << endl;
		// Step 2: find median of medians
		p = computePositionK(medians, medians.size() / 2);
//		cout << "\tpivot: " << p << endl;
		// Step 3: partition into LESS, EQUAL and GREATER
		vector<double> LESS;
		vector<double> EQUAL;
		vector<double> GREATER;
		for (i = 0; i < size; i++){
			if (V[i] < p){
				LESS.push_back(V[i]);
			}else if (V[i] == p){
				EQUAL.push_back(p);
			}else{
				GREATER.push_back(V[i]);
			}
		}
//		cout << "LESS:   \t"; for (int ii = 0; ii < (int)LESS.size(); ii++) cout << LESS[ii] << ", "; cout << endl;
//		cout << "EQUAL:  \t"; for (int ii = 0; ii < (int)EQUAL.size(); ii++) cout << EQUAL[ii] << ", "; cout << endl;
//		cout << "GREATER:\t"; for (int ii = 0; ii < (int)GREATER.size(); ii++) cout << GREATER[ii] << ", "; cout << endl;
		size_t sizeLESS = LESS.size();
		size_t sizeEQUALLESS = EQUAL.size() + sizeLESS;
		if (sizeLESS > posK){
			return computePositionK(LESS, posK);
		}else if (sizeEQUALLESS > posK){
			return p;
		}else{
			return computePositionK(GREATER, posK - sizeEQUALLESS);
		}
	}
	
	return 0;
}

void computePositionKExample(){
	vector<double> V;
	
	int numEntries = 200;
	
	for (int i = 0; i < numEntries; i++){
		V.push_back((double)(rand() % numEntries));
		cout << V[i] << "\t";
	}
	cout << endl;
	cout << endl;
	cout << endl;
	
	for (int i = 0; i < numEntries; i++){
		cout << computePositionK(V, i) << "\t";
	}
	cout << endl;
		
	
}

