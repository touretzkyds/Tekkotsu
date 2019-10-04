#ifndef __KDTREE_H
#define __KDTREE_H

#include <vector>
class keypoint;

class KDTree{
private:
	bool                   isLeaf;
	std::vector<keypoint*> keypts;
	int                    axis;
	std::vector<double>    variance;
	std::vector<double>    mean;
	double                 median;
	KDTree*                leftChild;
	KDTree*                rightChild;
	int                    height;
	
	void makeLeaf();
	void computeMeans(std::vector< std::vector<double> >& vals);
	int computeVariances();
	
public:
	KDTree(const std::vector<keypoint*>& C);
	~KDTree();
	
	bool isleaf();
	const std::vector<keypoint*>& getKeypoints();
	int                      getAxis();
	double                   getVariance(int x);
	double                   getMean(int x);
	double                   getMedian();
	KDTree*                  getLeftChild();
	KDTree*                  getRightChild();
	int                      getHeight();
	
	void                     getBestNKeypointMatch(keypoint& key, int maxSearches, int n);
	
private:
	KDTree(const KDTree&); // Do not call
	KDTree& operator=(const KDTree&); // Do not call
};


class axisVarPair{
public:
	int axis;
	double variance;
};

bool operator<(const axisVarPair& p1, const axisVarPair& p2);

class nodeSqDistPair{
public:
	KDTree* node;
	double sqDist;
	std::vector<bool>  activeAxes;
	std::vector<double> activeDist;
	
	nodeSqDistPair();
	nodeSqDistPair(KDTree* N, nodeSqDistPair& pair);
	nodeSqDistPair(const nodeSqDistPair&);
	nodeSqDistPair& operator=(const nodeSqDistPair&);
	~nodeSqDistPair();
	
	bool operator<(const nodeSqDistPair& p);
	
private:
};

double computePositionK(std::vector<double> V, size_t posK);

#endif
