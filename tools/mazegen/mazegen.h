#ifndef INCLUDED_mazegen_h
#define INCLUDED_mazegen_h

#include <vector>
#include <string>
#include <cassert>

class Cell {
public:
	Cell() : right(NULL), bottom(NULL), root(NULL) {}
	Cell& findRoot() { return root ? *(root = &root->findRoot()) : *this; }
	void unionRight(Cell& x) {
		assert(right==NULL);
		//assert(&x.findRoot()!=&findRoot());
		if(&x.findRoot()==&findRoot())
			right=&x; // already unioned, just remove wall
		else
			findRoot().root = &(right=&x)->findRoot();
	}
	void unionBottom(Cell& x) {
		assert(bottom==NULL);
		//assert(&x.findRoot()!=&findRoot());
		if(&x.findRoot()==&findRoot())
			bottom=&x; // already unioned, just remove wall
		else
			findRoot().root = &(bottom=&x)->findRoot();
	}
	bool rightWall() { return right==NULL; }
	bool bottomWall() { return bottom==NULL; }
protected:
	Cell* right;
	Cell* bottom;
	Cell* root;
};

typedef std::vector<Cell> row_t;
typedef std::vector<row_t> map_t;

extern float cellDim;
extern float wallHeight;
extern float wallDepth;
extern std::string mirageHost;
extern size_t startX,startY;

#endif
