//-*-c++-*-
#ifndef _GENERIC_RRT_H_
#define _GENERIC_RRT_H_

#include <vector>
#include <ostream>
#include <cmath>

#include "Shared/Measures.h"  // AngTwoPi

using namespace DualCoding;

//================ RRTNodeBase ================

//! Base class for RRT nodes used by GenericRRT
/*! Subclasses of RRTNodeBase must provide a NodeValue_t type, a
  CollisionChecker class, and the following methods: distance,
  generateSample, interpolate, toString.
*/
class RRTNodeBase {
public:
  RRTNodeBase(unsigned int p) : parent(p) {}
  unsigned int parent;
  enum Interp_t { COLLISION, APPROACHED, REACHED };
  //! returns a value in the range [minVal, maxVal), i.e. inclusive minVal, exclusive maxVal
  static float randRange(float minVal, float maxVal) {
    return (maxVal - minVal) * (rand()/((float)RAND_MAX)) + minVal;
  }
  virtual ~RRTNodeBase() {}
  virtual std::string toString() const = 0;
};

//================ AdmissibilityPredicate ================

//! Base class for admissibility predicates to determine whether a node is admissible.
/*! Subclasses of AdmissibilityPredicateBase must provide the following method:
  admissible(NodeValue_t, std::vector<NODE>&, unsigned int);
*/

template<typename NODE>
class AdmissibilityPredicate {
public:
  //! constructor
  AdmissibilityPredicate() {}
  
  //! destructor
  virtual ~AdmissibilityPredicate() {}
  
  //! test to see if node is admissible
  virtual bool admissible(typename NODE::NodeValue_t q, std::vector<NODE>& tree, unsigned int parent) = 0;
};

//================ GenericRRT ================

class GenericRRTBase {
public:
  enum PlanPathResultCode {SUCCESS, START_COLLIDES, END_COLLIDES, MAX_ITER};
  
  //! Relays planner success/failure, as well as obstacles in the case of initial collision states
  /*! If a collision occurs, the colliding obstacles will also be included.
   Otherwise, they will be set to NULL. */
  template <size_t N>
  class PlannerResult {
  public:
    GenericRRTBase::PlanPathResultCode code;
    PlannerObstacle<N> *movingObstacle;
    PlannerObstacle<N> *collidingObstacle;
    
    PlannerResult() : code(), movingObstacle(NULL), collidingObstacle(NULL) {}
    
    ~PlannerResult() {
      delete movingObstacle;
      delete collidingObstacle;
    }

    //! Copy constructor
    PlannerResult(const PlannerResult &other) : 
      code(other.code), 
      movingObstacle(other.movingObstacle ? dynamic_cast<PlannerObstacle<N>*>(other.movingObstacle->clone()) : NULL), 
      collidingObstacle(other.collidingObstacle ? dynamic_cast<PlannerObstacle<N>*>(other.collidingObstacle->clone()) : NULL) {}

    //! Assignment operator
    PlannerResult& operator=(const PlannerResult &other) {
      delete movingObstacle;
      delete collidingObstacle;
      code = other.code;
      movingObstacle = other.movingObstacle ? dynamic_cast<PlannerObstacle<N>*>(other.movingObstacle->clone()) : NULL;
      collidingObstacle = other.collidingObstacle ? dynamic_cast<PlannerObstacle<N>*>(other.collidingObstacle->clone()) : NULL;
      return *this;
    }

  };
  
  virtual ~GenericRRTBase() {}

  typedef PlannerResult<2> PlannerResult2D;
  typedef PlannerResult<3> PlannerResult3D;
};

//! Generic RRT implementation that makes no assumptions about the nature of the state space; those details are handled by the NODE template argument, which must be a subclass of RRTNodeBase
template<typename NODE, size_t N>
class GenericRRT : public GenericRRTBase {
public:
  typedef typename NODE::NodeValue_t NodeValue_t;
protected:

  NodeValue_t lowerLimits;  //!< lower limits on state q

  NodeValue_t upperLimits;  //!< upper limits on state q

  NodeValue_t extendingInterpolationStep;  //!< interpolation steps for q while extending

  NodeValue_t smoothingInterpolationStep;  //!< interpolation steps for q while smoothing

  typename NODE::CollisionChecker *cc;  //!< collision checker

  AdmissibilityPredicate<NODE> *predicate;  //!< admissibility predicate

public:
  //! Constructor; will delete @a collCheck argument when destructed
  GenericRRT(typename NODE::CollisionChecker *collCheck, AdmissibilityPredicate<NODE> *predicate=NULL);

  //! Copy constructor
  GenericRRT(const GenericRRT &other) :
    GenericRRTBase(other), 
    lowerLimits(other.lowerLimits), upperLimits(other.upperLimits),
    extendingInterpolationStep(other.extendingInterpolationStep),
    smoothingInterpolationStep(other.smoothingInterpolationStep),
    cc(other.cc), predicate(other.predicate) {}


  //! Destructor: deletes the collision checker
  virtual ~GenericRRT() { delete cc; delete predicate; }

  //! Set lower and upper value limits for each dimension of the space
  void setLimits(const NodeValue_t &_lowerLimits, const NodeValue_t &_upperLimits) {
    lowerLimits = _lowerLimits;
    upperLimits = _upperLimits;
  }

  //! Set interpolation step size for each dimension of the space for both extending and smoothing
  virtual void setInterpolation(const NodeValue_t &_interpolationStep) {
    setSmoothingInterpolation(_interpolationStep);
    setExtendingInterpolation(_interpolationStep);
  }

  //! Set interpolation step size for each dimension of the space for extending
  virtual void setExtendingInterpolation(const NodeValue_t &_extendingInterpolationStep) {
    extendingInterpolationStep = _extendingInterpolationStep;
  }

  //! Set interpolation step size for each dimension of the space for smoothing
  virtual void setSmoothingInterpolation(const NodeValue_t &_smoothingInterpolationStep) {
    smoothingInterpolationStep = _smoothingInterpolationStep;
  }

  //! Plan a path from start to end
  virtual PlannerResult<N> planPath(const NodeValue_t &start,
                                    const NodeValue_t &end,
                                    unsigned int maxIterations,
                                    std::vector<NodeValue_t> *pathResult=NULL,
                                    std::vector<NODE> *treeStartResult=NULL,
                                    std::vector<NODE> *treeEndResult=NULL);

  typename NODE::CollisionChecker* getCC() { return cc; }

  virtual void dumpTree(const std::vector<NODE> &tree, const std::string &header="");

  void addObstaclesToShapeSpace(ShapeSpace &space, const fmat::Transform &t=fmat::Transform()) const;

protected:
  //! Set up initial search trees
  virtual void initialize(const NodeValue_t &start, std::vector<NODE> &treeStartResult,
			  const NodeValue_t &end, std::vector<NODE> &treeEndResult);
  virtual void buildPath(const std::vector<NODE> *treeStart, const std::vector<NODE> *treeEnd, 
			 std::vector<NodeValue_t> &path);

  void addNode(std::vector<NODE> *tree, const NodeValue_t &q, unsigned int parent);

private:
  unsigned int nearestNode(std::vector<NODE> *tree, const NodeValue_t &target);
  RRTNodeBase::Interp_t extend(std::vector<NODE> *tree, const NodeValue_t &q, bool truncate, bool searchingBackwards);

  GenericRRT& operator=(const GenericRRT&);   //!< Don't call this
};

//================ IMPLEMENTATION ================

template<typename NODE, size_t N>
GenericRRT<NODE, N>::GenericRRT(typename NODE::CollisionChecker *collCheck,
			     AdmissibilityPredicate<NODE> *_predicate) :
  lowerLimits(), upperLimits(), extendingInterpolationStep(), smoothingInterpolationStep(), cc(collCheck), predicate(_predicate) {}

template<typename NODE, size_t N>
void GenericRRT<NODE, N>::initialize(const NodeValue_t &start, std::vector<NODE> &treeStart,
																		 const NodeValue_t &end, std::vector<NODE> &treeEnd) {
  addNode(&treeStart, start, 0);
  addNode(&treeEnd, end, 0);
}

template<typename NODE, size_t N>
GenericRRTBase::PlannerResult<N>
GenericRRT<NODE, N>::planPath(const NodeValue_t &start, const NodeValue_t &end,
                              unsigned int maxIterations,
                              std::vector<NodeValue_t> *pathResult,
                              std::vector<NODE> *treeStartResult, std::vector<NODE> *treeEndResult) {
  PlannerResult<N> result;
  
  // setup state storage
  std::vector<NODE> privateTreeStart, privateTreeEnd;
  std::vector<NODE> *treeStart = treeStartResult ? treeStartResult : &privateTreeStart;
  std::vector<NODE> *treeEnd = treeEndResult ? treeEndResult : &privateTreeEnd;
  treeStart->clear();
  treeEnd->clear();

  // add initial configs in if we're searching in a circle around the designated endpoint
  initialize(start, *treeStart, end, *treeEnd);

  std::vector<NODE> *A = treeStart;
  std::vector<NODE> *B = treeEnd;
  
  // check start/end configurations
  if ( cc->collides(start, &result) ) {
    result.code = START_COLLIDES;
    return result;
  }
  
  // Don't check for end collision if we're using an array of endpoints,
  // because the end node is a dummy at the center of the actual endpoint set.
	// Check size <= 2 instead of size == 1 because we might need one extra node
	// if walking backward.
  if ( treeEnd->size() <= 2 && cc->collides(end, &result) ) {
    result.code = END_COLLIDES;
    return result;
  }

  unsigned int iter = 0;
  bool searchingBackwards = false;
  // first test for direct path from start to end if there is a unique end
  if ( treeEnd->size() == 1 && extend(A, end, false, searchingBackwards) == RRTNodeBase::REACHED )
    /* we're done */ ;
  else
    // there is no direct path, so build the tree
    while ( iter++ < maxIterations ) {
      NodeValue_t qrand;
      if ( iter == 1 && (*B)[0].parent < B->size() )  // tree must have at least one non-dummy node
				qrand = (*B)[nearestNode(B, start)].q;
      else
				NODE::generateSample(lowerLimits,upperLimits,qrand);
      if ( extend(A, qrand, true, searchingBackwards) != RRTNodeBase::COLLISION ) {
        if ( extend(B, A->back().q, true, !searchingBackwards) == RRTNodeBase::REACHED )
          break;
	swap(A,B);
	searchingBackwards = ! searchingBackwards;
      }
    }

  if ( iter >= maxIterations ) {
    // dumpTree(*treeStart,"treeStart: ================");
    // dumpTree(*treeEnd,"treeEnd: ================");
    result.code = MAX_ITER;
    return result;
  }

  if ( pathResult != NULL )
    buildPath(treeStart, treeEnd, *pathResult);

  result.code = SUCCESS;
  return result;
}

// nearestNode
template<typename NODE, size_t N>
unsigned int GenericRRT<NODE, N>::nearestNode(std::vector<NODE> *tree, const NodeValue_t &target) {
  // Root node's parent field contains the index of the first
  // matchable node for nearestNode.  This allows us to initialize the
  // end tree with special nodes that contribute to a path but can not
  // be matched directly.  Use in ShapeSpacePlannerXYTheta.
  unsigned int nearest = (*tree)[0].parent;  // index of first matchable node
  float dist = (*tree)[nearest].distance(target);
  for (unsigned int i = nearest+1; i < tree->size(); i++) {
    float d = (*tree)[i].distance(target);
    if (d < dist) {
      nearest = i;
      dist = d;
    }
  }
  return nearest;
}

// extend
template<typename NODE, size_t N>
RRTNodeBase::Interp_t GenericRRT<NODE, N>::extend(std::vector<NODE> *tree, const NodeValue_t &target,
						  bool truncate, bool searchingBackwards) {
  unsigned int nearest = nearestNode(tree, target);
  NodeValue_t reached;
  // interpolate towards target
  RRTNodeBase::Interp_t result = 
    NODE::interpolate((*tree)[nearest].q, target, extendingInterpolationStep, truncate, cc, reached, searchingBackwards);
  if ( result != RRTNodeBase::COLLISION ) {
    if ( predicate != NULL && predicate->admissible(reached, *tree, nearest) == false )
      result = RRTNodeBase::COLLISION;  // pretend inadmissible node causes a collision
    else
      addNode(tree, reached, nearest);
  }
  return result;
}

// addNode
template<typename NODE, size_t N>
void GenericRRT<NODE, N>::addNode(std::vector<NODE> *tree, const NodeValue_t &q, unsigned int parent) {
  tree->push_back(NODE(q, parent));
}

// buildPath
template<typename NODE, size_t N>
void GenericRRT<NODE, N>::buildPath(const std::vector<NODE> *treeStart, const std::vector<NODE> *treeEnd,
				  std::vector<NodeValue_t> &path) {

  // connection point is last value in each tree
  unsigned int n = treeStart->size() - 1;
  while (n != 0) {
    n = (*treeStart)[n].parent;
    path.push_back((*treeStart)[n].q);
  }

  std::reverse(path.begin(), path.end());
  path.push_back(treeEnd->back().q);

  n = treeEnd->size() - 1;
  while (n != 0) {
    n = (*treeEnd)[n].parent;
    path.push_back((*treeEnd)[n].q);
  }

  // smooth path
  size_t maxIter = max((size_t)20, 2*path.size());

  for (size_t i = 0; i < maxIter; i++) {
    int a = rand() % path.size();
    int b = rand() % path.size();
    if (a > b)
      std::swap(a,b);
    else if (a == b)
      continue;

    NodeValue_t dummy;
    if ( NODE::interpolate(path[a], path[b], smoothingInterpolationStep, false, cc, dummy, true) == RRTNodeBase::REACHED ) {
      path.erase(path.begin()+a+1,path.begin()+b);
    }
  }

  for (size_t i = 0; i+2 < path.size(); i++) {
    size_t j;
    for (j = i + 2; j < path.size(); j++) {
      NodeValue_t dummy;
      if ( NODE::interpolate(path[i], path[j], smoothingInterpolationStep, false, cc, dummy, true) != RRTNodeBase::REACHED )
	break;
    }
    if (j > i + 3 && j < path.size())
      path.erase(path.begin()+i+1, path.begin()+j-1);
  }
}

template<typename NODE, size_t N>
void GenericRRT<NODE, N>::dumpTree(const std::vector<NODE> &tree, const std::string &header) {
  if ( header.size() > 0 )
    std::cout << header << std::endl;
  for ( size_t i = 0; i < tree.size(); i++ )
    if ( tree[i].parent != 0 )
      std::cout << " " << tree[i].toString() << "   " << tree[tree[i].parent].toString() << std::endl;
}

template<typename NODE, size_t N>
void GenericRRT<NODE, N>::addObstaclesToShapeSpace(ShapeSpace &space, const fmat::Transform &t) const {
  cc->addObstaclesToShapeSpace(space,t);
}

#endif
