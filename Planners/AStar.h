//-*-c++-*-
#ifndef INCLUDED_AStar_h_
#define INCLUDED_AStar_h_

#include <vector>
#include <deque>
#include <set>
#include <algorithm>
#include <functional>
#include <tr1/unordered_set>

//! Holds data structures for search context, results, and implementation of A★ path planning algorithm, see AStar::astar
namespace AStar {

	//! Stores user state along with search context
	template<class State>
	struct Node {
		//! constructor, pass parent node @a p, cost so far @a c, remaining cost heuristic @a r, and user state @a st
		Node(const Node* p, float c, float r, const State& st) : parent(p), cost(c), remain(r), total(c+r), state(st) {}
		
		const Node* parent; //!< source for this search node
		float cost; //!< cost to reach this node from start state
		float remain; //!< estimated cost remaining to goal
		float total; //!< cached value of #cost + #remain
		State state; //!< user state
		
		//! Search nodes should be sorted based on total cost (#total)
		/*! Note the inverted comparison, STL heap operations put max at the heap root, but we want the min */
		struct CostCmp : public std::binary_function<Node*, Node*, bool> {
			bool operator()(const Node* left, const Node* right) const {
				//return left->total > right->total; // or should cost so far be a tie breaker?
				return left->total > right->total || ( left->total==right->total && left->cost < right->cost );
			}
		};
	};

	//! For efficient lookup of existance of states in open or closed list, uses user's Cmp on the user state within the search node
	template<class State, class Cmp>
	struct StateCmp : public std::binary_function<Node<State>*, Node<State>*, bool> {
		bool operator()(const Node<State>* left, const Node<State>* right) const {
			return Cmp()(left->state,right->state);
		}
	};
	
	//! Calls State::hash(), which should return value distributed over size_t
	template<class State>
	struct StateHash : public std::unary_function<Node<State>*,size_t> {
		size_t operator()(const Node<State>* n) const {
			return n->state.hash();
		}
	};
	
	//! Tests equality of states using State::operator==
	template<class State>
	struct StateEq : public std::binary_function<Node<State>*, Node<State>*, bool> {
		bool operator()(const Node<State>* left, const Node<State>* right) const {
			return left->state == right->state;
		}
	};
	
	//! Search results, including unexpanded nodes
	/*! Returning full search state for reporting of search statistics and 
	 *  also future expansion to support efficient re-planning, as well
	 *  as to allow user to free resources in State (e.g. if State is a pointer type) */
	template<class State, class Cmp=std::less<State> >
	struct Results {
		typedef Node<State> Node;
		typedef std::tr1::unordered_set<Node*, StateHash<State>, StateEq<State> > NodeSet;
		
		typedef typename std::vector<State>::iterator path_iterator;
		typedef typename std::vector<State>::const_iterator path_const_iterator;
		typedef typename NodeSet::iterator set_iterator;
		typedef typename NodeSet::const_iterator set_const_iterator;
		typedef typename std::deque<Node*>::iterator priority_iterator;
		typedef typename std::deque<Node*>::const_iterator priority_const_iterator;
		
		float cost;
		std::vector<State> path;
		NodeSet closed;
		NodeSet open;
		std::deque<Node*> priorities;
		~Results() {
			// don't need to delete from priorities... everything is either open or closed
			/*for(typename std::deque<Node*>::const_iterator it=priorities.begin(); it!=priorities.end(); ++it)
				delete *it;*/
			priorities.clear();
			for(set_const_iterator it=open.begin(); it!=open.end(); ++it)
				delete *it;
			open.clear();
			for(set_const_iterator it=closed.begin(); it!=closed.end(); ++it)
				delete *it;
			closed.clear();
		}
	};
	
	//! A★ search using custom comparison on State type
	/*! Expand is expected to be compatible with const std::vector<std::pair<float,State> >& (Context::*expand)(const State& st, const State& goal) const \n
	 * Heuristic is expected to be compatible with float (Context::*heuristic)(const State& st, const State& goal) const \n
	 * The Cmp argument is not actually used, but the function accepts an instance so you can avoid specifying all of the template parameters */
	template<class Context, class State, class Expand, class Heuristic, class Validate, class Cmp>
	Results<State,Cmp>
	astar(const Context& ctxt, const State& initial, const State& goal, Expand expand, Heuristic heuristic, Validate validate, const Cmp&, float bound=0) {
		typedef Node<State> Node;
		typedef StateCmp<State,Cmp> StateCmp;
		Results<State,Cmp> results;
		typename Results<State,Cmp>::NodeSet& closed = results.closed;
		typename Results<State,Cmp>::NodeSet& open = results.open;
		std::deque<Node*>& priorities = results.priorities;
		typename Node::CostCmp costCmp;
		
		{
			Node* n = new Node(NULL,0,(ctxt.*heuristic)(initial,goal),initial);
			open.insert(n);
			priorities.push_back(n);
		}
		
		while(!open.empty()) {
			Node* cur = priorities.front();
			bool valid = (ctxt.*validate)(cur->state);
			
			if(valid && cur->state == goal) { // or test (cur->remain==0) ?
				reconstruct(cur,results.path);
				results.cost = cur->cost;
				return results;
			}
			
			std::pop_heap(priorities.begin(),priorities.end(),costCmp);
			priorities.pop_back();
			open.erase(cur);
			closed.insert(cur);
			if(!valid)
				continue;
			//std::cout << "Closing " << cur->state << " cost " << cur->cost << " total " << cur->total << std::endl;
			
			const State* parentState = (cur->parent!=NULL) ? &cur->parent->state : static_cast<State*>(NULL);
			const std::vector<std::pair<float,State> >& neighbors = (ctxt.*expand)(parentState,cur->state, goal);
			for(typename std::vector<std::pair<float,State> >::const_iterator it=neighbors.begin(); it!=neighbors.end(); ++it) {
				Node n(cur, cur->cost+it->first, (ctxt.*heuristic)(it->second,goal), it->second);
				if(closed.count(&n) > 0 || (bound>0 && n.total>bound))
					continue;
				typename Results<State,Cmp>::set_const_iterator op = open.find(&n);
				if(op == open.end()) {
					// new node, add it
					Node * hn = new Node(n); // clone to get heap storage
					open.insert(hn);
					//std::cout << "open insert " << hn->cost << ' ' << hn->remain << ' ' << hn->total << ' ' << open.size() << std::endl;
					priorities.push_back(hn);
					std::push_heap(priorities.begin(), priorities.end(),costCmp);
				} else if(n.cost < (*op)->cost) {
					// better path to previous node, update it
					**op = n;
					std::make_heap(priorities.begin(), priorities.end(),costCmp); // can we do something more efficient?
				} else {
					// we can already do better, drop new node
					// since we used stack allocation for n, this is a no-op
				}
			}
		}
		return results;
	}
	
	//! A★ search using operator< to sort user State type
	/*! Expand is expected to be compatible with const std::vector<std::pair<float,State> >& (Context::*expand)(const State& st, const State& goal) const \n
	 * Heuristic is expected to be compatible with float (Context::*heuristic)(const State& st, const State& goal) const */
	template<class Context, class State, class Expand, class Heuristic, class Validate>
	Results<State, std::less<State> >
	astar(const Context& ctxt, const State& initial, const State& goal, Expand expand, Heuristic heuristic, Validate validate, float bound=0) {
		return astar(ctxt,initial,goal,expand,heuristic,validate,std::less<State>(),bound);
	}
	
	//! A★ search using operator< to sort user State type, assumes Context has functions named "expand" and "heuristic"
	template<class Context, class State>
	Results<State, std::less<State> >
	astar(const Context& ctxt, const State& initial, const State& goal, float bound=0) {
		return astar(ctxt,initial,goal,&Context::expand,&Context::heuristic,&Context::validate,std::less<State>(),bound);
	}
	
	//! constructs @a path by following parent pointers from @a n; the specified node is included in the path
	template<class Node, class State>
	void reconstruct(const Node* n, std::vector<State>& path, size_t depth=1) {
		if(n->parent != NULL)
			reconstruct(n->parent, path, depth+1);
		else
			path.reserve(depth);
		path.push_back(n->state);
	}

}

#endif
