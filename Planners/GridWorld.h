//-*-c++-*-
#ifndef INCLUDED_GridWorld_h_
#define INCLUDED_GridWorld_h_

#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

class GridWorld {
	static const float HCOST; //!< cost for horizontal moves
	static const float VCOST; //!< cost for vertical moves
	static const float DCOST; //!< cost for diagonal moves: <=0 means disable diagonals
	
	//! can read from istream (until end of stream), spaces are empty, non-spaces are blocked
	friend std::istream& operator>>(std::istream& is, GridWorld& gw);
	//! writes map to ostream
	friend std::ostream& operator<<(std::ostream& os, const GridWorld& gw);
	
	//! storage for map, spaces are empty, non-spaces are blocked
	std::vector< std::vector<char> > world;
	
public:
	//! Stores context used per node in path planning, stores row and column position
	struct State {
		//! default constructor, initializes to invalid position for fail-fast
		State() : r((size_t)-1), c((size_t)-1) {}
		//! constructor from a coordinate pair
		State(size_t row, size_t col) : r(row), c(col) {}
		//! a silly function to scatter row and column across the span of size_t
		size_t hash() const {
			size_t s1 = (c << (sizeof(size_t)*8/4*1));
			size_t s2 = (r << (sizeof(size_t)*8/4*2));
			size_t s3 = (c << (sizeof(size_t)*8/4*3));
			return s3+s2+s1+r;
		}
		//! comparison operator to allow sorting states for faster lookup in std::set
		bool operator<(const State& other) const { return r<other.r || (r==other.r && c<other.c); }
		//! equality is used to test against goal
		bool operator==(const State& other) const { return r==other.r && c==other.c; }
		//! just for debugging
		friend std::ostream& operator<<(std::ostream& os, const State& st) { return os << st.r << ',' << st.c; }

		size_t r; //!< row
		size_t c; //!< column
	};
	
	//! default constructor, does nothing
	GridWorld() : world() {}
	
	//! constructs empty world of the specified size
	GridWorld(size_t r, size_t c) : world(r,std::vector<char>(c,' ')) {}
	
	//! allows validation on queue-pop instead of before queue-push
	bool validate(const State& st) const { return true; }
	
	//! The heuristic function accepts the current state, and goal state, and should return an admissable (aka optimistic) estimate of the remaining cost
	float heuristic(const State& st, const State& goal) const;
	
	//! Generates a vector of successor states, paired with the cost to get to that state from @a st
	/*! Note that this implementation returns a reference to a static instance, which is not thread safe but slightly faster */
	const std::vector<std::pair<float,GridWorld::State> >& expand(const State* parent, const State& st, const State& goal) const;
	
	char& operator()(size_t r, size_t c) { return world[r][c]; } //!< map cell accessor
	char operator()(size_t r, size_t c) const { return world[r][c]; } //!< map cell accessor
	
	std::vector<char>& operator[](size_t r) { return world[r]; } //!< map row accessor
	const std::vector<char>& operator[](size_t r) const { return world[r]; } //!< map row accessor
	
	char& operator[](const State& x) { return world[x.r][x.c]; } //!< map cell accessor
	char operator[](const State& x) const { return world[x.r][x.c]; } //!< map cell accessor
};

#endif
