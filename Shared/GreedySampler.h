#ifndef INCLUDED_GreedySampler_h_
#define INCLUDED_GreedySampler_h_

#include <vector>
#include <stdexcept>
#include <iosfwd>

//! Iteratively samples over a one dimensional numeric region by subdividing the largest unsampled region
/*! This could be implemented 'closed form' to compute the samples iteratively
 *  without explicitly storing the regions, but one of the goals of this class
 *  is to 'seed' the initial samples based on domain-specific heuristics, and
 *  then use the sampler to pick any additional samples to explore the remaining
 *  space efficiently. */
class GreedySampler {
public:
	//! Construct with the range to be sampled, and mark if the range is circular
	GreedySampler(float min, float max, bool circ) : full(min,max-min), reqMin(min), ranges(), circular(circ) {
		if(!circular)
			ranges.push_back(full);
	}
	
	//! Subdivides the largest region, returns the midpoint
	float sample();
	
	//! returns the span of the largest unsampled region
	float resolution();
	
	//! Informs the sampler of an externally taken sample
	/*! If using a circular region, @a x will be internally normalized for processing,
	 *  but it is still more efficient to pass a 'close' value */
	void remove(float x);
	
	//! remove sampled points, start fresh sampling
	void reset();
	
	//! remove sampled points, start fresh sampling
	void reset(float min, float max, bool circ);
	
protected:
	//! assumes a circular range, puts x in the range which is being sampled (#full) not the requested (#reqMin)
	float normalize(float x);
	
	//! output for debugging, displays regions in the heap
	friend std::ostream& operator<<(std::ostream& os, const GreedySampler& gs);
	
	struct range {
		range(float x, float r) : min(x), span(r) {}
		float min, span;
		bool operator<(const range& r) const { return span < r.span; }
	};
	range full; //!< the range being sampled (may differ from #reqMin for circular regions with initial samples removed)
	float reqMin; //!< requested output range (may differ from #full.min for circular regions with initial samples removed)
	std::vector<range> ranges; //!< stores the unsampled regions
	bool circular; //!< if true, affects initialization and first sample, also causes remove to normalize its argument
};

#endif
