#include "GreedySampler.h"
#include <algorithm>
#include <iostream>
#include <cmath>

float GreedySampler::sample() {
	if(ranges.size()==0) { // implies circular
		float r = full.span/2;
		float x = full.min+r;
		full.min-=r;
		ranges.push_back(full);
		return x;
	} else {
		pop_heap(ranges.begin(), ranges.end());
		float r2 = ranges.back().span / 2;
		float x = ranges.back().min + r2;
		ranges.back().span = r2;
		push_heap(ranges.begin(), ranges.end());
		ranges.push_back(range(x,r2));
		push_heap(ranges.begin(), ranges.end());
		if(x < reqMin) // implies circular with an initial removal
			x+=full.span;
		return x;
	}
}

float GreedySampler::resolution() {
	if(ranges.size()==0) {
		return full.span;
	} else {
		return ranges.front().span;
	}
}

void GreedySampler::remove(float x) {
	if(ranges.size()==0) { // implies circular
		full.min = normalize(x) - full.span;
		ranges.push_back(full);
	} else {
		if(circular)
			x = normalize(x);
		if(x <= full.min || full.min+full.span <= x) {
			if(x==full.min || x==full.min+full.span)
				return; // technically not in the 'sampled' range, but we'll ignore it
			throw std::range_error("takeSample(x): x out of the sampled range");
		}
		std::vector<range>::iterator it=ranges.begin();
		for(; it!=ranges.end(); ++it) {
			if(it->min <= x && x < it->min+it->span) {
				break;
			}
		}
#ifdef DEBUG
		if(it==ranges.end()) // this shouldn't be possible (already did range check), but just in case...
			throw std::runtime_error("takeSample(x): could not find x in sampled range");
#endif
		if(it->min == x)
			return; // duplicate sample, ignore it
		float r = x - it->min;
		range n(x, it->span - r);
		it->span = r;
		ranges.push_back(n);
		make_heap(ranges.begin(),ranges.end());
	}
}

void GreedySampler::reset() {
	full.min=reqMin;
	ranges.clear();
	if(!circular)
		ranges.push_back(full);
}

void GreedySampler::reset(float min, float max, bool circ) {
	reqMin = full.min = min;
	full.span = max-min;
	circular=circ;
	ranges.clear();
	if(!circular)
		ranges.push_back(full);
}

float GreedySampler::normalize(float value) {
	const float max = full.min + full.span;
	if ( value >= max ) {
		if ( value <= max+full.span ) {
			value -= full.span;
		} else { // use fmod only if necessary
			value = std::fmod(value,full.span);
			if( value > max )
				value -= full.span;
		}
	} else if ( value < full.min ) {
		if ( value >= full.min-full.span ) {
			value += full.span;
		} else { // use fmod only if necessary
			value = std::fmod(value,full.span);
			if( value <= full.min )
				value += full.span;
		}
	}
	return value;
}

std::ostream& operator<<(std::ostream& os, const GreedySampler& gs) {
	for(std::vector<GreedySampler::range>::const_iterator it=gs.ranges.begin(); it!=gs.ranges.end(); ++it)
		os << "(" << it->min << "," << (it->min+it->span) << ") ";
	return os << "\n";
}

