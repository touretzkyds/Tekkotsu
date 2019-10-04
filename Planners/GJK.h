#include "Planners/PlannerObstacles.h"
#include "Shared/plistSpecialty.h"
#include <vector>

namespace GJK {
	
	//! Returns (@a a x @a b) x @a c = @a b * (@a a • @a c) - @a a * (@a b • @a c)
	/*! Commutative only if @a a = @a c: @a a x (@a b x @a a) == (@a a x @a b) x @a a
	 *  Otherwise, to get @a a x (@a b x @a c), call tripleProduct(@a b, @a c, -@a a) */
	template<size_t N>
	fmat::Column<N> tripleProduct(const fmat::Column<N>& a, const fmat::Column<N>& b, const fmat::Column<N>& c) {
		return b * fmat::dotProduct(a, c) - a * fmat::dotProduct(b, c);
	}
	
	template<size_t N>
	bool processLine(std::vector<fmat::Column<N> >& simplex, fmat::Column<N>& direction) {
		fmat::Column<N>& a = simplex[1], b = simplex[0];
		fmat::Column<N> ab = b - a;
		
		if (fmat::dotProduct(ab, -a) > 0)
			direction = tripleProduct(ab, -a, ab);
		else {
			simplex.erase(simplex.begin());
			direction = -a;
		}
		return false;
	}
	
	template<size_t N>
	bool processTriangle(std::vector<fmat::Column<N> >& simplex, fmat::Column<N>& direction) {
		fmat::Column<N>& a = simplex[2], b = simplex[1], c = simplex[0];
		
		fmat::Column<N> ab = b - a;
		fmat::Column<N> ac = c - a;
		
		// ababc = ab x (ab x ac) = (ab x ac) x -ab
		fmat::Column<N> ababc = tripleProduct(ab, ac, -ab);
		// abcac = (ab x ac) x ac
		fmat::Column<N> abcac = tripleProduct(ab, ac, ac);
		
		if (fmat::dotProduct(abcac, -a) > 0) {
			// ac simplex
			if (fmat::dotProduct(ac, -a) > 0) {
				// erase b
				simplex.erase(simplex.begin()+1);
				direction = tripleProduct(ac, -a, ac);
			}
			else {
				// erase c
				simplex.erase(simplex.begin());
				return processLine(simplex, direction);
			}
		}
		else {
			if (fmat::dotProduct(ababc, -a) > 0) {
				// erase c
				simplex.erase(simplex.begin());
				return processLine(simplex, direction);
			}
			/* for 2D case, we collided if we got to here.
			 but for 3D case, we test if the point is above or below the point */
			else {
				if (N==2)
					return true;
				else if (N==3) {
					fmat::Column<N> abc = fmat::Column<N>(fmat::crossProduct(ab, ac));
					if (fmat::dotProduct(abc, -a) > 0) {
						direction = abc;
					}
					else {
						// swap b and c
						std::swap(b, c);
						direction = -abc;
					}
				}
			}
		}
		return false;
	}
	
	// the 2D case will never get here anyway, but it has to be templated for consistency.
	// TODO: fix so that this is not templated, and is only defined for N=3
	template<size_t N>
	bool processTetrahedron(std::vector<fmat::Column<N> >& simplex, fmat::Column<N>& direction) {
		fmat::Column<N>& a = simplex[3], b = simplex[2], c = simplex[1], d = simplex[0];
		
		fmat::Column<N> ab = b - a;
		fmat::Column<N> ac = c - a;
		fmat::Column<N> ad = d - a;
		// casting needed because of templating
		fmat::Column<N> abc = fmat::Column<N>(fmat::crossProduct(ab, ac));
		fmat::Column<N> acd = fmat::Column<N>(fmat::crossProduct(ac, ad));
		fmat::Column<N> adb = fmat::Column<N>(fmat::crossProduct(ad, ab));
		
		if (fmat::dotProduct(abc, -a) > 0) {
			if (fmat::dotProduct(acd, -a) > 0) {
				if (fmat::dotProduct(adb, -a) > 0) {
					// erase b, c, d
					simplex.erase(simplex.begin(), simplex.begin()+3);
					direction = -simplex[0];
					return false;
				}
				else {
					// erase b, d
					simplex.erase(simplex.begin()+2);
					simplex.erase(simplex.begin());
					return processLine(simplex, direction);
				}
			}
			else {
				if (fmat::dotProduct(adb, -a) > 0) {
					// erase c, d
					simplex.erase(simplex.begin(), simplex.begin()+2);
					return processLine(simplex, direction);
				}
				else {
					// erase d
					simplex.erase(simplex.begin());
					return processTriangle(simplex, direction);
				}
			}
		}
		else {
			if (fmat::dotProduct(acd, -a) > 0) {
				if (fmat::dotProduct(adb, -a) > 0) {
					// erase b, c
					simplex.erase(simplex.begin()+1, simplex.begin()+3);
					return processLine(simplex, direction);
				}
				else {
					// erase b
					simplex.erase(simplex.begin()+2);
					return processTriangle(simplex, direction);
				}
			}
			else {
				if (fmat::dotProduct(adb, -a) > 0) {
					// erase c
					simplex.erase(simplex.begin()+1);
					return processTriangle(simplex, direction);
				}
				else
					return true;
			}
		}
	}
	
	template<size_t N>
	bool processSimplex(std::vector<fmat::Column<N> >& simplex, fmat::Column<N>& direction) {
		switch (simplex.size()) {
			case 2:
				return processLine(simplex, direction);
			case 3:
				return processTriangle(simplex, direction);
			case 4:
				// only for 3D case, so we cast just to avoid error
				return processTetrahedron(simplex, direction);
			default: // should never happen
				return false;
		}
	}
	
	template<size_t N>
	fmat::Column<N> getSupport(const PlannerObstacle<N>* obs1, const PlannerObstacle<N>* obs2, const fmat::Column<N>& direction) {
		return obs1->getSupport(direction) - obs2->getSupport(-direction);
	}
	
	template<size_t N>
	bool collides(const PlannerObstacle<N>* obs1, const PlannerObstacle<N>* obs2) {
		// choose initial direction
		fmat::Column<N> direction;
		direction[0] = 1;
		
		// initialize simplex
		std::vector<fmat::Column<N> > simplex;
		
		// for the 2 case, we needs 3 pts, for the 3 case we need 4 pts.
		simplex.reserve(N+1);
		
		// get first point
		simplex.push_back(getSupport(obs1, obs2, direction));
		
		// direction should point towards the origin
		direction = -simplex[0];
		
		int maxIterations = 20;
		
		while (maxIterations--) {
			// add new point
			simplex.push_back(getSupport(obs1, obs2, direction));
			
			// see if the new point was on the correct side of the origin
			if (fmat::dotProduct(simplex.back(), direction) < 0)
				return false;
			
			// process the simplex
			if (processSimplex(simplex, direction))
				return true;
		}
		
		// too many iterations... algorithm should converge in very few. assuming collision
		// std::cout << "WARNING: GJK: too many iterations, assuming collision." << std::endl;
		return true;
	}
	
}
