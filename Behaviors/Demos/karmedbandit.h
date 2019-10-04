//-*-c++-*-
#ifndef INCLUDED_karmedbandit_h_
#define INCLUDED_karmedbandit_h_

#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//!Makes decisions regarding an adversarial k-armed bandit
/*! Uses algorithms described in:
 *  The non-stochastic multi-armed bandit problem
 *  Auer, Cesa-Bianchi, Freund, and Schapire
 *  October 14, 2002
 */
class karmedbanditExp3 {
 public:
	//!constructor, pass the number of arms
	karmedbanditExp3(unsigned int k,double gammap)
		: w(k,1),lastp(0),last(-1U),g(gammap)
	{}

	//!returns the next choice, [0:k-1]
	unsigned int decide() {
		std::vector<double> p(w.size());
		double wsum=0;
		std::cout << "w =";
		for(unsigned int i=0; i<w.size(); i++)
			std::cout << ' ' << w[i];
		std::cout << std::endl;
		for(unsigned int i=0; i<w.size(); i++)
			wsum+=w[i];
		for(unsigned int i=0; i<w.size(); i++)
			p[i]=(1-g)*w[i]/wsum+g/w.size();
		std::cout << "p =";
		for(unsigned int i=0; i<w.size(); i++)
			std::cout << ' ' << p[i];
		std::cout << std::endl;
		double psum=0;
		for(unsigned int i=0; i<w.size(); i++)
			psum+=p[i];
		double pick=(rand()/(double)RAND_MAX)*psum;
		for(unsigned int i=0; i<w.size(); i++) {
			pick-=p[i];
			if(pick<=0) {
				lastp=p[i];
				return last=i;
			}
		}
		return -1U;
	}
	//!call this if you want to reward (r==true) or penalize (r==false) the previous decision
	void reward(bool r) {
		if(r) {
			w[last]*=exp(g/lastp/w.size());
			std::cout << "REWARD! :)" << std::endl;
		} else
			std::cout << "no reward. :(" << std::endl;
	}
	//!resets weights
	void reset() {
		for(unsigned int i=0; i<w.size(); i++)
			w[i]=1;
	}
	//!gets gamma parameter
	double getGamma() { return g; }
	//!sets gamma parameter
	void setGamma(double gammap) { g=gammap; }
	//!gets k parameter
	unsigned int getK() { return w.size(); }
 protected:
	std::vector<double> w; //!< the weights
	double lastp; //!< prob of last choice
	unsigned int last; //!< the last choice
	double g; //!< gamma
};

//!Makes decisions regarding an adversarial k-armed bandit
/*! Uses algorithms described in:
 *  The non-stochastic multi-armed bandit problem
 *  Auer, Cesa-Bianchi, Freund, and Schapire
 *  October 14, 2002
 */
class karmedbanditExp3_1 {
 public:
	//!constructor, pass the number of arms
	karmedbanditExp3_1(unsigned int k)
		: r(0), gr(0), last(0), G(k,0), exp3(k,0)
	{
		restart();
	}

	//!returns the next choice, [0:k-1]
	unsigned int decide() {
		double maxG=G[0];
		for(unsigned int i=1;i<G.size();i++)
			if(G[i]>maxG)
				maxG=G[i];
		if(maxG>gr-exp3.getK()/exp3.getGamma()) {
			restart();
			return last=decide();
		}
		return last=exp3.decide();
	}
	//!call this if you want to reward (r==true) or penalize (r==false) the previous decision
	void reward(bool rew) {
		if(rew)
			G[last]+=1;
		exp3.reward(rew);
	}
 protected:
	//!restarts exp3
	void restart() {
		std::cout << "Exp3 restart, g=" << std::flush;
		unsigned int k=exp3.getK();
		gr=(k*log((double)k))/(M_E-1)*pow(4.0,(double)r);
		double gammap=sqrt(k*log((double)k)/(M_E-1)/gr);
		//		exp3.reset(); //not sure if we're supposed to do this
		exp3.setGamma(gammap<1?gammap:1);
		std::cout << (gammap<1?gammap:1) << std::endl;
		r++;
	}
	unsigned int r; //!< the number of restarts
	double gr; //!< the gamma_r parameter
	unsigned int last; //!< the last choice
	std::vector<double> G; //!< the G-hat's
	karmedbanditExp3 exp3; //!< runs exp3 within this
};

/*! @file
 * @brief Defines karmedbandit - implements an algorithm which makes decisions regarding an adversarial k-armed bandit
 * @author ejt (Creator)
 */

#endif
