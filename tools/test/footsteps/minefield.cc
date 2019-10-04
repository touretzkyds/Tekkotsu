#include "Planners/PlannerObstacles.h"
#include "Shared/plist.h"
#include "Shared/zignor.h"
#include "local/DeviceDrivers/MirageComm.h"
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <sys/time.h>

int usage(const char name[], const char* msg) {
	if(msg!=NULL)
		std::cerr << msg << "\n\n";
	std::cerr << name << " [-m|--mirage [host]] [-x|--overlap] [-z|--resize] [-r|--rect] [-s|--seed value] n radius x_min x_max y_min y_max " << std::endl;
	return (msg==NULL) ? EXIT_SUCCESS : 2;
}

int main(int argc, const char* argv[]) {
	size_t nobs;
	bool overlap=false;
	bool resize=false;
	bool rectangles=false;
	float radius, xmin,xmax, ymin,ymax;
	std::string mirage;
	
	struct timeval tp;
	gettimeofday(&tp,NULL);
	tp.tv_sec+=tp.tv_usec;
	
	for(int i=1; i<argc; ++i) {
		std::string arg = argv[i];
		if(arg=="-h" || arg=="--help") {
			return usage(argv[0], NULL);
			
		} else if(arg=="-m" || arg=="--mirage") {
			if(i+1 < argc && isalpha(argv[i+1][0])) {
				mirage = argv[++i];
			} else {
				mirage = "localhost";
			}
			
		} else if(arg=="-s" || arg=="--seed") {
			if(++i >= argc) return usage(argv[0], "Seed missing value");
			if(std::stringstream(argv[i])>>tp.tv_sec) {} else { return usage(argv[0],"Bad seed value"); }
			
		} else if(arg=="-x" || arg=="--overlap") {
			overlap=true;

		} else if(arg=="-z" || arg=="--resize") {
			resize=true;
			
		} else if(arg=="-r" || arg=="--rectangles") {
			rectangles=true;

		} else {
			std::stringstream ss(arg);
			if(ss>>nobs) {} else { return usage(argv[0],"Bad obstacle count"); }
			
			if(++i >= argc) return usage(argv[0], "Not enough arguments!");
			ss.str(argv[i]); ss.clear();
			if(ss>>radius) {} else { return usage(argv[0],"Bad radius value"); }
			
			if(++i >= argc) return usage(argv[0], "Not enough arguments!");
			ss.str(argv[i]); ss.clear();
			if(ss>>xmin) {} else { return usage(argv[0],"Bad minimum x value"); }
			
			if(++i >= argc) return usage(argv[0], "Not enough arguments!");
			ss.str(argv[i]); ss.clear();
			if(ss>>xmax) {} else { return usage(argv[0],"Bad maximum x value"); }
			
			if(++i >= argc) return usage(argv[0], "Not enough arguments!");
			ss.str(argv[i]); ss.clear();
			if(ss>>ymin) {} else { return usage(argv[0],"Bad minimum y value"); }
			
			if(++i >= argc) return usage(argv[0], "Not enough arguments!");
			ss.str(argv[i]); ss.clear();
			if(ss>>ymax) {} else { return usage(argv[0],"Bad maximum y value"); }
		}
	}
	
	RanNormalSetSeedZig32((int*)&tp.tv_sec,sizeof(tp.tv_sec));
	
	plist::ArrayOf<PlannerObstacle> obs;
	std::stringstream config;
	config << "-s " << tp.tv_sec << ' ' << nobs << ' ' << radius << ' ' << xmin << ' ' << xmax << ' ' << ymin << ' ' << ymax;
	obs.setComment(0,config.str());
	float xavg=(xmin+xmax)/2, xrng=xmax-xmin;
	float yavg=(ymin+ymax)/2, yrng=ymax-ymin;
	while(obs.size()<nobs) {
		float x = DRanNormalZig32()*xrng/4 + xavg;
		float y = DRanNormalZig32()*yrng/4 + yavg;
		//std::cout << x << ' ' << y << std::endl;
		CircularObstacle co(x,y,radius);
		RectangularObstacle ro(fmat::pack(x,y),fmat::pack(radius,radius),0);
		PlannerObstacle& o = rectangles ? (PlannerObstacle&)ro : (PlannerObstacle&)co;
		bool collision=false;
		if(!overlap) {
			for(plist::ArrayOf<PlannerObstacle>::const_iterator it=obs.begin(); it!=obs.end(); ++it) {
				if(o.collides(**it)) {
					if(resize) {
						if(rectangles) {
							for(int i=0; i<4; ++i) {
								fmat::Column<2> p = ro.getCorner(static_cast<RectangularObstacle::CornerOrder>(i));
								if((*it)->collides(p)) {
									fmat::Column<2> v = (*it)->gradient(p);
									//v-=v/v.norm()*0.1; // leave a little overlap
									fmat::Column<2> poff = v / 2;
									fmat::Column<2> coff = poff;
									switch(static_cast<RectangularObstacle::CornerOrder>(i)) {
										case RectangularObstacle::TOP_RIGHT: break;
										case RectangularObstacle::BOTTOM_RIGHT: coff[1] = -coff[1]; break;
										case RectangularObstacle::BOTTOM_LEFT: coff = -coff; break;
										case RectangularObstacle::TOP_LEFT: coff[0] = -coff[0]; break;
										case RectangularObstacle::NUM_CORNERS: abort();
									}
									ro.reset(ro.getCenter()+poff,ro.getExtents()+coff,0);
								}
							}
							if(!o.collides(**it)) {
								//const RectangularObstacle& oro = dynamic_cast<const RectangularObstacle&>(**it);
								continue;
							}
						} else {
							const CircularObstacle& oco = dynamic_cast<const CircularObstacle&>(**it);
							fmat::fmatReal dist = (co.center - oco.center).norm() - oco.radius;
							if(dist > radius/3) {
								co.radius=dist;
								continue;
							}
						}
					}
					collision=true;
					break;
				}
			}
		}
		if(!collision)
			obs.addEntry(o.clone());
	}
	obs.saveStream(std::cout);
	
	if(mirage.size()>0) {
		ionetstream netcomm;
		MirageComm mcomm(netcomm);
		mcomm.setPersist(true);
		mcomm.setName("Obstacles");
		KinematicJoint * kj = new KinematicJoint;
		mcomm.setKinematics(kj);
		const float OBSTACLE_HEIGHT=50;
		for(plist::ArrayOf<PlannerObstacle>::const_iterator it=obs.begin(); it!=obs.end(); ++it) {
			const PlannerObstacle* ob = *it;
			LinkComponent * lc = new LinkComponent;
			kj->components.addEntry(lc);
			if(const RectangularObstacle* r = dynamic_cast<const RectangularObstacle*>(ob)) {
				lc->collisionModel="Cube";
				r->getCenter().exportTo(lc->collisionModelOffset);
				lc->collisionModelScale[0] = r->getWidth();
				lc->collisionModelScale[1] = r->getHeight();
				lc->collisionModelScale[2] = OBSTACLE_HEIGHT;
				fmat::Quaternion::aboutZ(r->getOrientation()).exportTo(lc->collisionModelRotation[0],lc->collisionModelRotation[1],lc->collisionModelRotation[2]);
			} else if(const CircularObstacle* c = dynamic_cast<const CircularObstacle*>(ob)) {
				lc->collisionModel="Cylinder";
				c->center.exportTo(lc->collisionModelOffset);
				lc->collisionModelScale[0] = c->radius*2;
				lc->collisionModelScale[1] = c->radius*2;
				lc->collisionModelScale[2] = OBSTACLE_HEIGHT;
			} else {
				std::cerr << "WARNING unknown obstacle type " << ob->getTypeName() << std::endl;
				delete lc;
				lc = NULL;
			}
		}
	}
	
	return EXIT_SUCCESS;
}