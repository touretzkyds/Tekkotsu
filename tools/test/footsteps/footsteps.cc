#include "dumpFigures.h"
#include "Motion/XWalkParameters.h"
#include "local/DeviceDrivers/MirageComm.h"
#include "Wireless/netstream.h"
#include "Shared/TimeET.h"
#include <iostream>

const float OBSTACLE_HEIGHT=2;
const float PATH_NODE_DIA=15;
const float STEP_DIA=10;

typedef std::set<const AStarNode*> DisplaySet;

void dumpObstacles(ionetstream& io, const GaitedFootsteps& f);
void dumpPath(ionetstream& io, const AStarResults& res);
void dumpState(ionetstream& io, const GaitedFootsteps& f, const AStarResults& res, size_t i);
void dumpWork(ionetstream& io, const AStarResults& res, bool extended);

void subsample(const AStarResults::NodeSet& full, DisplaySet& sub, size_t n);
void displayPath(KinematicJoint& kj, const AStarNode& node, DisplaySet& displayed, const std::string& nodeClr, const std::string& linkClr);

int usage(const std::string& name, const std::string& msg) {
	if(msg.size()>0)
		std::cerr << msg << std::endl;
	std::cerr << "Usage: " << name << " [-w|--walk gait-file] [-k|--kin kinematics-file]\n";
	std::cerr << "       [-e|--env environment-file] [-m|--mirage [host]] [-b|--bound dist]\n";
	std::cerr << "       [-r|--relax relaxation] [-s|--start x y] [-g|--goal x y] [--no-reorient] [--no-rehab]\n";
	std::cerr << "       [-f|--fig file] [--hide-path] [--show-work] [--show-steps] [--show-support] [--show-grid]\n";
	std::cerr << "       [-o|--ori len] [-v|--view x y w h] [--style css-file] [--embed-style]\n";
	return msg.size()==0 ? EXIT_SUCCESS : 2;
}

static std::string mirage;

int main(int argc, const char* argv[]) {
	std::string kinematicsFile("robot.kin");
	XWalkParameters xp;
	plist::ArrayOf<PlannerObstacle> obs;
	GaitedFootsteps f;
	GaitedFootsteps::State initial, goal;
	goal.pos[0]=1100;
	//goal.pos[1]=600;
	std::string figures;
	int figFlags=FIGURES_SHOW_PATH;
	fmat::Column<4> viewBox;
	std::string styleFile="style.css";
	std::string obsFile;
	float boundDist=0;
	
	for(int i=1; i<argc; ++i) {
		std::string arg = argv[i];
		try {
			if(arg=="-w" || arg=="--walk") {
				if(++i==argc) throw std::invalid_argument("Missing file argument for "+arg);
				xp.loadFile(argv[i]);
			} else if(arg=="-k" || arg=="--kin") {
				if(++i==argc) throw std::invalid_argument("Missing file argument for "+arg);
				kinematicsFile = argv[i];
			} else if(arg=="-e" || arg=="--env") {
				if(++i==argc) throw std::invalid_argument("Missing file argument for "+arg);
				obsFile=argv[i];
				obs.loadFile(argv[i]);
			} else if(arg=="-r" || arg=="--relax") {
				if(++i==argc) throw std::invalid_argument("Missing argument for "+arg);
				std::stringstream ss(argv[i]);
				if(!(ss >> f.relaxation)) throw std::invalid_argument("Bad value for "+arg);
				std::cout << "Relaxation set to " << f.relaxation << std::endl;
			} else if(arg=="-b" || arg=="--bound") {
				if(++i==argc) throw std::invalid_argument("Missing argument for "+arg);
				std::stringstream ss(argv[i]);
				if(!(ss >> boundDist)) throw std::invalid_argument("Bad value for "+arg);
			} else if(arg=="-s" || arg=="--start") {
				if((i+=2)>=argc) throw std::invalid_argument("Missing argument for "+arg);
				std::stringstream ss(argv[i-1]);
				if(!(ss >> initial.pos[0])) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i-1]);
				ss.clear(); ss.str(argv[i]);
				if(!(ss >> initial.pos[1])) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i]);
			} else if(arg=="-g" || arg=="--goal") {
				if((i+=2)>=argc) throw std::invalid_argument("Missing argument for "+arg);
				std::stringstream ss(argv[i-1]);
				if(!(ss >> goal.pos[0])) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i-1]);
				ss.clear(); ss.str(argv[i]);
				if(!(ss >> goal.pos[1])) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i]);
			} else if(arg=="-m" || arg=="--mirage") {
				if(i+1<argc && argv[i+1][0]!='-') {
					mirage=argv[++i];
				} else {
					mirage="localhost";
				}
			} else if(arg=="-f" || arg=="--fig") {
				if(++i==argc) throw std::invalid_argument("Missing argument for "+arg);
				figures=argv[i];
			} else if(arg=="--hide-path") {
				figFlags &= ~FIGURES_SHOW_PATH;
			} else if(arg=="--show-work") {
				figFlags |= FIGURES_SHOW_WORK;
			} else if(arg=="--show-steps") {
				figFlags |= FIGURES_SHOW_STEPS;
			} else if(arg=="--show-support") {
				figFlags |= FIGURES_SHOW_SUPPORT;
			} else if(arg=="--show-grid") {
				figFlags |= FIGURES_SHOW_GRID;
			} else if(arg=="--embed-style") {
				figFlags |= FIGURES_EMBED_STYLE;
			} else if(arg=="-o" || arg=="--ori") {
				if(++i==argc) throw std::invalid_argument("Missing file argument for "+arg);
				std::stringstream ss(argv[i]);
				if(!(ss >> figureOriLen)) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i]);
			} else if(arg=="style") {
				if(++i==argc) throw std::invalid_argument("Missing file argument for "+arg);
				styleFile = argv[i];
			} else if(arg=="--no-reorient") {
				f.stepReorient=false;
			} else if(arg=="--no-rehab") {
				f.stepRehab=false;
			} else if(arg=="-v" || arg=="--view") {
				if((i+=4)>=argc) throw std::invalid_argument("Missing argument for "+arg);
				std::stringstream ss;
				for(size_t j=0; j<4; ++j) {
					ss.clear(); ss.str(argv[i-3+j]);
					if(!(ss >> viewBox[j])) throw std::invalid_argument("Bad argument for "+arg+": "+argv[i-3+j]);
				}
			} else if(arg=="-h" || arg=="--help") {
				return usage(argv[0],"");
			} else {
				throw std::invalid_argument("Uknown argument '"+arg+"'");
			}
		} catch(const std::invalid_argument& ex) {
			return usage(argv[0],ex.what());
		} catch(const std::exception& ex) {
			std::cerr << "Exception: " << ex.what() << std::endl;
			return 1;
		}
	}
	
	{
		KinematicJoint kj;
		kj.loadFile(kinematicsFile.c_str());
		f.setGait(kj,xp,4);
	}
	
	for(size_t i=0; i<NumLegs; ++i)
		initial.footPos[i] = f.neutrals[i];
	
	// display candidates at root
	/*const std::vector<std::pair<float,GaitedFootsteps::State> >& candidates = f.expand(NULL,initial,goal);
	for(size_t i=0; i<candidates.size(); ++i) {
		std::cout << candidates[i].second.pos[0] << '\t' << candidates[i].second.pos[1] << '\t' << candidates[i].first << std::endl;
	}*/
	
	for(size_t i=0; i<obs.size(); ++i)
		f.addObstacle(obs[i].clone());
	
	TimeET planningTime;
	AStarResults res = AStar::astar(f,initial,goal,boundDist);
	planningTime = planningTime.Age();
	
	ionetstream netcomm;
	
	//dumpPath(netcomm,res);
	std::cout << res.path.size() << " steps, " << res.closed.size() <<" closed " << res.open.size() << " open" << std::endl;
	std::cout << "Total cost: " << res.cost << std::endl;
	std::cout << "Planning time: " << planningTime << std::endl;
	// DATA FORMAT: steps open closed dist time groupSize environment
	std::cout << "DATA " << res.path.size() << '\t' << res.closed.size() << '\t' << res.open.size() << '\t' << res.cost << '\t' << planningTime << '\t' << f.groups.front().size() << '\t' << obsFile << std::endl;
	
	if(figures.size()>0)
		dumpFigures(figures, f, res, goal.pos, viewBox, styleFile, figFlags);
	
	if(mirage.size()==0)
		return EXIT_SUCCESS;
	
	dumpObstacles(netcomm,f);
	bool showWork=false;
	dumpWork(netcomm,res,showWork);
	
	std::string in, last;
	size_t i=0;
	while(std::getline(std::cin,in)) {
		if(in.size()==0)
			in=last;
		if(in=="n") {
			++i;
		} else if(in=="p") {
			--i;
		} else if(in=="w") {
			showWork = !showWork;
			dumpWork(netcomm,res,showWork);
		} else {
			std::stringstream ss(in);
			if(!(ss >> i)) {
				std::cerr << "Bad input " << in << std::endl;
				continue;
			}
		}
		if(i>=res.path.size()) {
			std::cerr << "State " << i << " is out of range." << std::endl;
			continue;
		}
		last=in;
		dumpState(netcomm, f, res, i);
	}
	return EXIT_SUCCESS;
}

void dumpObstacles(ionetstream& io, const GaitedFootsteps& f) {
	MirageComm mcomm(io,mirage);
	mcomm.setName("Obstacles");
	KinematicJoint * kj = new KinematicJoint;
	mcomm.setKinematics(kj);
	for(size_t i=0; i<f.getObstacles().size(); ++i) {
		PlannerObstacle* ob = f.getObstacles()[i];
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

void dumpPath(ionetstream& io, const AStarResults& res) {
	for(size_t i=0; i<res.path.size(); ++i) {
		std::cout << res.path[i].pos[0] << ' ' << res.path[i].pos[1] << std::endl;
	}
}

void dumpState(ionetstream& io, const GaitedFootsteps& f, const AStarResults& res, size_t i) {
	MirageComm mcomm(io,mirage);
	mcomm.setName("Steps");
	KinematicJoint * kj = new KinematicJoint;
	mcomm.setKinematics(kj);
	std::set<size_t> moved;
	if(i>0)
		moved=f.groups[res.path[i-1].phase];
	for(std::set<size_t>::const_iterator it=moved.begin(); it!=moved.end(); ++it) {
		typedef typeof(res.path[i].footPos[*it]) pos_t;
		pos_t prev = res.path[i-1].footPos[*it];
		LinkComponent * lc = new LinkComponent;
		kj->components.addEntry(lc);
		lc->model="Sphere";
		prev.exportTo(lc->modelOffset);
		lc->modelScale[0] = lc->modelScale[1] = lc->modelScale[2] = STEP_DIA;
		lc->material="Pink";
		
		pos_t next = res.path[i].footPos[*it];
		pos_t mid = (next + prev)/2;
		pos_t dv = (next - prev);
		fmat::Quaternion dir = fmat::Quaternion::aboutZ(atan2(dv[1],dv[0])) * fmat::Quaternion::aboutY(M_PI/2);
		lc = new LinkComponent;
		kj->components.addEntry(lc);
		lc->model="Cylinder";
		mid.exportTo(lc->modelOffset);
		dir.exportTo(lc->modelRotation[0],lc->modelRotation[1],lc->modelRotation[2]);
		lc->modelScale[0]=lc->modelScale[1]=STEP_DIA/2;
		lc->modelScale[2]=dv.norm();
		lc->material="Pink";
	}
	for(size_t leg=0; leg<NumLegs; ++leg) {
		LinkComponent * lc = new LinkComponent;
		kj->components.addEntry(lc);
		lc->model="Sphere";
		res.path[i].footPos[leg].exportTo(lc->modelOffset);
		lc->modelScale[0] = lc->modelScale[1] = lc->modelScale[2] = STEP_DIA;
		lc->material = (moved.count(leg)==0) ? "Orange" : "Green";
	}
	LinkComponent * lc = new LinkComponent;
	kj->components.addEntry(lc);
	lc->model="Sphere";
	res.path[i].pos.exportTo(lc->modelOffset);
	lc->modelScale[0] = lc->modelScale[1] = lc->modelScale[2] = PATH_NODE_DIA*1.5;
	lc->material="Blue";
}

void subsample(const AStarResults::NodeSet& full, DisplaySet& sub, size_t n) {
	if(n >= full.size()) {
		sub.insert(full.begin(),full.end());
		return;
	}
	float inc = n/float(full.size());
	float level = 0;
	for(AStarResults::set_const_iterator it=full.begin(); it!=full.end(); ++it) {
		if( (level+=inc) >= 1) {
			level -= 1;
			sub.insert(*it);
		}
	}
}

void displayPath(KinematicJoint& kj, const AStarNode& node, DisplaySet& displayed, const std::string& nodeClr, const std::string& linkClr) {
	if(displayed.count(&node))
		return;
	displayed.insert(&node);
	LinkComponent * lc = new LinkComponent;
	kj.components.addEntry(lc);
	lc->model="Sphere";
	node.state.pos.exportTo(lc->modelOffset);
	lc->modelScale[0] = lc->modelScale[1] = lc->modelScale[2] = PATH_NODE_DIA;
	lc->material=nodeClr;
	if(node.parent!=NULL) {
		fmat::Column<2> dv = node.state.pos - node.parent->state.pos;
		if(dv.norm() > PATH_NODE_DIA) {
			fmat::Column<2> mid = (node.state.pos + node.parent->state.pos)/2;
			fmat::Quaternion dir = fmat::Quaternion::aboutZ(atan2(dv[1],dv[0])) * fmat::Quaternion::aboutY(M_PI/2);
			lc = new LinkComponent;
			kj.components.addEntry(lc);
			lc->model="Cylinder";
			mid.exportTo(lc->modelOffset);
			dir.exportTo(lc->modelRotation[0],lc->modelRotation[1],lc->modelRotation[2]);
			lc->modelScale[0]=lc->modelScale[1]=PATH_NODE_DIA/2;
			lc->modelScale[2]=dv.norm();
			lc->material=linkClr;
		}
		displayPath(kj,*node.parent,displayed,nodeClr,linkClr);
	}
}

void dumpWork(ionetstream& io, const AStarResults& res, bool extended) {
	MirageComm mcomm(io,mirage);
	mcomm.setName("Work");
	KinematicJoint * kj = new KinematicJoint;
	mcomm.setKinematics(kj);
	
	TimeET dispTime;
	
	DisplaySet displayed;
	displayPath(*kj,*res.priorities.front(),displayed,"Yellow","Yellow");
	
	if(!extended)
		return;
	
	DisplaySet closed, open;
	subsample(res.closed,closed,500);
	subsample(res.open,open,500);
	for(DisplaySet::const_iterator it=open.begin(); it!=open.end(); ++it) {
		const AStarNode* n = (*it)->parent;
		while(n!=NULL) {
			if(res.closed.count(const_cast<AStarNode*>(n))!=0)
				closed.insert(n);
			n = n->parent;
		}
	}
	
	for(DisplaySet::const_iterator it=closed.begin(); it!=closed.end(); ++it)
		displayPath(*kj,**it,displayed,"Pink","Pink");
	
	for(DisplaySet::const_iterator it=open.begin(); it!=open.end(); ++it)
		displayPath(*kj,**it,displayed,"Blue","Blue");
	
	std::cout << "Display Time " << dispTime.Age() << std::endl;
}

#include "Motion/Kinematics.h"
Kinematics * kine=NULL;

