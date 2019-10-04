#include "Shared/fmatSpatial.h"
#include "Motion/KinematicJoint.h"
#include "Wireless/netstream.h"

#include <iostream>
#include <sstream>
#include <limits>
#include <cmath>
#include <vector>
#include <set>
#include <map>
#include <string>

using namespace std;

void usage(const std::string& execName) {
	cerr << "Usage: " << execName << " [-m [host[:port]]] -w base [distscale] [--offset x y z]\n"
	<< execName << " will read an input file from standard input, and write\n"
	"the kinematic data on standard output.  Use standard command line redirection\n"
	"if you wish to save or load from a file.\n"
	"\n"
	"The per-line format is one of:\n"
	"    Name jointName Origin x y z [Z | RotR | RotD] x y z [Others] [Component ..]\n"
	"    Branch jointName\n"
	"\n"
	"The first line will declare joint, with origin specified in world coordinates,\n"
	"and a z-axis orientation, either directly as a vector, or x/y/z rotations.\n"
	"(Rotations given in radians (RotR) or degrees (RotD).)  Other KinematicJoint\n"
	"fields can also be specified.\n"
	"\n"
	"A line can end with one or more Component specifications, which are typically\n"
	"used to specify a rough model for collision simulation, e.g.:\n"
	"  ... Component CollisionModel Cube CollisionModelScale 10 20 30\n"
	"\n"
	"A joint specification can be continued across lines by placing a backslash '\\'\n"
	"at the end of the line.\n"
	"\n"
	"The second style line will declare a branch on a previously named link, such\n"
	"that any joints declared after the branch statement will be attached following\n"
	"the specified joint.\n"
	"\n"
	"The -m (--mirage) option will send the kinematics file to a Mirage visualizer,\n"
	"to be placed at (0,0,-lowest point in model), i.e. \"internal\" BaseFrames will\n"
	"be raised such that the entire model is placed above ground.\n"
	"\n"
	"The -w (--weight) option will apply automatic mass values:\n"
	"    mass = base + distscale·√(r^2+d^2)\n"
	"\n"
	"The --offset will cause the provided (x,y,z) values to be subtracted from all\n"
	"frame origins.  This is useful if you are pulling coordinates from a model\n"
	"which is not centered on the desired final baseframe.\n"
	;
}

static const fmat::fmatReal EPSILON = numeric_limits<float>::epsilon()*10;
static const KinematicJoint& emptyKJ() {
	static KinematicJoint kj;
	return kj;
}
/*static const LinkComponent& emptyLink() {
	static LinkComponent link;
	return link;
}*/
bool massSet=false;
float baseMass=0;
float distMass=0;

//! Used with fmat::Column::apply(), map(), or directly to clip floating point rounding errors back to 0
/*! Main concern here is to provide prettier user output */
fmat::fmatReal shave(fmat::fmatReal x) { return std::abs(x)<EPSILON ? 0 : x; }

//! Stores some extra data for each KinematicJoint during construction and some utility functions
struct Frame : public KinematicJoint {
	Frame() : KinematicJoint(), z(fmat::UNIT3_Z), wtr(), origOrigin() {
		qmin=-M_PI/2;
		qmax=M_PI/2;
	}
	Frame* getParent() const { return dynamic_cast<Frame*>(KinematicJoint::getParent()); }
	Frame* nextJoint() const { return dynamic_cast<Frame*>(KinematicJoint::nextJoint()); }
	
	fmat::Column<3> z; //!< requested z axis from input
	fmat::Transform wtr; //!< initially requested transform, after calculation cached getFullT()
	fmat::Column<3> origOrigin; //!< the initially requested origin so we can report the offset at the end

	void filterUnused(); //!< for output readability, skip un-customized values
	void makeCollisionModel(); //!< provide some simple link and motor graphics so we can visualize off the bat.
};

//! report a message on cerr and return EXIT_FAILURE, to be used from main() during input processing
int error(size_t line, const plist::OutputSelector& name, const string& msg) {
	cerr << "ERROR line " << line << ": ";
	if(name!=plist::OutputSelector::UNUSED)
		cerr << name.get() << " ";
	cerr << msg << endl;
	return EXIT_FAILURE;
}

//! the big cheese: performs computations to solve for D-H parameters
void computeDH(KinematicJoint* kj);

//! inform the user of frames that could not be placed at the requested position
void reportOffsets(KinematicJoint* kj);

//! find lowest boundary point in model
float findBottom(const KinematicJoint* kj);

//! begins execution
int main(int argc, char* argv[]) {
	Frame root;
	string mirageHost;
	fmat::Column<3> originOffset;
	
	// *****************************
	// **   PROCESS COMMAND LINE  **
	// *****************************
	for(int x=1; x<argc; x++) {
		if(strcmp(argv[x],"-m")==0 || strcmp(argv[x],"--mirage")==0) {
			if(x==argc-1 || argv[x+1][0]=='-') {
				mirageHost = "localhost:19785";
			} else if(strchr(argv[x+1],':')==NULL) {
				mirageHost = argv[++x];
				mirageHost += ":19785";
			} else {
				mirageHost = argv[++x];
			}
		} else if(strcmp(argv[x],"-w")==0 || strcmp(argv[x],"--weight")==0) {
			if(x==argc-1 || argv[x+1][0]=='-') {
				cerr << "weight requires a non-negative parameter" << endl;
				return EXIT_FAILURE;
			}
			baseMass = atof(argv[++x]);
			if(x<argc-1 && argv[x+1][0]!='-')
				distMass = atof(argv[++x]);
			massSet=true;
		} else if(strcmp(argv[x],"--offset")==0) {
			if(x>=argc-3 || (argv[x+1][0]=='-' && isalpha(argv[x+1][1])) || (argv[x+2][0]=='-' && isalpha(argv[x+2][1])) || (argv[x+3][0]=='-' && isalpha(argv[x+3][1]))) {
				cerr << "origin offset requires three parameters" << endl;
				return EXIT_FAILURE;
			}
			originOffset[0]=atof(argv[++x]);
			originOffset[1]=atof(argv[++x]);
			originOffset[2]=atof(argv[++x]);
		} else if(strcmp(argv[x],"-h")==0 || strcmp(argv[x],"--help")==0) {
			usage(argv[0]);
			return 0;
		} else {
			cerr << "Unrecognized argument " << argv[x] << endl;
			usage(argv[0]);
			return 2;
		}
	}
	
	
	// *****************************
	// **     PROCESS INPUT       **
	// *****************************
	string lineStr;
	size_t lineNum=0;
	Frame * parent=&root;
	while(getline(cin,lineStr)) {
		++lineNum;
		while(lineStr.size()>0 && lineStr[lineStr.size()-1]=='\\') {
			lineStr.erase(lineStr.size()-1); // remove trailing '\'
			std::string continuation;
			if(!getline(cin,continuation))
				break;
			lineStr+=continuation;
		}
		lineStr = string_util::trim(lineStr);
		if(lineStr[0]=='#')
			continue;
		bool hasContent=false;
		stringstream line(lineStr);
		Frame& f = *new Frame;
		while(line) {
			string section;
			if(!(line >> section))
				break;
			if(section=="Component") {
				LinkComponent* lc = new LinkComponent;
				lc->model="";
				f.components.addEntry(lc);
				while((line >> section)) {
					if(section.size()==0)
						continue;
					if(section=="Component") {
						lc = new LinkComponent;
						lc->model="";
						f.components.addEntry(lc);
						continue;
					}
					if(section[0]=='#')
						break;
					if(lc->findEntry(section)!=lc->end()) {
						if(plist::PrimitiveBase * prim = dynamic_cast<plist::PrimitiveBase*>(&(*lc)[section])) {
							string str;
							if(!(line >> str)) return error(lineNum, f.outputOffset, section + " missing value");
							prim->set(str);
						} else if(plist::ArrayBase * arr = dynamic_cast<plist::ArrayBase*>(&(*lc)[section])) {
							for(size_t i=0; i<arr->size(); ++i) {
								string str;
								if(!(line >> str)) return error(lineNum, f.outputOffset, section + " missing value");
								dynamic_cast<plist::PrimitiveBase&>((*arr)[i]).set(str);
							}
						}
						hasContent=true;
					} else {
						return error(lineNum, f.outputOffset, "Unknown component data tag '"+section+"'");
					}
				}
				break; // Component always finishes the line
			}
			if(section.size()==0)
				continue;
			if(section[0]=='#')
				break;
			if(section=="Name") {
				string x;
				if(!(line >> x)) return error(lineNum, f.outputOffset, section + " missing string");
				f.outputOffset = x;
			} else if(section=="Branch") {
				string x;
				if(!(line >> x)) return error(lineNum, f.outputOffset, section + " missing name");
				map<unsigned int,KinematicJoint*> children;
				root.buildChildMap(children,0,-1U);
				parent = dynamic_cast<Frame*>(children[capabilities.getOutputOffset(x)]);
				if(parent==NULL)
					return error(lineNum, f.outputOffset, section + " could not find joint named "+x);
			} else if(section=="Origin") {
				if(!(line >> f.origOrigin[0])) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> f.origOrigin[1])) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> f.origOrigin[2])) return error(lineNum, f.outputOffset, section + " missing elements");
				f.origOrigin-=originOffset;
				f.wtr.translation() = f.origOrigin;
				hasContent=true;
			} else if(section=="Z") {
				if(!(line >> f.z[0])) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> f.z[1])) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> f.z[2])) return error(lineNum, f.outputOffset, section + " missing elements");
				if(f.z.norm()<EPSILON)
					return error(lineNum, f.outputOffset, section + " must be non-zero vector");
				f.z/=f.z.norm();
				hasContent=true;
			} else if(section=="RotR" || section=="RotD") {
				double x,y,z;
				if(!(line >> x)) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> y)) return error(lineNum, f.outputOffset, section + " missing elements");
				if(!(line >> z)) return error(lineNum, f.outputOffset, section + " missing elements");
				if(section=="RotD") {
					x*=M_PI/180;
					y*=M_PI/180;
					z*=M_PI/180;
				}
				f.wtr.rotation() = fmat::rotationZ(z) * fmat::rotationY(y) * fmat::rotationX(x);
				f.z = f.wtr.rotation().column(2);
				hasContent=true;
			} else if(f.findEntry(section)!=f.end()) {
				if(plist::PrimitiveBase * prim = dynamic_cast<plist::PrimitiveBase*>(&f[section])) {
					string str;
					if(!(line >> str)) return error(lineNum, f.outputOffset, section + " missing value");
					prim->set(str);
				} else if(plist::ArrayBase * arr = dynamic_cast<plist::ArrayBase*>(&f[section])) {
					for(size_t i=0; i<arr->size(); ++i) {
						string str;
						if(!(line >> str)) return error(lineNum, f.outputOffset, section + " missing value");
						dynamic_cast<plist::PrimitiveBase&>((*arr)[i]).set(str);
					}
				}
				hasContent=true;
			} else {
				return error(lineNum, f.outputOffset, "Unknown data tag '"+section+"'");
			}
		}
		if(!hasContent && parent==&root && f.outputOffset!=plist::OutputSelector::UNUSED) {
			hasContent=true;
			f.qmin=f.qmax=0;
		}
		if(hasContent) {
			parent->addBranch(&f);
			parent = &f;
		} else {
			delete &f;
		}
	}
	if(root.nextJoint()==NULL) {
		cerr << "Empty input" << endl;
		return EXIT_FAILURE;
	}
	
	
	// *****************************
	// **     COMPUTE/SAVE DH     **
	// *****************************
	for_each(root.getBranches().begin(),root.getBranches().end(),computeDH);
	root.makeCollisionModel();
	root.filterUnused();
	float bodyOffset = -findBottom(&root);
	
	// generally first frame is the base frame, no transform
	// so skip these parameters in output for readability
	Frame& base = *root.nextJoint();
	base.removeEntry("JointType");
	base.removeEntry("d");
	base.removeEntry("r");
	base.removeEntry("θ");
	base.removeEntry("α");
	base.removeEntry("Min");
	base.removeEntry("Max");

	base.saveStream(cout);
	cout << endl;
	usleep(10000); // if piping to 'tee' or such, give it a little time to show up before continuing
	


	// *****************************
	// **    DISPLAY IN MIRAGE    **
	// *****************************
	if(mirageHost.size()>0) {
		ionetstream mirage;

		// open connection
		if(!mirage.open(mirageHost)) {
			std::cerr << "Connection to " << mirageHost << " refused." << std::endl;
			//return EXIT_FAILURE;
		}
		mirage << "<messages>\n";

		// this will hold the keys and values we want to send to mirage
		plist::Dictionary msg;
		msg.addEntry("ID",new plist::Primitive<std::string>("dhcalc"));
		msg.addEntry("Persist",new plist::Primitive<bool>(true));
		msg.addEntry("Model",new KinematicJointSaver(root));
		msg.addEntry("Location",new plist::Point(0,0,bodyOffset));

		// write the message and exit
		msg.saveStream(mirage,true);
		mirage << "</messages>";
		mirage.flush();
		mirage.close();
	}
	


	// *****************************
	// **      Report Offsets     **
	// *****************************
	if(bodyOffset>EPSILON) {
		cerr << "Body offset by " << bodyOffset << "\n";
		cerr << "  (use this as Drivers.Mirage.InitialLocation.2 value to be placed entirely above ground plane)" << endl;
	}
	for_each(root.getBranches().begin(),root.getBranches().end(),reportOffsets);
	
	return EXIT_SUCCESS;
}



/*! This is a recursive function, so it will solve for the requested
 *  joint and then call itself on sub-branches */
void computeDH(KinematicJoint* kj) {
	Frame& cur = dynamic_cast<Frame&>(*kj);
	Frame& parent=*cur.getParent();
	
	cur.alpha = acos( fmat::dotProduct(parent.z, cur.z) );
	
	if(std::abs(cur.alpha) < EPSILON || std::abs(cur.alpha-M_PI) < EPSILON) {
		// parallel z-axis case
		
		// find current origin relative to parent's frame
		fmat::Transform wToP = parent.wtr.rigidInverse(); // world to parent transform
		fmat::Column<3> pp = wToP * cur.wtr.translation(); // use it
		
		cur.d = pp[2];
		cur.theta = atan2(pp[1],pp[0]);
		cur.r = fmat::SubVector<2>(pp).norm();

	} else {
		// non-parallel z-axis case

		/* Need to solve for intersection of common normal:
		 * Say we have:
		 *     normal n
		 *     parent z vector t through point p
		 *     current z vector u through q
		 * Then we want to solve for scalars a, b, and c in:
		 *     p+at + cn = q+bu
		 *     at - bu + cn = q-p
		 *     at + b(-u) + cn = q-p
		 *     [t -u n] [a b c]ᵀ = q-p
		 *     [a b c]ᵀ = [t -u n]'(q-p) */
		
		fmat::Column<3> n = fmat::crossProduct(cur.z,parent.z);
		n/=n.norm();
		
		// we want the normal to point at the new frame so we have positive d
		fmat::Column<3> dv = cur.wtr.translation() - parent.wtr.translation();
		if(fmat::dotProduct(dv,n)<0)
			n=-n;
		
		// fix sign on alpha now that we know orientation of normal
		if(fmat::dotProduct(n, fmat::crossProduct(parent.z, cur.z))<0)
			cur.alpha=-cur.alpha;
		
		fmat::Matrix<3,3> x;
		x.column(0) = parent.z;
		x.column(1) = -cur.z;
		x.column(2) = n;
		
		fmat::Column<3> dist = fmat::invert(x) * (cur.wtr.translation() - parent.wtr.translation());
		fmat::Column<3> px = parent.wtr.rotation().column(0);
		
		cur.d = dist[0];
		cur.theta = acos( fmat::dotProduct(n, px) );
		if(fmat::dotProduct(parent.z, fmat::crossProduct(px,n))<0)
			cur.theta=-cur.theta;
		cur.r = dist[2];
	}
	
	// clean up "stubble" from floating point errors
	cur.d = shave(cur.d);
	cur.theta = shave(cur.theta);
	cur.r = shave(cur.r);
	cur.alpha = shave(cur.alpha);
	
	// manual calcuations:
	/*fmat::Transform zTr(fmat::rotationZ<3,fmat::fmatReal>(cur.theta), fmat::pack(0,0,cur.d));
	fmat::Transform xTr(fmat::rotationX<3,fmat::fmatReal>(cur.alpha), fmat::pack(cur.r,0,0));
	
	fmat::Transform toParent = zTr * xTr;
	cout << "Parent:\n" << toParent << endl;
	
	cur.wtr = parent.wtr * toParent;
	cout << "World:\n" << cur.wtr << endl;*/
	
	// or just use KJ's own computation:
	cur.wtr = cur.getFullT();
	
	// recurse on children
	for_each(cur.getBranches().begin(),cur.getBranches().end(),computeDH);
}



void reportOffsets(KinematicJoint* kj) {
	Frame& cur = dynamic_cast<Frame&>(*kj);
	fmat::Column<3> woff = (cur.wtr.translation() - cur.origOrigin);
	if(woff.norm()>EPSILON) {
		fmat::Column<3> loff = cur.getFullInvT()*cur.origOrigin;
		if(fmat::SubVector<2>(loff).norm() > EPSILON)
			cerr << "WARNING: final DH origin signficantly offset from z axis " << loff << endl;
		cerr << cur.outputOffset << " original origin offset by " << woff << " (local z offset " << -loff[2] << ")" << endl;
	}

	// recurse on children
	for_each(cur.getBranches().begin(),cur.getBranches().end(),reportOffsets);
}



void Frame::filterUnused() {
	std::set<string> removals;
	for(iterator it=dict.begin(); it!=dict.end(); ++it) {
		if(plist::PrimitiveBase* prim = dynamic_cast<plist::PrimitiveBase*>(it->second)) {
			if(*prim == dynamic_cast<plist::PrimitiveBase&>(*emptyKJ().findEntry(it->first)->second))
				removals.insert(it->first);
		} else if(plist::Collection* col = dynamic_cast<plist::Collection*>(it->second)) {
			if((it->first!="ModelScale" || model=="CollisionObject" || (modelScale[0]==1 && modelScale[1]==1 && modelScale[2]==1))
			&& (it->first!="ModelOffset" || model=="CollisionObject" || (modelOffset[0]==0 && modelOffset[1]==0 && modelOffset[2]==0))
			&& (it->first!="ModelRotation" || model=="CollisionObject" || (modelRotation[0]==0 && modelRotation[1]==0 && modelRotation[2]==0))
			&& (it->first!="CollisionModelScale" || (collisionModelScale[0]==1 && collisionModelScale[1]==1 && collisionModelScale[2]==1))
			&& (it->first!="CollisionModelOffset" || (collisionModelOffset[0]==0 && collisionModelOffset[1]==0 && collisionModelOffset[2]==0))
			&& (it->first!="CollisionModelRotation" || (collisionModelRotation[0]==0 && collisionModelRotation[1]==0 && collisionModelRotation[2]==0))
			&& (it->first!="CenterOfMass" || mass==0 || (centerOfMass[0]==0 && centerOfMass[1]==0 && centerOfMass[2]==0))
			&& (it->first!="Components" || col->size()==0))
				removals.insert(it->first);
		} else {
			cerr << "Unknown plist::Dictionary entry type for " << it->first << "!" << endl;
		}
	}
	// we always include these
	removals.erase("θ");
	removals.erase("d");
	removals.erase("α");
	removals.erase("r");

	if(outputOffset == plist::OutputSelector::UNUSED) {
		removals.insert("Name");
		removals.insert("JointType");
		removals.insert("Min");
		removals.insert("Max");
	}
	for(std::set<string>::const_iterator it=removals.begin(); it!=removals.end(); ++it)
		removeEntry(*it);
	for(component_iterator it=components.begin(); it!=components.end(); ++it) {
		(*it)->clear();
		if((*it)->model!="CollisionModel") {
			(*it)->addEntry("Model",(*it)->model);
			if((*it)->model.size()>0) {
				if((*it)->modelScale[0]!=1 || (*it)->modelScale[1]!=1 || (*it)->modelScale[2]!=1)
					(*it)->addEntry("ModelScale",(*it)->modelScale);
				if((*it)->modelOffset[0]!=0 || (*it)->modelOffset[1]!=0 || (*it)->modelOffset[2]!=0)
					(*it)->addEntry("ModelOffset",(*it)->modelOffset);
				if((*it)->modelRotation[0]!=0 || (*it)->modelRotation[1]!=0 || (*it)->modelRotation[2]!=0)
					(*it)->addEntry("ModelRotation",(*it)->modelRotation);
			}
		}
		
		if((*it)->collisionModel.size()>0) {
			(*it)->addEntry("CollisionModel",(*it)->collisionModel);
			if((*it)->collisionModelScale[0]!=1 || (*it)->collisionModelScale[1]!=1 || (*it)->collisionModelScale[2]!=1)
				(*it)->addEntry("CollisionModelScale",(*it)->collisionModelScale);
			if((*it)->collisionModelOffset[0]!=0 || (*it)->collisionModelOffset[1]!=0 || (*it)->collisionModelOffset[2]!=0)
				(*it)->addEntry("CollisionModelOffset",(*it)->collisionModelOffset);
			if((*it)->collisionModelRotation[0]!=0 || (*it)->collisionModelRotation[1]!=0 || (*it)->collisionModelRotation[2]!=0)
				(*it)->addEntry("CollisionModelRotation",(*it)->collisionModelRotation);
		}
		
		if((*it)->mass!=0)
			(*it)->addEntry("Mass",(*it)->mass);
		if((*it)->material.size()>0)
			(*it)->addEntry("Material",(*it)->material);
		if(!(*it)->visible)
			(*it)->addEntry("Visible",(*it)->visible);
	}
	
	// recurse on children
	for(branch_iterator it = branches.begin(); it!=branches.end(); ++it)
		dynamic_cast<Frame&>(**it).filterUnused();
}



/*! Also sets the mass by assigning mass for each component */
void Frame::makeCollisionModel() {
	if(collisionModel.size()>0 || components.size()>0) {
		/* user defined collision model */
		if(massSet && !hasMass())
			mass = baseMass;
		
	} else if(branches.size()==0) {
		// no branches, this is a leaf
		
		fmat::fmatReal mindim = max(r/2,d/2);
		if(mindim==0)
			mindim=shave(getParent()->getTo().translation().norm()*.75);
		if(mindim==0)
			mindim=4;
		
		if(model=="CollisionModel") {
			fmat::Column<3> center(0.f);
			fmat::Column<3> dim(mindim/50);
			LinkComponent * link = new LinkComponent;
			center.exportTo(link->modelOffset);
			dim.exportTo(link->modelScale);
			link->model="ReferenceFrame";
			link->visible=false;
			components.addEntry(link);
		}

		if(isMobile() && collisionModel.size()==0) {
			fmat::Column<3> center = ( getFullInvT() * origOrigin ).map(shave);
			//center[2] -= mindim*3/4/2;
			fmat::Column<3> dim(mindim*3/4);
			LinkComponent * link = new LinkComponent;
			center.exportTo(link->collisionModelOffset);
			dim.exportTo(link->collisionModelScale);
			link->collisionModel="Cylinder";
			link->mass = baseMass;
			components.addEntry(link);
		}
		
	} else {
		// one or more branches, draw link(s) to the child joints
		bool useLink=true; // if this remains true, we draw one bar, otherwise a bounding box
		
		fmat::Column<3> childP; // position of first non-zero child point
		for(branch_iterator it=branches.begin(); it!=branches.end(); ++it) {
			childP = ( getFullInvT() * dynamic_cast<Frame*>(*it)->origOrigin ).map(shave);
			if(childP.norm()>EPSILON)
				break;
		}
		// see if there are any links at a different position and also not at current origin
		for(branch_iterator it=branches.begin(); it!=branches.end(); ++it) {
			fmat::Column<3> p = ( getFullInvT() * dynamic_cast<Frame*>(*it)->origOrigin ).map(shave);
			if( ((p - childP).norm() > childP.norm()/100) && (p.norm() > EPSILON) ) {
				useLink=false;
				break;
			}
		}
		
		LinkComponent * link;
		fmat::Column<3> center,dim;
		double mindim;
		if(useLink) {
			// only one destination (although branches may split from there...)
			// use a thin bar to connect origin to destination
			
			if(childP.norm()>EPSILON) {
				mindim = fmat::SubVector<2>(childP).norm()/20;
				bool doRot=false;
				if(mindim==0) { // purely vertical
					mindim = std::abs(childP[2])/20;
					dim = fmat::pack(mindim,mindim,std::abs(childP[2]));
				} else {
					dim = fmat::pack(mindim*20,mindim,max(std::abs(childP[2]),mindim));
					doRot=true;
				}
				if(collisionModel.size()==0) {
					link = new LinkComponent;
					(childP/2).exportTo(link->collisionModelOffset);
					dim.exportTo(link->collisionModelScale);
					if(doRot)
						fmat::pack(0,0,sin(atan2(childP[1],childP[0])/2)).exportTo(link->collisionModelRotation);
					link->collisionModel="Cube";
					link->material="Steel";
					link->mass = dim.norm() * distMass;
					components.addEntry(link);
				}
			}
			
		} else {
			// multiple destinations (distinct branch points)
			// make a axis-aligned box encompasing the points

			boundingBox.min = boundingBox.max = 0.f;
			for(branch_iterator it=branches.begin(); it!=branches.end(); ++it) {
				fmat::Column<3> p = ( getFullInvT() * dynamic_cast<Frame*>(*it)->origOrigin ).map(shave);
				boundingBox.expand(p);
			}
			center = (boundingBox.max + boundingBox.min)/2;
			dim = boundingBox.max - boundingBox.min;
			mindim = dim.max()/20;
			boundingBox.expand(fmat::Column<3>(mindim));
			boundingBox.expand(fmat::Column<3>(-mindim));
			dim = boundingBox.max - boundingBox.min;
			
			if(collisionModel.size()==0) {
				link = new LinkComponent;
				center.exportTo(link->collisionModelOffset);
				dim.exportTo(link->collisionModelScale);
				link->collisionModel="Cube";
				link->material="Steel";
				link->mass = dim.norm() * distMass;
				components.addEntry(link);
			}
		}
		
		mindim=shave(mindim);
		if(mindim==0)
			mindim=shave(getTo().translation().norm()/8);
		
		if(model=="CollisionModel" && dim.norm()>EPSILON) {
			center = 0.f;
			dim = dim.norm()/100;
			link = new LinkComponent;
			center.exportTo(link->modelOffset);
			dim.exportTo(link->modelScale);
			link->model="ReferenceFrame";
			link->visible=false;
			components.addEntry(link);
		}
		
		if(isMobile() && collisionModel.size()==0) {
			center = ( getFullInvT() * origOrigin ).map(shave);
			//center[2] -= mindim*2.5;
			dim = mindim*5;
			link = new LinkComponent;
			center.exportTo(link->collisionModelOffset);
			dim.exportTo(link->collisionModelScale);
			link->collisionModel="Cylinder";
			link->mass = baseMass;
			components.addEntry(link);
		}
	}

	// recurse on children
	for(branch_iterator it=branches.begin(); it!=branches.end(); ++it)
		dynamic_cast<Frame&>(**it).makeCollisionModel();
}



float findBottom(const KinematicJoint* kj) {
	const fmat::Transform& wtr = dynamic_cast<const Frame&>(*kj).wtr;
	fmat::Column<3> bbLow = kj->getAABB().min, bbHigh = kj->getAABB().max;
	fmat::Column<3> p = bbLow; fmat::fmatReal low = (wtr * p)[2];
	p[2] = bbHigh[2]; low = min(low,(wtr * p)[2]); // LLH
	p[1] = bbHigh[1]; low = min(low,(wtr * p)[2]); // LHH
	p[2] = bbLow[2]; low = min(low,(wtr * p)[2]); // LHL
	p[0] = bbHigh[0]; low = min(low,(wtr * p)[2]); // HHL
	p[2] = bbHigh[2]; low = min(low,(wtr * p)[2]); // HHH
	p[1] = bbLow[1]; low = min(low,(wtr * p)[2]); // HLH
	p[2] = bbLow[2]; low = min(low,(wtr * p)[2]); // HLL
	for(KinematicJoint::branch_iterator it=kj->getBranches().begin(); it!=kj->getBranches().end(); ++it) {
		float blow = findBottom(*it);
		if(blow<low)
			low=blow;
	}
	return low;
}
