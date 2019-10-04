#include "Motion/KinematicJoint.h"
#include "Motion/IKSolver.h"
#include <cstdlib>
#include <map>
#include <stack>
#include <string>
#include <fstream>

using namespace std;
using namespace fmat;

int usage(const char name[], int exitval) {
	cerr
	<< name << ": kinFile effectorFrame (-r|--res c [w]) [-e|--eff x y z] [-x|--ext x] [-o|--out file]\n"
	<< name << ": kinFile effectorFrame (-w|--wres w) (-b|--bnds lx ly lz hx hy hz) [-e|--eff x y z] [-o|--out file]\n"
	<< name << ": kinFile effectorFrame [-o|--obj x y z] [-a|--ang q1 q2 …] [-e|--eff x y z]\n"
	"\n"
	"  --res  maps the entire configuration space with resolution c, and\n"
	"         then optionally remaps the work space with resolution w\n"
	"\n"
	"  --wres maps the work space at the specified resolution\n"
	"\n"
	"  --bnds specifies the bounds of the work space to map\n"
	"\n"
	"  --obj  sets the objective point for a single solution\n"
	"\n"
	"  --ang  sets the initial joint angles leading up to effectorFrame\n"
	"\n"
	"  --eff  sets an effector offset relative to effectorFrame\n"
	"\n"
	"  --ext  sets how far to extend c-space search beyond joint min and max\n"
	"\n"
	"  --out  specifies a file for the data output, otherwise stdout\n"
	"\n"
	;
	return exitval;
}

const float LARGE_PRISMATIC = 1000;
const float LARGE_REVOLUTE = M_PI*2.5;

void setQ(KinematicJoint* kj, stack<float>& q);
void dumpQ(KinematicJoint* kj);
float readFloat(const char v[]);

void singleSolve(const Column<3>& pEff, KinematicJoint& eff, const Column<3>& pObj);
void mapCSpace(float cRes, const Column<3>& pEff, KinematicJoint& eff, KinematicJoint* fwd, Column<3>& bndLow, Column<3>& bndHigh);
void mapWSpace(float wRes, const Column<3>& pEff, KinematicJoint& eff, Column<3>& bndLow, Column<3>& bndHigh);

struct CData {
	vector<float> qSol;
	Column<3> pObj;
	vector<float> qIK;
	Column<3> pFin;
	float qdist; //!< c-space dist from previous sample
	bool alt; //!< qSol != qIK
	bool converged; //!< return value from IK (true: successful)
	bool invalid; //!< pFin == pObj
	bool discontinuity; //!< previous qIK not close to this qIK
	friend ostream& operator<<(ostream& os, const CData& d) {
		for(vector<float>::const_iterator it=d.qSol.begin(); it!=d.qSol.end(); ++it)
			os << *it << ' ';
		os << "  " << d.pObj[0] << ' ' << d.pObj[1] << ' ' << d.pObj[2] << "  ";
		for(vector<float>::const_iterator it=d.qIK.begin(); it!=d.qIK.end(); ++it)
			os << ' ' << *it;
		os << "   " << d.pFin[0] << ' ' << d.pFin[1] << ' ' << d.pFin[2] << "   ";
		return os << d.qdist << ' ' << d.alt << ' ' << d.converged << ' ' << d.invalid << ' ' << d.discontinuity;
	}
};
vector<CData> cdata;

struct WData {
	Column<3> pObj;
	vector<float> qIK;
	Column<3> pFin;
	float qdist; //!< c-space dist from previous sample
	bool converged; //!< return value from IK (true: successful)
	bool invalid; //!< pFin == pObj
	bool discontinuity; //!< previous qIK not close to this qIK
	friend ostream& operator<<(ostream& os, const WData& d) {
		os << d.pObj[0] << ' ' << d.pObj[1] << ' ' << d.pObj[2] << "  ";
		for(vector<float>::const_iterator it=d.qIK.begin(); it!=d.qIK.end(); ++it)
			os << ' ' << *it;
		os << "   " << d.pFin[0] << ' ' << d.pFin[1] << ' ' << d.pFin[2] << "   ";
		return os << d.qdist << ' ' << d.converged << ' ' << d.invalid << ' ' << d.discontinuity;
	}
};
vector<WData> wdata;

static float ext=0;
static stack<float> q;

int main(int argc, const char* argv[]) {
	try {
		
		// *** PARSE COMMAND LINE ARGUMENTS *** //
		
		if(argc<4) {
			return usage(argv[0],2);
		}
		
		size_t argi=1;
		string kinFile = argv[argi++];
		string effName = argv[argi++];
		float cRes=0, wRes=0;
		Column<3> pObj, pEff;
		Column<3> bndLow, bndHigh;
		string outFile;
	
		bool hasObj=false;
		while(argi!=argc) {
			string flag=argv[argi++];
			if(flag=="-r" || flag=="--res") {
				if(argi==argc) 
					throw invalid_argument(flag+" requires configuration space resolution value");
				cRes = readFloat(argv[argi++]);
				if(argi<argc) try {
					wRes = readFloat(argv[argi]);
					++argi;
				} catch(const exception& ex) {}
			} else if(flag=="-w" || flag=="--wres") {
				if(argi==argc) 
					throw invalid_argument(flag+" requires work space resolution value");
				wRes = readFloat(argv[argi++]);
			} else if(flag=="-o" || flag=="--out") {
				if(argi==argc) 
					throw invalid_argument(flag+" requires output file name");
				outFile = argv[argi++];
			} else if(flag=="-b" || flag=="--bnds") {
				if(argi+6>argc) 
					throw invalid_argument(flag+" requires six values for bounding box to map");
				bndLow = fmat::pack(readFloat(argv[argi]),readFloat(argv[argi+1]),readFloat(argv[argi+2]));
				argi+=3;
				bndHigh = fmat::pack(readFloat(argv[argi]),readFloat(argv[argi+1]),readFloat(argv[argi+2]));
				argi+=3;
			} else if(flag=="-o" || flag=="--obj") {
				if(argi+3>argc) 
					throw invalid_argument(flag+" requires x y z values");
				pObj = fmat::pack(readFloat(argv[argi]),readFloat(argv[argi+1]),readFloat(argv[argi+2]));
				argi+=3;
				hasObj=true;
			} else if(flag=="-a" || flag=="--ang") {
				for(;argi<argc;++argi) {
					try {
						q.push(readFloat(argv[argi]));
					} catch(...) {
						break;
					}
				}
			} else if(flag=="-x" || flag=="--ext") {
				if(argi==argc) 
					throw invalid_argument(flag+" requires value to extend search");
				ext = readFloat(argv[argi++]);
			} else if(flag=="-e" || flag=="--eff") {
			} else {
				throw invalid_argument("Unknown argument: "+flag);
			}
		}

		// *** LOAD KINEMATICS FILE *** //
	
		KinematicJoint root;
		if(!root.loadFile(kinFile.c_str()))
			throw runtime_error("Could not load kinematics file "+kinFile);
		map<unsigned int, KinematicJoint*> childMap;
		root.buildChildMap(childMap,0,-1U);
	
	
		// *** PROCESS KINEMATICS *** //
	
		// Get effector joint (and do sanity checking)
		KinematicJoint& eff = *childMap[capabilities.getOutputOffset(effName)];
		if(&eff == NULL)
			throw runtime_error("Requested effector "+effName+" has unknown kinematics (not found in "+kinFile+")");
		
		if(cRes<=0 && wRes<=0) {
			// Single kinematics query
			
			// Setup effector and objective positions
			setQ(&eff,q);
			if(!hasObj)
				pObj = eff.getWorldPosition();
			
			singleSolve(pEff, eff, pObj);

		} else {
			// Map configuration space
			if(cRes>0) {
				auto_ptr<KinematicJoint> fwd(eff.cloneBranch());
				mapCSpace(cRes, pEff, eff, fwd.get(), bndLow, bndHigh);
				// expand bounds a bit
				bndLow-=wRes;
				bndHigh+=wRes;
			}
			
			if(wRes>0) {
				mapWSpace(wRes, pEff, eff, bndLow, bndHigh);
				ofstream of(outFile.c_str());
				ostream& os = (outFile.size()>0) ? of : cout;
				for(vector<WData>::const_iterator it = wdata.begin(); it!=wdata.end(); ++it)
					os << *it << '\n';
			} else {
				ofstream of(outFile.c_str());
				ostream& os = (outFile.size()>0) ? of : cout;
				for(vector<CData>::const_iterator it = cdata.begin(); it!=cdata.end(); ++it)
					os << *it << '\n';
			}
			
		}
		
	} catch(const exception& ex) {
		cerr << ex.what() << endl;
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

void setQ(KinematicJoint* kj, stack<float>& q) {
	if(q.empty()) {
		return;
	} else if(kj==NULL) {
		throw runtime_error("Extra q value(s)");
	} else if(!kj->isMobile()) {
		setQ(kj->getParent(),q);
	} else {
		kj->setQ(q.top());
		q.pop();
		setQ(kj->getParent(),q);
	}
}

void dumpQ(KinematicJoint* kj) {
	if(kj==NULL)
		return;
	dumpQ(kj->getParent());
	if(kj->isMobile())
		cout << ' ' << kj->getQ();
}

float readFloat(const char v[]) {
	char* endp;
	float x=strtof(v,&endp);
	if(endp==v || *endp!='\0')
		throw invalid_argument(string("Bad floating point value ").append(v));
	return x;
}

void singleSolve(const Column<3>& pEff, KinematicJoint& eff, const Column<3>& pObj) {
	IKSolver& solver = eff.getIK();

	cout << "Moving " << pEff << " on " << eff.outputOffset.get() << endl;
	cout << "From " << eff.getWorldPosition() << " to " << pObj << endl;
	cout << "Using solver " << string_util::demangle(typeid(solver).name()) << endl;
	cout << "Initial q:";
	dumpQ(&eff);
	cout << endl;

	bool converged = solver.solve(pEff,eff,IKSolver::Point(pObj));
	
	cout << (converged?"Success":"Failure") << endl;
	cout << "Final position: " << eff.getWorldPosition() << " error " << eff.getWorldPosition()-pObj << endl;
	cout << "Final q:";
	dumpQ(&eff);
	cout << endl;
}

void getQBack(KinematicJoint& kj, vector<float>& q) {
	if(kj.getParent())
		getQBack(*kj.getParent(),q);
	if(kj.isMobile())
		q.push_back(kj.getQ());
}

KinematicJoint* getQFwd(KinematicJoint& kj, vector<float>& q) {
	KinematicJoint* it=&kj;
	if(it->isMobile())
		q.push_back(it->getQ());
	while(it->nextJoint()) {
		it=it->nextJoint();
		if(it->isMobile())
			q.push_back(it->getQ());
	}
	return it;
}

void mapCSpace(float cRes, const Column<3>& pEff, KinematicJoint& eff, KinematicJoint* fwd, Column<3>& bndLow, Column<3>& bndHigh) {
	KinematicJoint* mob = fwd;
	while(mob!=NULL && !mob->isMobile())
		mob=mob->getParent();
	if(mob==NULL) {
		cdata.push_back(CData());
		CData& d = cdata.back();
		Column<3> x = getQFwd(*fwd,d.qSol)->getWorldPosition();
		d.pObj = x;
		if(x[0]<bndLow[0])
			bndLow[0]=x[0];
		else if(x[0]>bndHigh[0])
			bndHigh[0]=x[0];
		if(x[1]<bndLow[1])
			bndLow[1]=x[1];
		else if(x[1]>bndHigh[1])
			bndHigh[1]=x[1];
		if(x[2]<bndLow[2])
			bndLow[2]=x[2];
		else if(x[2]>bndHigh[2])
			bndHigh[2]=x[2];
		try {
			d.converged = eff.getIK().solve(pEff,eff,IKSolver::Point(x));
		} catch(...) {
			cerr << "Exception solving for " << x << endl;
			throw;
		}
		getQBack(eff,d.qIK);
		d.pFin = eff.getWorldPosition();
		d.invalid = (d.pObj - d.pFin).norm() > 0.1f;
		d.alt = false;
		d.qdist=0;
		d.discontinuity = false;
		if(cdata.size()>1 && cdata[cdata.size()-2].qSol[0]<d.qSol[0]) {
			CData& last = cdata[cdata.size()-2];
			// check whether any of the joints moved more than would be expected
			// "expected" depends on distance from that joint...
			for(size_t i=d.qIK.size()-1; i!=(size_t)-1; --i) {
				float err = std::abs(d.qIK[i]-last.qIK[i]);
				d.qdist += err*err;
				float maxexp = cRes*2;
				if(err > maxexp) {
					d.discontinuity=true;
				}
			}
			d.qdist=std::sqrt(d.qdist);
		} else {
			for(size_t i=0; i<d.qIK.size(); ++i) {
				if(std::abs(d.qIK[i]-d.qSol[i])>0.005f) {
					d.alt=true;
					break;
				}
			}
		}
	} else {
		float inc=cRes;
		float qmin=mob->qmin;
		float qmax=mob->qmax;
		float qext=ext;
		if(mob->jointType==KinematicJoint::PRISMATIC) {
			if(!isfinite(qmin))
				qmin=-LARGE_PRISMATIC; // override to a "large number"
			if(!isfinite(qmax))
				qmax=LARGE_PRISMATIC; // override to a "large number"
			inc = (qmax-qmin) / (M_PI*2/cRes); // convert to linear increments
			qext = (qmax-qmin) / (M_PI*2/ext);
		} else {
			// if infinite bounds (some kind of wheel?) test for finite amount of wrap-around
			if(!isfinite(qmin))
				qmin=-LARGE_REVOLUTE;
			if(!isfinite(qmax))
				qmax=LARGE_REVOLUTE;
		}
		for(float q=qmin-qext; q<qmax+qext; q+=inc) {
			mob->setQ(q);
			mapCSpace(cRes,pEff,eff,mob->getParent(),bndLow,bndHigh);
		}
	}
}

void mapWSpace(float wRes, const Column<3>& pEff, KinematicJoint& eff, Column<3>& bndLow, Column<3>& bndHigh) {
	cout << "Workspace low bounds: " << bndLow << endl;
	cout << "Workspace high bounds: " << bndHigh << endl;
	for(float x=bndLow[0]; x<bndHigh[0]; x+=wRes) {
		for(float y=bndLow[1]; y<bndHigh[1]; y+=wRes) {
			for(float z=bndLow[2]; z<bndHigh[2]; z+=wRes) {
				wdata.push_back(WData());
				WData& d = wdata.back();
				d.pObj = pack(x,y,z);
				setQ(&eff,q);
				try {
					d.converged = eff.getIK().solve(pEff,eff,IKSolver::Point(d.pObj));
				} catch(...) {
					cerr << "Exception solving for " << x << endl;
					throw;
				}
				getQBack(eff,d.qIK);
				d.pFin = eff.getWorldPosition();
				d.invalid = (d.pObj - d.pFin).norm() > 0.1f;
				d.qdist=0;
				d.discontinuity = false;
				if(z!=bndLow[2]) {
					// check whether any of the joints moved more than would be expected
					// "expected" depends on distance from that joint...
					WData& last = wdata[wdata.size()-2];
					KinematicJoint * cur=&eff;
					for(size_t i=d.qIK.size()-1; i!=(size_t)-1; --i) {
						while(!cur->isMobile())
							cur=cur->getParent();
						float err = std::abs(d.qIK[i]-last.qIK[i]);
						d.qdist += err*err;
						float maxexp = atan2(wRes, (d.pFin - cur->getWorldPosition()).norm())*5;
						if(err > maxexp) {
							d.discontinuity=true;
						}
						cur = cur->getParent();
					}
					d.qdist=std::sqrt(d.qdist);
				}
			}
		}
	}
}
