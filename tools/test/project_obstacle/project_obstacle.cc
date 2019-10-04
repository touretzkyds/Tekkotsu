#include "Motion/KinematicJoint.h"
#include "Planners/PlannerObstacles.h"
#include <cmath>

void testBB(); // does some tests on KinematicJoint::getAABB

// tests some code for finding a 2D bounding box for the 3D collision shape
void projectCollisionModel(const fmat::Transform& worldT, const LinkComponent& lc, RectangularObstacle& r);

// these do display/visualization of projectCollisionModel
void displayKJ(const fmat::Transform& worldT, const LinkComponent& lc);
void displayProjection(const RectangularObstacle& r);
#ifdef HAVE_CAIRO
void makeMovie(const KinematicJoint& lc);
#endif

int main(int argc, char** argv) {
	std::cout << std::setprecision(5);
	testBB();
	
	KinematicJoint kj;
	kj.loadFile("object.kin");
	
	fmat::Transform basePose; // position of base in world...
	basePose.rotation() = fmat::rotationZ(30*M_PI/180) * fmat::rotationY(20*M_PI/180) * fmat::rotationX(10*M_PI/180);
	basePose.translation() = fmat::pack(15,30,0);
	fmat::Transform linkPose = basePose * kj.getFullT(); // world to base, base to link
	
	RectangularObstacle r;
	
	// just using the KJ's own collision model (ignoring components array)
	kj.getBB2D(linkPose,r);
	
	// display results
	std::cout << "object.kin @ \n" << basePose << "\nCorners:\n";
	displayKJ(linkPose,kj);
	std::cout << "\nBounding box corners:\n";
	displayProjection(r);

#ifdef HAVE_CAIRO
	if(argc>1) {
		makeMovie(kj);
		std::cout << "To stitch the frames into a movie, try:\n"
			"ffmpeg -f image2 -r 50 -i 'frames/%06d.png' -b 2000k -y frames.mp4" << std::endl;
	} else {
		std::cout << "@VAR projectCollisionModel movie generation available, pass an argument to trigger." << std::endl;
	}
#else
	std::cout << "@VAR Cairo not found, projectCollisionModel movie generation disabled." << std::endl;
#endif
}

void displayBB(const std::string& label, const fmat::Column<3>& low, const fmat::Column<3>& high) {
	std::cout << label << " Low: " << low << '\n';
	std::cout << label << " High: " << high << '\n';
	std::cout << label << " for plotting:\n";
	std::cout << std::setw(10) << low[0] << ' ' << std::setw(10) << low[1] << ' ' << std::setw(10) << low[2] << '\n';
	std::cout << std::setw(10) << low[0] << ' ' << std::setw(10) << high[1] << ' ' << std::setw(10) << low[2] << '\n';
	std::cout << std::setw(10) << high[0] << ' ' << std::setw(10) << high[1] << ' ' << std::setw(10) << low[2] << '\n';
	std::cout << std::setw(10) << high[0] << ' ' << std::setw(10) << low[1] << ' ' << std::setw(10) << low[2] << '\n';
	std::cout << std::setw(10) << low[0] << ' ' << std::setw(10) << low[1] << ' ' << std::setw(10) << low[2] << '\n';
	std::cout << std::endl;
}

void testBB() {
	KinematicJoint kj;
	kj.collisionModel="Cube";
	kj.collisionModelScale.set(100, 50, 25);
	kj.collisionModelRotation.set(0.3, 0.2, 0.3);
	kj.collisionModelOffset.set(40,20,0);
	std::cout << "KJ:\n";
	displayKJ(fmat::Transform::IDENTITY, kj);
	
	LinkComponent * lc = new LinkComponent;
	lc->collisionModel="Cube";
	lc->collisionModelScale.set(50, 25, 25);
	lc->collisionModelRotation.set(-0.3, -0.2, -0.3);
	lc->collisionModelOffset.set(80,20,0);
	std::cout << "LC:\n";
	displayKJ(fmat::Transform::IDENTITY, *lc);
	std::cout << '\n';
	
	BoundingBox3D bb = kj.getAABB();
	displayBB("KJ",bb.min,bb.max);
	
	bb = lc->getAABB();
	displayBB("LC",bb.min,bb.max);
	
	kj.components.addEntry(lc);
	bb = kj.getAABB();
	displayBB("KJ+LC",bb.min,bb.max);
}

template<class T> void disp2(const T& x) {
	std::cout << std::setw(10) << x[0] << ' ' << std::setw(10) << x[1] << '\n';
}

// display the 8 corner points
void displayKJ(const fmat::Transform& worldT, const LinkComponent& lc) {
	fmat::Transform obT;
	lc.getCollisionModelTransform(obT);
	fmat::Column<3> obD;
	obD.importFrom(lc.collisionModelScale);
	
	fmat::Transform fullT = worldT * obT;
	
	disp2(fullT * (fmat::pack( obD[0], obD[1], obD[2])/2));
	disp2(fullT * (fmat::pack( obD[0], obD[1],-obD[2])/2));
	disp2(fullT * (fmat::pack( obD[0],-obD[1], obD[2])/2));
	disp2(fullT * (fmat::pack( obD[0],-obD[1],-obD[2])/2));
	disp2(fullT * (fmat::pack(-obD[0], obD[1], obD[2])/2));
	disp2(fullT * (fmat::pack(-obD[0], obD[1],-obD[2])/2));
	disp2(fullT * (fmat::pack(-obD[0],-obD[1], obD[2])/2));
	disp2(fullT * (fmat::pack(-obD[0],-obD[1],-obD[2])/2));
}

void displayProjection(const RectangularObstacle& r) {
	for(size_t i=0; i<4; ++i) {
		disp2(r.getCorner(static_cast<RectangularObstacle::CornerOrder>(i)));
	}
	// repeat to close the box when graphed
	disp2(r.getCorner(static_cast<RectangularObstacle::CornerOrder>(0)));
}
