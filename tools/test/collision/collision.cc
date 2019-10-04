#include "Shared/Draw.h"
#include "Planners/PlannerObstacles.h"
#include "Shared/plistSpecialty.h"
#include "Shared/TimeET.h"
#include <vector>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h>

using namespace fmat;
using namespace std;

float IMGDIM=500;
bool pngOutput=false;
std::string imgOutputPrefix;
bool extendedTests=false;
size_t pageCnt=0;

int usage(const std::string& name, const std::string& msg="") {
	if(msg.size()>0)
		std::cerr << msg << std::endl;
	std::cout << "Usage: " << name << " [-x|--extended] [--dim size] [--png] [--prefix img-prefix]" << std::endl;
	return msg.size()>0 ? EXIT_FAILURE : 2;
}

struct TestSetup : public plist::Dictionary {
	plist::Primitive<size_t> visSamples;
	plist::Primitive<size_t> testSamples;
	plist::Angle reposRotationAngle;
	plist::ArrayOf<plist::Primitive<float> > reposRotationCenter;
	plist::ArrayOf<plist::Primitive<float> > reposOffset;
	plist::Primitive<size_t> reposSteps;
	plist::ArrayOf<PlannerObstacle2D> inner;
	plist::DictionaryOf<PlannerObstacle2D> outer;
	TestSetup()
		: visSamples(50), testSamples(1000),
		reposRotationAngle(0), reposRotationCenter(2,0,false), reposOffset(2,0,false), reposSteps(10),
		inner(), outer()
	{
		addEntry("TestSamples", testSamples, "Samples used for extended testing (per axis, so squared)");
		addEntry("VisSamples", visSamples, "Samples used for visualization (per axis, so squared)");
		addEntry("ReposRotationAngle", reposRotationAngle, "For reposition test, angle to be applied to each element of outer");
		addEntry("ReposRotationCenter", reposRotationCenter, "For reposition test, center of rotation to be applied to each element of outer");
		addEntry("ReposOffset", reposOffset, "For reposition test, offset to be applied to each element of outer");
		addEntry("ReposSteps", reposSteps, "Number of steps to be taken over the course of ReposRotationAngle and reposOffset");
		addEntry("Inner", inner, "The obstacles which will be moved around (“inner loop”)");
		addEntry("Outer", outer, "The obstacles which will hold a static position (“outer loop”)");
	}
	PLIST_CLONE_DEF(TestSetup, new TestSetup(*this));
};
PLIST_CLONE_IMP(TestSetup, new TestSetup(*this));

void rectTest();
void convexHullTest();
ConvexPolyObstacle& randomHull(ConvexPolyObstacle& poly, fmat::Column<2> c, float w, float h);
void startPage(Draw& d, const std::string& base, const std::string& test);
void endPage(Draw& d);

int main(int argc, const char* argv[]) {
	//srandomdev();
	srandom(81487); // just some consistent random sequence (if there is a problem, want to reproduce it)
	
	for(int a=1; a<argc; ++a) {
		std::string s = argv[a];
		if(s=="-x" || s=="--extended") {
			extendedTests = true;
		} else if(s=="--dim") {
			if(++a==argc)
				return usage(argv[0],"--prefix requires an argument");
			std::stringstream(argv[a]) >> IMGDIM;
		} else if(s=="--png") {
			pngOutput=true;
		} else if(s=="--prefix") {
			if(++a==argc)
				return usage(argv[0],"--prefix requires an argument");
			imgOutputPrefix = argv[a];
		} else if(s=="-h" || s=="--help") {
			return usage(argv[0]);
		}
	}
	std::cout << "Running visual tests, verify image files after execution." << std::endl;
	if(!extendedTests) {
		std::cout << "Pass -x or --extended to enable additional testing" << std::endl;
	}
	
	/* Programmatically generate shape test data, might just stick with this
	 * or might switch to load/save from a config file (also test serialization?) */
	plist::ArrayOf<TestSetup> tests;
	{
		fmat::Column<2> c=fmat::pack(1,2);
		fmat::Column<2> ex=fmat::pack(1.5,0.7);
		fmat::fmatReal ori = 23*M_PI/180;
		TestSetup * t = new TestSetup;
		
		t->reposRotationAngle = -90*M_PI/180;
		t->reposRotationCenter[0] = 1.8;
		t->reposRotationCenter[1] = 1.3;
		t->reposOffset[0] = .75;
		t->reposOffset[1] = .5;
		
		t->outer.addEntry("Circle",new CircularObstacle(c,ex[1]*2));
		t->outer.addEntry("CircEllipse",new EllipticalObstacle(c, ex[1]*2));
		t->outer.addEntry("Ellipse",new EllipticalObstacle(c, ex[1], ex[0], ori+M_PI_2));
		t->outer.addEntry("PolyRand",&randomHull(*new ConvexPolyObstacle,c,ex[0]*2,ex[0]*2));
		
		RectangularObstacle * rect = new RectangularObstacle(c, ex, ori);
		t->outer.addEntry("Rectangle",rect);
		
		ConvexPolyObstacle * poly = new ConvexPolyObstacle;
		for(size_t i=0; i<RectangularObstacle::NUM_CORNERS; ++i)
			poly->addPoint(rect->getCorner(i));
		poly->close();
		t->outer.addEntry("PolyRect",poly);
		
		HierarchicalObstacle * ho1 = new HierarchicalObstacle;
		ho1->rotate(fmat::rotation2D(M_PI/8));
		ho1->updatePosition(fmat::pack(0.2,0.2));
		RectangularObstacle * hor = new RectangularObstacle(c,fmat::pack(1.2,1),ori);
		hor->name = "Rect";
		ho1->add(hor);
		CircularObstacle * hoc = new CircularObstacle((hor->getCorner(RectangularObstacle::TOP_LEFT) + hor->getCorner(RectangularObstacle::BOTTOM_LEFT))/2,1);
		hoc->name = "Circ";
		ho1->add(hoc);
		t->outer.addEntry("Hierarchical",ho1);
		
		RectangularObstacle * rect2 = new RectangularObstacle(c, ex/3, ori-80*M_PI/180);
		t->inner.addEntry(rect2);
		ConvexPolyObstacle * poly2 = new ConvexPolyObstacle;
		for(size_t i=0; i<RectangularObstacle::NUM_CORNERS; ++i)
			poly2->addPoint(rect2->getCorner(i));
		poly2->close();
		t->inner.addEntry(poly2);
		t->inner.addEntry(new CircularObstacle(c,ex[1]/2));
		t->inner.addEntry(new EllipticalObstacle(c,ex[0]/3,ex[1]/3,ori));
		HierarchicalObstacle * ho2 = new HierarchicalObstacle;
		ho2->rotate(fmat::rotation2D(M_PI/8));
		ho2->updatePosition(fmat::pack(0.2,0.2));
		RectangularObstacle * hor2 = new RectangularObstacle(c,fmat::pack(0.6,0.5),ori);
		hor2->name = "Rect";
		ho2->add(hor2);
		CircularObstacle * hoc2 = new CircularObstacle((hor2->getCorner(RectangularObstacle::TOP_LEFT) + hor2->getCorner(RectangularObstacle::BOTTOM_LEFT))/2, 0.5);
		hoc2->name = "Circ";
		ho2->add(hoc2);
		t->inner.addEntry(ho2);
		tests.addEntry(t);
		//tests.saveFile("tests.plist");
	}
	
	// Quick BB rotation test
	{
		float rot = 30*M_PI/180;
		BoundingBox2D bb(fmat::pack(1,3), fmat::pack(8,8));
		RectangularObstacle ro(bb.getCenter(), bb.getDimensions()/2, rot);
		bb.rotate(fmat::rotation2D(rot));
		if(ro.getBoundingBox().min != bb.min || ro.getBoundingBox().max != bb.max) {
			std::cerr << "Rotated bounding box does not match rectangular obstacle bounding box" << std::endl;
			std::cerr << "Bounding box: " << bb << std::endl;
			std::cerr << "Rectangular obstacle:\n" << ro << "\nbb " << ro.getBoundingBox() << std::endl;
		}
	}
		
	
	
	/* Now run the main collision tests!!! */
	
	for(size_t test=0; test<tests.size(); ++test) {
		if(tests.size()>0)
			std::cout << "Test " << test << ":" << std::endl;
		for(plist::DictionaryOf<PlannerObstacle2D>::const_iterator oit=tests[test].outer.begin(); oit!=tests[test].outer.end(); ++oit) {
			PlannerObstacle2D& outer = *oit->second;
			BoundingBox2D bb = outer.getBoundingBox();
			float range = bb.getDimensions().max()*1.5;
			bb = BoundingBox2D(bb.getCenter()-range/2,bb.getCenter()+range/2);
			
			// High resolution tests for precision and performance testing
			const fmat::fmatReal TRES = range/tests[test].testSamples;
			const size_t TCOLS = bb.getDimension(0) / TRES;
			const double TEDGE = TRES / std::sqrt(3.0);
			const size_t TROWS = std::ceil(bb.getDimension(1) / (TEDGE*3/2));
			
			// Run tests at visual sampling res for display
			const fmat::fmatReal VRES = range/tests[test].visSamples;
			const size_t VCOLS = std::ceil(bb.getDimension(0) / VRES) + 1;
			const double VEDGE = VRES / std::sqrt(3.0);
			const size_t VROWS = std::ceil(bb.getDimension(1) / (VEDGE*3/2));
			
			Draw d;
			if(!pngOutput)
				d.setFile(imgOutputPrefix+oit->first+".pdf", IMGDIM, IMGDIM);
			pageCnt=1;
			
			/**** Page 1: Display bounding box and center ****/
			startPage(d, oit->first, "bb");
			d.setScale(IMGDIM/range);
			d.centerView(bb.getCenter());
			
			d.setStrokeWidth(0.5);
			d.setColor(Draw::BLACK, 0.5);
			// draw origin axes (might not be visible...)
			d.strokeLine(0, bb.getCenter()[1]-range/2, 0, bb.getCenter()[1]+range/2);
			d.strokeLine(bb.getCenter()[0]-range/2, 0, bb.getCenter()[0]+range/2, 0);
			// shade bounding box
			d.rect(outer.getBoundingBox()).fill(Draw::RED,0.4);
			// draw shape itself
			d.setStrokeWidth(2);
			// cairo pdf clipping bug - order matters, clip before fill or clip is lost ( https://bugs.freedesktop.org/show_bug.cgi?id=39653 )
			d.draw(outer).setClip().fill(Draw::WHITE).stroke(Draw::BLACK).clearClip();
			// draw center marker
			double markSize = d.displayToScaled(IMGDIM/40);
			fmat::Column<2> c = outer.getCenter();
			d.holdPath();
			d.rect(c, c+markSize*2).rect(c,c-markSize*2).setClip();
			d.releasePath();
			d.circle(c, markSize).fill(Draw::GRAY);
			d.clearClip();
			d.setScaledStrokeWidth(markSize/3.5);
			d.stroke(Draw::GRAY).stroke(); // cairo pdf clipping bug - need extra stroke ( https://bugs.freedesktop.org/show_bug.cgi?id=39654 )
			//d.circle(c, markSize/2).fill(Draw::GRAY);
			//d.setColor(Draw::BLACK).setStrokeWidth(0.5);
			//d.strokeLine(c[0]-markSize*2,c[1],c[0]+markSize*2,c[1]);
			//d.strokeLine(c[0],c[1]-markSize*2,c[0],c[1]+markSize*2);
			endPage(d);
			
			
			/**** Page 2: Clone and reposition test ****/
			{
				/* This is a whole-lotta-math to find the rotation AND offset STEPS which
				 * reproduce the requested angle and offset.  The downside is that the
				 * actual rotation center applied at each step is not the 'requested' one,
				 * but these individual steps are verified against the full transformation
				 * as a single step at the end (yellow highlight should overlap black border) */
				fmat::Column<2> reqCenter; reqCenter.importFrom(tests[test].reposRotationCenter);
				fmat::Column<2> reqOffset; reqOffset.importFrom(tests[test].reposOffset);
				fmat::Matrix<2,2> totalRotation = fmat::rotation2D(tests[test].reposRotationAngle);
				fmat::Column<2> totalOffset = totalRotation * -reqCenter + reqCenter + reqOffset;
				fmat::Column<2> totalCenter = invert(fmat::Matrix<2,2>::identity() - totalRotation) * totalOffset;
				fmat::Matrix<2,2> stepRotation = fmat::rotation2D(tests[test].reposRotationAngle/tests[test].reposSteps);
				fmat::Column<2> stepOffset = stepRotation * -totalCenter + totalCenter - reqOffset/tests[test].reposSteps;
				fmat::Column<2> stepCenter = invert(fmat::Matrix<2,2>::identity() - stepRotation) * stepOffset;
				
				startPage(d, oit->first, "repos");
				d.setScale(IMGDIM/(range+(outer.getCenter()-totalCenter).norm()));
				//d.centerView(fmat::rotation2D(tests[test].reposRotationAngle/2)*(outer.getCenter()-totalCenter)+totalCenter);
				d.centerView((outer.getCenter() + totalRotation*outer.getCenter()+totalOffset)/2);
				
				// show motion of obstacle about totalCenter
				PlannerObstacle2D * o = outer.clone();
				d.setStrokeWidth(2);
				d.draw(*o).setClip().fill(Draw::WHITE).stroke(Draw::BLACK).clearClip();
				for(unsigned int i=0; i<tests[test].reposSteps; ++i) {
					o->rotate(stepCenter, stepRotation);
					o->updatePosition(o->getCenter() + reqOffset/tests[test].reposSteps);
					d.draw(*o).setClip().fill(Draw::WHITE,0.4).stroke(Draw::BLACK).clearClip();
				}
				
				// verify bounding box has kept up...
				d.rect(o->getBoundingBox()).fill(Draw::RED,0.4);
				// redraw initial pose to verify clone and provide visual reference of total rotation
				d.setStrokeWidth(3);
				d.draw(outer).stroke(Draw::YELLOW,0.5);
				// redraw hilighted outline on top to verify against single-step transform
				d.setStrokeWidth(3);
				d.draw(*o).fill(Draw::WHITE).stroke(Draw::YELLOW);
				delete o;
				o = outer.clone();
				o->rotate(reqCenter, totalRotation);
				o->updatePosition(o->getCenter() + reqOffset);
				d.setStrokeWidth(2);
				d.draw(*o).setClip().fill(Draw::LT_GRAY).stroke(Draw::BLACK).clearClip();
				
				// show rotate and then offset in blue
				d.setColor(Draw::BLUE);
				d.setStrokeWidth(1);
				d.strokeLine(reqCenter,outer.getCenter());
				d.strokeLine(reqCenter,totalRotation * (outer.getCenter()-reqCenter) + reqCenter);
				d.setStrokeWidth(2);
				d.holdPath();
				fmat::Column<2> c1 = outer.getCenter() - reqCenter;
				d.arc(reqCenter,c1.norm(),fmat::atan(c1),fmat::atan(c1) + tests[test].reposRotationAngle);
				d.lineTo(totalRotation * outer.getCenter() + totalOffset);
				d.releasePath().stroke();
				d.point(reqCenter, 6).fill();
				
				// show equivalent single-step transformation in black
				d.setColor(Draw::BLACK);
				d.setStrokeWidth(1);
				d.strokeLine(totalCenter,outer.getCenter());
				d.strokeLine(totalCenter,totalRotation * (outer.getCenter()-totalCenter) + totalCenter);
				d.setStrokeWidth(2);
				fmat::Column<2> c2 = outer.getCenter() - totalCenter;
				d.arc(totalCenter,c2.norm(),fmat::atan(c2),fmat::atan(c2)+tests[test].reposRotationAngle).stroke();
				d.point(totalCenter, 6).fill();
				
				// indicate center points
				d.setColor(Draw::RED).setStrokeWidth(4);
				d.point(outer.getCenter(),10).fill(); // begin point
				d.point(totalRotation * outer.getCenter() + totalOffset,10).fill(); // expected point
				d.point(o->getCenter(),18).stroke(); // actual point
				
				//d.point(stepCenter,10).fill(Draw::MAGENTA);
				
				endPage(d);
				delete o;
			}
			
			
			/**** Page 3: Point collision tests ****/
			if(extendedTests) {
				TimeET time;
				size_t hits=0;
				for(size_t i=0; i<TROWS; ++i) {
					fmat::fmatReal y = i*(TEDGE*3/2) + bb.min[1];
					for(size_t j=0; j<TCOLS; ++j) {
						fmat::Column<2> p = fmat::pack(j*TRES - TRES + bb.min[0], y);
						if(i&1)
							p[0]+=TRES/2;
						if(outer.collides(p))
							++hits;
					}	
				}
				time = time.Age();
				std::cout << "Test " << test << " " << std::setw(16) << (oit->first+": ") 
					<< std::setw(8) << TROWS*TCOLS << " points, "
					<< std::setw(7) << hits << " hits,  @VAR time " << time << std::endl;
			}
			
			std::vector<fmat::Column<2> > hit, miss;
			hit.reserve(VROWS*VCOLS);
			miss.reserve(VROWS*VCOLS);
			for(size_t i=0; i<VROWS; ++i) {
				fmat::fmatReal y = i*(VEDGE*3/2) + bb.min[1];
				for(size_t j=0; j<VCOLS; ++j) {
					fmat::Column<2> p = fmat::pack(j*VRES + bb.min[0], y);
					if(i&1)
						p[0]+=VRES/2;
					if(outer.collides(p)) {
						hit.push_back(p);
					} else {
						miss.push_back(p);
					}
				}	
			}
			
			// display point results
			startPage(d, oit->first, "points");
			d.setScale(IMGDIM/range);
			d.centerView(bb.getCenter());
			
			double pointSize = d.scaledToDisplay(VRES)/2;
			d.setStrokeWidth(1);
			
			for(size_t i=0; i<hit.size(); ++i)
				d.point(hit[i], pointSize).fill(Draw::RED);
			for(size_t i=0; i<miss.size(); ++i)
				d.point(miss[i], pointSize).fill(Draw::GREEN,0.1666666);
			
			d.draw(outer).setClip().fill(Draw::GRAY);
			
			for(size_t i=0; i<hit.size(); ++i)
				d.point(hit[i], pointSize).fill(Draw::RED,0.1666666);
			for(size_t i=0; i<miss.size(); ++i)
				d.point(miss[i], pointSize).fill(Draw::GREEN);
			
			d.clearClip();
			
			d.setColor(Draw::BLACK,0.5);
			for(size_t i=0; i<hit.size(); ++i)
				d.point(hit[i], pointSize/6).fill();
			for(size_t i=0; i<miss.size(); ++i)
				d.point(miss[i], pointSize/6).fill();
			endPage(d);
			
			
			/**** Page 4: Gradient tests ****/
			if(extendedTests) {
				TimeET time;
				double gradLen=0, gradLen2=0;
				for(size_t i=0; i<TROWS; ++i) {
					fmat::fmatReal y = i*(TEDGE*3/2) + bb.min[1];
					for(size_t j=0; j<TCOLS; ++j) {
						fmat::Column<2> p = fmat::pack(j*TRES - TRES + bb.min[0], y);
						if(i&1)
							p[0]+=TRES/2;
						fmat::Column<2> x = outer.gradient(p);
						gradLen += x.norm();
						gradLen2 += outer.gradient(p+x).norm();
					}	
				}
				time = time.Age();
				std::cout << "Test " << test << " " << std::setw(16) << (oit->first+": ") 
				<< std::setw(8) << TROWS*TCOLS << " points, "
				<< std::setw(4) << gradLen2 << " error, total gradient " << std::setw(4) << gradLen
				<< ", @VAR time " << time << std::endl;
			}
			
			std::vector<fmat::Column<2> > hitGradients, missGradients;
			hitGradients.reserve(hit.size());
			missGradients.reserve(miss.size());
			for(size_t i=0; i<hit.size(); ++i)
				hitGradients.push_back(hit[i]+outer.gradient(hit[i]));
			for(size_t i=0; i<miss.size(); ++i)
				missGradients.push_back(miss[i]+outer.gradient(miss[i]));
			
			// display gradient results
			startPage(d, oit->first, "grad");
			d.setStrokeWidth(1);
			d.setColor(Draw::RED);
			for(size_t i=0; i<hit.size(); ++i)
				d.strokeLine(hit[i], hitGradients[i]);
			d.draw(outer).fill(Draw::GRAY,0.5);
			d.setColor(Draw::GREEN);
			for(size_t i=0; i<miss.size(); ++i)
				d.strokeLine(miss[i], missGradients[i]);
			d.setColor(Draw::BLACK);
			for(size_t i=0; i<missGradients.size(); ++i)
				d.point(missGradients[i],3).fill();
			for(size_t i=0; i<hitGradients.size(); ++i)
				d.point(hitGradients[i],3).fill();
			d.setStrokeWidth(1);
			d.draw(outer).stroke(Draw::YELLOW);
			endPage(d);
			
			/**** Page 5+inner.size()*2: Shape-shape collision tests ****/
			for(size_t s=0; s!=tests[test].inner.size(); ++s) {
				PlannerObstacle2D& inner = tests[test].inner[s];
				hit.clear(); miss.clear();
				for(size_t i=0; i<VROWS; i+=2) {
					fmat::fmatReal y = i*(VEDGE*3/2) + bb.min[1];
					for(size_t j=0; j<VCOLS; j+=2) {
						fmat::Column<2> p = fmat::pack(j*VRES + bb.min[0], y);
						if(i&1)
							p[0]+=VRES/2;
						inner.updatePosition(p);
						if(outer.collides(inner)) {
							hit.push_back(p);
						} else {
							miss.push_back(p);
						}
					}	
				}
				
				// draw misses
				std::stringstream ss;
				ss << "s" << s << "m";
				startPage(d, oit->first, ss.str());
				d.setStrokeWidth(4);
				d.draw(outer).setClip().fill(Draw::LT_GRAY).stroke(Draw::BLACK).clearClip();
				d.setStrokeWidth(1);
				d.setColor(Draw::GREEN);
				for(size_t i=0; i<miss.size(); ++i) {
					inner.updatePosition(miss[i]);
					d.draw(inner).fill();
				}
				d.setColor(Draw::RED);
				d.draw(outer).setClip().holdPath();
				for(size_t i=0; i<miss.size(); ++i) {
					inner.updatePosition(miss[i]);
					d.draw(inner);
				}
				d.releasePath().fill();
				d.clearClip();
				d.setColor(Draw::BLACK);
				for(size_t i=0; i<miss.size(); ++i) {
					inner.updatePosition(miss[i]);
					d.draw(inner).setClip().stroke().clearClip();
				}
				endPage(d);
				
				// draw hits
				ss.str("");
				ss << "s" << s << "h";
				startPage(d, oit->first, ss.str());
				d.setStrokeWidth(1);
				d.setColor(Draw::RED);
				for(size_t i=0; i<hit.size(); ++i) {
					inner.updatePosition(hit[i]);
					d.draw(inner).fill();
				}
				d.setColor(Draw::BLACK);
				for(size_t i=0; i<hit.size(); ++i) {
					inner.updatePosition(hit[i]);
					d.draw(inner).setClip().stroke().clearClip();
				}
				d.setStrokeWidth(4);
				d.draw(outer).setClip().fill(Draw::LT_GRAY,0.5).stroke(Draw::YELLOW).clearClip();
				endPage(d);
			}
			std::cout << std::endl;
		}
	}
	
	
	/**** Some numeric tests... ****/
	if(extendedTests) {
		rectTest();
		convexHullTest();
	}
	
	return EXIT_SUCCESS;
}

void startPage(Draw& d, const std::string& base, const std::string& test) {
	if(extendedTests || pageCnt==1) {
		if(pngOutput) {
			std::cout << "Rendering";
		} else {
			std::cout << "Rendering " << imgOutputPrefix << base << ".pdf -";
		}
	}
	if(pngOutput) {
		d.setFile(imgOutputPrefix+base+"-"+test+".png", IMGDIM, IMGDIM);
		std::cout << " " << imgOutputPrefix+base+"-"+test+".png";
	} else {
		std::cout << " " << pageCnt << ":" << test;
	}
	if(extendedTests)
		std::cout << std::endl;
	else
		std::cout << std::flush;
	++pageCnt;
}
void endPage(Draw& d) {
	if(!pngOutput)
		d.flushPage();
}

void rectTest() {
	std::cout << "\nTesting Rectangle initialization:\n";
	Column<2> rc = pack(10,15);
	Column<2> rx = pack(5,7);
	fmatReal ra = 15*M_PI/180;
	RectangularObstacle r(rc,rx,ra);
	cout << "Center: " << r.getCenter() << endl;
	cout << "Dimensions: " << r.getWidth() << ' ' << r.getHeight() << endl;
	cout << "Orientation: " << r.getOrientation() << endl;
	BoundingBox2D bb;
	bb = r.getBoundingBox();
	cout << "Extents min " << bb.min << " max " << bb.max << endl;

	rc = pack(11,16);
	r.updatePosition(rc);
	cout << "Center: " << r.getCenter() << endl;
	cout << "Dimensions: " << r.getWidth() << ' ' << r.getHeight() << endl;
	cout << "Orientation: " << r.getOrientation() << endl;
	bb = r.getBoundingBox();
	cout << "Extents min " << bb.min << " max " << bb.max << endl;
}

ConvexPolyObstacle& randomHull(ConvexPolyObstacle& poly, fmat::Column<2> c, float w, float h) {
	std::vector<fmat::Column<2> > p;
	for(size_t n=(random()%17)+3; n>0; --n) {
		float x = random()/float((1<<31)-1)*w - w/2;
		float y = random()/float((1<<31)-1)*h - h/2;
		p.push_back(c+fmat::pack(x,y));
	}
	poly.hull(std::set<fmat::Column<2> >(p.begin(),p.end()));
	return poly;
}

void convexHullTest() {
	std::cout << "\nTesting convex hull generation\n";
	const unsigned int N_HULL_TESTS=15000;
	TimeET time;
	for(size_t h=0; h<N_HULL_TESTS; ++h) {
		std::vector<fmat::Column<2> > p;
		for(size_t n=(random()%17)+3; n>0; --n) {
			float x = random()/float((1<<31)-1)*10 - 5;
			float y = random()/float((1<<31)-1)*10 - 5;
			p.push_back(fmat::pack(x,y));
		}
		ConvexPolyObstacle poly;
		poly.hull(std::set<fmat::Column<2> >(p.begin(),p.end()));
		// Verification
		for(size_t i=0; i<poly.getNormals().size(); ++i) {
			for(size_t j=0; j<poly.getPoints().size(); ++j) {
				if(i==j || j==i+1 || (i==poly.getNormals().size()-1 && j==0))
					continue;
				fmat::Column<2> d = poly.getPoints()[j] - poly.getPoints()[i];
				if(fmat::dotProduct(d,poly.getNormals()[i])>=0) {
					std::cout << "\nErr " << i << ' ' << j << ' ' << fmat::dotProduct(d,poly.getNormals()[i]) << std::endl;
					std::cout << "Normal " << poly.getNormals()[i] << std::endl;
					std::cout << "ALL" << std::endl;
					for(std::vector<fmat::Column<2> >::const_iterator it=p.begin(); it!=p.end(); ++it) {
						std::cout << (*it)[0] << ' ' << (*it)[1] << std::endl;
					}
					std::cout << "\nHULL" << std::endl;
					for(std::vector<fmat::Column<2> >::const_iterator it=poly.getPoints().begin(); it!=poly.getPoints().end(); ++it) {
						std::cout << (*it)[0] << ' ' << (*it)[1] << std::endl;
					}
				}
			}
		}
	}
	std::cout << "@VAR " << N_HULL_TESTS << " convex hull tests took " << time.Age() << std::endl; 
}


