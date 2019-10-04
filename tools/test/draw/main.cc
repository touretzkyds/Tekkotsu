#include "Shared/Draw.h"
#include <cstdlib>

void test(const char* file);

int main(int argc, const char* argv[]) {
	try {
		test("test.pdf");
		test("test.png");
	} catch(const std::exception& ex) {
		std::cerr << "Exception: " << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

void test(const char* file) {
	BoundingBox2D r(10,10,75,50);
	Draw d(file, 250, 250);
	d.setOrigin(-125,-125);
	d.setStrokeWidth(1);
	d.strokeLine(-125,0.5,125,0.5);
	d.strokeLine(0.5,-125,0.5,125);
	d.ellipse(fmat::pack(0,0),250,125,-M_PI_4).stroke(Draw::BLUE).fill(Draw::GREEN,0.5);
	d.setStrokeWidth(2);
	d.setScale(2.5,true);
	d.setOrigin((r.getDimensions()-100)/2+r.min);
	d.circle(r.getCenter(),50-1).stroke(Draw::BLACK,0.5);
	d.setScaledStrokeWidth(2);
	d.circle(r.getCenter(),50-3).stroke(Draw::BLACK,0.5);
	d.setOrigin(0,0);
	d.setStrokeWidth(2);
	d.setColor(Draw::RED);
	d.rect(r).stroke();
	d.circle(r.getCenter(),r.getDimension(1)/2).stroke(Draw::BLACK);
	d.ellipse(r.getCenter(),r.getDimension(0),r.getDimension(1)).stroke(Draw::BLUE);
	d.setStrokeWidth(1);
	for(size_t i=0; i<4; ++i)
		d.point(r.getCorner(i),4).fill(Draw::WHITE).stroke(Draw::BLACK);
	d.setScale(1);
	d.setOrigin(-125,-125);
	d.point(0.5,0.5,4).fill(Draw::WHITE).stroke(Draw::BLACK);
}
