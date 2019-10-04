#include "Draw.h"

Draw& Draw::setColor(Colors c, double alpha) {
	switch(c) {
		case BLACK:
			setColorRGB(0,0,0,alpha); break;
		case DK_GRAY:
			setColorRGB(0.25,0.25,0.25,alpha); break;
		case GRAY:
			setColorRGB(0.5,0.5,0.5,alpha); break;
		case LT_GRAY:
			setColorRGB(0.75,0.75,0.75,alpha); break;
		case WHITE:
			setColorRGB(1,1,1,alpha); break;
		case RED:
			setColorRGB(1,0,0,alpha); break;
		case YELLOW:
			setColorRGB(1,1,0,alpha); break;
		case GREEN:
			setColorRGB(0,1,0,alpha); break;
		case CYAN:
			setColorRGB(0,1,1,alpha); break;
		case BLUE:
			setColorRGB(0,0,1,alpha); break;
		case MAGENTA:
			setColorRGB(1,0,1,alpha); break;
	}
	return *this;
}

#ifndef HAVE_CAIRO

struct _cairo {
	_cairo() : lineWidth(1) {}
	double lineWidth;
};

Draw& Draw::setFile(const std::string&, double width, double height) {
	teardown();
	cr = new cairo_t;
	surfaceWidth=width;
	surfaceHeight=height;
	return *this;
}
Draw& Draw::setScale(double x, bool scaleStroke) {
	if(!scaleStroke && cr!=NULL)
		cr->lineWidth *= scale/x;
	scale=x;
	return *this;
}
Draw& Draw::setRegion(const BoundingBox2D& region, bool scaleStroke/*=false*/) {
	double s0 = surfaceWidth / region.getDimension(0);
	double s1 = surfaceHeight / region.getDimension(1);
	double s = s0 < s1 ? s0 : s1;
#ifndef PLATFORM_APERIOS // missing isfinite
	if(std::isfinite(s) && s>0)
#endif
		setScale(s,scaleStroke);
	centerView(region.getCenter());
	return *this;
}
Draw& Draw::drawGrid(double xRes, double yRes, double xOff/*=0*/, double yOff/*=0*/) { return *this; }
Draw& Draw::drawAxes() { return *this; }
Draw& Draw::setStrokeWidth(double x) { if(cr!=NULL) cr->lineWidth = x/scale; return *this; }
Draw& Draw::setScaledStrokeWidth(double x) { if(cr!=NULL) cr->lineWidth = x; return *this; }
double Draw::getScaledStrokeWidth() const { return (cr!=NULL)?cr->lineWidth:1; }
void Draw::teardown() { delete cr; cr=NULL; }
 
Draw& Draw::flushPage() { return *this; }
Draw& Draw::centerView(double, double) { return *this; }
Draw& Draw::setClip() { return *this; }
Draw& Draw::clearClip() { return *this; }
Draw& Draw::setColorRGB(double, double, double, double) { return *this; }
Draw& Draw::stroke() { return *this; }
Draw& Draw::strokeLine(double, double, double, double) { return *this; }
Draw& Draw::fill() { return *this; }
Draw& Draw::point(double, double, double, PointStyles) { return *this; }
Draw& Draw::arc(double, double, double, double, double) { return *this; }
Draw& Draw::ellipse(const fmat::Column<2>&, double, double, double) { return *this; }
Draw& Draw::rect(const fmat::Column<2>&, const fmat::Column<2>&) { return *this; }
Draw& Draw::arrow(const fmat::Column<2>& p1, const fmat::Column<2>& p2, double headWidth, double headLength) { return * this; }
Draw& Draw::draw(const PlannerObstacle2D&) { return *this; }
Draw& Draw::draw(const RectangularObstacle&) { return *this; }
Draw& Draw::draw(const CircularObstacle&) { return *this; }
Draw& Draw::draw(const EllipticalObstacle&) { return *this; }
Draw& Draw::draw(const ConvexPolyObstacle&) { return *this; }
Draw& Draw::draw(const HierarchicalObstacle&) { return *this; }
Draw& Draw::moveTo(double, double) { return *this; }
Draw& Draw::lineTo(double, double) { return *this; }
Draw& Draw::arcTo(double, double, double, double, double) { return *this; }
Draw& Draw::closePath() { return *this; }
Draw& Draw::clearPath() { return *this; }
void Draw::resetTransform() {}
void Draw::checkSurfaceStatus(const std::string&) {}
void Draw::checkStatus(const std::string&) {}

#else

#include <cairo-pdf.h>
#include <stdexcept>
#include "Planners/PlannerObstacles.h"

void Draw::teardown() {
	if(filename.size()>=4 && filename.substr(filename.size()-4)==".png") {
		cairo_surface_write_to_png(surface, filename.c_str());
	}
	cairo_destroy(cr);
	cr=NULL;
	cairo_surface_destroy(surface);
	surface=NULL;
}

Draw& Draw::setFile(const std::string& file, double width, double height) {
	teardown();
	filename = file;
	if(file.size()>=4 && file.substr(file.size()-4)==".pdf") {
		surface = cairo_pdf_surface_create(file.c_str(), width, height);
	} else {
		surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
	}
	checkSurfaceStatus("cairo_pdf_surface_create");
	cr = cairo_create(surface);
	checkStatus("cairo_create");
	surfaceWidth=width;
	surfaceHeight=height;
	resetTransform();
	return *this;
}

Draw& Draw::flushPage() {
	cairo_surface_show_page(surface);
	checkStatus("cairo_surface_show_page");
	resetTransform();
	return *this;
}

Draw& Draw::centerView(double x, double y) {
	origin[0] = x - surfaceWidth/scale/2;
	origin[1] = y - surfaceHeight/scale/2;
	resetTransform();
	return *this;
}

Draw& Draw::setClip() {
	cairo_clip_preserve(cr);
	return *this;
}

Draw& Draw::clearClip() {
	cairo_reset_clip(cr);
	return *this;
}

Draw& Draw::setScale(double x, bool scaleStroke/*=false*/) {
	if(!scaleStroke)
		cairo_set_line_width(cr, cairo_get_line_width(cr)*scale/x);
	scale=x;
	resetTransform();
	return *this;
}

Draw& Draw::setRegion(const BoundingBox2D& region, bool scaleStroke/*=false*/) {
	double s0 = surfaceWidth / region.getDimension(0);
	double s1 = surfaceHeight / region.getDimension(1);
	double s = s0 < s1 ? s0 : s1;
#ifndef PLATFORM_APERIOS // missing isfinite
	if(std::isfinite(s) && s>0)
#endif
		setScale(s,scaleStroke);
	centerView(region.getCenter());
	return *this;
}

Draw& Draw::drawGrid(double xRes, double yRes, double xOff/*=0*/, double yOff/*=0*/) {
	if(!addPath)
		cairo_new_path(cr);
	double right = origin[0] + surfaceWidth/scale;
	double top = origin[1] + surfaceHeight/scale;
	if(xRes>0) {
		xOff = std::fmod(xOff,xRes);
		double begin = std::ceil((origin[0]-xOff) / xRes) * xRes + xOff;
		for(double x=begin; x<=right; x+=xRes) {
			cairo_move_to(cr, x, origin[1]);
			cairo_line_to(cr, x, top);
		}
	}
	if(yRes>0) {
		yOff = std::fmod(yOff,yRes);
		double begin = std::ceil((origin[1]-yOff) / yRes) * yRes + yOff;
		for(double y=begin; y<=top; y+=yRes) {
			cairo_move_to(cr, origin[0], y);
			cairo_line_to(cr, right, y);
		}
	}
	return *this;
}

Draw& Draw::drawAxes() {
	if(!addPath)
		cairo_new_path(cr);
	double right = origin[0] + surfaceWidth/scale;
	double top = origin[1] + surfaceHeight/scale;
	cairo_move_to(cr, 0, origin[1]);
	cairo_line_to(cr, 0, top);
	cairo_move_to(cr, origin[0], 0);
	cairo_line_to(cr, right, 0);
	return *this;
}

Draw& Draw::setStrokeWidth(double x) {
	cairo_set_line_width(cr, x/scale);
	return *this;
}

Draw& Draw::setScaledStrokeWidth(double x) {
	cairo_set_line_width(cr, x);
	return *this;
}

double Draw::getScaledStrokeWidth() const {
	return cairo_get_line_width(cr);
}

Draw& Draw::setColorRGB(double red, double green, double blue, double alpha/*=1*/) {
	cairo_set_source_rgba(cr, red, green, blue, alpha);
	return *this;
}

Draw& Draw::stroke() {
	cairo_stroke_preserve(cr);
	return *this;
}

Draw& Draw::strokeLine(double x1, double y1, double x2, double y2) {
	if(!addPath)
		cairo_new_path(cr);
	cairo_move_to(cr, x1, y1);
	cairo_line_to(cr, x2, y2);
	cairo_stroke_preserve(cr);
	return *this;
}


Draw& Draw::fill() {
	cairo_fill_preserve(cr);
	return *this;
}

Draw& Draw::point(double x, double y, double size, PointStyles style/*=CIRCLE*/) {
	double r=size/2/scale;
	switch(style) {
		case CIRCLE: {
			if(!addPath)
				cairo_new_path(cr);
			else
				cairo_new_sub_path(cr);
			cairo_arc(cr, x, y, r, 0, 2*M_PI);
		} break;
		case SQUARE: {
			if(!addPath)
				cairo_new_path(cr);
			cairo_move_to(cr, x-r, y-r);
			cairo_line_to(cr, x+r, y-r);
			cairo_line_to(cr, x+r, y+r);
			cairo_line_to(cr, x-r, y+r);
			cairo_close_path(cr);
		} break;
		case DIAMOND: {
			if(!addPath)
				cairo_new_path(cr);
			cairo_move_to(cr, x, y-r);
			cairo_line_to(cr, x+r, y);
			cairo_line_to(cr, x, y+r);
			cairo_line_to(cr, x-r, y);
			cairo_close_path(cr);
		} break;
	}
	return *this;
}

Draw& Draw::arc(double x, double y, double r, double begin, double end) {
	if(!addPath)
		cairo_new_path(cr);
	else
		cairo_new_sub_path(cr);
	return arcTo(x,y,r,begin,end);
}

Draw& Draw::ellipse(const fmat::Column<2>& c, double width, double height, double orientation/*=0*/) {
	cairo_matrix_t om;
	cairo_get_matrix(cr, &om);
	cairo_matrix_t m;
	cairo_matrix_init(&m, scale, 0, 0, -scale, (-origin[0]+c[0])*scale, (origin[1]-c[1])*scale+surfaceHeight);
	cairo_set_matrix(cr, &m);
	cairo_rotate(cr, orientation);
	cairo_scale(cr, width/2, height/2);
	circle(0,0,1);
	cairo_set_matrix(cr, &om);
	return *this;
}

Draw& Draw::rect(const fmat::Column<2>& p1, const fmat::Column<2>& p2) {
	const fmat::Column<2> d = p2-p1;
	if(!addPath)
		cairo_new_path(cr);
	cairo_rectangle(cr, p1[0],p1[1], d[0],d[1]);
	return *this;
}

Draw& Draw::arrow(const fmat::Column<2>& p1, const fmat::Column<2>& p2, double headWidth, double headLength) {
	fmat::Column<2> d = p2-p1;
	d/=d.norm();
	fmat::Matrix<2,2> r;
	r(0,0) = r(1,1) = d[0];
	r(0,1) = -(r(1,0) = d[1]);
	fmat::Column<2> a = fmat::pack(-headLength,headWidth) * getScaledStrokeWidth();
	fmat::Column<2> p3 = r * a;
	a[1]=-a[1];
	fmat::Column<2> p4 = r * a;
	if(!addPath)
		cairo_new_path(cr);
	cairo_move_to(cr, p1[0], p1[1]);
	cairo_line_to(cr, p2[0], p2[1]);
	cairo_move_to(cr, p2[0]+p3[0], p2[1]+p3[1]);
	cairo_line_to(cr, p2[0], p2[1]);
	cairo_line_to(cr, p2[0]+p4[0], p2[1]+p4[1]);
	return *this;
}


Draw& Draw::draw(const PlannerObstacle2D& o) {
	switch(o.getObstacleGeometry()) {
		case PlannerObstacle2D::RECTANGULAR_OBS: return draw(static_cast<const RectangularObstacle&>(o));
		case PlannerObstacle2D::CIRCULAR_OBS: return draw(static_cast<const CircularObstacle&>(o));
		case PlannerObstacle2D::ELLIPTICAL_OBS: return draw(static_cast<const EllipticalObstacle&>(o));
		case PlannerObstacle2D::CONVEX_POLY_OBS: return draw(static_cast<const ConvexPolyObstacle&>(o));
		case PlannerObstacle2D::HIERARCHICAL_OBS: return draw(static_cast<const HierarchicalObstacle&>(o));
	}
	return *this;
}

Draw& Draw::draw(const RectangularObstacle& o) {
	if(!addPath)
		cairo_new_path(cr);
	cairo_move_to(cr, o.getCorner(0)[0], o.getCorner(0)[1]);
	for(size_t i=1; i<RectangularObstacle::NUM_CORNERS; ++i) {
		cairo_line_to(cr, o.getCorner(i)[0], o.getCorner(i)[1]);
	}
	cairo_close_path(cr);
	return *this;
}
Draw& Draw::draw(const CircularObstacle& o) {
	return circle(o.getCenter(), o.getRadius());
}
Draw& Draw::draw(const EllipticalObstacle& o) {
	return ellipse(o.center, o.semimajor*2, o.semiminor*2, o.getAngle());
}
Draw& Draw::draw(const ConvexPolyObstacle& o) {
	if(!addPath)
		cairo_new_path(cr);
	cairo_move_to(cr, o.getPoints()[0][0], o.getPoints()[0][1]);
	for(std::vector<fmat::Column<2> >::const_iterator it=o.getPoints().begin()+1; it!=o.getPoints().end(); ++it) {
		cairo_line_to(cr, (*it)[0], (*it)[1]);
	}
	cairo_close_path(cr);
	return *this;
	return *this;
}
Draw& Draw::draw(const HierarchicalObstacle& o) {
	bool oldHold = getHoldMode();
	if(!oldHold)
		holdPath();
	const std::vector<PlannerObstacle2D*>& sub = o.getObstacles();
	for(size_t i=0; i<sub.size(); ++i) {
		sub[i]->rotate(fmat::ZERO2, o.getOrientation());
		fmat::Column<2> oldCenter = sub[i]->getCenter();
		sub[i]->updatePosition(oldCenter+o.getCenter());
		draw(*sub[i]);
		sub[i]->updatePosition(oldCenter);
		sub[i]->rotate(fmat::ZERO2, o.getOrientation().transpose());
	}
	if(!oldHold)
		releasePath();
	return *this;
}

Draw& Draw::moveTo(double x, double y) {
	if(!addPath)
		cairo_new_path(cr);
	cairo_move_to(cr, x, y);
	return *this;
}

Draw& Draw::lineTo(double x, double y) {
	cairo_line_to(cr, x, y);
	return *this;
}

Draw& Draw::arcTo(double x, double y, double r, double begin, double end) {
	if(begin<end)
		cairo_arc(cr, x, y, r, begin, end);
	else
		cairo_arc_negative(cr, x, y, r, begin, end);
	return *this;
}

Draw& Draw::closePath() {
	cairo_close_path(cr);
	return *this;
}

Draw& Draw::clearPath() {
	cairo_new_path(cr);
	return *this;
}

void Draw::resetTransform() {
	cairo_matrix_t m;
	cairo_matrix_init(&m, scale, 0, 0, -scale, -origin[0]*scale, origin[1]*scale+surfaceHeight);
	cairo_set_matrix(cr, &m);
	checkStatus("cairo_create");
}

void Draw::checkSurfaceStatus(const std::string& msg) {
	if(cairo_surface_status(surface)!=CAIRO_STATUS_SUCCESS) {
		throw std::runtime_error(msg+": " +cairo_status_to_string(cairo_surface_status(surface)));
	}
}

void Draw::checkStatus(const std::string& msg) {
	checkSurfaceStatus(msg);
	if(cairo_status(cr)!=CAIRO_STATUS_SUCCESS) {
		throw std::runtime_error(msg+": " +cairo_status_to_string(cairo_surface_status(surface)));
	}
}

#endif
