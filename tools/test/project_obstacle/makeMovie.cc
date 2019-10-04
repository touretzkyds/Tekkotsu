/******************* DRAWING STUFF **************************
 * This is just to do a bunch of graphics for visualization *
 * to verify things are working correctly.                  *
 ************************************************************/

#ifdef HAVE_CAIRO
#include "Motion/KinematicJoint.h"
#include "Planners/PlannerObstacles.h"
#include <sys/stat.h>
#include <cairo.h>

void makeMovie(const KinematicJoint& kj) {
	const fmat::Column<2> ASPECT = fmat::pack(1,1);
	const fmat::Column<2> FRAME_DIM = ASPECT * 480;
	const fmat::Column<2> FRAME_RANGE = ASPECT * 120;
	const fmat::Column<2> OFFSET = fmat::pack(15,30);
	const float PIXEL = FRAME_RANGE[0]/FRAME_DIM[0];
	const float FRONT_LINE = 3*PIXEL, MID_LINE = 2*PIXEL, BACK_LINE = 1.25*PIXEL;
	const float BB_LINE = 4*PIXEL;
	
	mkdir("frames",0755);
	std::cout << std::endl;
	
	fmat::Transform obT;
	kj.getCollisionModelTransform(obT);
	fmat::Column<3> obD;
	obD.importFrom(kj.collisionModelScale);

	fmat::Matrix<3,8> prism;
	prism.column(0) = fmat::pack( obD[0], obD[1], obD[2])/2;
	prism.column(1) = fmat::pack( obD[0], obD[1],-obD[2])/2;
	prism.column(2) = fmat::pack( obD[0],-obD[1],-obD[2])/2;
	prism.column(3) = fmat::pack( obD[0],-obD[1], obD[2])/2;
	prism.column(4) = fmat::pack(-obD[0], obD[1], obD[2])/2;
	prism.column(5) = fmat::pack(-obD[0], obD[1],-obD[2])/2;
	prism.column(6) = fmat::pack(-obD[0],-obD[1],-obD[2])/2;
	prism.column(7) = fmat::pack(-obD[0],-obD[1], obD[2])/2;

	// edges connections of a prism, given index ordering above
	std::vector<std::set<size_t> > edges(12);
	for(size_t i=0; i<4; ++i) {
		edges[i].insert((i+1)%4);
		edges[i+4].insert((i+1)%4 + 4);
		edges[i].insert(i+4);
		// edges are bi-directional, but we only need one direction...
		//edges[(i+1)%4].insert((i+1)%4);
		//edges[(i+1)%4 + 4].insert(i + 4);
		//edges[i+4].insert(i);
	}
	
	RectangularObstacle r;
	
	fmat::Transform basePose; // position of base in world...
	basePose.translation() = fmat::pack(OFFSET,0);

	cairo_surface_t * surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, FRAME_DIM[0], FRAME_DIM[1]);

	const size_t FRAMES=60*30, X1=11, X2=6;
	for(size_t frame=0; frame<FRAMES; ++frame) {
		float t = frame/(float)FRAMES;
		std::cout << '\r' << std::setw(6) << (frame+1) << " / " << FRAMES << std::flush;
		
		// for testing corner tracking on minor perterbations (use square x-y dimensions on object)
		//basePose.rotation() = fmat::rotationZ(2*M_PI*t * 3) * fmat::rotationY(std::sin(-2*M_PI * t * X1)*.1) * fmat::rotationX(std::sin(-2*M_PI * t * X2)*.1);
		
		basePose.rotation() = fmat::rotationX(-2*M_PI * t * X2) * fmat::rotationY(M_PI * t) * fmat::rotationX(2*M_PI * t * X1);
		fmat::Transform linkPose = basePose * kj.getFullT(); // world to base, base to link

		// just using the KJ's own collision model (ignoring components array)
		kj.getBB2D(linkPose, r);
		
		cairo_t * cr = cairo_create (surface);
		cairo_set_source_rgba(cr,1,1,1,1);
		cairo_paint(cr);
		
		// set up drawing space so positive Y is up and scaling is applies
		cairo_translate(cr, FRAME_DIM[0]/2, FRAME_DIM[1]/2);
		cairo_scale(cr, FRAME_DIM[0]/FRAME_RANGE[0], -FRAME_DIM[1]/FRAME_RANGE[1]);
		cairo_translate(cr, -OFFSET[0], -OFFSET[1]);
		
		// draw XY axes just for reference
		cairo_move_to(cr,0,OFFSET[1]-FRAME_RANGE[1]/2);
		cairo_line_to(cr,0,OFFSET[1]+FRAME_RANGE[1]/2);
		cairo_move_to(cr,OFFSET[0]-FRAME_RANGE[0]/2,0);
		cairo_line_to(cr,OFFSET[0]+FRAME_RANGE[0]/2,0);
		cairo_set_source_rgba(cr,0,0,0,0.5);
		cairo_set_line_width(cr,PIXEL);
		cairo_stroke(cr);
		
		// fill the fitted box area
		cairo_move_to(cr,r.getCorner(RectangularObstacle::TOP_RIGHT)[0],r.getCorner(RectangularObstacle::TOP_RIGHT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::BOTTOM_RIGHT)[0],r.getCorner(RectangularObstacle::BOTTOM_RIGHT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::BOTTOM_LEFT)[0],r.getCorner(RectangularObstacle::BOTTOM_LEFT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::TOP_LEFT)[0],r.getCorner(RectangularObstacle::TOP_LEFT)[1]);
		cairo_close_path(cr);
		cairo_set_source_rgba(cr,1,0,0,0.4);
		cairo_fill(cr);

		cairo_set_line_join(cr,CAIRO_LINE_JOIN_BEVEL);
		
		// compute corner points of 3D shape
		fmat::Transform fullT = linkPose * obT;
		fmat::Matrix<3,8> corners = fullT * prism;
		
		// draw edges from deepest point below highlighting
		size_t deepest=0,highest=0;
		for(size_t i=1; i<8; ++i) {
			if(corners(2,i)<corners(2,deepest))
				deepest=i;
			if(corners(2,i)>corners(2,highest))
				highest=i;
		}
		for(std::set<size_t>::const_iterator it=edges[deepest].begin(); it!=edges[deepest].end(); ++it) {
			cairo_move_to(cr,corners(0,deepest),corners(1,deepest));
			cairo_line_to(cr,corners(0,*it),corners(1,*it));
		}
		for(size_t i=0; i<12; ++i) {
			if(edges[i].count(deepest)) {
				cairo_move_to(cr,corners(0,deepest),corners(1,deepest));
				cairo_line_to(cr,corners(0,i),corners(1,i));
			}
		}
		cairo_set_line_width(cr,BACK_LINE);
		cairo_set_source_rgba(cr,0,0,0,0.9);
		cairo_stroke(cr);
		
		// cross hatch the small back side
		size_t side = (deepest<4)?0:4;
		cairo_move_to(cr,corners(0,0 + side),corners(1,0 + side));
		cairo_line_to(cr,corners(0,2 + side),corners(1,2 + side));
		cairo_move_to(cr,corners(0,1 + side),corners(1,1 + side));
		cairo_line_to(cr,corners(0,3 + side),corners(1,3 + side));
		cairo_set_line_width(cr,MID_LINE);
		cairo_set_source_rgba(cr,0.75,0.25,0.75,1);
		cairo_stroke(cr);
		
		// highlight projected area of 3D shape
		ConvexPolyObstacle hull;
		std::set<fmat::Column<2> > projCorners;
		for(size_t i=0; i<8; ++i) // have to cut off the z dimension for the hull call below
			projCorners.insert(fmat::SubVector<2>(corners.column(i)));
		hull.hull(projCorners);
		cairo_move_to(cr,hull.getPoints()[0][0],hull.getPoints()[0][1]);
		for(unsigned int i=1; i<hull.getPoints().size(); ++i) {
			cairo_line_to(cr,hull.getPoints()[i][0],hull.getPoints()[i][1]);
		}
		cairo_close_path(cr);
		cairo_set_source_rgba(cr,0.3,1,0.5,0.6);
		cairo_fill(cr);
		
		// cross hatch the small front side
		side = (highest<4)?0:4;
		cairo_move_to(cr,corners(0,0 + side),corners(1,0 + side));
		cairo_line_to(cr,corners(0,2 + side),corners(1,2 + side));
		cairo_move_to(cr,corners(0,1 + side),corners(1,1 + side));
		cairo_line_to(cr,corners(0,3 + side),corners(1,3 + side));
		cairo_set_line_width(cr,MID_LINE);
		cairo_set_source_rgba(cr,0.4,0.4,0.4,1);
		cairo_stroke(cr);
		
		// draw remaining edges of 3D shape
		cairo_set_source_rgba(cr,0,0,0,1);
		for(size_t i=0; i<12; ++i) {
			if(i==deepest)
				continue;
			for(std::set<size_t>::const_iterator it=edges[i].begin(); it!=edges[i].end(); ++it) {
				if(*it!=deepest) {
					cairo_move_to(cr,corners(0,i),corners(1,i));
					cairo_line_to(cr,corners(0,*it),corners(1,*it));
					cairo_set_line_width(cr,(*it==highest || i==highest)?FRONT_LINE:MID_LINE);
					cairo_stroke(cr);
				}
			}
		}
		
		// highlight calculated bounding box itself
		cairo_set_line_join(cr,CAIRO_LINE_JOIN_MITER);
		cairo_move_to(cr,r.getCorner(RectangularObstacle::TOP_RIGHT)[0],r.getCorner(RectangularObstacle::TOP_RIGHT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::BOTTOM_RIGHT)[0],r.getCorner(RectangularObstacle::BOTTOM_RIGHT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::BOTTOM_LEFT)[0],r.getCorner(RectangularObstacle::BOTTOM_LEFT)[1]);
		cairo_line_to(cr,r.getCorner(RectangularObstacle::TOP_LEFT)[0],r.getCorner(RectangularObstacle::TOP_LEFT)[1]);
		cairo_close_path(cr);
		cairo_set_source_rgba(cr,1,0,0,0.55);
		cairo_set_line_width(cr,BB_LINE);
		cairo_stroke(cr);
		
		cairo_arc(cr,r.getCorner(RectangularObstacle::TOP_RIGHT)[0],r.getCorner(RectangularObstacle::TOP_RIGHT)[1],BB_LINE*2,0,2*M_PI);
		cairo_set_source_rgba(cr,1,0,0,0.55);
		cairo_fill(cr);
		
		std::stringstream filename;
		filename << "frames/" << std::setw(6) << std::setfill('0') << frame << ".png";
		cairo_surface_write_to_png (surface, filename.str().c_str());
		
		cairo_destroy (cr);
	}
	std::cout << std::endl;

	cairo_surface_destroy (surface);
}
#endif
