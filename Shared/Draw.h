#ifndef INCLUDED_Draw_h_
#define INCLUDED_Draw_h_

#include "Shared/BoundingBox.h"
#include "Planners/PlannerObstacles.h"

typedef struct _cairo cairo_t;
typedef struct _cairo_surface cairo_surface_t;

//! A very simple drawing environment, originally to visualize planner stuff
/*! Default origin is in the lower left corner, positive x and y to the right and up */
class Draw {
public:
	//! Default constructor, must still call setFile() before any drawing commands
	Draw()
		: cr(NULL), surface(NULL), filename(), origin(), scale(1), surfaceWidth(), surfaceHeight(), addPath(false) {}
	
	//! Constructor, sets up output
	Draw(const std::string& file, double width, double height)
		: cr(NULL), surface(NULL), filename(), origin(), scale(1), surfaceWidth(), surfaceHeight(), addPath(false) { setFile(file,width,height); }
	
	//! Destructor
	~Draw() { teardown(); }
	
	//! This must be called to setup output destination before calling any drawing commands
	Draw& setFile(const std::string& file, double width, double height);
	
	//! Writes the current page and starts a new one
	Draw& flushPage();
	
	//! Moves the origin (lower left corner)
	Draw& setOrigin(const fmat::Column<2>& p) { origin=p; resetTransform(); return *this;}
	//! Moves the origin (lower left corner)
	Draw& setOrigin(double x, double y) { origin=fmat::pack(x,y); resetTransform(); return *this;}
	//! Moves the origin in order to center the viewport on the specified point
	Draw& centerView(const fmat::Column<2>& p) { return centerView(p[0],p[1]); }
	//! Moves the origin in order to center the viewport on the specified point
	Draw& centerView(double x, double y);
	
	//! Clips further drawing to the current path area
	Draw& setClip();
	//! Clears the clipping region
	Draw& clearClip();
	
	//! Changes the scaling factor, to display more or less within the bounds given to setFile()
	/*! The apparent stroke width is unchanged unless you set @a scaleStroke */
	Draw& setScale(double x, bool scaleStroke=false);
	//! Converts from display units to scaled coordinates
	double displayToScaled(double x) const { return x/scale; }
	//! Converts from scaled coordinates to display units
	double scaledToDisplay(double x) const { return x*scale; }
	
	//! Changes scale and origin to fit the specified drawing region to the output
	Draw& setRegion(const BoundingBox2D& region, bool scaleStroke=false);
	//! Draws a grid of the specified horizontal and vertical resolution across the visible region (doesn't stroke)
	Draw& drawGrid(double xRes, double yRes, double xOff=0, double yOff=0);
	//! Draws the axes across the visible region (doesn't stroke)
	Draw& drawAxes();
	
	//! Sets the stroke width using output dimension units, i.e. scale factor invariant
	Draw& setStrokeWidth(double x);
	//! Sets the stroke width using coordinate-space units, i.e. relative to scale factor
	Draw& setScaledStrokeWidth(double x);
	//! Returns the stroke width in output dimension units
	double getStrokeWidth() const  { return getScaledStrokeWidth()*scale; }
	//! Returns the stroke width in coordinate-space units
	double getScaledStrokeWidth() const;
	
	//! Convenient color constants
	enum Colors {
		BLACK, DK_GRAY, GRAY, LT_GRAY, WHITE, RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA
	};
	
	//! Sets the color for future drawing operations
	Draw& setColorRGB(double red, double green, double blue, double alpha=1);
	//! Sets the color for future drawing operations
	Draw& setColorGray(double x, double alpha=1) { return setColorRGB(x,x,x,alpha); }
	//! Sets the color for future drawing operations
	Draw& setColor(Colors c, double alpha=1);
	
	//! Outline the current path
	Draw& stroke();
	//! Outline the current path with specified color
	Draw& stroke(Colors c, double alpha=1) { return setColor(c,alpha).stroke(); }
	//! Outline the current path with specified color
	Draw& stroke(double red, double green, double blue, double alpha=1) { return setColorRGB(red,green,blue,alpha).stroke(); }
	//! Stroke a single line; if drawing a polygon, better to use lineTo and stroke as a batch
	Draw& strokeLine(const fmat::Column<2>& p1, const fmat::Column<2>& p2) { return strokeLine(p1[0],p1[1],p2[0],p2[1]); }
	//! Stroke a single line; if drawing a polygon, better to use lineTo and stroke as a batch
	Draw& strokeLine(double x1, double y1, double x2, double y2);

	
	//! Paint the current path
	Draw& fill();
	//! Paint the current path with specified color
	Draw& fill(Colors c, double alpha=1) { return setColor(c,alpha).fill(); }
	//! Paint the current path with specified color
	Draw& fill(double red, double green, double blue, double alpha=1) { return setColorRGB(red,green,blue,alpha).fill(); }
	
	
	//! Provides different point() styles
	enum PointStyles {
		CIRCLE, SQUARE, DIAMOND
	};
	
	//! Produces a output-dimension (i.e. unscaled) symbol at the specified point (follow up with fill() and/or stroke())
	Draw& point(const fmat::Column<2>& p, double size, PointStyles style=CIRCLE) { return point(p[0],p[1],size,style); }
	//! Produces a output-dimension (i.e. unscaled) symbol at the specified point (follow up with fill() and/or stroke())
	Draw& point(double x, double y, double size, PointStyles style=CIRCLE);
	
	//! Produces an arc of the specified range
	Draw& arc(const fmat::Column<2>& p, double r, double begin, double end) { return arc(p[0],p[1],r,begin,end); }
	//! Produces an arc of the specified range
	Draw& arc(double x, double y, double r, double begin, double end);
	
	//! Produces a closed path for a circle at the specified point
	Draw& circle(const fmat::Column<2>& c, double r) { return arc(c[0],c[1],r,0,2*M_PI); }
	//! Produces a closed path for a circle at the specified point
	Draw& circle(double x, double y, double r) { return arc(x,y,r,0,2*M_PI); }
	
	//! Produces an ellipse, note width and height, not major/minor
	Draw& ellipse(const fmat::Column<2>& c, double width, double height, double orientation=0);
	
	//! Produces a closed path for an axis-aligned rectangle
	Draw& rect(const BoundingBox2D& r) { return rect(r.min,r.max); }
	//! Produces a closed path for an axis-aligned rectangle between the specified points
	Draw& rect(const fmat::Column<2>& p1, const fmat::Column<2>& p2);
	
	//! Draws a line between points with an arrowhead at @a p2, the @a headWidth and @a headLength are multiples of the stroke width
	/*! You can stroke to get a barbed head, or fill to get a little triangle.  For a really
	 *  perfect filled arrowhead, call closePath() first to avoid a little bit of mitred corners sticking out. */
	Draw& arrow(const fmat::Column<2>& p1, const fmat::Column<2>& p2, double headWidth=2, double headLength=5);

	//! Produces a closed path for a planner obstacle
	Draw& draw(const PlannerObstacle2D& o);
	//! Produces a closed path for a planner obstacle
	Draw& draw(const RectangularObstacle& o);
	//! Produces a closed path for a planner obstacle
	Draw& draw(const CircularObstacle& o);
	//! Produces a closed path for a planner obstacle
	Draw& draw(const EllipticalObstacle& o);
	//! Produces a closed path for a planner obstacle
	Draw& draw(const ConvexPolyObstacle& o);
	//! Produces a closed path for a planner obstacle
	Draw& draw(const HierarchicalObstacle& o);
	
	//! Set the pen location without adding a line segment to the current path.
	Draw& moveTo(const fmat::Column<2>& p) { return moveTo(p[0],p[1]); }
	//! Set the pen location without adding a line segment to the current path.
	/*! Clears previous path unless holdPath() has been called. */
	Draw& moveTo(double x, double y);
	
	//! Add a line segment to the current path
	Draw& lineTo(const fmat::Column<2>& p) { return lineTo(p[0],p[1]); }
	//! Add a line segment to the current path
	Draw& lineTo(double x, double y);
	
	//! Produces an arc of the specified range (continuing the current path)
	Draw& arcTo(const fmat::Column<2>& p, double r, double begin, double end) { return arcTo(p[0],p[1],r,begin,end); }
	//! Produces an arc of the specified range (continuing the current path)
	Draw& arcTo(double x, double y, double r, double begin, double end);
	
	//! Add a line segment back to the last moveTo location
	Draw& closePath();
	
	//! Clears path unless @a continuePath is set, additional paths will be collected together until releasePath() is called
	Draw& holdPath(bool continuePath=false) { if(!continuePath) clearPath(); addPath=true; return *this; }
	//! Clears the hold flag from holdPath(), but does not actually clear the path
	Draw& releasePath() { addPath=false; return *this; }
	//! Returns current hold setting
	bool getHoldMode() const { return addPath; }
	
	//! Erase current path, but does not reset holdPath() mode
	Draw& clearPath();
	
protected:
	//! Flush pending output and release resources
	void teardown();
	//! Recomputes and applies the surface transform
	void resetTransform();
	//! Throws an exception if surface status is not success
	void checkSurfaceStatus(const std::string& msg);
	//! Throws an exception if either cairo or surface status is not success
	void checkStatus(const std::string& msg);
	
	cairo_t *cr;
	cairo_surface_t *surface;
	std::string filename;
	
	fmat::Column<2> origin;
	double scale;
	double surfaceWidth;
	double surfaceHeight;
	
	//! Indicates state of holdPath()/releasePath(), if true the path isn't cleared between draw calls.
	bool addPath;
	
private:
	Draw(const Draw& o); //!< Do not call
	Draw& operator=(const Draw& o); //!< Do not call
};

/*! @file
 * @brief Describes Draw, which provides a simple interface to draw images
 * @author ejt (Creator)
 */

#endif
