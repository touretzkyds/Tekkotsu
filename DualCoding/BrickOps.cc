#include "BrickOps.h"
#include "visops.h"

using namespace std;

namespace DualCoding {

  /* Begin functionality for distance-from-center methods of finding the corners of a blob*/


  /* Find the point at which the given line hits the edge of the rendered region */
  Point findEdgePoint(Point start, Point end, Sketch<bool>& rendering) 
  {
    int gap_tolerance=2;

    float startx = start.coordX();
    float starty = start.coordY();
    float dx = end.coordX() - startx;
    float dy = end.coordY() - starty;

    float dist = sqrt(dx*dx + dy*dy);
    dx = dx/dist;
    dy = dy/dist;
    int maxi=0, gap=0;
    int x,y;
    for (int i=0; i<dist; i++) {
      x = (int)(startx+i*dx);
      y = (int)(starty+i*dy);
      if (rendering(x,y)) {
	maxi = i;
	gap = 0;
      }
      else 
	gap++;
      if (gap>gap_tolerance)
	break;
    }
    float fx = startx+maxi*dx;
    float fy = starty+maxi*dy;
    Point result(fx,fy);
    return result;
  }


  /* 
   * Finds the edge points of the region by drawing lines out from the center to the edges
   * Points and distances returned are indexed by angle, so some overlap is possible. 
   * The return value is the index of the most distant point. 
   */
  int findRadialDistancesFromPoint(Point center, float radius, 
					     Sketch<bool>& rendering,
					     float distances[],
					     std::vector<Point>& points)
  {
    NEW_SKETCH(circle, bool, visops::zeros(rendering));

    int maxi = 0, origmaxi = 0;
    float max = -1;
    bool stillmax = false;
    Point edge;
    for (int i=0; i<2*M_PI*radius; i++){
      float x = center.coordX()+radius*cos(i/radius);
      float y = center.coordY()+radius*sin(i/radius);
      if (x<0) x = 0; 
      else if (x>=rendering->getWidth()) x = rendering->getWidth()-1;
      if (y<0) y = 0;
      else if (y>=rendering->getHeight()) y = rendering->getHeight()-1;
      edge.setCoords(x,y,0);
      circle->at((int)(edge.coordX()), (int)(edge.coordY())) = 1;
      points[i] = findEdgePoint(center,edge, rendering);
      distances[i] = points[i].distanceFrom(center);
      if (distances[i] > max) {
	max = distances[i];
	maxi = i;
	origmaxi = i;
	stillmax = true;
      }
      else if (distances[i] == max && stillmax) {
	maxi = (origmaxi + i) / 2;
      }
      else {
	stillmax = false;
      }
      
    }

    return maxi;
  }


  /*
   * Takes the derivative of the x array, returned in dx
   */
  void takeDerivative(float x[], float dx[], int len) 
  {
    for (int i=0; i<len; i++) {
      dx[i] = x[(i+1)%len]+x[(i+2)%len]-x[(i-1+len)%len]-x[(i-2+len)%len];
    }
  }

  // Applies a "gaussian" to x, returned in gx
  // very lazy implementation
  // "gaussian" == averaging
  void applyGaussian(float x[], float gx[], int len)
  {
    int smooth_n = 5;
    float smooth_x = .2f;
    for (int i=0; i<len; i++) {
      gx[i] = 0;
      for (int j=0; j<smooth_n; j++) {
	gx[i]+=x[(i+len+j-smooth_n/2)%len]*smooth_x;
      }
    }
  }


  /*
   * Draws a histogram of the array, to be viewed like a normal sketch. 
   * Purely for debugging / presentation output. 
   */
  void drawHist(float distances[], unsigned int len, Sketch<bool>& parent) 
  {
    float xscale = 1;
    if (len > parent->getWidth() - 20) xscale = (parent->getWidth()-20)/(1.0f*len);
    float ymax = 0;
    for (unsigned int i=0; i<len; i++)
      ymax = max(ymax,abs(distances[i]));
    int const yscale = 2;
    ymax *= yscale; // scale up the histogram
    int const maxheight = 70;
    float const yshrink =  (ymax > maxheight) ? maxheight/ymax : 1.0f;
    // Draw a histogram
    NEW_SKETCH(hist,bool, visops::zeros(parent));
    for(unsigned int i=0; i<len; i++) {
      if (distances[i] > 0) {
	for (unsigned int j=0; j<yscale*distances[i]; j++) {
	  hist->at((int)(i*xscale+10), (int)(maxheight - j*yshrink + 5)) = 1;		
	}
      }
      else {
	for (int j=-1; j>yscale*distances[i]; j--) {
	  hist->at((int)(i*xscale+10), (int)(maxheight - j*yshrink + 5)) = 1;		
	}
      }
    }

    hist->at(5,(int)(maxheight*(1-yshrink)+5)) = 1;
    hist->at(5,(int)(maxheight*(1+yshrink)+5)) = 1;
    hist->at(203,(int)(maxheight*(1-yshrink)+5)) = 1;
    hist->at(203,(int)(maxheight*(1+yshrink)+5)) = 1;

  }


  /* Helper functions for finding corners by fitting a quadrilateral to a blob */


  /*
   * Gets the score of the given set of corners as a fit to the blob. 
   * 
   * The score is a combination of the area of the quadrilateral and the number of edge pixels
   * that its lines lie on. 
   *
   * A lower score is better, scores can go negative
   */
  float getBoundingQuadrilateralScore(BlobData &blob, vector<Point>& corners, 
				      Sketch<bool> edgeImage, 
				      int& borderCount, ShapeSpace &space)
  {
    const float EDGE_SCALAR = 50;

    borderCount = countBorderPixelFit(blob, corners, edgeImage, space);
    return getQuadrilateralArea(corners) - EDGE_SCALAR*borderCount;
  }



  /*
   * assuming a convex quadrilateral
   * splits the quadrilateral into n-2 triangles and uses the
   * edge-length method to find the area of each triangle:
   * s = perimeter / 2
   * area = sqrt(s*(s-a)(s-b)(s-c))
   */
  float getQuadrilateralArea(vector<Point>& corners) 
  {
    float totalArea = 0;
    for (unsigned int i=2; i<corners.size(); i++) {
      float e1 = corners[0].distanceFrom(corners[i-1]);
      float e2 = corners[i-1].distanceFrom(corners[i]);
      float diag = corners[0].distanceFrom(corners[i]);
      float s1 = (e1+e2+diag)/2.0f;
      totalArea += sqrt(s1*(s1-e1)*(s1-e2)*(s1-diag));
    }
    
    return totalArea;
  }

  
  /*
   * Computes the percentage of the blob points that are inside the quadrilateral
   *
   * A point is inside if it is on the same side of each line as one of the opposite corners
   * This is assuming a convex quadrilateral
   */
  float getBoundingQuadrilateralInteriorPointRatio(BlobData &blob, 
						   vector<Point>& corners, 
						   ShapeSpace &space)
  {
    int ncorners = corners.size();

    vector<LineData> lines;
    for (int i=0; i<ncorners; i++) {
      lines.push_back(LineData(space,corners[(i+1)%ncorners], corners[(i+2)%ncorners]));
    }

    int totalInside = 0;
    int totalPixelArea = 0;

    for ( std::vector<BlobData::run>::const_iterator it = blob.runvec.begin();
	  it != blob.runvec.end(); it++ ) {
      const BlobData::run &r = *it;
      int const xstop = r.x + r.width;

      totalPixelArea += r.width;

      Point p1(r.x,r.y);
      Point p2(xstop, r.y);
      
      // Check if the endpoints are inside the polygon
      bool pt1Inside = true, pt2Inside = true;
      for (int i=0; i<ncorners; i++) {
	if (!lines[i].pointsOnSameSide(p1,corners[i])) {
	  pt1Inside = false;
	  break;
	}
      }
      for (int i=0; i<ncorners; i++) {
	if (!lines[i].pointsOnSameSide(p2,corners[i])) {
	  pt2Inside = false;
	  break;
	}
      }

      // if the whole run is inside, we can just add the whole run and move on
      if (pt1Inside) {
	if (pt2Inside) {
	  totalInside += r.width;
	}
	else {
	  // If one end of the run is inside the quadrilateral, walk along it until you enter
	  // This could be made faster with a binary search
	  // It could also be made faster by just checking the lines that the other endpoint was outside of
	  totalInside++;
	  for (int x=xstop-1; x>r.x; x--) {
	    Point p(x,r.y);
	    bool ptInside = true;
	    for (int i=0; i<ncorners; i++) {
	      if (!lines[i].pointsOnSameSide(p,corners[i])) {
		ptInside = false;
		break;
	      }
	    }
	    if (ptInside) {
	      totalInside+=x-r.x;
	      break;
	    }
	  }
	}
      }
      else {
	// Same case as above, with the other endpoint
	if (pt2Inside) {
	  totalInside++;
	  for (int x=r.x+1; x<xstop; x++) {
	    Point p(x,r.y);
	    bool ptInside = true;
	    for (int i=0; i<ncorners; i++) {
	      if (!lines[i].pointsOnSameSide(p,corners[i])) {
		ptInside = false;
		break;
	      }
	    }
	    if (ptInside) {
	      totalInside+=xstop-x;
	      break;
	    }
	  }
	}
	else {
	  // If both endpoints are outside, we still need to check the whole run.
	  bool beenInside = false;
	  int xstart = xstop;
	  for (int x=r.x+1; x<xstop; x++) {
	    Point p(x,r.y);
	    bool ptInside = true;
	    for (int i=0; i<ncorners; i++) {
	      if (!lines[i].pointsOnSameSide(p,corners[i])) {
		ptInside = false;
		break;
	      }
	    }
	    if (ptInside) {
	      xstart = x;
	      beenInside = true;
	      break;
	    }
	  }
	  if (beenInside) {
	    for (int x=xstop-1; x>=xstart; x--) {
	      Point p(x,r.y);
	      bool ptInside = true;
	      for (int i=0; i<ncorners; i++) {
		if (!lines[i].pointsOnSameSide(p,corners[i])) {
		  ptInside = false;
		  break;
		}
	      }
	      if (ptInside) {
		totalInside+=x-xstart+1;
		break;
	      }
	    }
	  }
	}
      }
  
    }
  
    
    return totalInside*1.0f/totalPixelArea;
  }


  /*
   * Counts the number of border pixels of the blob that lie udner one of the lines
   */
  int countBorderPixelFit(BlobData &blob, const vector<Point> &corners, 
			  Sketch<bool> edges, ShapeSpace &space)
  {
    int ncorners = corners.size();

    vector<LineData> lines;
    vector<float> dx, dy;
    for (int i=0; i<ncorners; i++) {
      lines.push_back(LineData(space, corners[(i+1)%ncorners], corners[(i+2)%ncorners]));
    }

    int count = 0;

    for (int x=max((int)blob.topLeft.coordX()-5,0); x<=min((int)blob.topRight.coordX()+5,edges.width); x++) {
      for (int y=max((int)blob.topLeft.coordY()-5,0); y<=min((int)blob.bottomRight.coordY()+5,edges.height); y++) {
	if (edges(x,y)) {
	  Point p(x,y);
	  bool onLine = false;
	  for (int i=0; i<ncorners; i++) {
	    if (lines[i].pointOnLine(p)) {
	      onLine = true;
	      break;
	    }
	  }
	  if (onLine)
	    count++;
	}
      }
    }
    
    return count;
  }


  /*
   * Probabilistically pick the next move based on the scores
   * Lower is better for the score
   */
  int pickMove(vector<float> scores, float weight) 
  {
    unsigned int i;
    float min = scores[0], max = scores[0];
    for (i=1; i<scores.size(); i++) {
      if (scores[i] < min)
	min = scores[i];
      else if (scores[i] > max)
	max = scores[i];
    }
    
    max -= min;
    for (i=0; i<scores.size(); i++) {
      scores[i]-=min;
      if (max > 0) {
	scores[i]*=weight/max;
      }
    }

    vector<float> exps;
    float expsum = 0;
    for (i=0; i<scores.size(); i++) {
      float curexp = exp(-scores[i]);
      exps.push_back(curexp);
      expsum+=curexp;
    }

    for (i=0; i<scores.size(); i++) {
      exps[i]/=expsum;
    }    

    float randval = (rand()%1000000)/1000000.0f;

    for (i=0; i<scores.size(); i++) {
      randval -= exps[i];
      if (randval <= 0)
	return i;
    }

    return -1;
  }


} // namespace

