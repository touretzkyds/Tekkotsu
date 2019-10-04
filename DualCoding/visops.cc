//-*-c++-*-

#include <math.h>
#include "susan.h"

#include "visops.h"
#include "Vision/cmvision.h"

using namespace DualCoding;

namespace visops {

Sketch<bool> zeros(SketchSpace& space) {
  Sketch<bool> result(space,"zeros()");
  *result.pixels = 0;  // valarray assignment
  return result;
}

Sketch<bool> zeros(const SketchRoot& other) {
  const Sketch<bool>& fake = reinterpret_cast<const Sketch<bool>&>(other);
  Sketch<bool> result("zeros("+fake->getName()+")", fake);
  *result.pixels = 0;  // valarray assignment
  return result;
}

Sketch<bool> colormask(const Sketch<uchar>& src, const std::string &colorname) {
  return colormask(src, ProjectInterface::getColorIndex(colorname));
}

Sketch<bool> colormask(const Sketch<uchar>& src, color_index cindex) {
  Sketch<bool> result(src == cindex);
  result->setColor(ProjectInterface::getColorRGB(cindex));
  return result;
}

std::vector<xyPair> boundaryPoints(const Sketch<uint>& dest, uint& mindist, const uint maxval) {
  SketchSpace &space = dest->getSpace();
  const int width = space.getWidth();
  const int height = space.getHeight();
  std::vector<xyPair> boundaryPt;
  boundaryPt.clear();
  mindist = maxval;
  
  for (int i=1; i<width-1; i++)
    for (int j=1; j<height-1; j++)
      if (dest(i,j)<maxval)
      /*
        if ( dest(i-1,j-1)>=maxval || dest(i,j-1)>=maxval || dest(i+1,j-1)>=maxval || 
             dest(i-1,j)>=maxval || dest(i+1,j)>=maxval || 
             dest(i-1,j+1)>=maxval || dest(i,j+1)>=maxval || dest(i+1,j+1)>=maxval )
        {
          boundaryPt.push_back(xyPair(i,j));
          if (dest(i,j)<mindist) mindist = dest(i,j);
        }
      *//*
        if ( dest(i-1,j-1)==maxval || dest(i,j-1)==maxval || dest(i+1,j-1)==maxval || 
             dest(i-1,j)==maxval || dest(i+1,j)==maxval || 
             dest(i-1,j+1)==maxval || dest(i,j+1)==maxval || dest(i+1,j+1)==maxval )
        {
          boundaryPt.push_back(xyPair(i,j));
          if (dest(i,j)<mindist) mindist = dest(i,j);
        }
      */
        if ( dest(i,j-1)==maxval || 
             dest(i-1,j)==maxval || 
             dest(i+1,j)==maxval || 
             dest(i,j+1)==maxval )
        {
          boundaryPt.push_back(xyPair(i,j));
          if (dest(i,j)<mindist) mindist = dest(i,j);
        }
      
  return boundaryPt;
}

bool radiate(const xyPair center, Sketch<uint>& dist, const Sketch<bool>& dest, const Sketch<bool>& obst, const uint maxval) {
  SketchSpace &space = dist->getSpace();
  const int width = space.getWidth();
  const int height = space.getHeight();
  const int ctxiw = center.first;
  const int ctyjh = center.second;
  const uint centerdis = dist(ctxiw,ctyjh);
  
  float r[360];
  int theta[5];
  for (int i=0; i<360; i++) r[i]=maxval;
  for (int i=1; i<width-1; i++)
    for (int j=1; j<height-1; j++) {
      if (i==ctxiw && j== ctyjh) continue;
      //else if (obst(i,j)||dest(i,j)) {
      else if (obst(i,j)) {
        if ((i-1==ctxiw && j== ctyjh)) {r[0]=1;continue;}
        if ((i+1==ctxiw && j==ctyjh)) {r[180]=1;continue;}
        if ((i==ctxiw && j-1==ctyjh)) {r[270]=1;continue;}
        if ((i==ctxiw && j+1==ctyjh)) {r[90]=1;continue;}
        float temp = std::sqrt( (j-ctyjh)*(j-ctyjh) + (i-ctxiw)*(i-ctxiw) );
        theta[0] = int(std::atan2(j-ctyjh, i-ctxiw)*180/M_PI+360)%360;
        theta[1] = int(std::atan2(j-ctyjh, i+1-ctxiw)*180/M_PI+360)%360;
        theta[2] = int(std::atan2(j-1-ctyjh, i-ctxiw)*180/M_PI+360)%360;
        theta[3] = int(std::atan2(j-ctyjh, i-1-ctxiw)*180/M_PI+360)%360;
        theta[4] = int(std::atan2(j+1-ctyjh, i-ctxiw)*180/M_PI+360)%360;
        bool area1 = false;
        bool area4 = false;
        for (int k=0; k<5; k++) {
          if (theta[k] >=0 && theta[k] < 90) area1 = true;
          else if (theta[k] >=270 && theta[k] < 360) area4 = true;
        }
        if (area1 && area4) {
          for (int k=0; k<5; k++)
            if (theta[k]<=90) theta[k]+=360;
        }
        int start = 1000;
        int end = -1;
        for (int k=0; k<5; k++) {
          if (start>theta[k]) start = theta[k];
          if (end<theta[k]) end = theta[k];
        }
        for (int k=start; k<=end+1; k++)
          if (r[k%360] > temp) r[k%360] = temp;
      }
    }
  
  bool changed = false;
  for (int i=0; i<width; i++)
    for (int j=0; j<height; j++) {
      if (obst(i,j) || dest(i,j)) continue;
      int thet = int(std::atan2(j-ctyjh, i-ctxiw)*180/M_PI+360)%360;
      float temp = std::sqrt( (j-ctyjh)*(j-ctyjh) + (i-ctxiw)*(i-ctxiw) );
      //if ( temp<r[thet] && dist(i,j)>centerdis+temp && dist(i,j)!=maxval+1) {
      if ( temp<r[thet] && dist(i,j)>centerdis+temp) {
        dist(i,j) = centerdis+temp;
        changed = true;
      }
    }
  
  return changed;
}

Sketch<uint> ebdist(const Sketch<bool>& dest, const Sketch<bool>& obst, const uint maxdist, const uint time) {
  SketchSpace &space = dest->getSpace();
  const int width = space.getWidth();
  const int height = space.getHeight();
  const uint maxval = (uint)-2;
  
  Sketch<uint> distSk("ebdist("+dest->getName()+","+obst->getName()+")", dest);
  distSk = maxval;
  distSk->setColorMap(jetMapScaled);
  
  for (int i=0; i<width; i++)
    for (int j=0; j<height; j++)
      if (dest(i,j)) distSk(i,j)=0;
      
  for (int i=0; i<width; i++)
    for (int j=0; j<height; j++)
      if (obst(i,j)) distSk(i,j)=maxval+1;
      
  bool finish = false;
  uint t = 0;
  uint mindist;
  std::vector<xyPair> boundary = boundaryPoints(distSk, mindist, maxval);
  if (boundary.size() == 0) finish = true;
  else if (mindist >= maxval) finish = true;
  while (!finish) {
    bool changed = false;
    for(std::vector<xyPair>::iterator it=boundary.begin(); it!=boundary.end(); it++) {
      if (radiate(*it, distSk, dest, obst, maxval)) changed = true;
    }
    //for display only
    //for(std::vector<xyPair>::iterator it=boundary.begin(); it!=boundary.end(); it++) {
    //  distSk(it->first, it->second) = maxval+1;
    //}
    //end display
    t++;
    if (!changed) break;
    if (t >= time && time!=0) break;
    boundary = boundaryPoints(distSk, mindist, maxval);
    if (boundary.size() == 0) break;
    else if (mindist >= maxval) break;
  }
  
  std::cout << "Run radiate for " << t << " times." << std::endl;
  for (int i=0; i<width; i++)
    for (int j=0; j<height; j++)
      if ( distSk(i,j) > maxdist ) distSk(i,j)=maxdist;
      
  return distSk;
}

Sketch<uint> bdist(const Sketch<bool>& dest, 
			    const Sketch<bool>& obst, 
			    const uint maxdist) {
  SketchSpace &space = dest->getSpace();
  space.requireIdx4way();
  Sketch<uint> result("bdist("+dest->getName()+","+obst->getName()+")", dest);
  result = maxdist;
  result->setColorMap(jetMapScaled);
  
  // Start at the target and iteratively expand out the frontier
  SketchIndices frontier;
  frontier.addIndices(dest);
  result.setIndices(frontier, 0);
  SketchIndices obstInds;
  obstInds.addIndices(obst);
  SketchIndices newFrontier, oldFrontier;

  for (uint dist = 1; dist < maxdist; dist++) {
    // newFrontier should become all values adjacent to frontier, not
    // actually in the frontier, not a boundary, or in oldFrontier
    newFrontier = frontier[*space.idxN] + frontier[*space.idxS] 
      + frontier[*space.idxW] + frontier[*space.idxE]
      - (obstInds + frontier + oldFrontier);
    newFrontier.trimBounds(space);
    // if no more frontier, done
    if (newFrontier.table.empty())
      break;
    result.setIndices(newFrontier, dist);
    oldFrontier = frontier;
    frontier = newFrontier;
  }
  
  return result;
}

Sketch<uint> mdist(const Sketch<bool>& targetSk) {
  SketchSpace &space = targetSk->getSpace();
  const size_t width = space.getWidth();
  const size_t height = space.getHeight();
  const uint maxdist = width + height + 1;
  // at the moment doing 4-pass linear Manhattan distance, but should
  // probably use linear-time Euclidean algorithm described in 
  Sketch<uint> distSk("mdist("+targetSk->getName()+")", targetSk);
  distSk = maxdist;
  distSk->setColorMap(jetMapScaled);
  
  SketchData<bool>& target = *targetSk.data;
  SketchData<uint>& dist = *distSk.data;
  uint cur_dist;
  
  // find min distances within each row
  for (size_t j = 0; j < height; j++) {
    cur_dist = maxdist;
    const bool* targetrow = &target(0,j);
    uint* distrow = &dist(0,j);
    for (size_t i = 0; i < width; i++) {
      if (targetrow[i] == 1)
				distrow[i] = cur_dist = 0;
      else if (distrow[i] < (uint)cur_dist)
				cur_dist = distrow[i];
      else distrow[i] = cur_dist;
      cur_dist++;
    }
    cur_dist = maxdist;
    for (size_t i = width; i > 0; ) {
      --i;
      if (targetrow[i] == true)
	distrow[i] = cur_dist = 0;
      else if (distrow[i] < (uint)cur_dist)
	cur_dist = distrow[i];
      else
	distrow[i] = cur_dist;
      cur_dist++;
    }
  }
  // find min distances within each column
  for (size_t i = 0; i < width; i++) {
    cur_dist = maxdist;
    const bool* targetcol = &target(i,0);
    uint* distcol = &dist(i,0);
    for (size_t j = 0; j < height*width; j+=width) {
      if (targetcol[j] == 1)
				distcol[j] = cur_dist = 0;
      else if (distcol[j] < (uint)cur_dist)
				cur_dist = distcol[j];
      else
				distcol[j] = cur_dist;
      cur_dist++;
    }
    cur_dist = maxdist;
    for (size_t j = height*width; j  > 0; ) {
      j-=width;
      if (targetcol[j] == 1)
				distcol[j] = cur_dist = 0;
      else if (distcol[j] < (uint)cur_dist)
				cur_dist = distcol[j];
      else
				distcol[j] = cur_dist;
      cur_dist++;
    }
  }
  return distSk;
}

  Sketch<float> edist(const Sketch<bool>& targetSk) {
    SketchSpace &space = targetSk->getSpace();
    const size_t width = space.getWidth();
    const size_t height = space.getHeight();
    const uint maxdist = width + height + 1;
	
    Sketch<uint> xVals("xVals(" + targetSk->getName() + ")", targetSk);
    Sketch<uint> yVals("yVals(" + targetSk->getName() + ")", targetSk);
    xVals = maxdist;
    yVals = maxdist;
    xVals->setColorMap(jetMapScaled);
    yVals->setColorMap(jetMapScaled);
    Sketch<float> distSk("edist("+targetSk->getName()+")", targetSk);
    distSk = maxdist;
    distSk->setColorMap(jetMapScaled);
	
    uint cur_xdist, cur_ydist;
    float cur_dist, this_dist;

    // There is probably a smarter way to compute this.  The present
    // algorithm is copied from mdist but requires at least two passes
    // because Euclidean distance requires us to maintain separate
    // xVal and yVal arrays.

    for (int pass=0; pass<2; pass++) {
      // find min distances within each row
      for (size_t j = 0; j < height; j++) {
	cur_xdist = maxdist;
	cur_ydist = maxdist;
	for (size_t i = 0; i < width; i++) {
	  if (targetSk(i,j) == true) {
	    xVals(i,j) = cur_xdist = 0;
	    yVals(i,j) = cur_ydist = 0;
	  } else {
	    cur_dist = cur_xdist*cur_xdist + cur_ydist*cur_ydist;
	    this_dist = xVals(i,j)*xVals(i,j) + yVals(i,j)*yVals(i,j);
	    if (this_dist < cur_dist) {
	      cur_xdist = xVals(i,j);
	      cur_ydist = yVals(i,j);
	    } else {
	      xVals(i,j) = cur_xdist;
	      yVals(i,j) = cur_ydist;
	    }
	  }
	  cur_xdist++;
	}
	cur_xdist = maxdist;
	cur_ydist = maxdist;
	for (size_t i = width; i > 0; ) {
	  i--;
	  if (targetSk(i,j) == true) {
	    xVals(i,j) = cur_xdist = 0;
	    yVals(i,j) = cur_ydist = 0;
	  } else {
	    cur_dist = cur_xdist*cur_xdist + cur_ydist*cur_ydist;
	    this_dist = xVals(i,j)*xVals(i,j) + yVals(i,j)*yVals(i,j);
	    if (this_dist < cur_dist) {
	      cur_xdist = xVals(i,j);
	      cur_ydist = yVals(i,j);
	    } else {
	      xVals(i,j) = cur_xdist;
	      yVals(i,j) = cur_ydist;
	    }
	  }
	  cur_xdist++;
	}
      }
	
      // find min distances within each column
      for (size_t i = 0; i < width; i++) {
	cur_xdist = maxdist;
	cur_ydist = maxdist;
		
	for (size_t j = 0; j < height; j++) {
	  if (targetSk(i,j) == true) {
	    xVals(i,j) = cur_xdist = 0;
	    yVals(i,j) = cur_ydist = 0;
	  } else {
	    cur_dist = cur_xdist*cur_xdist + cur_ydist*cur_ydist;
	    this_dist = xVals(i,j)*xVals(i,j) + yVals(i,j)*yVals(i,j);
	    if (this_dist < cur_dist) {
	      cur_xdist = xVals(i,j);
	      cur_ydist = yVals(i,j);
	    } else {
	      xVals(i,j) = cur_xdist;
	      yVals(i,j) = cur_ydist;
	    }
	  }
	  cur_ydist++;
	}
	cur_xdist = maxdist;
	cur_ydist = maxdist;
	for (size_t j = height; j > 0; ) {
	  j--;
	  if (targetSk(i,j) == true) {
	    xVals(i,j) = cur_xdist = 0;
	    yVals(i,j) = cur_ydist = 0;
	  } else {
	    cur_dist = cur_xdist*cur_xdist + cur_ydist*cur_ydist;
	    this_dist = xVals(i,j)*xVals(i,j) + yVals(i,j)*yVals(i,j);
	    if (this_dist < cur_dist) {
	      cur_xdist = xVals(i,j);
	      cur_ydist = yVals(i,j);
	    } else {
	      xVals(i,j) = cur_xdist;
	      yVals(i,j) = cur_ydist;
	    }
	  }
	  cur_ydist++;
	}
      }
    }
	
    for(unsigned i = 0; i < width; i++)
      for(unsigned j = 0; j < height; j++) {
	float x = (float)xVals(i,j), y = (float)yVals(i,j);
	distSk(i,j) = std::sqrt(x*x + y*y);
      }
	
    return distSk;
  }
	
Sketch<uint> labelcc(const Sketch<bool>& sketch, int minarea) {
  Sketch<uchar> temp;
  uchar *pixels;
  if ( sizeof(bool) == sizeof(uchar) )
    pixels = reinterpret_cast<uchar*>(&((*sketch.pixels)[0]));
  else {
    temp.bind((Sketch<uchar>)sketch);
    pixels = &((*temp.pixels)[0]);
  }

  // convert pixel array to RLE
  int const maxRuns = (sketch.width * sketch.height) / 8;
  CMVision::run<uchar> *rle_buffer = new CMVision::run<uchar>[maxRuns];
  unsigned int const numRuns = CMVision::EncodeRuns(rle_buffer, pixels, sketch.width, sketch.height, maxRuns);

  // convert RLE to region list
  CMVision::ConnectComponents(rle_buffer,numRuns);
  int const maxRegions = (sketch.width * sketch.height) / 16;   // formula from RegionGenerator.h
  CMVision::region *regions = new CMVision::region[maxRegions];
  unsigned int numRegions = CMVision::ExtractRegions(regions, maxRegions, rle_buffer, numRuns);
  int const numColors = 2;  // only two colors in a Sketch<bool>: 0 and 1
  CMVision::color_class_state *ccs = new CMVision::color_class_state[numColors];
  unsigned int const maxArea = CMVision::SeparateRegions(ccs, numColors, regions, numRegions);
  CMVision::SortRegions(ccs, numColors, maxArea);
  CMVision::MergeRegions(ccs, numColors, rle_buffer);

  // extract regions from region list
  NEW_SKETCH_N(result, uint, visops::zeros(sketch));
  result->setColorMap(jetMapScaled);
  const CMVision::region* list_head = ccs[1].list;
  if ( list_head != NULL ) {
    for (int label=1; list_head!=NULL && list_head->area >= minarea;
	   list_head = list_head->next, label++) {
      // the first run might be array element 0, so use -1 as end of list test
      for (int runidx = list_head->run_start; runidx != -1;
	   runidx = rle_buffer[runidx].next ? rle_buffer[runidx].next : -1) {
	const CMVision::run<uchar> &this_run = rle_buffer[runidx];
	const int xstop = this_run.x + this_run.width;
	const int yi = this_run.y;
	for ( int xi = this_run.x; xi < xstop; xi++ )
	  result(xi,yi) = label * sketch(xi,yi); // the * undoes some of CMVision's noise removal
      }
    }
  }

  delete[] ccs;
  delete[] regions;
  delete[] rle_buffer;
  return result;
}

// written with guidance from this page: http://www.dai.ed.ac.uk/HIPR2/label.htm
Sketch<uint> oldlabelcc(const Sketch<bool>& source, Connectivity_t connectivity)
{
	bool conn8 = (connectivity == EightWayConnect);
	const int width = source.width;
	const int height = source.height;
	Sketch<uint> labels("oldlabelcc("+source->getName()+")",source);
	labels = 0;
	labels->setColorMap(jetMapScaled);
	uint* data = labels->getRawPixels();
	const bool* srcdata = source->getRawPixels();
	
	// First scan: Give initial labels and sort connected label classes
	// into equivalence classes using UNION-FIND
	// Doing something similar to tree-based UNION-FIND, without the tree
	std::vector<int> eq_classes(500); // vector of equivalence classes for union-find
	eq_classes.clear();
	eq_classes.push_back(0); // added just so that indices match up with labels
	int highest_label = 0;
	int up_label = 0; // value above current pixel
	int left_label = 0; // value to left of current pixel
	int ul_label = 0; // value to upper-left of current pixel
	int ur_label = 0; // value to upper-right of current pixel
	for(int j = 0; j < height; j++) {
		uint* row = &data[j*width];
		const bool* srcrow = &srcdata[j*width];
		for(int i = 0; i < width; i++) {
			uint* p = &row[i];
			if (srcrow[i]) {
				up_label = (j == 0) ? 0 : *(p-width);
				left_label = (i==0) ? 0 : *(p-1); 
				ul_label = (i==0||j==0) ? 0 : *(p-1-width);
				ur_label = (i==(width-1)||j==0) ? 0 : *(p+1-width);
				if (up_label == 0 && left_label == 0 && (!conn8 || (ul_label == 0 && ur_label == 0))) {
					*p = ++highest_label;  // create new label
					// push back a new root label
					eq_classes.push_back(highest_label); // label value will be equal to index
				} else if (up_label && !left_label) {
					*p = up_label;
				} else if (conn8 && !up_label && ur_label) {
					*p = ur_label;
				} else if (left_label && !up_label) {
					*p = left_label;
				} else if (conn8 && !left_label && !up_label && ur_label && !ul_label) {
					*p = ur_label; 
				} else if (conn8 && !left_label && !up_label && ul_label && !ur_label){
					*p = ul_label;	
				}
				
				if (up_label && left_label && (up_label != left_label)) {
					// form union between two equivalence classes
					// if upper-left, assume equivalence class already made
					
					int root = up_label;
					while (eq_classes[root] != root) {
						root = eq_classes[root]; // "FIND" of UNION-FIND
					}
					// should do path compression to make more efficient
					int tmp_root = up_label, next_root;
					while(eq_classes[tmp_root] != root) {
						next_root = eq_classes[tmp_root];
						eq_classes[tmp_root] = root; // compress
						tmp_root = next_root;
					}
					
					eq_classes[left_label] = root; // "UNION" of UNION-FIND
					*p = root; // not sure why putting this here works, but it does
				} else if (up_label && (up_label == left_label)) {
					*p = up_label;	
				} else if (conn8 && ur_label && left_label && (ur_label != left_label)) {
					// form union between two equivalence classes
					int root = ur_label;
					while (eq_classes[root] != root) {
						root = eq_classes[root]; // "FIND" of UNION-FIND
					}
					// should do path compression to make more efficient
					int tmp_root = ur_label, next_root;
					while(eq_classes[tmp_root] != root) {
						next_root = eq_classes[tmp_root];
						eq_classes[tmp_root] = root; // compress
						tmp_root = next_root;
					}
					
					eq_classes[left_label] = root; // "UNION" of UNION-FIND
					*p = root; // not sure why putting this here works, but it does
				}
			}
		}
	}
	
	// Second scan: 
	uint *p = data, *end = &data[width*height];
	for(; p!=end; ++p) {
		int cur_label=*p;
		if (cur_label != 0) {
			while(eq_classes[cur_label] != cur_label) {
				cur_label = eq_classes[cur_label];	
			}
			*p = cur_label;
		}
	}
	
	return labels;
}

Sketch<uint> areacc(const Sketch<bool>& source, Connectivity_t connectivity) {
  NEW_SKETCH_N(labels, uint, visops::oldlabelcc(source,connectivity));
  return visops::areacc(labels);
}

Sketch<uint> areacc(const Sketch<uint>& labels) {
	int const numlabels = 1 + labels->max();
  std::vector<int> areas(numlabels, 0);
  for (uint i = 0; i<labels->getNumPixels(); i++)
    ++areas[labels[i]];
  areas[0] = 0;
  Sketch<uint> result("areacc("+labels->getName()+")",labels);
  for (uint i = 0; i<labels->getNumPixels(); i++)
    result[i] = areas[labels[i]];
  return result;
}

Sketch<bool> minArea(const Sketch<bool>& sketch, int minval) {
  NEW_SKETCH_N(labels, uint, visops::labelcc(sketch));
  NEW_SKETCH_N(areas, uint, visops::areacc(labels));
  NEW_SKETCH_N(result, bool, areas >= minval);
  return result;
}

Sketch<uchar> neighborSum(const Sketch<bool>& im, Connectivity_t connectivity) 
{
	im.checkValid();
	const bool* imdata = im->getRawPixels();
	// using index redirection method
	SketchSpace &space = im->getSpace();
	
	space.requireIdx4way();
	
	skindex* idxN = (*space.idxN)->getRawPixels();
	skindex* idxS = (*space.idxS)->getRawPixels();
	skindex* idxE = (*space.idxE)->getRawPixels();
	skindex* idxW = (*space.idxW)->getRawPixels();
	skindex *idxNE=NULL, *idxNW=NULL, *idxSE=NULL, *idxSW=NULL;
	
	if (connectivity == EightWayConnect) {
		space.requireIdx8way();
		idxNE = (*space.idxNE)->getRawPixels();
		idxNW = (*space.idxNW)->getRawPixels();
		idxSE = (*space.idxSE)->getRawPixels();
		idxSW = (*space.idxSW)->getRawPixels();
	}
	
	Sketch<uchar> result("neighborSum("+im->getName()+")", im);
	uchar* rdata = result->getRawPixels();
	result->setColorMap(jetMapScaled);
	const unsigned int length = im->getNumPixels();
	for ( unsigned int i = 0; i < length; i++ ) {
		uchar cnt = imdata[idxN[i]] + imdata[idxS[i]] + imdata[idxE[i]] + imdata[idxW[i]];
		if (connectivity == EightWayConnect)
			cnt += imdata[idxNE[i]] + imdata[idxNW[i]] + imdata[idxSE[i]] + imdata[idxSW[i]];
		rdata[i] = cnt;
	}
	return result;
}
		
Sketch<bool> fillin(const Sketch<bool>& im, int iter, 
			    uchar min_thresh, uchar max_thresh, bool remove_only)
{
  Sketch<bool> result(im);
  if ( remove_only )
    result.bind(visops::copy(im));
  Sketch<uchar> neighborCount(im);
  if (remove_only) {
    neighborCount.bind(neighborSum(result,EightWayConnect));
    result &= (neighborCount <= max_thresh);
    for (int i = 0; i < iter; i++)
      result &= (neighborCount >= min_thresh);
  }
  else {
    for (int i = 0; i < iter; i++) {
      neighborCount.bind(neighborSum(result,EightWayConnect));
      result.bind((neighborCount >= min_thresh) & (neighborCount <= max_thresh));
    }
  }
  result->setName("fillin("+im->getName()+")");
  result->setColor(im->getColor());
  return result;
}

Sketch<bool> edge(const Sketch<bool> &im) {
  im->getSpace().requireIdx4way();
  SketchSpace &space = im->getSpace();
  return ((im != im[*space.idxS]) | (im != im[*space.idxE]));
}


Sketch<bool> horsym(const Sketch<bool> &im, size_t minskip, size_t maxskip)
{
  NEW_SKETCH_N(result, bool, visops::zeros(im));
  result->setName("horsym("+im->getName()+")");
  bool * rdata = result->getRawPixels();
  const bool * imdata = im->getRawPixels();
  const size_t width = im->getWidth();
  const size_t height = im->getHeight();

  for (size_t j = 0; j < height; j++) {
    const bool * imrow = &imdata[j*width];
    for (size_t i = 0; i < width; i++) {
      while (i < width && !imrow[i]) i++; // skip over empty pixels
      for (size_t k = i+1; k <= width; k++) {
	  if ( k==width || !imrow[k]) {
	    if ( (k-i) >= minskip && (k-i) <= maxskip ) {
	      const size_t u = (i+k)/2;
	      rdata[j*width+u] = true;
	    }
	    i=k+1;
	    break;
	  }
      }
    }
  }
  return result;
}

Sketch<bool> versym(const Sketch<bool> &im, size_t minskip, size_t maxskip)
{
  NEW_SKETCH_N(result, bool, visops::zeros(im));
  result->setName("horsym("+im->getName()+")");
  bool * rdata = result->getRawPixels();
  const bool * imdata = im->getRawPixels();
  const size_t width = im->getWidth();
  const size_t height = im->getHeight();

  for (size_t i = 0; i < width; i++) {
    const bool * imcol = &imdata[i];
    for (size_t j = 0; j < height; j++) {
      while (j < height && !imcol[j*width]) j++; // skip over empty pixels
      for (size_t k = j+1; k <= height; k++) {
	  if ( k==height || !imcol[k*width]) {
	    if ( (k-j) >= minskip && (k-j) <= maxskip ) {
	      const size_t u = (j+k)/2;
	      rdata[u*width+i] = true;
	    }
	    j=k+1;
	    break;
	  }
      }
    }
  }
  return result;
}


/*
Sketch<bool> versym(const Sketch<bool>& im, int minskip, int maxskip)
{
  NEW_SKETCH_N(result, bool, visops::zeros(im));
  result->setName("versym("+im->getName()+")");
  int height = im->getWidth();
  int width = im->getHeight();
  
  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      if (im(j,i)) {
	while (i < width-1 && im(j,i+1)) {
	  i++; // skip over contiguous pixels
	}
	for (int k = i+1; k < width; k++) {
	  if (k-i > maxskip)
	    break;
	  else if (im(j,k)) {
	    if ((k-i) < minskip)
	      break;
	    int u = (i+k)/2;
	    if (!im(j,u))
	      result(j,u) = true; // was result(j,u) = k-i when this returned a Sketch<int>
	    break; // only works well for non-'donut' ellipses
	  }
	}
      }
    }
  }
  return result;
}
*/

Sketch<bool> skel(const Sketch<bool>& im) {
  NEW_SKETCH_N(result, bool, horsym(im) | versym(im));
  result->setName("skel("+im->getName()+")");
  return result;
}

Sketch<bool> seedfill(const Sketch<bool>& borders, size_t index) {
  // use four-way connect so thin diagonal line can function as a boundary
  NEW_SKETCH_N(regions, uint, oldlabelcc(! borders, visops::FourWayConnect));
  NEW_SKETCH_N(result, bool, regions == regions->at(index));  // use at() to do bounds checking
  return result;
}

// helper function called only inside visops::fillExterior
void fillExteriorHelper(const Sketch<uint> &regions, Sketch<bool> &result, std::vector<bool> &processed, 
			const int x, const int y) {
  const uint c = regions(x,y);
  if ( c > 0 && !processed[c] ) {
    result |= (regions == c);
    processed[c] = true;
  }
}

Sketch<bool> fillExterior(const Sketch<bool>& borders) {
  // use four-way connect so thin diagonal line can function as a boundary
  NEW_SKETCH_N(regions, uint, oldlabelcc(! borders, visops::FourWayConnect));
  const uint numreg = regions->max();
  std::vector<bool> processed(numreg+1,false);
  NEW_SKETCH_N(result, bool, visops::zeros(borders));
  for ( int x = 0; x < result.width; x++ ) {
    fillExteriorHelper(regions,result,processed,x,0);
    fillExteriorHelper(regions,result,processed,x,result.height-1);
  }
  for ( int y = 0; y < result.height; y++ ) {
    fillExteriorHelper(regions,result,processed,0,y);
    fillExteriorHelper(regions,result,processed,result.width-1,y);
  }
  return result;
}

Sketch<bool> fillInterior(const Sketch<bool>& borders) {
  return ! (borders | fillExterior(borders));
}

Sketch<bool> leftHalfPlane(const Shape<LineData> &ln) {
  SketchSpace &SkS = ln->getSpace().getDualSpace();
  //! @todo **** THIS visops::leftHalfPlane CODE NEEDS TO CHECK THE SketchSpace ReferenceFrameType **** BECAUSE "left" MEANS DIFFERENT THINGS IN DIFFERENT FRAMES 
  float const x1 = ln->end1Pt().coordX();
  float const y1 = ln->end1Pt().coordY();
  float const x2 = ln->end2Pt().coordX();
  float const y2 = ln->end2Pt().coordY();
  bool const roughlyVertical = fabs(x1-x2) < 0.001;
  float const m = roughlyVertical ? BIG_SLOPE : (y2-y1) / (x2-x1);
  float const b = y1 - x1*m;
  int seed;
  if ( roughlyVertical )
    seed = ( x1 <= 0 ) ? -1 : 0;
  else if ( ln->getOrientation() > M_PI/2 )
    seed =  ( b <= 0) ? -1 : 0;
  else {
    int const lim = SkS.getHeight() - 1;
    seed =  ( b < lim ) ? (int)(*SkS.idx)(0,lim) : -1;
	}
  if ( seed == -1 ) {
    NEW_SKETCH_N(result, bool, visops::zeros(SkS));
    result->inheritFrom(ln);
    return result;
  } else {
    NEW_SHAPE_N(line_copy, LineData, ln->copy());
    line_copy->setInfinite();
    NEW_SKETCH_N(bounds, bool, line_copy->getRendering());
    bounds->inheritFrom(ln);
    return visops::seedfill(bounds,seed);
  }
}

Sketch<bool> rightHalfPlane(const Shape<LineData> &ln) {
    NEW_SHAPE_N(line_copy, LineData, ln->copy());
    line_copy->setInfinite();
    NEW_SKETCH_N(bounds, bool, line_copy->getRendering());
    bounds->inheritFrom(ln);
    return ! (visops::leftHalfPlane(ln) | bounds);
}

Sketch<bool> topHalfPlane(const Shape<LineData> &ln) {
  SketchSpace &SkS = ln->getSpace().getDualSpace();
  //! @todo **** visops::topHalfPlane needs to check the SketchSpace ReferenceFrameType because "left" means different things in different reference frames 
  float const x1 = ln->end1Pt().coordX();
  float const y1 = ln->end1Pt().coordY();
  float const x2 = ln->end2Pt().coordX();
  float const y2 = ln->end2Pt().coordY();
  bool const roughlyVertical = fabs(x1-x2) < 0.001;
  float const m = roughlyVertical ? BIG_SLOPE : (y2-y1) / (x2-x1);
  float const b = y1 - x1*m;
  int seed;
  if ( roughlyVertical )
    seed = ( y1 <= 0 ) ? -1 : 0;
  else if ( ln->getOrientation() > M_PI/2 )
    seed =  ( b <= 0) ? -1 : 0;
  else {
    int const lim = SkS.getWidth() - 1;
    seed =  ( int(m*lim+b) > 0 ) ? (int)(*SkS.idx)(lim,0) : -1;
	}
  if ( seed == -1 ) {
    NEW_SKETCH_N(result, bool, visops::zeros(SkS));
    result->inheritFrom(ln);
    return result;
  } else {
    NEW_SHAPE_N(line_copy, LineData, ln->copy());
    line_copy->setInfinite();
    NEW_SKETCH_N(bounds, bool, line_copy->getRendering());
    bounds->inheritFrom(ln);
    return visops::seedfill(bounds,seed);
  }
}

Sketch<bool> bottomHalfPlane(const Shape<LineData> &ln) {
    NEW_SHAPE_N(line_copy, LineData, ln->copy());
    line_copy->setInfinite();
    NEW_SKETCH_N(bounds, bool, line_copy->getRendering());
    bounds->inheritFrom(ln);
    return ! (visops::topHalfPlane(ln) | bounds);
}

Sketch<bool> non_bounds(const Sketch<bool>& im, int offset) {
  const int width = im->getWidth();
  const int height = im->getHeight();
  NEW_SKETCH_N(nbresult,bool,visops::copy(im));
  nbresult->setName("non_bounds("+im->getName()+")");

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < offset; j++) {
      nbresult(i,j) = false;
      nbresult(i,height-j-1) = false;
    }
  }
  for (int i = 0; i < offset; i++) {
    for (int j = offset; j < height-offset; j++) {
      nbresult(i,j) = false;
      nbresult(width-i-1,j) = false;
    }
  }
  return nbresult;
}


Sketch<uchar> susan_edges(const Sketch<uchar>& im, int brightness)
{
  const int width = im->getWidth();
  const int height = im->getHeight();
  unsigned char *bp;
  Sketch<uchar> edges(visops::copy(im));

  int *r = (int *)malloc(width*height*sizeof(int));

  unsigned char *mid = (unsigned char *)malloc(width*height);
  memset (mid,100,width * height); /* note not set to zero */

  setup_brightness_lut(&bp,brightness,6);

  // susan_principle(im->getRawPixels(),edges->getRawPixels(), &bp, 2650, width, height);

  susan_edges_internal(edges->getRawPixels(), r, mid, bp, 2650, width, height);

  susan_thin(r, mid, width, height);

  edge_draw(edges->getRawPixels(),mid,width,height,0);

   free(r);
   free(mid);
   free(bp-258);

  return edges;
}


// Default brightness was 20 for original algorithm
Sketch<bool> susan_edge_points(const Sketch<uchar>& im, int brightness)
{
  const int width = im->getWidth();
  const int height = im->getHeight();
  unsigned char *bp;
  Sketch<uchar> orig(im);
  Sketch<uchar> edges(visops::zeros(im));
  int *r = (int *)malloc(width*height*sizeof(int));
  unsigned char *mid = (unsigned char *)malloc(width*height);
  memset(mid,100,width * height); /* note not set to zero */
  setup_brightness_lut(&bp,brightness,6);
  susan_edges_internal(orig->getRawPixels(), r, mid, bp, 2650, width, height);
  susan_thin(r, mid, width, height);
  edge_draw(edges->getRawPixels(),mid,width,height,1);
  free(r);
  free(mid);
  Sketch<bool> result(edges);
  return result;
}

Sketch<uint> convolve(const Sketch<uchar> &sketch, Sketch<uchar> &kernel, 
		       int istart, int jstart, int width, int height) {
  Sketch<uint> result("convolve("+sketch->getName()+")",sketch);
  result->setColorMap(jetMapScaled);
  int const di = - (int)(width/2);
  int const dj = - (int)(height/2);
  for (int si=0; si<sketch.width; si++)
    for (int sj=0; sj<sketch.height; sj++) {
      int sum = 0;
      for (int ki=0; ki<width; ki++)
	for (int kj=0; kj<height; kj++)
	  if ( si+di+ki >= 0 && si+di+ki < sketch.width &&
	       sj+dj+kj >= 0 && sj+dj+kj < sketch.height )
	    sum += (uint)sketch(si+di+ki,sj+dj+kj) * (uint)kernel(istart+ki,jstart+kj);
      result(si,sj) = sum/(width*height);
    }
  return result;      
}

Sketch<uint> templateMatch(const Sketch<uchar> &sketch, Sketch<uchar> &kernel, 
		       int istart, int jstart, int width, int height) {
  Sketch<uint> result("convolve0("+sketch->getName()+")",sketch);
  result->setColorMap(jetMapScaled);
  int const npix = width * height;
  int const di = - (int)(width/2);
  int const dj = - (int)(height/2);
  for (int si=0; si<sketch.width; si++)
    for (int sj=0; sj<sketch.height; sj++) {
      int sum = 0;
      for (int ki=0; ki<width; ki++)
	for (int kj=0; kj<height; kj++) {
	  int k_pix = kernel(istart+ki,jstart+kj);
	  if ( si+di+ki >= 0 && si+di+ki < sketch.width &&
	       sj+dj+kj >= 0 && sj+dj+kj < sketch.height ) {
	    int s_pix = sketch(si+di+ki,sj+dj+kj);
	    sum +=  (s_pix - k_pix) * (s_pix - k_pix);
	  }
	  else
	    sum += k_pix * k_pix;
	}
      result(si,sj) =  65535 - uint(sqrt(sum/float(npix)));
    }
  result = result - result->min();
  return result;
}


} // namespace
