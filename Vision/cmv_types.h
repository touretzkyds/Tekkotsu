#ifndef __CMVISION_TYPES_H__
#define __CMVISION_TYPES_H__

/*! @file
* @brief Base types for #CMVision
* @author James R. Bruce, School of Computer Science, Carnegie Mellon University
*
* Licensed under the <a href="../gpl-2.0.txt">GNU GPL version 2</a>
*/

#include "colors.h"

namespace CMVision{

  //typedef unsigned char uchar;

// uncomment if your compiler supports the "restrict" keyword
// #define restrict __restrict__
#define restrict

template <class pixel>
class image{
public:
  pixel *buf;
  int width,height,row_stride;
};

template <class pixel>
class image_idx{
public:
  pixel *buf;
  int width,height,row_stride;
};

template <class element>
class image_yuv {
public:
  element *buf_y,*buf_u,*buf_v;
  int width,height;
  int row_stride; // number of elements to skip to move one row in buf_[yuv] (handle row interleaving)
  int col_stride; // number of elements to move one column in buf_[yuv] (handle pixel interleaving)
};

#define CMV_PARAM <class pixel,class vimage,class threshold>
#define CMV_TEM template CMV_PARAM

#define CMV_NONE ((unsigned)(-1))

/*
// Options for level of processing
//   use enable()/disable() to change 
#define CMV_THRESHOLD      0x01
#define CMV_COLOR_AVERAGES 0x02
#define CMV_DUAL_THRESHOLD 0x04
#define CMV_DENSITY_MERGE  0x08

#define CMV_VALID_OPTIONS  0x0F
*/

template <class cclass>
class run{
public:
  short x,y,width;    // location and width of run
  cclass color;       // which color(s) this run represents
  int parent,next;    // parent run and next run in run list
};

template <class cclass>
class run_mini{
public:
  short x,y,width;    // location and width of run
  cclass color;       // which color(s) this run represents
  short parent,next;  // parent run and next run in run list
};

struct region{
  int color;         // id of the color
  int x1,y1,x2,y2;   // bounding box (x1,y1) - (x2,y2)
  float cen_x,cen_y; // centroid
  int area;          // occupied area in pixels
  int run_start;     // first run index for this region
  int iterator_id;   // id to prevent duplicate hits by an iterator
  region *next;      // next region in list
};

struct region_small{
  short color;       // id of the color
  short x1,y1,x2,y2; // bounding box (x1,y1) - (x2,y2)
  short iterator_id; // id to prevent duplicate hits by an iterator
  float cen_x,cen_y; // centroid
  int area;          // occupied area in pixels
  int run_start;     // first run index for this region
  region *next;      // next region in list
};

struct region_tiny{
  uchar color;       // id of the color
  uchar x1,y1,x2,y2; // bounding box (x1,y1) - (x2,y2)
  float cen_x,cen_y; // centroid
  short area;        // occupied area in pixels
  short run_start;   // first run index for this region
  short iterator_id; // id to prevent duplicate hits by an iterator
  region *next;      // next region in list
};

struct color_class_state{
	color_class_state() : list(), num(), min_area(), total_area(), merge_threshold(), color(), name() {}
	color_class_state(const color_class_state& c) : list(c.list), num(c.num), min_area(c.min_area), total_area(c.total_area), merge_threshold(c.merge_threshold), color(c.color), name(c.name) {}
	color_class_state& operator=(const color_class_state& c) { list=c.list; num=c.num; min_area=c.min_area; total_area=c.total_area; merge_threshold=c.merge_threshold; color=c.color; name=c.name; return *this;}
  region *list;          // head of region list for this color
  int num;               // number of regions of this color
  int min_area;          // minimum area for a meaningful region
  int total_area;        // total area covered by color
  float merge_threshold; // minimum density of merged region
  rgb color;             // example color (such as used in test output)
  char *name;            // color's meaningful name (e.g. orange ball, goal)
};

} // namespace

#endif
