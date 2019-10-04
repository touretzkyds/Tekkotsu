#ifndef __CMV_REGION_H__
#define __CMV_REGION_H__

/*! @file
* @brief Region and connected component support for #CMVision
* @author James R. Bruce, School of Computer Science, Carnegie Mellon University
*
* Licensed under the <a href="../gpl-2.0.txt">GNU GPL version 2</a>
*/

//@todo this should go away - ET
const unsigned int MAX_COLORS=20;


#include <stdio.h>
//#include "Shared/Util.h"
//#include "Visiondefines.h"

#include "cmv_types.h"
#include <map>

struct hashcmp_eqstr { bool operator()(const char* s1, const char* s2) const
                     { return strcasecmp(s1, s2) == 0; } };

namespace CMVision{

template<class T>
class DummyT1 {
};

//==== Utility Functions ===========================================//

// sum of integers over range [x,x+w)
inline int range_sum(int x,int w)
{
  return(w*(2*x + w-1) / 2);
}

// table used by bottom_bit
const int log2modp[37] = {
  0, 1, 2,27, 3,24,28, 0, 4,17,25,31,29,12, 0,14, 5, 8,18,
  0,26,23,32,16,30,11,13, 7, 0,22,15,10, 6,21, 9,20,19
};

// returns index of least significant set bit
template <class num>
inline int bottom_bit(num n)
{
  return(log2modp[(n & -n) % 37]);
}

// returns index of most significant set bit
template <class num>
inline num top_bit(num n)
{
  /*
  n = n | (n >>  1);
  n = n | (n >>  2);
  n = n | (n >>  4);
  n = n | (n >>  8);
  n = n | (n >> 16);
  n = n - (n >>  1);
  return(log2modp[n % 37]);
  */

  n |= (n >> 1) | (n >> 2) | (n >>  3);
  n |= (n >> 4) | (n >> 8) | (n >> 12);
  n |= (n >> 16);
  n -= (n >>  1);
  return(log2modp[n % 37]);

  /*
  int i = 1;
  if(!n) return(0);
  while(n>>i) i++;
  return(i);
  */
}

//==== Main Library Functions ======================================//

#define REMOVE_NOISE

template <class rle_t,class tmap_t>
int EncodeRuns(rle_t *rle,tmap_t *map,int width,int height,int max_runs)
// Changes the flat array version of the thresholded image into a run
// length encoded version, which speeds up later processing since we
// only have to look at the points where values change.
{
  tmap_t m,save;
  tmap_t *row;
  int x,y,j,l;
 
#ifdef REMOVE_NOISE
  int lastcolor;
  int noise;
  int noise_back;
#endif
  
  rle_t r;

  r.next = 0;

  // initialize terminator restore
  save = map[0];

  j = 0;

  for(y=0; y<height; y++){
    row = &map[y * width];

    // restore previous terminator and store next
    // one in the first pixel on the next row
    row[0] = save;
    save = row[width];
    row[width] = MAX_COLORS;
    r.y = y;

    x = 0;

#ifdef REMOVE_NOISE
    lastcolor=0;
    noise=0;
    noise_back=0;
#endif
    while(x < width){
      m = row[x];
      r.x = x;

      l = x;
      while(row[x] == m) x++;

#ifdef REMOVE_NOISE
      if (x - l > 4) {
        if (noise>=4) {
          j=j-noise_back;
          lastcolor=0;
        }
        noise=0; 
        noise_back=0;
      } else {
        noise++;
        if (m) noise_back++;
      }
#endif

      if(m!=0 || x>=width){
        r.color = m; 
        r.width = x - l;
        r.parent = j;
        rle[j++] = r;

        if(j >= max_runs) {
          row[width] = save;
          return(j);
        }
      }
#ifdef REMOVE_NOISE
      else if (!m && lastcolor && x-l<5) {
        rle[j-1].width+=x-l;
      }
      
      lastcolor=m;
#endif
    }
  }

  return(j);
}

template <class rle_t,class tmap_t,class edge_t>
int EncodeRunsUseEdges(rle_t *rle,tmap_t *map,edge_t *edge_map,int width,int height,int max_runs)
// Changes the flat array version of the thresholded image into a run
// length encoded version, which speeds up later processing since we
// only have to look at the points where values change.
{
  tmap_t m,save;
  tmap_t *row;
  int x,y,j,l;
  rle_t r;

  r.next = 0;

  // initialize terminator restore
  save = map[0];

  j = 0;
  for(y=0; y<height; y++){
    row = &map[y * width];

    // restore previous terminator and store next
    // one in the first pixel on the next row
    row[0] = save;
    save = row[width];
    row[width] = MAX_COLORS;
    r.y = y;

    x = 0;
    while(x < width){
      m = row[x];
      r.x = x;

      l = x;
      while(row[x] == m) x++;

      if(m!=0 || x>=width){
	r.color = m; 
	r.width = x - l;
	r.parent = j;
	rle[j++] = r;

        // printf("run (%d,%d):%d %d\n",r.x,r.y,r.width,r.color);

	if(j >= max_runs) return(j);
      }
    }
  }

  return(j);
}

#ifdef ENABLE_JOIN_NEARBY
template<class rle_t>
inline int AdvanceToNextRun(int run_idx, rle_t *runs) {
  run_idx++;
  if(runs[run_idx].x==-1)
    run_idx=runs[run_idx].next;
  return run_idx;
}
#else
#define AdvanceToNextRun(x,y) (x+1)
#endif

//#define DUMP_RUNS

// returns true if the runs are ok, false otherwise
template<class rle_t>
bool CheckRuns(rle_t *rle,int num_runs,int width,int height) {
  bool error;
  int x,y;
  int run_idx;

  error=false;
  x = y = 0;

  run_idx = 0;
  while(run_idx < num_runs && !error) {
    rle_t *cur_run=&rle[run_idx];

    if(cur_run->x < x) {
      printf("\nat (%d,%d) backwards x movement, cur x=%d run x=%d\n",x,y,x,cur_run->x);
      error=true;
    }

    if(cur_run->width < 0) {
      printf("\nat (%d,%d) negative run width %d\n",x,y,cur_run->width);
      error=true;
    }

    if(cur_run->y != y) {
      printf("\nat (%d,%d) wrong y value, cur y=%d run y=%d\n",x,y,y,cur_run->y);
      error=true;
    }

    x = cur_run->x + cur_run->width;
    if(x == width) {
      x = 0;
      y++;
    }

    run_idx++;
  }

  if(!error && (x!=0 || y!=height)) {
    printf("at (%d,%d) wrong ending position\n",x,y);
    error=true;
  }

  return !error;
}

#ifdef ENABLE_JOIN_NEARBY
// join nearby runs of the same color
// specifically those seperated by only one pixel of black
template <class rle_t>
int JoinNearbyRuns(rle_t *rle_from,rle_t *rle_to,int num_runs,int width,int height) {
  int fup,fon,fdn; // runs with x that is largest but still <= x
  int ton;
  int fup_n,fon_n,fdn_n;
  int x,y; // location on fill row of output
  int next_x,cand_next_x; // x to move to next
  int which_bump; // which row to bump
  rle_t rup,ron,rdn;
  rle_t rup_n,ron_n,rdn_n;
  rle_t rout;

  fup = -1;
  fup=AdvanceToNextRun(fup,rle_from);
  rup = rle_from[fup];
  fup_n=AdvanceToNextRun(fup,rle_from);
  rup_n=rle_from[fup_n];

  fon = fup;
  fon=AdvanceToNextRun(fon,rle_from);
  ron = rle_from[fon];
  fon_n=AdvanceToNextRun(fon,rle_from);
  ron_n=rle_from[fon_n];

  fdn = fon;
  while(rle_from[fdn].y==0)
    fdn=AdvanceToNextRun(fdn,rle_from);
  rdn = rle_from[fdn];
  
  ton = 0;

  // store black row before image
  rout.color = 0;
  rout.width = width;
  rout.x = 0;
  rout.y = -1;
  rout.parent = ton;
  rle_to[ton++] = rout;

  rout = ron;

  y = 0;
  x = ron.x+ron.width-1;

  while(rup_n.x <= x && rup_n.y < y) {
    fup=fup_n;
    rup=rup_n;
    fup_n=AdvanceToNextRun(fup,rle_from);
    rup_n=rle_from[fup_n];
  }

  while(y < height && fon < num_runs-1) {
    static const int bridge_width=1;
    
    bool output=true;
    bool advance=true;

    // check if should merge based on this row's runs
    // case on row left, on row right
    if(rout.x+rout.width+bridge_width >= ron_n.x && rout.color == ron_n.color && rout.y == ron_n.y) {
      // merge
      rout.width = ron_n.x+ron_n.width - rout.x;
      output = false;
    }
    else {
      // check if should merge based on this row and row above
      // case on row left, up row right
      int x_diff;
      x_diff = rup_n.x - (rout.x+rout.width-1);
      if(0 < x_diff && x_diff<=2 && rout.color == rup_n.color && rout.y == rup_n.y+1 && (rup_n.x < ron_n.x || rout.y != ron_n.y)) {
        // extend
        rout.width += x_diff;
        output = false;
        advance = false;
      }
    }
    
    if(output) {
      // output
      rout.parent = ton;
      rle_to[ton++] = rout;
      rout = ron_n;
    }

    if(advance) {
      fon=fon_n;
      fon_n=AdvanceToNextRun(fon,rle_from);
      ron_n=rle_from[fon_n];
    }

    next_x = rout.x + rout.width - 1;
    x = next_x;
    if(x==width-1) {
      y++;
      x=0;
    }

    while(rup.y < y-1 || (rup_n.x <= x && rup_n.y < y)) {
      fup=fup_n;
      rup=rup_n;
      fup_n=AdvanceToNextRun(fup,rle_from);
      rup_n=rle_from[fup_n];
    }
  }

  // store pending output run
  rout.parent = ton;
  rle_to[ton++] = rout;

  // store black row after image
  rout.color = 0;
  rout.width = width;
  rout.x = 0;
  rout.y = height;
  rout.parent = ton;
  rle_to[ton++] = rout;

  return ton;
}
#endif

template <class rle_t>
void ConnectComponents(rle_t *map,int num)
// Connect components using four-connecteness so that the runs each
// identify the global parent of the connected region they are a part
// of.  It does this by scanning adjacent rows and merging where
// similar colors overlap.  Used to be union by rank w/ path
// compression, but now is just uses path compression as the global
// parent index is a simpler rank bound in practice.
// WARNING: This code is complicated.  I'm pretty sure it's a correct
//   implementation, but minor changes can easily cause big problems.
//   Read the papers on this library and have a good understanding of
//   tree-based union find before you touch it
{
  int l1,l2;
  rle_t r1,r2;
  int i,j,s;

  // l2 starts on first scan line, l1 starts on second
  l2 = 0;
#ifdef ENABLE_JOIN_NEARBY
  l2=AdvanceToNextRun(l2,map);
#endif
  l1 = 1;
  while(map[l1].y == 0) 
    l1=AdvanceToNextRun(l1,map);

  // Do rest in lock step
  r1 = map[l1];
  r2 = map[l2];
  s = l1;
  while(l1 < num){
    /*
    printf("%6d:(%3d,%3d,%3d) %6d:(%3d,%3d,%3d)\n",
	   l1,r1.x,r1.y,r1.width,
	   l2,r2.x,r2.y,r2.width);
    */

    if(r1.color==r2.color && r1.color){
      // case 1: r2.x <= r1.x < r2.x + r2.width
      // case 2: r1.x <= r2.x < r1.x + r1.width
      if((r2.x<=r1.x && r1.x<r2.x+r2.width) ||
	 (r1.x<=r2.x && r2.x<r1.x+r1.width)){
        if(s != l1){
          // if we didn't have a parent already, just take this one
          map[l1].parent = r1.parent = r2.parent;
          s = l1;
        }else if(r1.parent != r2.parent){
          // otherwise union two parents if they are different

          // find terminal roots of each path up tree
          i = r1.parent;
          while(i != map[i].parent) i = map[i].parent;
          j = r2.parent;
          while(j != map[j].parent) j = map[j].parent;

          // union and compress paths; use smaller of two possible
          // representative indicies to preserve DAG property
          if(i < j){
	    map[j].parent = i;
            map[l1].parent = map[l2].parent = r1.parent = r2.parent = i;
          }else{
            map[i].parent = j;
            map[l1].parent = map[l2].parent = r1.parent = r2.parent = j;
          }
        }
      }
    }

    // Move to next point where values may change
    i = (r2.x + r2.width) - (r1.x + r1.width);
    if(i >= 0) r1 = map[l1=AdvanceToNextRun(l1,map)];
    if(i <= 0) r2 = map[l2=AdvanceToNextRun(l2,map)];
  }
 
  // Now we need to compress all parent paths
  i=0;
#ifdef ENABLE_JOIN_NEARBY
  if(map[i].x==-1)
    i = map[i].next;
#endif
  while(i<num) {
    j = map[i].parent;
    map[i].parent = map[j].parent;

    i=AdvanceToNextRun(i,map);
  }
}

template <class region_t,class rle_t>
int ExtractRegions(region_t *reg,int max_reg,rle_t *rmap,int num)
// Takes the list of runs and formats them into a region table,
// gathering the various statistics along the way.  num is the number
// of runs in the rmap array, and the number of unique regions in
// reg[] (bounded by max_reg) is returned.  Implemented as a single
// pass over the array of runs.
{
  int b,i,n,a;
  rle_t r;

  n = 0;
  i=0;
#ifdef ENABLE_JOIN_NEARBY
  i=AdvanceToNextRun(i,rmap);
#endif
  while(i<num) {
    if(rmap[i].color){
      r = rmap[i];
      if(r.parent == i){
        // Add new region if this run is a root (i.e. self parented)
        rmap[i].parent = b = n;  // renumber to point to region id
        reg[b].color = r.color;
        reg[b].area = r.width;
        reg[b].x1 = r.x;
        reg[b].y1 = r.y;
        reg[b].x2 = r.x + r.width;
        reg[b].y2 = r.y;
        reg[b].cen_x = range_sum(r.x,r.width);
        reg[b].cen_y = r.y * r.width;
	reg[b].run_start = i;
	reg[b].iterator_id = i; // temporarily use to store last run
        n++;
        if(n >= max_reg) return(max_reg);
      }else{
        // Otherwise update region stats incrementally
        b = rmap[r.parent].parent;
        rmap[i].parent = b; // update parent to identify region id
        reg[b].area += r.width;
        reg[b].x2 = std::max(r.x + r.width,reg[b].x2);
        reg[b].x1 = std::min((int)r.x,reg[b].x1);
        reg[b].y2 = r.y; // last set by lowest run
        reg[b].cen_x += range_sum(r.x,r.width);
        reg[b].cen_y += r.y * r.width;
	// set previous run to point to this one as next
	rmap[reg[b].iterator_id].next = i;
	reg[b].iterator_id = i;
      }
    }

    i=AdvanceToNextRun(i,rmap);
  }

  // calculate centroids from stored sums
  i=0;
#ifdef ENABLE_JOIN_NEARBY
  i=AdvanceToNextRun(i,rmap);
#endif
  while(i<n) {
    a = reg[i].area;
    reg[i].cen_x = (float)reg[i].cen_x / a;
    reg[i].cen_y = (float)reg[i].cen_y / a;
    reg[i].iterator_id = 0;
    reg[i].x2--; // change to inclusive range
    i=AdvanceToNextRun(i,rmap);
  }

  return(n);
}

template <class color_class_state_t,class region_t>
int SeparateRegions(color_class_state_t *color,int colors,
		    region_t *reg,int num)
// Splits the various regions in the region table a separate list for
// each color.  The lists are threaded through the table using the
// region's 'next' field.  Returns the maximal area of the regions,
// which can be used later to speed up sorting.
{
  region_t *p;
  int i;
  int c;
  int area,max_area;

  // clear out the region list head table
  for(i=0; i<colors; i++){
    color[i].list = NULL;
    color[i].num  = 0;
  }

  // step over the table, adding successive
  // regions to the front of each list
  max_area = 0;
  for(i=0; i<num; i++){
    p = &reg[i];
    c = p->color;
    area = p->area;

    if(area >= color[c].min_area){
      if(area > max_area) max_area = area;
      color[c].num++;
      p->next = color[c].list;
      color[c].list = p;
    }
  }

  return(max_area);
}

// These are the tweaking values for the radix sort given below
// Feel free to change them, though these values seemed to work well
// in testing.  Don't worry about extra passes to get all 32 bits of
// the area; the implementation only does as many passes as needed to
// touch the most significant set bit (MSB of largest region's area)
#define CMV_RBITS 6
#define CMV_RADIX (1 << CMV_RBITS)
#define CMV_RMASK (CMV_RADIX-1)

template <class region_t>
region_t *SortRegionListByArea(region_t *list,int passes)
// Sorts a list of regions by their area field.
// Uses a linked list based radix sort to process the list.
{
  region_t *tbl[CMV_RADIX],*p,*pn;
  int slot,shift;
  int i,j;

  // Handle trivial cases
  if(!list || !list->next) return(list);

  // Initialize table
  for(j=0; j<CMV_RADIX; j++) tbl[j] = NULL;

  for(i=0; i<passes; i++){
    // split list into buckets
    shift = CMV_RBITS * i;
    p = list;
    while(p){
      pn = p->next;
      slot = ((p->area) >> shift) & CMV_RMASK;
      p->next = tbl[slot];
      tbl[slot] = p;
      p = pn;
    }

    // integrate back into partially ordered list
    list = NULL;
    for(j=0; j<CMV_RADIX; j++){
      p = tbl[j];
      tbl[j] = NULL; // clear out table for next pass
      while(p){
        pn = p->next;
        p->next = list;
        list = p;
        p = pn;
      }
    }
  }

  return(list);
}

template <class color_class_state_t>
void SortRegions(color_class_state_t *color,int colors,int max_area)
// Sorts entire region table by area, using the above
// function to sort each threaded region list.
{
  int i,p;

  // do minimal number of passes sufficient to touch all set bits
  p = (top_bit(max_area) + CMV_RBITS-1) / CMV_RBITS;

  // sort each list
  for(i=0; i<colors; i++){
    color[i].list = SortRegionListByArea(color[i].list,p);
  }
}

//#define DEBUG_REGION_MERGE
//#define STATS_REGION_MERGE
//#define DEBUG_CONSIDER_REGION_MERGE

template<class region,class rle_t>
void MergeRegions(region *p,region *q,region **q_prev_next,rle_t *runs) {
  int l,r,t,b;
  int a;

  // find union box
  l = std::min(p->x1,q->x1);
  r = std::max(p->x2,q->x2);
  t = std::min(p->y1,q->y1);
  b = std::max(p->y2,q->y2);

  // merge them to create a new region
  a = p->area + q->area;
  p->x1 = l;
  p->x2 = r;
  p->y1 = t;
  p->y2 = b;
  p->cen_x = ((p->cen_x * p->area) + (q->cen_x * q->area)) / a;
  p->cen_y = ((p->cen_y * p->area) + (q->cen_y * q->area)) / a;
  p->area = a;

  // reparent runs and hook up next pointers
  int parent_run_idx;
  int p_run_idx,q_run_idx;
  rle_t *p_run,*q_run,*last_run;

  last_run = NULL;
  p_run_idx = p->run_start;
  p_run = &runs[p_run_idx];
  parent_run_idx = p_run->parent;
  q_run_idx = q->run_start;
  q_run = &runs[q_run_idx];

  if(p_run_idx == 0) {
    last_run = p_run;
    p_run_idx = p_run->next;
    p_run = &runs[p_run_idx];
  } else if(q_run_idx == 0) {
    p->run_start = q_run_idx;
    last_run = q_run;
    last_run->parent = parent_run_idx;
    q_run_idx = q_run->next;
    q_run = &runs[q_run_idx];
  }

  if(p_run_idx!=0 &&
     (p_run_idx < q_run_idx || q_run_idx==0)) {
    if(last_run)
      last_run->next = p_run_idx;
    last_run = p_run;
    p_run_idx = p_run->next;
    p_run = &runs[p_run_idx];
    while(p_run_idx != 0 && p_run_idx < q_run_idx) {
      last_run = p_run;
      p_run_idx = p_run->next;
      p_run = &runs[p_run_idx];
    }
  } else {
    if(last_run)
      last_run->next = q_run_idx;
    else
      p->run_start = q_run_idx;
    last_run = q_run;
    last_run->parent = parent_run_idx;
    q_run_idx = q_run->next;
    q_run = &runs[q_run_idx];
  }

  // last_run is non-null at this point
  // p->run_start is correct at this point

  while(q_run_idx!=0 && p_run_idx!=0) {
    if(p_run_idx < q_run_idx) {
      last_run->next = p_run_idx;
      last_run = p_run;
      p_run_idx = p_run->next;
      p_run = &runs[p_run_idx];
    } else {
      last_run->next = q_run_idx;
      last_run = q_run;
      last_run->parent = parent_run_idx;
      q_run_idx = q_run->next;
      q_run = &runs[q_run_idx];
    }
  }

  if(p_run_idx != 0) {
    last_run->next = p_run_idx;
  }
  if(q_run_idx != 0) {
    do {
      last_run->next = q_run_idx;
      last_run = q_run;
      last_run->parent = parent_run_idx;
      q_run_idx = q_run->next;
      q_run = &runs[q_run_idx];
    } while(q_run_idx != 0);
    last_run->next = 0;
  }
  
  // remove q from list (old smaller region)
  *q_prev_next = q->next;
}

template<class region>
void CalcXYBounds(region *p,double density_thresh,int area,int &xl,int &xh,int &yl,int& yh) {
  int a,l,r,t,b;
  int width,height;
  int xexp,yexp;

  a = p->area;
  l = p->x1;
  r = p->x2;
  t = p->y1;
  b = p->y2;

  width  = r - l + 1;
  height = b - t + 1;

  // derived from:
  // (a + area) / ((width + xexp) * height) > density_thresh
  xexp = (int) ((a + area) / (density_thresh * height) - width); // round down

  xl = l - xexp;
  xh = r + xexp;

  // derived from:
  // (a + area) / (width * (height + yexp)) > density_thresh
  yexp = (int) ((a + area) / (density_thresh * width) - height); // round down

  yl = t - yexp;
  yh = b + yexp;
}

template<class region,class rle_t>
int MergeRegions(region *p,double density_thresh,rle_t *runs)
{
  region *q,*s;
  int l,r,t,b;
  int a;
  int merged;
  int last_area; // last size used to calculate bounds
  int xbl,xbh; // x bound low/high
  int ybl,ybh; // y bound low/high
  
  merged = 0;
  last_area = 0;

  while(p){
    q = p->next;
    s = p;

    if(q) {
      CalcXYBounds(p,density_thresh,q->area,xbl,xbh,ybl,ybh);
      last_area = q->area;
    }

    while(q){
      bool advance=true;

      if(q->area > last_area) {
        CalcXYBounds(p,density_thresh,q->area,xbl,xbh,ybl,ybh);
        last_area = q->area;
      }

      if(q->x1 >= xbl &&
         q->x2 <= xbh &&
         q->y1 >= ybl &&
         q->y2 <= ybh) {
        // find union box and get its total area
        l = std::min(p->x1,q->x1);
        r = std::max(p->x2,q->x2);
        t = std::min(p->y1,q->y1);
        b = std::max(p->y2,q->y2);
        a = (r-l+1) * (b-t+1);

        CalcXYBounds(p,density_thresh,q->area,xbl,xbh,ybl,ybh);
        last_area = q->area;

        // if density of merged region is still above threshold
        if((double)(p->area + q->area) / a > density_thresh){

          MergeRegions(p,q,&s->next,runs);

          // return to next region after p
          q = p->next;
          s = p;
          merged++;
          advance=false;
        }
      }

      if(advance) {
	s = q;
	q = q->next;
      }
    }
    p = p->next;
  }

  return(merged);
}

template<class color_class_state_t,class rle_t>
int MergeRegions(color_class_state_t *color,int colors,rle_t *runs)
{
  int i,m;
  int num;

  num = 0;

  for(i=0; i<colors; i++){
    if(color[i].merge_threshold >= 1.0)
      m = 0;
    else
      m = MergeRegions(color[i].list,color[i].merge_threshold,runs);
    num += m;
    color[i].num -= m;
  }

  return(num);
}

template<class region, class rle_t>
bool CheckRegions(region *p,rle_t *runs) {
  bool ok=true;

  while(p && ok) {
    int area;
    int run_idx,next_run_idx;
    rle_t *currun,*next_run;

    run_idx = p->run_start;
    currun = &runs[run_idx];
    area = 0;

    do {
      area += currun->width;
      
      if(currun->parent != runs[p->run_start].parent) {
        printf("wrong parent id on run %d (claims parent %d should be %d)\n",
                run_idx,currun->parent,runs[p->run_start].parent);
        ok = false;
      }
      
      next_run_idx = currun->next;
      next_run = (next_run_idx!=0 ? &runs[next_run_idx] : NULL);
      
      if(next_run_idx != 0 && 
         next_run_idx <= run_idx) {
        printf("failed to progress, going from run %d to run %d\n",run_idx,next_run_idx);
        ok = false;
      }
      
      run_idx = next_run_idx;
      currun = next_run;
    } while(run_idx != 0 && ok);

    if(p->area != area) {
      ok = false;
      printf("area mismatch, claimed %d actually %d\n",p->area,area);
    }

    p = p->next;
  }

  return ok;
}

template<class color_class_state_t,class rle_t>
bool CheckRegions(color_class_state_t *color,int colors,rle_t *runs) {
  bool ok=true;
  int i;

  for(i=0; i<colors && ok; i++){
    ok = CheckRegions(color[i].list,runs);
    if(!ok) {
      printf("region check failed for color %d\n",i);
    }
  }

  return(ok);
}

template <class region_t,class rle_t>
int FindStart(rle_t *rmap,int left,int right,int x,DummyT1<region_t> dummy=DummyT1<region_t>())
// This function uses binary search to find the leftmost run whose
// interval either contains or is greater than x.
{
  int m;

  while(right > left){
    m = (left + right) / 2;
    if(x > rmap[m].x+rmap[m].width){
      left = m + 1;
    }else if(x < rmap[m].x){
      right = m;
    }else{
      return(m);
    }
  }

  return(m);
}

template <class rle_t>
int FindStart(rle_t *rmap,int left,int right,int x,int y)
// This function uses binary search to find the leftmost run whose
// interval either contains or is greater than x and equals y
{
  int m=0;

  while(right > left){
    m = (left + right) / 2;
    if(y > rmap[m].y || 
       (y == rmap[m].y && x > rmap[m].x+rmap[m].width)){
      left = m + 1;
    }else if(y < rmap[m].y || 
             (y == rmap[m].y && x < rmap[m].x)){
      right = m;
    }else{
      return(m);
    }
  }

  return(m);
}

template <class region_t,class rle_t>
void CreateRunIndex(int *yindex,rle_t *rmap,int num,DummyT1<region_t> dummy=DummyT1<region_t>())
// Creates an index of where rows start in the run map.  This can be
// used to speed up searching over the map.  This function assumes
// there is at least one run in every row.
{
  int y = 0;
  yindex[y] = 0;

  for(int i=0; i<num; i++){
    if(rmap[i].y > y){
      y = rmap[i].y;
      yindex[y] = i;
    }
  }
}

template <class color_class_state_t>
void GetNextRegion(color_class_state_t *color,int colors,int max_area)
{
  // TODO
}

template <class color_class_state_t>
void CalcTotalArea(color_class_state_t *color)
{
  region *cur_reg;

  cur_reg = color->list;
  color->total_area = 0;
  while(cur_reg!=NULL) {
    color->total_area += cur_reg->area;
    cur_reg = cur_reg->next;
  }
}

template <class color_class_state_t>
void CalcTotalArea(color_class_state_t *color,int colors)
{
  for(int i=0; i<colors; i++)
    CalcTotalArea(&color[i]);
}

template <class data>
int find(data *arr,int start,int end,data key)
{
  int i;

  for(i=start; i<end; i++){
    if(arr[i] == key) return(i);
  }

  return(end);
}

typedef std::map<std::string, unsigned int> color_name_map;

template <class color_class_state_t>
int LoadColorInformation(color_class_state_t *color,int max,const char *filename, color_name_map &color_names)
{
  char buf[512];
  FILE *in;
  int len;
  int sl,sr;
  int num;

  int idx,r,g,b,ms;
  float merge_threshold;
  char *name;

  //printf("about to open color file\n");
  in = fopen(filename,"rt");
  if(!in) return(0);

  memset(color,0,sizeof(color_class_state_t)*max);
  num = 0;

  //printf("about to fgets\n");
  while(fgets(buf,256,in)){
    //printf("fgets result is '%s'\n",buf);
    len = strlen(buf) - 1;
    buf[len] = 0;

    if(len && buf[0]!='#'){
      sl = find(buf,   0,len,'"');
      sr = find(buf,sl+1,len,'"');
      if(sl<len && sr<len){
	buf[sl] = buf[sr] = 0;
	idx = r = g = b = ms = 0;
	sscanf(buf,"%d (%d %d %d)",&idx,&r,&g,&b);
	name = buf+sl+1;
        merge_threshold = 1.f;
	sscanf(buf+sr+1,"%d %g",&ms,&merge_threshold);

	if(idx>=0 && idx<max && ms>0){
	  color[idx].min_area = ms;
          color[idx].merge_threshold = merge_threshold;
	  color[idx].color.red   = r;
	  color[idx].color.green = g;
	  color[idx].color.blue  = b;
	  color[idx].name = strdup(name);
    color_names[color[idx].name]=idx;
	  if(idx >= num) num = idx+1;
	} else {
 	  printf("Options error: %2d (%3d %3d %3d) '%s' %d\n",
                  idx,r,g,b,name,ms);
	}
      }
    }
  }

  fclose(in);

  return(num);
}

} // namespace

#endif
