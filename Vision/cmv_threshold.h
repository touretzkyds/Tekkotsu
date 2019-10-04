#ifndef __CMV_THRESHOLD_H__
#define __CMV_THRESHOLD_H__

/*! @file
* @brief Color threshold support for #CMVision
* @author James R. Bruce, School of Computer Science, Carnegie Mellon University
*
* Licensed under the <a href="../gpl-2.0.txt">GNU GPL version 2</a>
*/

#include <stdio.h>
#include "cmv_types.h"

namespace CMVision{

template<int x,int y,int z>
class DummyI3 {
};

template<class T,int x,int y,int z>
class DummyT1I3 {
};

template <class cmap_t,class image,int bits_y,int bits_u,int bits_v>
void ThresholdImage(cmap_t *cmap,image &img,cmap_t *tmap,DummyI3<bits_y,bits_u,bits_v> dummy=DummyI3<bits_y,bits_u,bits_v>())
{
  // yuyv *buf,p;
  uyvy *buf,p;
  int i,m,size;

  int rshift_y,rshift_u,rshift_v;
  int lshift_y,lshift_u,lshift_v;

  rshift_y = 8 - bits_y;
  rshift_u = 8 - bits_u;
  rshift_v = 8 - bits_v;

  lshift_y = bits_u + bits_v;
  lshift_u = bits_v;
  lshift_v = 0;

  size = img.width * img.height;
  buf  = img.buf;

  for(i=0; i<size; i++){
    p = buf[i / 2];
    m = ((p.u >> rshift_u) << lshift_u) +
        ((p.v >> rshift_v) << lshift_v);
    cmap[i + 0] = tmap[m + ((p.y1 >> rshift_y) << lshift_y)];
    cmap[i + 1] = tmap[m + ((p.y1 >> rshift_y) << lshift_y)];
  }
}

template <class cmap_t,class image>
void ThresholdImageRGB16(cmap_t *cmap,image &img,cmap_t *tmap)
{
  unsigned short *buf;
  int i,size;

  size = img.width * img.height;
  buf  = (unsigned short*)img.buf;

  for(i=0; i<size; i++){
    cmap[i] = tmap[buf[i]];
  }
}

//void ThresholdImageYUVPlanar(cmap_t *cmap,image &img,cmap_t *tmap,DummyT1I3<element,bits_y,bits_u,bits_v> dummy=DummyT1I3<element,bits_y,bits_u,bits_v>())
template <class cmap_t,class image,class element,int bits_y,int bits_u,int bits_v>
void ThresholdImageYUVPlanar(cmap_t *cmap,image &img,cmap_t *tmap)
{
  //int nonzero_cnt=0;

  // element *buf_y,*buf_u,*buf_v;
  int row,col;
  int width,height;
  int py,pu,pv;
  int tmap_idx;
#ifdef CALC_AVG_IMG_COLOR
  ulong total_y;
  ulong total_u;
  ulong total_v;
#endif

  int rshift_y,rshift_u,rshift_v;
  int lshift_y,lshift_u,lshift_v;

  element *row_y,*row_u,*row_v;
  cmap_t *row_cmap;

  rshift_y = 8 - bits_y;
  rshift_u = 8 - bits_u;
  rshift_v = 8 - bits_v;

  lshift_y = bits_u + bits_v;
  lshift_u = bits_v;
  lshift_v = 0;

  width  = img.width;
  height = img.height;
#ifdef CALC_AVG_IMG_COLOR
  total_y = 0;
  total_u = 0;
  total_v = 0;
#endif

  for(row=0; row<height; row++) {
    row_y = img.buf_y + row*img.row_stride;
    row_u = img.buf_u + row*img.row_stride;
    row_v = img.buf_v + row*img.row_stride;
    row_cmap = cmap + row*width;

    int rowidx=0;
    for(col=0; col<width; col++) {
      py = row_y[rowidx] >> rshift_y;
      pu = row_u[rowidx] >> rshift_u;
      pv = row_v[rowidx] >> rshift_v;
      tmap_idx = 
        (py << lshift_y) +
        (pu << lshift_u) +
        (pv << lshift_v);
      row_cmap[col] = tmap[tmap_idx];
#ifdef CALC_AVG_IMG_COLOR
      total_y += row_y[rowidx];
      total_u += row_u[rowidx];
      total_v += row_v[rowidx];
#endif

      /*
      if(row==height/2 && col==width/2) {
        printf("py=%u pu=%u pv=%u tmap_idx=%d tmap val=%u\n",py,pu,pv,tmap_idx,tmap[tmap_idx]);
      }
      */
      rowidx+=img.col_stride;
    }
  }

}

//#define ENABLE_JOIN_NEARBY

template <class rle_t,class color_class_state_t>
void RmapToRgb(rgb *img,rle_t *map,int last_run,int width,int height,
		color_class_state_t *color,int num)
{
  int i,x,y=0,next_x;

  i=0;
  next_x=0;
#ifdef ENABLE_JOIN_NEARBY
  i=AdvanceToNextRun(i,map);
#endif  
  while(i < last_run) {
    rle_t *currun;
    currun = &map[i];
    
    y=currun->y;
    if(y>=height) {
      return;
    }

    x=currun->x;
    if(x<next_x) {
      return;
    }

    if(x!=next_x) {
      for(x=next_x; x<currun->x; x++)
        img[y*width + x] = color[0].color;
    }

    next_x = currun->x+currun->width;
    for(x=currun->x; x<next_x; x++)
      img[y*width + x] = color[currun->color].color;

    if(next_x == width) {
      y++;
      next_x = 0;
    }

    i=i+1;//AdvanceToNextRun(i,map);
  }
  for(x=next_x; x<width; x++)
    img[y*width + x] = color[0].color;
}

template <class cmap_t>
void RgbToIndex(cmap_t *map,rgb *img,int width,int height,
		rgb *colors,int num)
{
  int i,j,size;

  size = width * height;

  j = 0;
  for(i=0; i<size; i++){
    if(img[i] != colors[j]){
      j = 0;
      while(j<num && img[i]!=colors[j]) j++;
      if(j==num)
        j = 0;
    }
    map[i] = j;
  }
}

template <class cmap_t,class color_class_state_t>
void IndexToRgb(rgb *img,cmap_t *map,int width,int height,
		color_class_state_t *color,int num)
{
  int i,size;

  size = width * height;

  for(i=0; i<size; i++){
    img[i] = color[map[i]].color;
  }
}

template <class cmap_t>
void IndexToRgb(rgb *img,cmap_t *map,int width,int height,
		rgb *colors,int num)
{
  int i,size;

  size = width * height;

  for(i=0; i<size; i++){
    img[i] = colors[map[i]];
  }
}

template <class data>
data Get3D(data *arr,int num_i,int num_j,int num_k,int i,int j,int k)
{
  int l;
  l = i*num_j*num_k + j*num_k + k;
  return(arr[l]);
}

template <class data>
void Set3D(data *arr,int num_i,int num_j,int num_k,int i,int j,int k,data v)
{
  int l;
  l = i*num_j*num_k + j*num_k + k;
  arr[l] = v;
}

template <class tmap_t>
int RemapTMapColor(tmap_t *tmap,int num_y,int num_u,int num_v,int src_id,int dest_id)
{
  int i,n,size;

  size = num_y * num_u * num_v;
  n = 0;

  for(i=0; i<size; i++){
    if(tmap[i] == src_id){
      tmap[i] = dest_id;
      n++;
    }
  }

  return(n);
}

template <class tmap_t>
int CheckTMapColors(tmap_t *tmap,int num_y,int num_u,int num_v,int colors,int default_id)
{
  int i,n,size;

  size = num_y * num_u * num_v;
  n = 0;

  for(i=0; i<size; i++){
    if(tmap[i] >= colors){
      tmap[i] = default_id;
      n++;
    }
  }

  return(n);
}

template <class tmap_t>
bool LoadThresholdFile(tmap_t *tmap,int num_y,int num_u,int num_v,const char *filename)
{
  FILE *in;
  char buf[256];
  int ny,nu,nv;
//  int by,bu,bv;
  int size,read;
  bool invertUV;

  in = fopen(filename,"r");
  if(!in) return(false);

  // read magic
  if(!fgets(buf,256,in)) goto error;
  buf[4] = 0;
  if(strcmp(buf,"TMAP")) goto error;

  // read type
  if(!fgets(buf,256,in)) goto error;
  if(strcmp(buf,"YUV8\n")==0)
    invertUV=true;
  else if(strcmp(buf,"YUV'8\n")==0)
    invertUV=false;
  else goto error;

  // read size
  if(!fgets(buf,256,in)) goto error;
  ny = nu = nv = 0;
  sscanf(buf,"%d %d %d",&ny,&nu,&nv);
  if(invertUV) {
    if(num_y!=ny || num_u!=nv || num_v!=nu) goto error;
  } else {
    if(num_y!=ny || num_u!=nu || num_v!=nv) goto error;
  }
  /*//for(by=1; (num_y>>by)!=0; by++) {}
  for(bu=1; (num_u>>bu)!=0; bu++) {}
  for(bv=1; (num_v>>bv)!=0; bv++) {}
  by=bu+bv;
  bu=bv;
  bv=0;*/

  size = num_y * num_u * num_v;
  if(!invertUV) {
    read = fread(tmap,sizeof(tmap_t),size,in);
  } else {
    read=0;
    for(int yi=0; yi<num_y; yi++) {
      for(int vi=0; vi<num_v; vi++) {
        for(int ui=0; ui<num_u; ui++) {
          if(fread(tmap+ui*num_v+vi,sizeof(tmap_t),1,in)!=1) goto error;
        }
      }
      tmap+=num_u*num_v;
      read+=num_u*num_v;
    }
  }

  fclose(in);

  return(read == size);
error:
  if(in) fclose(in);
  return(false);
}

template <class tmap_t>
bool SaveThresholdFile(tmap_t *tmap,int num_y,int num_u,int num_v,char *filename)
{
  FILE *out;
  int size,wrote;

  out = fopen(filename,"w");
  if(!out) return(false);

  fprintf(out,"TMAP\nYUV%zu\n%d %d %d\n",
          sizeof(tmap_t),num_y,num_u,num_v);
  size = num_y * num_u * num_v;
  wrote = fwrite(tmap,sizeof(tmap_t),size,out);
  fclose(out);

  return(wrote == size);
}

} // namespace

#endif
