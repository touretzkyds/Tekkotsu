#include "DualCoding/ShapeEllipse.h"
#include "DualCoding/EllipseData.h"
#include "Graphics.h"
#include "FilterBankGenerator.h"

using namespace std;

//! empty definition of macro for checking for any exceeding of image's memory boundaries
//#define CHKBOUNDS(p) {}

// ! check memory boundaries of image, first that we are after the start of the image, second that we are before the end of the image, and third that we are within a valid row (in case of interleaved rows), fourth that we are within a valid column (interleaved pixels)
#define CHKBOUNDS(p,ERR_F) { \
	unsigned int rowoffset=(p-img1)-(p-img1)/yInc*yInc; \
	if(p<img1 || p>=img1+h*yInc || rowoffset>=w*xInc || rowoffset/xInc*xInc!=rowoffset) { \
		cout << "Graphics Bad draw! line:" << __LINE__ << " frame=" << (gen!=NULL?gen->getFrameNumber():0) << ' '  /*<< (void*)p << '-' << (void*)img1 << '='*/ << (int)(p-img1) << " w=" << w << " xInc=" << xInc << " h=" << h << " yInc=" << yInc << endl; \
		ERR_F; \
	} \
}

Graphics::Graphics(FilterBankGenerator& fbg, unsigned int layer, unsigned int channel)
	: gen(&fbg), genLayer(layer), genChan1(channel), genChan2(channel), genChan3(channel), 
	  img1(NULL), img2(NULL), img3(NULL), w(0), h(0), xInc(0), yInc(0), color(0, 0, 0) {
  updateFBG();
}

Graphics::Graphics(FilterBankGenerator& fbg, unsigned int layer, unsigned int chan1, unsigned int chan2, unsigned int chan3)
	: gen(&fbg), genLayer(layer), genChan1(chan1), genChan2(chan2), genChan3(chan3),
	  img1(NULL), img2(NULL), img3(NULL), w(0), h(0), xInc(0), yInc(0), color(0, 0, 0) {
	updateFBG();
}

Graphics::Graphics(unsigned char* base, unsigned int width, unsigned int height, unsigned int interval, unsigned int stride)
	: gen(NULL), genLayer(), genChan1(), genChan2(), genChan3(),
	  img1(base), img2(base), img3(base), w(width), h(height), xInc(interval), yInc(stride), color(0, 0, 0) {}

void Graphics::updateFBG() {
	if(gen==NULL)
		return;
	img1=gen->getImage(genLayer,genChan1);
	img2=gen->getImage(genLayer,genChan2);
	img3=gen->getImage(genLayer,genChan3);
	w=gen->getWidth(genLayer);
	h=gen->getHeight(genLayer);
	xInc=gen->getIncrement(genLayer);
	yInc=gen->getStride(genLayer);
}

void Graphics::drawRect(int x, int y, int width, int height) {
	if(width<0) {
		x+=width;
		width=-width;
	}
	if(height<0) {
		y+=height;
		height=-height;
	}
	if(x>=(int)w || y>=(int)h || x+width<0 || y+height<0) //completely out of bounds
		return;
	if(x<0 && y<0 && x+width>=(int)w && y+height>=(int)h) //out of bounds (circumscribed)
		return;
	unsigned int left=x>0?x:0;
	unsigned int top=y>0?y:0;
	unsigned int right=( (x+width>=(int)w) ? w : (unsigned int)(x+width)); //not inclusive
	unsigned int bot=( (y+height>=(int)h) ? h : (unsigned int)(y+height)); //not inclusive
	//left vertical
	if(x>=0) {
		unsigned int result = top*yInc+left*xInc;
		unsigned char* p1=img1+result;
		unsigned char* p2=img2+result;
		unsigned char* p3=img3+result;
		unsigned char* end1=img1+result;
		while (p1!=end1) {
			CHKBOUNDS(p1,return);
			*p1=color.y;
			*p2=color.u;
			*p3=color.v;
			p1+=yInc;
			p2+=yInc;
			p3+=yInc;
		}
	}		
	//top horizontal
	if(y>=0) {
		unsigned int result1 = top*yInc+left*xInc;
		unsigned int result2 = top*yInc+right*xInc;
		unsigned char* p1=img1+result1;
		unsigned char* p2=img2+result1;
		unsigned char* p3=img3+result1;
		unsigned char* end1=img1+result2;
		while (p1!=end1) {
			CHKBOUNDS(p1,return);
			*p1=color.y;
			*p2=color.u;
			*p3=color.v;
			p1+=xInc;
			p2+=xInc;
			p3+=xInc;
		}
	}
	//right vertical
	if(right<w && left!=right) {
		unsigned int result1 = top*yInc+right*xInc;
		unsigned int result2 = bot*yInc+right*xInc;
		unsigned char* p1=img1+result1;
		unsigned char* p2=img2+result1;
		unsigned char* p3=img3+result1;
		unsigned char* end1=img1+result2;
		while (p1!=end1) {
			CHKBOUNDS(p1,return);
			*p1=color.y;
			*p2=color.u;
			*p3=color.v;
			p1+=yInc;
			p2+=yInc;
			p3+=yInc;
		}
	}		
	//bottom horizontal
	if(bot<h && top!=bot) {
		unsigned int result1 = bot*yInc+left*xInc;
		unsigned int result2 = bot*yInc+right*xInc;
		unsigned char* p1=img1+result1;
		unsigned char* p2=img2+result1;
		unsigned char* p3=img3+result1;
		unsigned char* end1=img1+result2;
		unsigned char* end2=img2+result2;
		unsigned char* end3=img3+result2;
		while(p1!=end1 && p2!=end2 && p3!=end3) {
			CHKBOUNDS(p1,return);
			*p1=color.y;
			*p2=color.u;
			*p3=color.v;
			p1+=xInc;
			p2+=xInc;
			p3+=xInc;
		}
	}
	//bottom right corner
	if(right<w && bot<h) {
		unsigned int result = bot*yInc+right*xInc;
		unsigned char* p1=img1+result;
		unsigned char* p2=img2+result;
		unsigned char* p3=img3+result;
		CHKBOUNDS(p1,return);
		*p1=color.y;
		*p2=color.u;
		*p3=color.v;
		return;
	}
}
void Graphics::drawRect(float x, float y, float width, float height) {
	unsigned int left,top,right,bot;
	getPixelCoordinates(left,top,x,y);
	getPixelCoordinates(right,bot,x+width,y+height);
	drawRect((int)left,(int)top,(int)(right-left),(int)(bot-top));
}

/*! @todo I think this could be a little faster by writing two cases -- one that
 *  handles mostly-vertical lines, and one that handles mostly-horizontal ones.
 *  Then the primary loop could be over integer coordinates along that axis, and
 *  only the position along the other axis would have to be calculated as
 *  floating point */
void Graphics::drawLine(int ix1, int iy1, int ix2, int iy2) {
	// the "right" way, allows full range of unsigned int for width
	//if(ix1<0 && ix2<0 || iy1<0 && iy2<0 || ix1>0 && ix2>0 && (unsigned int)ix1>=w && (unsigned int)ix2>=w || iy1>0 && iy2>0 && (unsigned int)iy1>=h && (unsigned int)iy2>=h)
	//  return; //completely outside visible region

	// the "realistic way" saves some CPU
	if( (ix1<0 && ix2<0) || (iy1<0 && iy2<0) || (ix1>=(int)w && ix2>=(int)w) || (iy1>=(int)h && iy2>=(int)h) )
		return; //completely outside visible region

	float x1=ix1, y1=iy1, x2=ix2, y2=iy2;
	float width=x2-x1;
	float height=y2-y1;
	bool clipped=false;
	if(width!=0) {
		float slope=height/width;
		if(x1<x2) {
			if(x1<0) {
				y1-=x1*slope;
				x1=0;
				clipped=true;
			}
			if(x2>=w) {
				y2-=(x2-(w-1))*slope;
				x2=w-1;
				clipped=true;
			}
		} else {
			if(x2<0) {
				y2-=x2*slope;
				x2=0;
				clipped=true;
			}
			if(x1>=w) {
				y1-=(x1-(w-1))*slope;
				x1=w-1;
				clipped=true;
			}
		}
	}
	if(clipped) {
		if( (x1<0 && x2<0) || (y1<0 && y2<0) || (x1>=w && x2>=w) || (y1>=h && y2>=h) )
			return; //completely outside visible region
		clipped=false;
	}
	if(height!=0) {
		float invslope=width/height;
		if(y1<y2) {
			if(y1<0) {
				x1-=y1*invslope;
				y1=0;
				clipped=true;
			}
			if(y2>=h) {
				x2-=(y2-(h-1))*invslope;
				y2=h-1;
				clipped=true;
			}
		} else {
			if(y2<0) {
				x2-=y2*invslope;
				y2=0;
				clipped=true;
			}
			if(y1>=h) {
				x1-=(y1-(h-1))*invslope;
				y1=h-1;
				clipped=true;
			}
		}
	}
	if(clipped) {
		if((x1<0 && x2<0) || (y1<0 && y2<0) || (x1>=w && x2>=w) || (y1>=h && y2>=h) )
			return; //completely outside visible region
		clipped=false;
	}
	width=x2-x1;
	height=y2-y1;
	int aw=abs((int)width);
	int ah=abs((int)height);
	int d=aw>ah?aw:ah;
	float dx=width/d;
	float dy=height/d;
	for(float x=x1,y=y1;d>0;d--,x+=dx,y+=dy) {
		unsigned int result = ((int)y)*yInc+((int)x)*xInc;
		unsigned char* p1=img1+result;
		unsigned char* p2=img2+result;
		unsigned char* p3=img3+result;
		CHKBOUNDS(p1,continue);
		*p1=color.y;
		*p2=color.u;
		*p3=color.v;
	}
	unsigned int result = ((int)y2)*yInc+((int)x2)*xInc;
	unsigned char* p1=img1+result;
	unsigned char* p2=img2+result;
	unsigned char* p3=img3+result;
	CHKBOUNDS(p1,return);
	*p1=color.y;
	*p2=color.u;
	*p3=color.v;
}
void Graphics::drawLine(float x1, float y1, float x2, float y2) {
	unsigned int px1,py1,px2,py2;
	getPixelCoordinates(px1,py1,x1,y1);
	getPixelCoordinates(px2,py2,x2,y2);
	drawLine((int)px1,(int)py1,(int)px2,(int)py2);
}

void Graphics::drawEllipse(int x, int y, float semimajor, float semiminor, AngPi orientation) {
   const float cosT = cos(orientation);
   const float sinT = sin(orientation);
   const float xRange = semimajor;
   const float majorSq = xRange * xRange;
   const float aspectRatio = semiminor / semimajor;

   for(float xDist = -xRange; xDist <= xRange; xDist += 0.2f) {
       const float yRange = sqrt(max(0.f, majorSq - xDist*xDist)) * aspectRatio;
       int px = round(x + xDist*cosT + yRange*sinT);
       int py = round(y - yRange*cosT + xDist*sinT);
       drawPoint(px, py);
       px = round(x + xDist*cosT - yRange*sinT);
       py = round(y + yRange*cosT + xDist*sinT);
       drawPoint(px, py);
   }
}

void Graphics::drawEllipse(float x, float y, float semimajor, float semiminor, AngPi orientation ) {
  unsigned int px, py;
  getPixelCoordinates(px,py,x,y);
  drawEllipse((int)px,(int)py,semimajor,semiminor,orientation);
}

void Graphics::drawQuarterEllipse(int x, int y, float semimajor, float semiminor, AngPi orientation) {
   const float cosT = cos(orientation);
   const float sinT = sin(orientation);
   const float xmax = abs(semimajor);
   const float majorSq = semimajor * semimajor;
   const float xsign = semimajor > 0 ? 1 : -1;
   const float ysign = semiminor > 0 ? 1 : -1;
   const float aspectRatio = abs(semiminor / semimajor);
   for (float xval=0; xval <= xmax; xval += 0.1f) {
     const float yval = sqrt(max(0.f, majorSq-xval*xval)) * aspectRatio;
     int px = round(x + xval*cosT*xsign - yval*sinT*ysign);
     int py = round(y + yval*cosT*ysign + xval*sinT*xsign);
     drawPoint(px, py);
   }
}

void Graphics::getPixelCoordinates(unsigned int& px, unsigned int& py, float x, float y) const {
	if(gen!=NULL) {
		gen->getPixelCoordinates(px,py,x,y,genLayer);
	} else {
		//note width sets the scale for both, so coordinate system is square... is good? I'm up for debate.
		px=(unsigned int)((w-1)*(x+1)/2+.5f); //+.5 to round to nearest
		float aspect=h/(float)w;
		py=(unsigned int)((h-1)*(y+aspect)/(aspect*2)+.5f); //+.5 to round to nearest
	}
}
	
void Graphics::getRealCoordinates(float& x, float& y, unsigned int px, unsigned int py) const {
	if(gen!=NULL) {
		gen->getRealCoordinates(x,y,px,py,genLayer);
	} else {
		//note width sets the scale for both, so coordinate system is square... is good? I'm up for debate.
		x=px/(float)(w-1)*2-1;
		float aspect=h/(float)w;
		y=py/(float)(h-1)*aspect*2-aspect;
	}
}

/*! @file
 * @brief 
 * @author ejt (Creator)
 */
