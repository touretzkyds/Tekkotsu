//-*-c++-*-
#ifndef INCLUDED_Graphics_h_
#define INCLUDED_Graphics_h_

#include "Shared/Measures.h"

class FilterBankGenerator;

//! Provides basic graphics capabilities for drawing into any bitmap, particularly FilterBankGenerators
/*! Wherever possible, this should try to emulate the graphics API of Java 1 to minimize learning curve.
 *  For instance, the pen hangs down and to the right. */
class Graphics {
public:
	//! constructor, pass a FilterBankGenerator and layer/channel to draw into
	Graphics(FilterBankGenerator& fbg, unsigned int layer, unsigned int channel);

	//! constructor, pass a FilterBankGenerator and layer/channel to draw into
	Graphics(FilterBankGenerator& fbg, unsigned int layer, unsigned int chan1, unsigned int chan2, unsigned int chan3);

	//! constructor, directly specify an image buffer
	Graphics(unsigned char * base, unsigned int width, unsigned int height, unsigned int interval, unsigned int stride);

	//! If you want to reuse a graphics object across multiple frames from a FilterBankGenerator, call this after each new frame, but before you do any drawing
	/*! This is automatically called by the constructor, so you don't need to do it if you
	 *  constructor a fresh Graphics object for each frame.  But otherwise you'll need this
	 *  to update #img, #w, #h, #xInc, and #yInc from the current frame available in #gen */
	void updateFBG(); 

	//! Draws a rectange, upper left at @a x,@a y and extending right and down by @a width and @a height
	/*! This expects direct-pixel coordinates, so make sure you check the
	 *  width and height of the layer you are drawing into */
	void drawRect(int x, int y, int width, int height);

	//! Draws a rectange, upper left at @a x,@a y and extending right and down by @a width and @a height
	/*  This expects resolution independent coordinates, such that
	 *  x ranges [-1,1] and y ranges plus/minus the aspect ratio */
	void drawRect(float x, float y, float width, float height);

	//! Draws a line from (@a x1, @a y1) to (@a x2, @a y2)
	/*! This expects direct-pixel coordinates, so make sure you check the
	 *  width and height of the layer you are drawing into */
	void drawLine(int x1, int y1, int x2, int y2);

	//! Draws a line from (@a x1, @a y1) to (@a x2, @a y2)
	/*  This expects resolution independent coordinates, such that
	 *  x ranges [-1,1] and y ranges plus/minus the aspect ratio */
	void drawLine(float x1, float y1, float x2, float y2);

	//! Draws an ellipse at (x,y) with the specified parameters
	/*  This expects resolution independent coordinates, such that
	 *  x ranges [-1,1] and y ranges plus/minus the aspect ratio */
	void drawEllipse(float x, float y, float semimajor, float semiminor, AngPi orientation);

	//! Draws an ellipse at (x,y) with the specified parameters
	/*! This expects direct-pixel coordinates, so make sure you check the
	 *  width and height of the layer you are drawing into */
	void drawEllipse(int x, int y, float semimajor, float semiminor, AngPi orientation);

	//! Draws a quarter ellipse at (x,y) with the specified parameters; semimajor and/or semiminor may be negative.
	/*! This expects direct-pixel coordinates, so make sure you check the
	 *  width and height of the layer you are drawing into */
	void drawQuarterEllipse(int x, int y, float semimajor, float semiminor, AngPi orientation);

	//! Draws a single point at (@a x1, @a y1)
	/*! This expects direct-pixel coordinates, so make sure you check the
	 *  width and height of the layer you are drawing into */
	inline void drawPoint(int x, int y) {
		if(x<0 || y<0 || (unsigned int)x>=w || (unsigned int)y>=h)
			return;
		*(img1+y*yInc+x*xInc)=color.y;
		*(img2+y*yInc+x*xInc)=color.u;
		*(img3+y*yInc+x*xInc)=color.v;
	}
	//! Draws a single point at (@a x1, @a y1)
	/*  This expects resolution independent coordinates, such that
	 *  x ranges [-1,1] and y ranges plus/minus the aspect ratio */
	void drawPoint(float x, float y) {
		unsigned int px,py;
		getPixelCoordinates(px,py,x,y);
		drawPoint((int)px,(int)py);
	}

	//! Sets the "color" of the pen
	/*! Currently we don't support multi-channel drawing, so you have to draw into
	 *  each channel separately to do real color based drawing, but maybe someday
	 *  we'll add a color class.\n
	 *  In the mean time, this is just the byte that's going to be used to fill in
	 *  wherever the pen traces */
	void setColor(rgb _color) { color=rgb2yuv(_color); }

	//! Sets the "color" of the pen when drawing into a SegCam buffer
	void setColor(unsigned char _color) { color=yuv(_color, _color, _color); }

	//! returns the "color" of the pen
	/*! Currently we don't support multi-channel drawing, so you have to draw into
	 *  each channel separately to do real color based drawing, but maybe someday
	 *  we'll add a color class.\n
	 *  In the mean time, this is just the byte that's going to be used to fill in
	 *  wherever the pen traces */
	yuv getColor() const { return color; }

	//! sets the pixel-coordinate px and py parameters to the corresponding value of x and y
	/*! @param[out] px      the pixel position, relative to left edge, positive right, ranges 0 through width-1
	 *  @param[out] py      the pixel position, relative to top edge, positive down, ranges 0 through height-1
	 *  @param[in]  x       the horizontal position, relative to center of the image, left edge is -1 and right edge is 1; no boundary checking is done
	 *  @param[in]  y       the vertical pixel position, relative to center of the image, top edge is the negative aspect ratio, bottom edge is positive aspect ratio; no boundary checking is done
	 *
	 *  To keep the coordinate system square, the x is defined to range -1,1, but y's range depends on the
	 *  aspect ratio of the image, height/width.  Thus typically y will approx. -.75,.75 */
	void getPixelCoordinates(unsigned int& px, unsigned int& py, float x, float y) const;
	
	//! sets the x and y parameters from the pixel-coordinates px and py
	/*! @param[out] x       the horizontal position, relative to center of the image, left edge is -1 and right edge is 1; no boundary checking is done
	 *  @param[out] y       the vertical pixel position, relative to center of the image, top edge is the negative aspect ratio, bottom edge is positive aspect ratio; no boundary checking is done
	 *  @param[in]  px      the pixel position, relative to left edge, positive right, ranges 0 through width-1
	 *  @param[in]  py      the pixel position, relative to top edge, positive down, ranges 0 through height-1
	 *
	 *  To keep the coordinate system square, the x is defined to range -1,1, but y's range depends on the
	 *  aspect ratio of the image, height/width.  Thus typically y will approx. -.75,.75 */
	void getRealCoordinates(float& x, float& y, unsigned int px, unsigned int py) const;
	
protected:
	FilterBankGenerator* gen; //!< the filter bank generator we are drawing into, or NULL
	unsigned int genLayer; //!< the layer within #gen we are drawing into
	unsigned int genChan1; //!< the channel within #gen we are drawing into
	unsigned int genChan2; //!< the channel within #gen we are drawing into
	unsigned int genChan3; //!< the channel within #gen we are drawing into

	unsigned char* img1; //!< the image we are currently drawing into (may need to be updated if #gen is non-NULL, see updateFBG())
	unsigned char* img2; //!< the image we are currently drawing into (may need to be updated if #gen is non-NULL, see updateFBG())
	unsigned char* img3; //!< the image we are currently drawing into (may need to be updated if #gen is non-NULL, see updateFBG())
	unsigned int w; //!< the width of #img
	unsigned int h; //!< the height of #img
	unsigned int xInc; //!< the number of bytes to skip to move horizontally one pixel in #img
	unsigned int yInc; //!< the number of bytes to skip to move vertically one pixel in #img

	yuv color; //!< the current pen color

private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	Graphics(const Graphics&); //!< don't call (copy constructor)
	Graphics& operator=(const Graphics&); //!< don't call (assignment operator)
};

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

#endif
