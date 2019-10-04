//-*-c++-*-
#ifndef INCLUDED_VisionObjectEvent_h
#define INCLUDED_VisionObjectEvent_h

#include "EventBase.h"

//! Extends EventBase to also include location in the visual field and distance (though distance is not implimented yet)
class VisionObjectEvent : public EventBase {
public:
	//! Constructor, pass a source id and type id -- mainly useful for deactivate events since all object parameters are going to be set to 0
	/*! @param sid The source ID for the object being detected -- you can define your own values, some are already set in ProjectInterface, but can be reassigned during your project's startup
	 *  @param tid The type ID for the event
	 */
  explicit VisionObjectEvent(size_t sid=0,EventTypeID_t tid=EventBase::deactivateETID)
		: EventBase(EventBase::visObjEGID,sid,tid,0),
			_x1(0),_x2(0),_y1(0),_y2(0), _area(0),
			_clipLeft(false), _clipRight(false), _clipTop(false), _clipBottom(false),
			_xRange(0), _yRange(0), _frame(0)
	{}
		
	//! Constructor, pass the type id, source id, left, right, top, bottom, x range, and y range
	/*! @param sid The source ID for the object being detected -- you can define your own values, some are already set in ProjectInterface, but can be reassigned during your project's startup
	 *  @param tid The type ID for the event
	 *  @param x1 The leftmost extent of the object, in generalized coordinates (see #_x1)
	 *  @param x2 The rightmost extent of the object in generalized coordinates (see #_x2)
	 *  @param y1 The topmost extent of the object, in generalized coordinates (see #_y1)
	 *  @param y2 The bottommost extent of the object, in generalized coordinates (see #_y2)
	 *  @param objarea The area of the object being detected, in squared generalized coordinates (see #_area)
	 *  @param rx The plus/minus range of the x coordinates (generally xres/xres for cameras which are wider than they are high)
	 *  @param ry The plus/minus range of the y coordinates (camera yres/xres for cameras which are wider than they are high)
	 */
  VisionObjectEvent(size_t sid, EventTypeID_t tid, float x1, float x2, float y1, float y2,float objarea, float rx, float ry)
		: EventBase(EventBase::visObjEGID,sid,tid,0),
			_x1(x1),_x2(x2),_y1(y1),_y2(y2), _area(objarea),
			_clipLeft(_x1<=-rx), _clipRight(_x2>=rx), _clipTop(_y1<=-ry), _clipBottom(_y2>=ry),
			_xRange(rx), _yRange(ry), _frame(0)
	{}
	
 	//! Constructor, pass the type id, source id, left, right, top, bottom, x range, y range, and frame_number
	/*! @param sid The source ID for the object being detected -- you can define your own values, some are already set in ProjectInterface, but can be reassigned during your project's startup
	 *  @param tid The type ID for the event
	 *  @param x1 The leftmost extent of the object, in generalized coordinates (see #_x1)
	 *  @param x2 The rightmost extent of the object in generalized coordinates (see #_x2)
	 *  @param y1 The topmost extent of the object, in generalized coordinates (see #_y1)
	 *  @param y2 The bottommost extent of the object, in generalized coordinates (see #_y2)
	 *  @param objarea The area of the object being detected, in squared generalized coordinates (see #_area)
	 *  @param rx The plus/minus range of the x coordinates (generally xres/xres for cameras which are wider than they are high)
	 *  @param ry The plus/minus range of the y coordinates (camera yres/xres for cameras which are wider than they are high)
	 *  @param frame The camera frame number the object was detected in (see #_frame)
	 */
  VisionObjectEvent(size_t sid, EventTypeID_t tid, float x1, float x2, float y1, float y2, float objarea, float rx, float ry,unsigned int frame)
		: EventBase(EventBase::visObjEGID,sid,tid,0),
			_x1(x1),_x2(x2),_y1(y1),_y2(y2), _area(objarea),
			_clipLeft(_x1<=-rx), _clipRight(_x2>=rx), _clipTop(_y1<=-ry), _clipBottom(_y2>=ry),
			_xRange(rx),_yRange(ry), _frame(frame)
	{} 
  
	//! destructor
	virtual ~VisionObjectEvent() {}
  
	virtual EventBase* clone() const { return new VisionObjectEvent(*this); }

	virtual unsigned int getClassTypeID() const { return autoRegisterVisionObjectEvent; }

	//!@name Attribute Accessors
	float getLeft() const { return _x1;} //!< returns the initial x (#_x1) coordinate of the Bounding Box (inclusive value)
	VisionObjectEvent& setLeft(float x1) { _x1=x1; return *this;} //!< sets the initial x (#_x1) coordinate of the Bounding Box
	
	float getRight() const { return _x2;} //!< returns the final x (#_x2) coordinate of the Bounding Box (inclusive value)
	VisionObjectEvent& setRight(float x2) { _x2=x2; return *this;} //!< sets the final x (#_x2) coordinate of the Bounding Box
	
	float getTop() const { return _y1;} //!< returns the initial y (#_y1) coordinate of the Bounding Box (inclusive value)
	VisionObjectEvent& setTop(float y1) { _y1=y1; return *this;} //!< sets the initial y (#_y1) coordinate of the Bounding Box
	
	float getBottom() const { return _y2;} //!< returns the final y (#_y2) coordinate of the Bounding Box (inclusive value)
	VisionObjectEvent& setBottom(float y2) { _y2=y2; return *this;} //!< sets the final y (#_y2) coordinate of the Bounding Box
	
	float getObjectArea() const { return _area; } //!< returns the object's #_area within the camera, in squared generalized coordinates.  Multiply by the square of the major camera resolution (normally RobotInfo::CameraResolutionX if using full resolution) and divide by 4.0 to get pixel area.
	VisionObjectEvent& setObjectArea(float objarea) { _area=objarea; return *this; } //!< sets the object's #_area within the camera, in squared generalized coordinates (multiply by the major camera resolution to get pixel area)
	//@}
 
	//!@name Calculated Attributes
	float getDistanceEstimate(float diaMajor, float diaMinor=0) const; //!< returns an estimate of how far away the object is if its major (larger) physical dimension is @a diaMajor and the other dimension is @a diaMinor; pass 0 if to only use the major dimension
	static float calcDistance(float visArc, float physDia); //!< returns the distance of an object, given that it takes up @a visArc of the camera's visual arc, and the physical crossection is @a physDia
	float getCenterX() const { return (_x1+_x2)/2; } //!< returns the center along x
	float getCenterY() const { return (_y1+_y2)/2; } //!< returns the center along y
	float getWidth() const { return _x2-_x1; } //!< return width along x
	float getHeight() const { return _y2-_y1; } //!< return height along y
	float getBoundaryArea() const { return (_x2-_x1)*(_y2-_y1); } //!< returns the area of the bounding box, just multiplication of width*height, (multiply by the major camera resolution to get pixel area)
	float getXrange() const{ return  _xRange;}//!< returns the maximum x value
	float getYrange() const{return _yRange;}//!< returns the maximum y value
	unsigned int getFrame() const{return _frame;}//!< returns number of frame when the event was generated
	//@}

	//!@name Object out of bounds Detection Functions
	bool isClipped() const { return isClippedLeft() || isClippedRight() || isClippedTop() || isClippedBottom(); }
	bool isClippedLeft() const { return _clipLeft; } //!< returns #_clipLeft
	bool isClippedRight() const { return _clipRight; }  //!< returns #_clipRight
	bool isClippedTop() const { return _clipTop; } //!< returns #_clipTop
	bool isClippedBottom() const {return _clipBottom; } //!< returns #_clipBottom
	void setClipping(bool left, bool right, bool top, bool bottom) { _clipLeft=left; _clipRight=right; _clipTop=top; _clipBottom=bottom; } //!< sets clipping boundaries
	//@}
      
	virtual std::string getDescription(bool showTypeSpecific=true, unsigned int verbosity=0) const;
	
	virtual unsigned int getBinSize() const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;

protected:
	float _x1;  //!< a value representing location in visual field - from -1 if on the left edge to 1 if it's on the right edge, see Config::vision_config::x_range
	float _x2;  //!< a value representing location in visual field - from -1 if on the left edge to 1 if it's on the right edge, see Config::vision_config::x_range
	float _y1;  //!< top boundary, in range of ±1/Config::vision_config::aspectRatio, see Config::vision_config::y_range
	float _y2;  //!< bottom boundary, in range of ±1/Config::vision_config::aspectRatio, see Config::vision_config::y_range
	float _area; //!< area of the actual object within bounding box as set by generator, in same units as getBoundaryArea().  Multiply by the square of the major camera resolution (normally RobotInfo::CameraResolutionX if using full resolution) and divide by 4.0 to get pixel area.
	bool _clipLeft;   //!< flag to indicate left boundary is on or beyond the camera image's boundary
	bool _clipRight;  //!< flag to indicate right boundary is on or beyond the camera image's boundary
	bool _clipTop;    //!< flag to indicate top boundary is on or beyond the camera image's boundary
	bool _clipBottom; //!< flag to indicate bottom boundary is on or beyond the camera image's boundary
	float _xRange; //!< Max range of X dimension (typically 1.0 for AIBO)
	float _yRange; //!< Max range of Y dimension (typically around 0.8 for AIBO due to camera aspect ratio)
	unsigned int _frame; //!< Number of frame when the event was generated.
	
	//! causes class type id to automatically be regsitered with EventBase's FamilyFactory (getTypeRegistry())
	static const EventBase::classTypeID_t autoRegisterVisionObjectEvent;
};

/*! @file
 * @brief Describes VisionObjectEvent, which provides information about objects recognized in the camera image
 * @author alokl (Creator)
 * @author Ignacio Herrero Reder &lt; nhr at dte uma es &gt; (VisionObjectInfo Boundary Box - bug 74, frame number - bug 143)
 */

#endif
