//-*-c++-*-
#ifndef INCLUDED_ImageBuffer_h_
#define INCLUDED_ImageBuffer_h_

#include "Shared/ReferenceCounter.h"
#include "IPC/Thread.h"
#include <vector>

//! A buffer for storing a rendered image and its compressed forms for transmission to a camera subscriber (CommThread::Subscription)
class ImageBuffer : public ReferenceCounter {
public:
	//! constructor, pass width and height, which will remain constant for lifetime of object (must delete and recreate to change dimensions)
	ImageBuffer(unsigned int w, unsigned int h)
		: ReferenceCounter(), width(w), height(h), pixels(w*h*3), yuv(), png(), jpg()
	{
		//std::cout << "Created " << this << std::endl;
	}
	//! destructor, free associated buffers
	~ImageBuffer() {
		//std::cout << "Destroyed " << this << std::endl;
	}
	
	unsigned int getWidth() const { return width; } //!< returns the image #width assumed for input and output buffers
	unsigned int getHeight() const { return height; } //!< returns the image #height assumed for input and output buffers
	
	//! returns buffer where input data should be placed
	/*! This assumes incoming data will be RGB, but ideally we would have the renderer apply a
	 *  Cg or GLSL operation to convert this data to YUV444 which would then let us skip
	 *  the updateYUV() operation in this class */
	char* getRGBBuffer() const { return const_cast<char*>(&pixels[0]); }
	
	bool getYUV(const char*& buf, size_t& size); //!< convert input color space to YUV and return the buffer via arguments (input values ignored)
	bool getPNG(const char*& buf, size_t& size, int compressionLevel); //!< compress as PNG and return the buffer via arguments (input values ignored)
	bool getJPG(const char*& buf, size_t& size, int quality); //!< compress as JPEG and return the buffer via arguments (input values ignored)

	//! marks previously computed invalid, will reuse buffer allocation
	void clear() { yuv.clear(); png.clear(); jpg.clear(); }

protected:
	const unsigned int width; //!< stores image width
	const unsigned int height; //!< stores image height
	std::vector<char> pixels; //!< stores input image data (currently RGB, maybe someday YUV)
	
	//! convert from the input RGB (#pixels) to yuv444 in #yuv
	void updateYUV();
	
	//! stores a derivative encoding and buffer meta-data
	struct Encoding {
		//! constructor, does not create buffer
		Encoding() : lock(), used(0), valid(false), buf() {}
		//! destructor, free buffer
		~Encoding() { lock.lock(); buf.clear(); }
		//! marks #valid false, but does not free buffer
		void clear() { valid=false; }
		char* getBuffer() { return const_cast<char*>(&buf[0]); }
		const char* getBuffer() const { return &buf[0]; }
		void resize(size_t sz) { buf.resize(sz); }
		size_t size() const { return buf.size(); }
		
		Thread::Lock lock; //!< prevents multiple subscriptions trying to perform the same compression at the same time
		size_t used; //!< amount of buffer in use (only this portion needs to be transmitted to subscribers)
		bool valid; //!< false if new input is available and thus #buf needs to be recomputed
	protected:
		std::vector<char> buf; //!< stores compressed or transformed image data
	};
	Encoding yuv; //!< yuv444 data
	Encoding png; //!< PNG compressed data
	Encoding jpg; //!< JPEG compressed data
};

/*! @file
 * @brief Describes ImageBuffer, for storing a rendered image and its compressed forms for transmission to a camera subscriber (CommThread::Subscription)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
