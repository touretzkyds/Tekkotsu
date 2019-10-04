//-*-c++-*-
#ifndef INCLUDED_DumpFileControl_h_
#define INCLUDED_DumpFileControl_h_

#include "FileBrowserControl.h"
#include <fstream>

//! Upon activation, loads a position from a file name read from cin (stored in ms/data/motion...)
class DumpFileControl : public FileBrowserControl {
 public:
	//! Constructor
	DumpFileControl(const std::string& n,const std::string& r)
		: FileBrowserControl(n,"Dumps a user specified file to the console",r)
	{}
	//! Destructor
	virtual ~DumpFileControl() {}

protected:
	//!does the actual loading of the MotionSequence
	virtual ControlBase* selectedFile(const std::string& f) {
		const unsigned int BUFSIZE=128;
		doRewrite=false;
		std::ifstream in(f.c_str());
		while(in) {
			byte* buf=sout->getWriteBuffer(BUFSIZE);
			if(buf==NULL) {
				serr->printf("DumpFileControl - write failed\n");
				return this;
			}
			in.read(reinterpret_cast<char*>(buf),BUFSIZE);
			sout->write(in.gcount());
		}
		sout->printf("\n");
		return this;
	}
};

/*! @file
 * @brief Defines DumpFileControl, which when activated, plays a sound selected from the memory stick
 * @author ejt (Creator)
 */

#endif
