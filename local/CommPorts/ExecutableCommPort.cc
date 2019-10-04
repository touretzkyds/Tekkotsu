#include "ExecutableCommPort.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std; 

const std::string ExecutableCommPort::autoRegisterExecutableCommPort = CommPort::getRegistry().registerType<ExecutableCommPort>("ExecutableCommPort");

ExecutableCommPort::~ExecutableCommPort() {
	if(openedCnt>0) {
		cerr << "Connection still open in ExecutableCommPort destructor" << endl;
		command.removePrimitiveListener(this);
		shell.removePrimitiveListener(this);
	}
	if(!isChildRunning())
		return;
	cerr << "ExecutableCommPort destructing, but child is still running... waiting 3 seconds" << endl;
	if(waitChild(3000,500))
		return;
	cerr << "ExecutableCommPort child is STILL running... sending interrupt signal" << endl;
	if(kill(child,SIGINT)==-1) {
		perror("ExecutableCommPort unable to kill child");
		return;
	}
	if(waitChild(2000,500))
		return;
	cerr << "ExecutableCommPort child is STILL running... sending kill signal and moving on..." << endl;
	if(kill(child,SIGKILL)==-1) {
		perror("ExecutableCommPort unable to kill child");
		return;
	}
}

bool ExecutableCommPort::open() {
	if(openedCnt++>0)
		return true;
	command.addPrimitiveListener(this);
	shell.addPrimitiveListener(this);
	
	int s2c[2];
	int c2s[2];
	if(pipe(s2c)==-1) {
		perror("ERROR: ExecutableCommPort could not open simulator to child pipe");
		return false;
	}
	if(pipe(c2s)==-1) {
		perror("ERROR: ExecutableCommPort could not open child to simulator pipe");
		::close(s2c[0]);
		::close(s2c[1]);
		return false;
	}
	
	child = fork();
	if(child==-1) {
		perror("ERROR: ExecutableCommPort could not fork to launch executable");
		::close(s2c[0]);
		::close(s2c[1]);
		::close(c2s[0]);
		::close(c2s[1]);
		return false;
	}
	
	if(child==0) {
		// child process
		::close(s2c[WRITEPIPE]); // close write of sim to child (not used on our end!)
		::close(c2s[READPIPE]); // close read of child to sim (not used on our end!)
		
		::close(STDIN_FILENO); // theoretically not necessary (dup2 will close it), but we'll be paranoid
		if(::dup2(s2c[READPIPE],STDIN_FILENO)==-1)
			perror("ERROR: ExecutableCommPort could not dup2 the child's input to stdin");
		::close(s2c[READPIPE]); // now duplicated, close this reference
		
		::close(STDOUT_FILENO); // theoretically not necessary (dup2 will close it), but we'll be paranoid
		if(::dup2(c2s[WRITEPIPE],STDOUT_FILENO)==-1)
			perror("ERROR: ExecutableCommPort could not dup2 the child's output to stdout");
		::close(c2s[WRITEPIPE]); // now duplicated, close this reference
		
		// Launch executable!
		execlp(shell.c_str(), shell.c_str(), "-c", command.c_str(), NULL);
		
		// any return from here means error occurred!
		perror("ERROR: ExecutableCommPort could not launch the executable!");
		::close(STDIN_FILENO);
		::close(STDOUT_FILENO);
		_exit(EXIT_FAILURE); // note _exit() instead of exit(), avoids double-flushing buffers...
		
	} else {
		// parent process
		::close(s2c[READPIPE]); // close read of sim to child (not used on our end!)
		::close(c2s[WRITEPIPE]); // close write of child to sim (not used on our end!)
		rbuf.adoptFD(c2s[READPIPE]);
		wbuf.adoptFD(s2c[WRITEPIPE]);
	}
	opened();
	return true;
}

bool ExecutableCommPort::close() {
	if(openedCnt==0)
		std::cerr << "Warning: ExecutableCommPort close() without open()" << std::endl;
	if(--openedCnt>0)
		return false;
	closing();
	command.removePrimitiveListener(this);
	shell.removePrimitiveListener(this);
	rbuf.close();
	wbuf.close();
	return true;
}

void ExecutableCommPort::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&command || &pl==&shell) {
		if(openedCnt>0) {
			unsigned int tmp=openedCnt;
			openedCnt=1; // fake out close() and open() to make sure they trigger the action
			close();
			open();
			openedCnt=tmp; // reset original reference count
		}
	} else {
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

bool ExecutableCommPort::waitChild(unsigned int t, unsigned int p) {
	if(!isChildRunning())
		return true;
	for(unsigned int x=0; x<t; x+=p) {
		usleep(p*1000);
		if(!isChildRunning())
			return true;
	}
	return false;
}

bool ExecutableCommPort::isChildRunning() const {
	if(child==0)
		return false;
	int status=0;
	int ret=waitpid(child,&status,WNOHANG);
	if(ret==-1) {
		perror("ExecutableCommPort unable to check child status, waitpid");
		return false;
	}
	if(ret==0)
		return false;
	if(WIFEXITED(status) || WIFSIGNALED(status))
		return false;
	return true;
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
