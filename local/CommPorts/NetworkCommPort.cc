#include "NetworkCommPort.h"
#include "Shared/MarkScope.h"

using namespace std;

const std::string NetworkCommPort::autoRegisterNetworkCommPort = CommPort::getRegistry().registerType<NetworkCommPort>("NetworkCommPort");

bool NetworkCommPort::isReadable() {
	sbuf.update_status();
	if(sbuf.is_open())
		return true;
	if(recon.isStarted())
		return false;
	if(openedCnt>0) {
		MarkScope l(getLock());
		if(recon.isStarted())
			return false;
		if(!doOpen(true)) {
			recon.start();
			return false;
		}
	}
	return sbuf.is_open();
}

bool NetworkCommPort::open() {
	MarkScope l(getLock());
	if(openedCnt++>0)
		return true;
	sbuf.setReconnect(block);
	block.addPrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	host.addPrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	port.addPrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	transport.addPrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	server.addPrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	curaddr.set_name(host);
	curaddr.set_port(port);
	if(!doOpen(true)) {
		if(recon.isStarted())
			return false;
		recon.start();
		return false;
	}
	return true;
}

bool NetworkCommPort::doOpen(bool dispError) {
	MarkScope l(getLock());
	if(sbuf.is_open())
		return true;
	if(server) {
		if(block)
			cout << "Waiting for '" << instanceName << "' connection on port " << port << "... " << flush;
		if(!sbuf.listen(curaddr,transport==Config::UDP)) {
			Thread::testCurrentCancel();
			if(dispError) {
				stringstream ss;
				ss << "Could not listen '" << instanceName << "' on port " << port;
				connectionError(ss.str(),true);
			}
			return false;
		}
	} else {
		if(block)
			cout << "Waiting for '" << instanceName << "' connection to " << host << ':' << port << "... " << flush;
		if(!sbuf.open(curaddr,transport==Config::UDP)) {
			Thread::testCurrentCancel();
			if(dispError) {
				stringstream ss;
				ss << "Could not open '" << instanceName << "' connection to " << host << ":" << port;
				connectionError(ss.str(),true);
			}
			return false;
		}
	}
	if(block)
		cout << instanceName <<  " connected." << endl;
	sbuf.pubseekoff(0,ios_base::end,ios_base::in);
	opened();
	return true;
}

void NetworkCommPort::keepOpen() {
	while(!sbuf.is_open()) {
		sleep(1);
		Thread::testCurrentCancel();
		doOpen(false);
	}
}

bool NetworkCommPort::close() {
	MarkScope l(getLock());
	if(openedCnt==0)
		std::cerr << "Warning: NetworkCommPort close() without open()" << std::endl;
	if(--openedCnt>0)
		return false;
	sbuf.setReconnect(false);
	if(recon.isStarted())
		recon.stop().join();
	if(sbuf.is_open()) {
		closing();
		sbuf.close();
	}
	host.removePrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	port.removePrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	transport.removePrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	server.removePrimitiveListener(dynamic_cast<plist::PrimitiveListener*>(this));
	return true;
}

void NetworkCommPort::plistValueChanged(const plist::PrimitiveBase& pl) {
	if(&pl==&host || &pl==&port || &pl==&transport) {
		MarkScope l(getLock());
		if(host!=curaddr.get_name() || port!=curaddr.get_port() || curtrans!=transport) {
			close();
			open();
		}
	} else if(&pl==&server) {
		MarkScope l(getLock());
		close();
		open();
	} else if(&pl==&verbose) {
		sbuf.setEcho(verbose);
	} else if(&pl==&block) {
		MarkScope l(getLock());
		if(recon.isStarted())
			recon.stop().join();
		sbuf.setReconnect(block);
	} else {
		std::cerr << "Unhandled value change in " << getClassName() << ": " << pl.get() << std::endl;
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
