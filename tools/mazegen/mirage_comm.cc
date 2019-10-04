#include "mazegen.h"

#include "Shared/plist.h"
#include "Wireless/netstream.h"
#include "Motion/KinematicJoint.h"
#include "Shared/fmat.h"
#include <memory>

using namespace std;

enum walldir_t { HORIZ, VERT };
KinematicJoint* makeMaze(map_t map);
LinkComponent* makeWall(walldir_t dir, size_t low, size_t high, size_t cn);

void sendMazeMirage(map_t map) {
	ionetstream mirage;
	
	// open connection
	std::cout << "Connecting..." << std::flush;
	if(!mirage.open(mirageHost)) {
		std::cout << "refused." << std::endl;
		return;
	}
	std::cout << "connected " << mirage.getPeerAddress().get_rname() << ":" << mirage.getPeerAddress().get_port() << std::endl;
	mirage << "<messages>\n";
	
	// this will hold the keys and values we want to send to mirage
	plist::Dictionary msg;
	
	// name for the maze within mirage
	msg.addEntry("ID",new plist::Primitive<std::string>("maze"));
	
	// this will position the maze so 0,0 is the "start"
	plist::ArrayOf<plist::Primitive<float> > location(3,0);
	fmat::pack(startX*-cellDim,startY*-cellDim,0.f).exportTo(location);
	msg.addEntry("Location",location);
	
	// this keeps the maze instantiated in Mirage even when we close our connection
	msg.addEntry("Persist",new plist::Primitive<bool>(true));
	
	// create the maze walls as components of a immobile kinematic joint
	std::auto_ptr<KinematicJoint> maproot(makeMaze(map));
	msg.addEntry("Model",new KinematicJointSaver(*maproot));
	
	// write the message and exit
	msg.saveStream(mirage,true);
	mirage << "</messages>";
	mirage.flush();
	mirage.close();
}

KinematicJoint* makeMaze(map_t map) {
	KinematicJoint* wallHolder = new KinematicJoint;
	if(map.size()>0) {
		const size_t WIDTH=map.front().size();
		const size_t HEIGHT=map.size();
		wallHolder->components.addEntry(makeWall(HORIZ,0,WIDTH,HEIGHT));
		wallHolder->components.addEntry(makeWall(VERT,0,HEIGHT,0));
		for(size_t y=0; y<HEIGHT; ++y) {
			size_t begin=0, x=0;
			for(; x<WIDTH; ++x) {
				if(!map[y][x].bottomWall()) {
					if(begin!=x)
						wallHolder->components.addEntry(makeWall(HORIZ,begin,x,HEIGHT-y-1));
					begin=x+1;
				}
			}
			if(begin!=x)
				wallHolder->components.addEntry(makeWall(HORIZ,begin,x,HEIGHT-y-1));
		}
		for(size_t x=0; x<WIDTH; ++x) {
			size_t begin=0, y=0;
			for(; y<HEIGHT; ++y) {
				if(!map[y][x].rightWall()) {
					if(begin!=y)
						wallHolder->components.addEntry(makeWall(VERT,HEIGHT-y,HEIGHT-begin,x+1));
					begin=y+1;
				}
			}
			if(begin!=y)
				wallHolder->components.addEntry(makeWall(VERT,HEIGHT-y,HEIGHT-begin,x+1));
		}
	}
	return wallHolder;
}

LinkComponent* makeWall(walldir_t dir, size_t low, size_t high, size_t cn) {
	//cout << "Wall " << (dir==HORIZ?"HORIZ":"VERT") << " from " << low << " to " << high << " on " << cn << endl;
	LinkComponent* c = new LinkComponent;
	c->model="CollisionModel";
	c->material="Blue Plastic";
	c->collisionModel="Cube";
	float len=cellDim*(high-low) + wallDepth;
	float a = len/2 + cellDim*low - wallDepth/2 - cellDim/2;
	float b = cellDim*cn - cellDim/2;
	if(dir==HORIZ) {
		//a-=startX*cellDim;
		//b-=startY*cellDim;
		fmat::pack(len,wallDepth,wallHeight).exportTo(c->collisionModelScale);
		fmat::pack(a,b,wallHeight/2).exportTo(c->collisionModelOffset);
	} else {
		//a-=startY*cellDim;
		//b-=startX*cellDim;
		fmat::pack(wallDepth,len,wallHeight).exportTo(c->collisionModelScale);
		fmat::pack(b,a,wallHeight/2).exportTo(c->collisionModelOffset);
	}
	return c;
}


