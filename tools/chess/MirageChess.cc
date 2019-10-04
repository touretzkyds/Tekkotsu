#include "MirageChess.h"
#include "local/DeviceDrivers/MirageComm.h"

const std::string MirageChess::pieceNames[MirageChess::NUM_PLAYERS][MirageChess::NUM_PIECES] = {
	{
		"W_PAWN_A", "W_PAWN_B", "W_PAWN_C", "W_PAWN_D", "W_PAWN_E", "W_PAWN_F", "W_PAWN_G", "W_PAWN_H",
		"W_ROOK_A", "W_KNIGHT_A", "W_BISHOP_A", "W_KING", "W_QUEEN", "W_BISHOP_B", "W_KNIGHT_B", "W_ROOK_B"
	}, {
		"B_PAWN_A", "B_PAWN_B", "B_PAWN_C", "B_PAWN_D", "B_PAWN_E", "B_PAWN_F", "B_PAWN_G", "B_PAWN_H",
		"B_ROOK_A", "B_KNIGHT_A", "B_BISHOP_A", "B_KING", "B_QUEEN", "B_BISHOP_B", "B_KNIGHT_B", "B_ROOK_B"
	}, 
};

bool MirageChess::connect(const std::string& host) {
	MirageComm mirage(comm);
	if(!mirage.connect(host))
		return false;
	KinematicJoint * root = new KinematicJoint;
	LinkComponent c;
	c.model = "Plane";
	c.material = "Green";
	c.modelScale[0] = dim;
	c.modelScale[1] = dim;
	c.modelOffset[2] = .1f;
	for(float row=-3.5; row<4; row+=1) {
		for(float col=-3.5; col<4; col+=1) {
			if(!(static_cast<int>(std::floor(row+col+.5))&1))
				continue;
			c.modelOffset[0] = dim*col;
			c.modelOffset[1] = dim*row;
			LinkComponent * c2 = c.clone();
			root->components.addEntry(c2);
		}
	}
	mirage.setName("ChessBoard");
	mirage.setKinematics(root);
	mirage.setPosition(boardOffset[0],boardOffset[1],0);
	mirage.setOrientation(fmat::Quaternion::aboutZ(boardAngle));
	initPieces();
	return comm;
}

void MirageChess::initPieces() {
	MirageComm mirage(comm);
	for(size_t p=0; p<NUM_PLAYERS; ++p) {
		for(size_t i=0; i<NUM_PIECES; ++i) {
			// start a new object
			mirage.setName(pieceNames[p][i]);
			
			// calculate where this piece should appear
			int col = i%8;
			int row = 1 - i/8;
			if(p>0)
				row = 7-row;
			fmat::Column<2> loc = boardPos(row,col);
			mirage.setPosition(loc[0],loc[1],0);
			
			// Define physical characteristics of the object...
			// (here we just create and register the storage, note the NO_FLUSH!
			// So it won't be sent until we actually finish setting parameters)
			KinematicJoint * root = new KinematicJoint;
			mirage.setKinematics(root);
			
			// configure some dimensions unique to each piece
			float height=0;
			float bulb=0;
			switch(static_cast<Piece>(i)) {
				case PAWN_A:
				case PAWN_B:
				case PAWN_C:
				case PAWN_D:
				case PAWN_E:
				case PAWN_F:
				case PAWN_G:
				case PAWN_H: {
					height=47;
					bulb=19.75f;
					root->model="Chess/Pawn";
				} break;
				
				case ROOK_A:
				case ROOK_B: {
					height=53.5f;
					bulb=22.5f;
					root->model="Chess/Rook";
				} break;
				
				case KNIGHT_A:
				case KNIGHT_B: {
					height=55.6f;
					root->model="Chess/Knight";
				} break;
				
				case BISHOP_A:
				case BISHOP_B: {
					height=67.25f;
					bulb=20.85f;
					root->model="Chess/Bishop";
				} break;
				
				case QUEEN:  {
					height=77;
					bulb=20.5f;
					root->model="Chess/Queen";
				} break;
				
				case KING:  {
					height=90;
					bulb=24;
					root->model="Chess/King";
				} break;
			}
			// align pieces with board, facing opposing team
			// (graphics files are aligned to face along X axis)
			fmat::Quaternion::aboutZ(boardAngle+((p>0)?-M_PI:M_PI)/2).exportTo(root->modelRotation[0],root->modelRotation[1],root->modelRotation[2]);
			
			// what color is each player?
			root->material=(p>0)?"Blue":"Yellow";
			
			// configure mass for manipulation, place mass low
			root->mass=.2f;
			root->centerOfMass[2]=height/5;
			
			// place a collision column around each piece
			root->collisionModel="Cylinder";
			root->collisionModelScale=plist::Point(15,15,height);
			root->collisionModelOffset=plist::Point(0,0,height/2);
			
			// put a ball (or cylinder for knights) at the top so piece doesn't slide out of gripper
			// (i.e. provide a physical thing to grab on to, not just relying on friction)
			if(i==KNIGHT_A || i==KNIGHT_B) {
				LinkComponent* c = new LinkComponent;
				c->model="";
				c->material=root->material;
				c->collisionModel="Cylinder";
				c->collisionModelOffset[2]=height-11.5f;
				c->collisionModelScale = plist::Point(11.5f,11.5f,35);
				fmat::Quaternion::aboutY(M_PI/2).exportTo(c->collisionModelRotation[0],c->collisionModelRotation[1],c->collisionModelRotation[2]);
				root->components.addEntry(c);
			} else {
				LinkComponent* c = new LinkComponent;
				c->model="";
				c->material=root->material;
				c->collisionModel="Sphere";
				c->collisionModelOffset[2]=height-bulb/2;
				c->collisionModelScale = plist::Point(bulb,bulb,bulb);
				root->components.addEntry(c);
			}
		}
	}
}

void MirageChess::movePiece(bool player, Piece piece, int row, int col) {
	fmat::Column<2> loc = boardPos(row,col);
	setPosition(player,piece,loc[0],loc[1]);
}

void MirageChess::setPosition(bool player, Piece piece, float x, float y, float z/*=0*/) {
	MirageComm mirage(comm);
	mirage.setName(pieceNames[player][piece]);
	mirage.setPosition(x,y,z);
	// reset orientation
	mirage.setOrientation(fmat::Quaternion::aboutZ(boardAngle+((player>0)?-M_PI:M_PI)/2));
}

fmat::Column<2> MirageChess::boardPos(int row, int col) {
	fmat::Column<2> loc = fmat::pack(col-3.5f,row-3.5f)*dim;
	return fmat::rotation2D(boardAngle)*loc + boardOffset;
}

