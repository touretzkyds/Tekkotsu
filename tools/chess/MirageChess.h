#include "Wireless/netstream.h"
#include "Shared/fmat.h"
#include "Motion/KinematicJoint.h"
#include <string>
#include <stdexcept>

class MirageChess {
public:
	//! constructor, provide size of each board tile, the position of the center of the board, and the angle of the board
	MirageChess(float cellsize, float cx, float cy, float a)
	: comm(), dim(cellsize), boardOffset(fmat::pack(cx,cy)), boardAngle(a)
	{}
	
	//! destructor, tears down connection, removes board
	~MirageChess() {}
	
	//! tags to refer to pieces, see also #pieceNames
	enum Piece {
		PAWN_A, PAWN_B, PAWN_C, PAWN_D, PAWN_E, PAWN_F, PAWN_G, PAWN_H,
		ROOK_A, KNIGHT_A, BISHOP_A, KING, QUEEN, BISHOP_B, KNIGHT_B, ROOK_B
	};
	
	//! connects to Mirage and builds the chessboard
	bool connect(const std::string& host);
	
	//! moves a piece to a specified board position
	void movePiece(bool player, Piece piece, int row, int col);
	
	//! moves a piece to a position in world coordinates, also resets orientation in case it was knocked over
	void setPosition(bool player, Piece piece, float x, float y, float z=0);
	
	//! computes the world coordinates of a board position
	fmat::Column<2> boardPos(int row, int col);
	
protected:
	static const size_t NUM_PLAYERS=2;
	static const size_t NUM_PIECES=16;
	static const std::string pieceNames[NUM_PLAYERS][NUM_PIECES];
	
	//! places pieces on the board in default configuration
	void initPieces();
	
	ionetstream comm;
	float dim;
	fmat::Column<2> boardOffset;
	float boardAngle;
};
