#include "MirageChess.h"
#include <iostream>

using namespace std;

int main(int argc, const char* argv[]) {
	const float dim=70;
	MirageChess chess(dim, dim*4, -dim*4, M_PI/2);
	if(!chess.connect("localhost")) {
		cerr << "Connection refused" << endl;
		return EXIT_FAILURE;
	}
	cout << "Connected." << endl;
	chess.movePiece(0,MirageChess::PAWN_C,3,2);
	chess.setPosition(1,MirageChess::BISHOP_A,dim*6,dim*2);
	int x;
	cin >> x;
	return EXIT_SUCCESS;
}