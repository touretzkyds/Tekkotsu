#ifndef INCLUDED_dumpFigures_h
#define INCLUDED_dumpFigures_h

#include "Planners/GaitedFootsteps.h"
#include "Planners/AStar.h"

typedef AStar::Node<GaitedFootsteps::State> AStarNode;
typedef AStar::Results<GaitedFootsteps::State> AStarResults;

enum {
	FIGURES_SHOW_PATH=1<<0,
	FIGURES_SHOW_WORK=1<<1,
	FIGURES_SHOW_STEPS=1<<2,
	FIGURES_SHOW_SUPPORT=1<<3,
	FIGURES_SHOW_GRID=1<<4,
	FIGURES_EMBED_STYLE=1<<5
};

extern float figureOriLen;

void dumpFigures(const std::string& outfile, const GaitedFootsteps& f, const AStarResults& res, const fmat::Column<2>& goal, const fmat::Column<4> view, const std::string& styleFile, int flags);

#endif
