#include "dumpFigures.h"
#include <libxml/xmlwriter.h>
#include <stack>
#include <sstream>

typedef AStar::Node<GaitedFootsteps::State> AStarNode;
typedef AStar::Results<GaitedFootsteps::State> AStarResults;

float figureOriLen=0;

class Element {
public:
	Element(xmlTextWriter* w, const std::string& n) : writer(w), name(n) {
		if(xmlTextWriterStartElement(writer,BAD_CAST name.c_str()) < 0)
			throw std::runtime_error("Could not start '"+name+"' element");
	}
	Element(xmlTextWriter* w, const std::string& n, const std::string& ns) : writer(w), name(n) {
		if(xmlTextWriterStartElementNS(writer,NULL,BAD_CAST name.c_str(), BAD_CAST ns.c_str()) < 0)
			throw std::runtime_error("Could not start '"+name+"' element");
	}
	~Element() {
		if(xmlTextWriterEndElement(writer) < 0)
			throw std::runtime_error("Could not close '"+name+"' element");
	}
	template<typename T>
	Element& attribute(const std::string& aname, const T& val) {
		std::ostringstream ss; ss << val;
		if(xmlTextWriterWriteAttribute(writer, BAD_CAST aname.c_str(), BAD_CAST ss.str().c_str()) < 0)
			throw std::runtime_error("Could not write '"+aname+"'");
		return *this;
	}
protected:
	xmlTextWriter* writer;
	std::string name;
};

inline std::string stripExt(const std::string& s) { return s.substr(0,s.rfind('.')); }
inline void expandBB(float x, float y, fmat::Column<2>& min, fmat::Column<2>& max);
inline void expandBB(const fmat::Column<2>& p, fmat::Column<2>& min, fmat::Column<2>& max);
void obstacleBounds(const GaitedFootsteps& f, fmat::Column<2>& min, fmat::Column<2>& max);
void workBounds(const AStarResults& res, fmat::Column<2>& min, fmat::Column<2>& max);
void pathBounds(const AStarNode& node, fmat::Column<2>& min, fmat::Column<2>& max);
void feetBounds(const AStarNode& node, fmat::Column<2>& min, fmat::Column<2>& max);

void loadStyles(xmlTextWriter* writer, const std::string& file);
void writeNodeLine(xmlTextWriter* writer, const AStarNode& node, const std::string& cl);
void writeNode(xmlTextWriter* writer, const fmat::Column<2>& pos, const std::string& marker);
void writeObstacles(xmlTextWriter* writer, const GaitedFootsteps& f, const std::string& clName);
void writeViewboxComment(xmlTextWriter* writer, const std::string& name, const fmat::Column<2>& min, const fmat::Column<2>& max) {
	if(xmlTextWriterWriteFormatComment(writer, (name+" viewBox: %g %g %g %g").c_str(),min[0],min[1],max[0]-min[0],max[1]-min[1]) < 0)
		throw std::runtime_error("Could not write view comment");
	if(xmlTextWriterWriteString(writer, BAD_CAST "\n") < 0)
		throw std::runtime_error("Could not write newline");
}

void dumpFigures(const std::string& outFile, const GaitedFootsteps& f, const AStarResults& res, const fmat::Column<2>& goal, const fmat::Column<4> view, const std::string& styleFile, int flags) {
	std::cout << "Writing figures to " << outFile << std::endl;
	xmlTextWriter* writer = xmlNewTextWriterFilename(outFile.c_str(), 0);
	if(writer==NULL) throw std::runtime_error("Could not allocate XML writer");
	
	if(xmlTextWriterStartDocument(writer,NULL,NULL,NULL) < 0)
		throw std::runtime_error("Could not open document");
	
	// some SVG converters don't support this boo hiss :-P
	if(!(flags & FIGURES_EMBED_STYLE)) {
		if(xmlTextWriterWritePI(writer,BAD_CAST "xml-stylesheet",BAD_CAST ("href=\""+styleFile+"\" type=\"text/css\"").c_str()) < 0)
			throw std::runtime_error("Could not write style");
	}
	
	if(xmlTextWriterWriteString(writer, BAD_CAST "\n") < 0)
		throw std::runtime_error("Could not write newline");
	if(xmlTextWriterWriteDTD(writer,BAD_CAST "svg",BAD_CAST "-//W3C//DTD SVG 1.1//EN",BAD_CAST "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd",NULL) < 0)
		throw std::runtime_error("Could not write SVG doctype");
	if(xmlTextWriterWriteString(writer, BAD_CAST "\n") < 0)
		throw std::runtime_error("Could not write newline");
		
	const AStarNode* root = res.priorities.size()==0 ? *res.closed.begin() : res.priorities.front();
	while(root->parent != NULL)
		root = root->parent;
	
	fmat::Column<2> min,max;
	
	if(view.sumSq()!=0) {
		min = fmat::SubVector<2,const fmat::fmatReal>(view);
		max = min + fmat::SubVector<2,const fmat::fmatReal>(view,2);
	} else {
		const fmat::Column<2> GOAL_MARGIN = fmat::pack(70,70);
		min=max=goal;
		expandBB(goal-GOAL_MARGIN,min,max);
		expandBB(goal+GOAL_MARGIN,min,max);
		expandBB(root->state.pos - GOAL_MARGIN,min,max);
		expandBB(root->state.pos + GOAL_MARGIN,min,max);
		
		fmat::Column<2> obsMin,obsMax;
		obstacleBounds(f,obsMin,obsMax);
		expandBB(obsMin,min,max);
		expandBB(obsMax,min,max);
		writeViewboxComment(writer,"Obstacle",obsMin,obsMax);
		
		if(flags & FIGURES_SHOW_WORK) {
			fmat::Column<2> workMin,workMax;
			workBounds(res,workMin,workMax);
			expandBB(workMin,min,max);
			expandBB(workMax,min,max);
			writeViewboxComment(writer,"Work",obsMin,obsMax);
		}
		if((flags & FIGURES_SHOW_PATH) && res.priorities.size()>0) {
			fmat::Column<2> pathMin,pathMax;
			pathBounds(*res.priorities.front(),pathMin,pathMax);
			expandBB(pathMin,min,max);
			expandBB(pathMax,min,max);
			writeViewboxComment(writer,"Path",pathMin,pathMax);
		}
		if((flags & (FIGURES_SHOW_STEPS|FIGURES_SHOW_SUPPORT)) && res.priorities.size()>0) {
			fmat::Column<2> feetMin,feetMax;
			feetBounds(*res.priorities.front(),feetMin,feetMax);
			expandBB(feetMin,min,max);
			expandBB(feetMax,min,max);
			writeViewboxComment(writer,"Feet",feetMin,feetMax);
		}
	
		writeViewboxComment(writer,"Total",min,max);
		
		if(flags & FIGURES_SHOW_GRID) {
			fmat::Column<2> VIEW_MARGIN=(max-min)*.05;
			VIEW_MARGIN[0] = std::max<fmat::fmatReal>(VIEW_MARGIN[0],GOAL_MARGIN[0]);
			VIEW_MARGIN[1] = std::max<fmat::fmatReal>(VIEW_MARGIN[1],GOAL_MARGIN[1]);
			min-=VIEW_MARGIN;
			max+=VIEW_MARGIN;
		}
	}
	
	std::cout << "Figure viewBox: " << fmat::pack(min,max-min).fmt("%g","","","") << std::endl;
	
	{ // limiting scope of 'svg' so it closes before end document is called
		Element svg(writer,"svg","http://www.w3.org/2000/svg");
		svg.attribute("viewBox",fmat::pack(min,max-min).fmt("%g","","",""));
		svg.attribute("version","1.1");
		svg.attribute("xmlns:xlink","http://www.w3.org/1999/xlink");
		xmlTextWriterSetIndentString(writer, BAD_CAST "\t");
		xmlTextWriterSetIndent(writer, 1);
		
		// if we can't reference an external style file, read it directly
		if(flags & FIGURES_EMBED_STYLE) {
			loadStyles(writer,"style.css");
		}
		
		{ // limiting scope of 'defs'
			Element defs(writer,"defs");
			
			const float NODE_RADIUS=8;
			Element(writer,"circle")
				.attribute("id","ClosedNode")
				.attribute("r",NODE_RADIUS);
			
			Element(writer,"circle")
				.attribute("id","LiveNode")
				.attribute("r",NODE_RADIUS);

			Element(writer,"circle")
				.attribute("id","OpenNode")
				.attribute("r",5);

			Element(writer,"circle")
				.attribute("id","PathNode")
				.attribute("r",8);

			Element(writer,"circle")
				.attribute("id","Footstep")
				.attribute("r",4);
		}
		
		if(flags & FIGURES_SHOW_GRID) {
			const fmat::fmatReal GRID_SPACING=100;
			fmat::Column<2,int> gridMin; gridMin.importFrom((min/GRID_SPACING-1).map(floor));
			fmat::Column<2,int> gridMax; gridMax.importFrom((max/GRID_SPACING+1).map(ceil));
			for(int x=gridMin[0]; x<=gridMax[0]; ++x) {
				Element l(writer,"line");
				l.attribute("class",(x==0)?"OriginLine":"GridLine");
				l.attribute("x1",x*GRID_SPACING);
				l.attribute("x2",x*GRID_SPACING);
				l.attribute("y1",gridMin[1]*GRID_SPACING);
				l.attribute("y2",gridMax[1]*GRID_SPACING);
			}
			for(int y=gridMin[1]; y<=gridMax[1]; ++y) {
				Element l(writer,"line");
				l.attribute("class",(y==0)?"OriginLine":"GridLine");
				l.attribute("x1",gridMin[0]*GRID_SPACING);
				l.attribute("x2",gridMax[0]*GRID_SPACING);
				l.attribute("y1",y*GRID_SPACING);
				l.attribute("y2",y*GRID_SPACING);
			}
		}
		
		writeObstacles(writer,f,"Obstacle1");
		writeObstacles(writer,f,"Obstacle2");
		
		const fmat::fmatReal R=12;
		Element(writer,"rect")
			.attribute("id","Start")
			.attribute("x",root->state.pos[0]-R)
			.attribute("y",root->state.pos[1]-R)
			.attribute("width",R*2)
			.attribute("height",R*2);
		Element(writer,"rect")
			.attribute("id","Goal")
			.attribute("x",goal[0]-R)
			.attribute("y",goal[1]-R)
			.attribute("width",R*2)
			.attribute("height",R*2);
		
		std::set<const AStarNode*> seen;
		std::ostringstream solPts;
		if(res.priorities.size()>0) {
			for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
				solPts << cur->state.pos[0] << ',' << cur->state.pos[1] << ' ';
				seen.insert(cur);
			}
		}

		if(flags & FIGURES_SHOW_WORK) {
			std::ostringstream livePts;
			for(AStarResults::NodeSet::const_iterator it=res.open.begin(); it!=res.open.end(); ++it) {
				if(seen.count(*it)>0) // only applies to goal state of solution path...
					continue;
				if(seen.count((*it)->parent)==0) {
					const AStarNode* cur = (*it)->parent;
					livePts << " M " << cur->state.pos[0] << ',' << cur->state.pos[1];
					seen.insert(cur);
					for(cur = cur->parent; cur!=NULL; cur=cur->parent) {
						livePts << ' ' << cur->state.pos[0] << ',' << cur->state.pos[1];
						if(seen.count(cur)>0)
							break;
						seen.insert(cur);
					}
				}
			}

			std::stringstream closedPts;
			for(AStarResults::NodeSet::const_iterator it=res.closed.begin(); it!=res.closed.end(); ++it) {
				if(seen.count(*it)>0) // only applies to goal state of solution path...
					continue;
				closedPts << " M";
				for(const AStarNode* cur = (*it); cur!=NULL; cur=cur->parent) {
					closedPts << ' ' << cur->state.pos[0] << ',' << cur->state.pos[1];
					if(seen.count(cur)>0)
						break;
					seen.insert(cur);
				}
			}

			Element(writer,"path").attribute("class","Closed").attribute("fill","none").attribute("d",closedPts.str());
			Element(writer,"path").attribute("class","Live").attribute("fill","none").attribute("d",livePts.str());
			for(AStarResults::NodeSet::const_iterator it=res.open.begin(); it!=res.open.end(); ++it) {
				if(seen.count(*it)>0)
					continue;
				writeNodeLine(writer,**it,"Work Open");
			}
			for(AStarResults::NodeSet::const_iterator it=res.open.begin(); it!=res.open.end(); ++it) {
				if(seen.count(*it)>0)
					continue;
				writeNode(writer,(*it)->state.pos,"OpenNode");
			}
		}
		
		if(res.priorities.size()>0) {
			// *** SOLUTION PATH *** //
			if(flags & FIGURES_SHOW_PATH) {
				Element(writer,"polyline").attribute("class","Path").attribute("fill","none").attribute("points",solPts.str());
				for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
					writeNode(writer,cur->state.pos,"PathNode");
				}
			}
			
			// *** SUPPORT REGIONS *** //
			if(flags & FIGURES_SHOW_SUPPORT) {
				std::stack<std::string> supportStrs;
				for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
					std::vector<fmat::Column<2> > support;
					for(unsigned int i=0; i<NumLegs; ++i) {
						if(f.groups[cur->state.phase].count(i) == 0) {
							support.push_back(cur->state.footPos[i]);
						}
					}
					ConvexPolyObstacle supportPoly;
					supportPoly.hull(std::set<fmat::Column<2> >(support.begin(),support.end()));
			
					std::ostringstream footPts;
					for(unsigned int i=0; i<supportPoly.getPoints().size(); ++i)
						footPts << supportPoly.getPoints()[i][0] << ',' << supportPoly.getPoints()[i][1] << ' ';
					supportStrs.push(footPts.str());
				}
				for(;!supportStrs.empty();supportStrs.pop())
					Element(writer,"polygon").attribute("class","Support").attribute("points",supportStrs.top());
			}
		
			// *** FOOT PATHS *** //
			if(flags & FIGURES_SHOW_STEPS) {
				for(unsigned int i=0; i<NumLegs; ++i) {
					std::ostringstream footPts;
					for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
						footPts << cur->state.footPos[i][0] << ',' << cur->state.footPos[i][1] << ' ';
					}
					Element(writer,"polyline").attribute("class","Foot").attribute("fill","none").attribute("points",footPts.str());
					for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
						writeNode(writer,cur->state.footPos[i],"Footstep");
					}
				}
			}
			// *** ORIENTATION INDICATORS *** //
			if(flags & FIGURES_SHOW_PATH) {
				Element(writer,"polyline").attribute("class","Path").attribute("fill","none").attribute("points",solPts.str());
				for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
					writeNode(writer,cur->state.pos,"PathNode");
				}
			}
		
			// *** ORIENTATION INDICATORS *** //
			if(figureOriLen>0) {
				for(const AStarNode* cur = res.priorities.front(); cur!=NULL; cur=cur->parent) {
					Element l(writer,"line");
					l.attribute("class","OrientationLine");
					l.attribute("x1",cur->state.pos[0]);
					l.attribute("x2",cur->state.pos[0]+figureOriLen*std::cos(cur->state.oriAngle));
					l.attribute("y1",cur->state.pos[1]);
					l.attribute("y2",cur->state.pos[1]+figureOriLen*std::sin(cur->state.oriAngle));
				}
			}
		}
	}
	
	if(xmlTextWriterEndDocument(writer) < 0)
		throw std::runtime_error("Could not end document");
	
	xmlFreeTextWriter(writer);
}

void expandBB(float x, float y, fmat::Column<2>& min, fmat::Column<2>& max) {
	if(x<min[0]) min[0]=x;
	if(x>max[0]) max[0]=x;
	if(y<min[1]) min[1]=y;
	if(y>max[1]) max[1]=y;
}

void expandBB(const fmat::Column<2>& p, fmat::Column<2>& min, fmat::Column<2>& max) {
	if(!std::isfinite(p[0]) || !std::isfinite(p[1]))
		return;
	expandBB(p[0],p[1],min,max);
}

void obstacleBounds(const GaitedFootsteps& f, fmat::Column<2>& min, fmat::Column<2>& max) {
	min = std::numeric_limits<fmat::fmatReal>::infinity();
	max = -min;
	for(size_t i=0; i<f.getObstacles().size(); ++i) {
		PlannerObstacle* ob = f.getObstacles()[i];
		if(const RectangularObstacle* r = dynamic_cast<const RectangularObstacle*>(ob)) {
			expandBB(r->getCenter()[0] - r->getWidth()/2, r->getCenter()[1] - r->getHeight()/2, min, max);
			expandBB(r->getCenter()[0] + r->getWidth()/2, r->getCenter()[1] + r->getHeight()/2, min, max);
		} else if(const CircularObstacle* c = dynamic_cast<const CircularObstacle*>(ob)) {
			expandBB(c->center[0] - c->radius, c->center[1] - c->radius, min, max);
			expandBB(c->center[0] + c->radius, c->center[1] + c->radius, min, max);
		} else {
			std::cerr << "WARNING unknown obstacle type " << ob->getTypeName() << std::endl;
		}
	}
}
void workBounds(const AStarResults& res, fmat::Column<2>& min, fmat::Column<2>& max) {
	min = std::numeric_limits<fmat::fmatReal>::infinity();
	max = -min;
	for(AStarResults::NodeSet::const_iterator it=res.open.begin(); it!=res.open.end(); ++it)
		expandBB((*it)->state.pos,min,max);
	for(AStarResults::NodeSet::const_iterator it=res.closed.begin(); it!=res.closed.end(); ++it)
		expandBB((*it)->state.pos,min,max);
}
void pathBounds(const AStarNode& node, fmat::Column<2>& min, fmat::Column<2>& max) {
	min = std::numeric_limits<fmat::fmatReal>::infinity();
	max = -min;
	const AStarNode* cur=&node;
	while(cur!=NULL) {
		expandBB(cur->state.pos,min,max);
		cur=cur->parent;
	}
}
void feetBounds(const AStarNode& node, fmat::Column<2>& min, fmat::Column<2>& max) {
	min = std::numeric_limits<fmat::fmatReal>::infinity();
	max = -min;
	const AStarNode* cur=&node;
	while(cur!=NULL) {
		for(size_t i=0; i<NumLegs; ++i)
			expandBB(cur->state.footPos[i],min,max);
		cur=cur->parent;
	}
}

#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
void loadStyles(xmlTextWriter* writer, const std::string& file) {
	struct stat st;
	if(stat(file.c_str(),&st)<0) {
		perror(("Could not find style file "+file).c_str());
		return;
	}
	int fd = open(file.c_str(),O_RDONLY);
	if(fd<0) {
		perror(("Could not open style file "+file).c_str());
		return;
	}
	const char * d = static_cast<const char*>(mmap(NULL,st.st_size,PROT_READ,MAP_SHARED,fd,0));
	if(d==MAP_FAILED) {
		perror(("Could not read style file "+file).c_str());
		close(fd);
		return;
	}
	try {
		Element style(writer,"style");
		if(xmlTextWriterStartCDATA(writer) < 0)
			throw std::runtime_error("Could not start style dump");
		if(xmlTextWriterWriteRawLen(writer,BAD_CAST "\n",1) < 0)
			throw std::runtime_error("Could not perform style dump");
		if(xmlTextWriterWriteRawLen(writer,BAD_CAST d,st.st_size) < 0)
			throw std::runtime_error("Could not perform style dump");
		if(xmlTextWriterEndCDATA(writer) < 0)
			throw std::runtime_error("Could not end style dump");
	} catch(...) {
		munmap((void*)d,st.st_size);
		close(fd);
		throw;
	}
	munmap((void*)d,st.st_size);
	close(fd);
}

void writeNodeLine(xmlTextWriter* writer, const AStarNode& node, const std::string& cl) {
	if(node.parent!=NULL) {
		Element e(writer,"line");
		if(cl.size()>0)
			e.attribute("class",cl);
		e.attribute("x1",node.parent->state.pos[0]);
		e.attribute("y1",node.parent->state.pos[1]);
		e.attribute("x2",node.state.pos[0]);
		e.attribute("y2",node.state.pos[1]);
	}
}

void writeNode(xmlTextWriter* writer, const fmat::Column<2>& pos, const std::string& marker) {
	Element e(writer,"use");
	e.attribute("xlink:href","#"+marker);
	e.attribute("x",pos[0]);
	e.attribute("y",pos[1]);
}

void writeObstacles(xmlTextWriter* writer, const GaitedFootsteps& f, const std::string& clName) {
	for(size_t i=0; i<f.getObstacles().size(); ++i) {
		PlannerObstacle* ob = f.getObstacles()[i];
		if(const RectangularObstacle* r = dynamic_cast<const RectangularObstacle*>(ob)) {
			Element el(writer,"rect");
			el.attribute("class",clName);
			el.attribute("x",r->getCenter()[0] - r->getWidth()/2);
			el.attribute("y",r->getCenter()[1] - r->getHeight()/2);
			el.attribute("width",r->getWidth());
			el.attribute("height",r->getHeight());
			if(r->getOrientation()!=0)
				el.attribute("transform",fmat::pack(r->getOrientation(),r->getCenter()).fmt("%g","rotate(","",")"));

		} else if(const CircularObstacle* c = dynamic_cast<const CircularObstacle*>(ob)) {
			Element el(writer,"circle");
			el.attribute("class",clName);
			el.attribute("cx",c->center[0]);
			el.attribute("cy",c->center[1]);
			el.attribute("r",c->radius);

		} else {
			std::cerr << "WARNING unknown obstacle type " << ob->getTypeName() << std::endl;
		}
	}
}
