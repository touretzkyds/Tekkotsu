#include "Kinematics.h"
#include "Shared/Config.h"
#include "Shared/WorldState.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/string_util.h"
#include <sstream>
#include <iostream>
#include <limits>
#include <cmath>

#ifdef PLATFORM_APERIOS
//! this is normally defined in <math.h>, but the OPEN-R cross-compiler isn't configured right
template<typename T> static inline int signbit(T x) { return x<0; }
#endif

using namespace std;

Kinematics * kine = NULL;
Kinematics::InterestPointMap Kinematics::ips;
bool Kinematics::staticsInited=false;

void
Kinematics::init() {
	checkStatics();
	for(unsigned int i=0; i<NumReferenceFrames; ++i)
		jointMaps[i]=NULL;
	if(!root.loadFile(config->makePath(config->motion.kinematics).c_str())) {
		std::cerr << "Could not load " << config->motion.makePath(config->motion.kinematics) << ", forward and inverse kinematics unavailable." << std::endl;
		return;
	}
	root.buildChildMap(jointMaps,0,NumReferenceFrames);
}

void Kinematics::initStatics() {
	if(!staticsInited) {
		staticsInited=true;
	}
}

Kinematics::~Kinematics() {
}

fmat::Transform
Kinematics::baseToLocal(const PlaneEquation& plane) {
	std::cout << "plane " << plane.getDirection() << ' ' << plane.getDisplacement() << std::endl;
	fmat::Column<3> p = plane.getDirection() / plane.getDirection().norm();
	float s = hypotf(p[0],p[1]); // |cross(p,[0 0 -1])| = sin(an)
	if(std::abs(s)<std::numeric_limits<float>::epsilon()*100) 
		return fmat::Transform::IDENTITY;
	
	// cross(p,[0 0 -1]) : 
	float x = -p[1]/s;
	float y = p[0]/s;
	// float z=0
	
	float d = -plane.getDisplacement();
	
	float c = -p[2]; // dot(p,[0 0 -1]) = cos(an) = p[2]
	float t = 1 - c;
	//std::cout << "axis " << x << ' ' << y << " angle " << std::asin(s) << ' ' << std::acos(c) << std::endl;
	float v[fmat::Transform::CAP] = { t*x*x + c, t*x*y, -y*s, t*x*y, t*y*y + c, x*s, y*s, -x*s, c, p[0]*d, p[1]*d, p[2]*d };
	//std::cout << fmat::Transform(v) << std::endl;
	return fmat::Transform(v);
}

void 
Kinematics::getInterestPoint(const std::string& name, unsigned int& link, fmat::Column<3>& ip, bool convertToJoint/*=false*/) {
	link=-1U;
	ip=fmat::Column<3>();
	for(unsigned int i=0; i<NumReferenceFrames; ++i) {
		if(strcmp(outputNames[i],name.c_str())==0) {
			link = i;
			ip=pack(0,0,0);
		}
	}
	if(convertToJoint && NumOutputs<=link && link<NumReferenceFrames) {
		KinematicJoint * kj = jointMaps[link];
		while(kj!=NULL) {
			if(kj->outputOffset < NumOutputs) {
				ip = jointMaps[link]->getT(*kj) * ip;
				link = kj->outputOffset;
				return;
			}
		}
	}
}

fmat::Column<3>
Kinematics::getInterestPoint(unsigned int link, const std::string& name) {
	vector<string> ipNames = string_util::tokenize(name,",");
	fmat::Column<3> ans = pack(0,0,0);
	unsigned int cnt=0;
	for(vector<string>::const_iterator it=ipNames.begin(); it!=ipNames.end(); ++it) {
		string srcName = string_util::trim(*it);
		if(srcName.size()==0)
			continue;
		unsigned int srcLink=-1U;
		fmat::Column<3> srcPos;
		getInterestPoint(srcName,srcLink,srcPos);
		if(srcLink==-1U)
			throw std::runtime_error(std::string("Kinematics::getLinkInterestPoint: interest point unknown ").append(name));
		if(link!=srcLink)
			srcPos = linkToLink(srcLink,link)*srcPos;
		ans+=srcPos;
		++cnt;
	}
	if(cnt==0)
		throw std::runtime_error(std::string("Kinematics::getLinkInterestPoint called with empty string"));
	return ans/cnt;
}


#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_MANTIS) 
void
Kinematics::calcLegHeights(const fmat::Column<3>& down, float heights[NumLegs]) {
	update();
	//initialize to the height of the ball of the foot
	for(unsigned int i=0; i<NumLegs; i++) {
		if(jointMaps[FootFrameOffset+i]==NULL) {
			heights[i]=std::numeric_limits<typeof(heights[0])>::max();
		} else {
			fmat::Column<3> ip_b=jointMaps[FootFrameOffset+i]->getWorldPosition();
			float h = -fmat::dotProduct(ip_b,down);
			h-=BallOfFootRadius; //add the ball's radius
			heights[i]=h;
		}
	}
	//see if any interest points are lower
	for(InterestPointMap::const_iterator it=ips.begin(); it!=ips.end(); ++it) {
		unsigned int leg;
		if(jointMaps[it->second.output]==NULL)
			continue;
		else if(it->second.output>=LegOffset || it->second.output<LegOffset+NumLegJoints)
			leg = (it->second.output - LegOffset) / JointsPerLeg;
		else if(it->second.output>=FootFrameOffset || it->second.output<FootFrameOffset+NumLegs)
			leg = (it->second.output - FootFrameOffset);
		else
			continue;
		fmat::Column<3> ip_b=jointMaps[it->second.output]->getFullT() * it->second.p;
		float h = -fmat::dotProduct(ip_b,down);
		if(h<heights[leg])
			heights[leg]=h;
	}
}

LegOrder_t
Kinematics::findUnusedLeg(const fmat::Column<3>& down) {
	float heights[NumLegs];
	calcLegHeights(down,heights);
	//Find the highest foot
	unsigned int highleg=0;
	for(unsigned int i=1; i<NumLegs; i++)
		if(heights[i]>heights[highleg])
			highleg=i;
	//cout << "High: " << highleg << endl;
	return static_cast<LegOrder_t>(highleg);
}

PlaneEquation Kinematics::calculateGroundPlane() {
#ifdef TGT_HAS_ACCELEROMETERS
	fmat::Column<3> down=fmat::pack(state->sensors[BAccelOffset],-state->sensors[LAccelOffset],state->sensors[DAccelOffset]);
	if(down.sumSq()<0.01)
		down=fmat::pack(0,0,-1); //default to a down vector if sensors don't give a significant indication of gravity
	return calculateGroundPlane(down);
#else
	return calculateGroundPlane(fmat::pack(0,0,-1));
#endif
}

// TODO comment out name tracking (and this following #include) once we're sure it's working
#include "PostureEngine.h"
PlaneEquation
Kinematics::calculateGroundPlane(const fmat::Column<3>& down) {
	update();
	fmat::Matrix<3,3> lowip; //3 points define a plane
	float heights[3];
	unsigned int legs[3];
	std::string names[3];
	//initialize to max float
	for(unsigned int i=0; i<3; i++) {
		heights[i]=std::numeric_limits<typeof(heights[0])>::max();;
		legs[i]=-1U;
	}
	//Check the balls of the foot
	for(unsigned int i=0; i<NumLegs; i++) {
		if(jointMaps[FootFrameOffset+i]==NULL)
			continue;
		fmat::Column<3> ip_b=jointMaps[FootFrameOffset+i]->getWorldPosition();
		float h = -fmat::dotProduct(ip_b,down);
		h-=BallOfFootRadius; //add the ball's radius
		if(h<heights[0]) {
			if(h<heights[1]) {
				heights[0]=heights[1];
				lowip.column(0)=lowip.column(1);
				legs[0]=legs[1];
				names[0]=names[1];
				if(h<heights[2]) {
					heights[1]=heights[2];
					lowip.column(1)=lowip.column(2);
					legs[1]=legs[2];
					names[1]=names[2];
					
					heights[2]=h;
					lowip.column(2)=ip_b;
					legs[2]=i;
					names[2]="paw"; names[2]+=(char)('0'+i);
				} else {
					heights[1]=h;
					lowip.column(1)=ip_b;
					legs[1]=i;
					names[1]="paw"; names[1]+=(char)('0'+i);
				}
			} else {
				heights[0]=h;
				lowip.column(0)=ip_b;
				legs[0]=i;
				names[0]="paw"; names[0]+=(char)('0'+i);
			}
		}
	}
	//cout << "Ground plane initial: " << names[0] <<" ("<<heights[0]<<") " << names[1] << " ("<<heights[1]<<") " << names[2] << " ("<<heights[2]<<")"<< endl;
	
	//now check interest points
	for(InterestPointMap::const_iterator it=ips.begin(); it!=ips.end(); ++it) {
#ifdef TGT_IS_AIBO
		if(it->first.substr(0,3)=="Toe")
			continue;
#endif
		if(jointMaps[it->second.output]==NULL)
			continue;
		fmat::Column<3> ip_b=jointMaps[it->second.output]->getFullT() * it->second.p;
		float h = -fmat::dotProduct(ip_b,down);
		if(h<heights[0]) {
			unsigned int leg;
			if(it->second.output>=LegOffset && it->second.output<LegOffset+NumLegJoints)
				leg = (it->second.output - LegOffset) / JointsPerLeg;
			else if(it->second.output>=FootFrameOffset && it->second.output<FootFrameOffset+NumLegs)
				leg = (it->second.output - FootFrameOffset);
			else
				leg=-1U;
			
			if(h<heights[1]) {
				if(h<heights[2]) {
					if(leg==-1U || (legs[1]!=leg && legs[2]!=leg)) {
						heights[0]=heights[1];
						lowip.column(0)=lowip.column(1);
						legs[0]=legs[1];
						names[0]=names[1];
					}
					if(leg==-1U || legs[2]!=leg) {
						heights[1]=heights[2];
						lowip.column(1)=lowip.column(2);
						if(legs[2]!=leg)
							legs[1]=legs[2];
						names[1]=names[2];
					}
					
					heights[2]=h;
					lowip.column(2)=ip_b;
					legs[2]=leg;
					names[2]=(*it).first;
				} else {
					if(leg!=-1U && legs[2]==leg)
						continue;
					if(leg==-1U || legs[1]!=leg) {
						heights[0]=heights[1];
						lowip.column(0)=lowip.column(1);
						legs[0]=legs[1];
						names[0]=names[1];
					}
					heights[1]=h;
					lowip.column(1)=ip_b;
					legs[1]=leg;
					names[1]=(*it).first;
				}
			} else {
				if(leg!=-1U && (legs[1]==leg || legs[2]==leg))
					continue;
				heights[0]=h;
				lowip.column(0)=ip_b;
				legs[0]=leg;
				names[0]=(*it).first;
			}
		}
	}
	
	//Fit a plane to the remaining 3 feet
	fmat::Column<3> ones(1.f);
	fmat::Column<3> dir;
	try {
		dir = invert(lowip.transpose())*ones;
	} catch(...) {
		std::cout << "Exception during ground plane processing, saving current posture..." << std::flush;
		if(PostureEngine * tpose=dynamic_cast<PostureEngine*>(this)) {
			tpose->setSaveFormat(true,state);
			tpose->saveFile("/error.pos");
		} else {
			PostureEngine pose;
			pose.takeSnapshot();
			pose.setWeights(1);
			pose.setSaveFormat(true,state);
			pose.saveFile("/error.pos");
		}
		std::cout << "Wrote current sensor info \"error.pos\" on memstick" << std::endl;
		cout << "Ground plane was using " << names[0] <<" ("<<heights[0]<<") " << names[1] << " ("<<heights[1]<<") " << names[2] << " ("<<heights[2]<<")"<< endl;
		cout << lowip;
		throw;
	}
	// todo this doesn't handle plane through the origin, what if plane is z=0 (e.g. [0,0,1,0])
	return PlaneEquation(dir, 1);
}

#elif defined(TGT_IS_MANTIS)
// If the model is Mantis

PlaneEquation Kinematics::calculateGroundPlane() {
#ifdef TGT_HAS_ACCELEROMETERS
        fmat::Column<3> down=fmat::pack(state->sensors[BAccelOffset],-state->sensors[LAccelOffset],state->sensors[DAccelOffset]);
        if(down.sumSq()<0.01)
                down=fmat::pack(0,0,-1); //default to a down vector if sensors don't give a significant indication of gravity
        return calculateGroundPlane(down);
#else
        return calculateGroundPlane(fmat::pack(0,0,-1));
#endif
}

#include "PostureEngine.h"
#include <utility>
PlaneEquation
Kinematics::calculateGroundPlane(const fmat::Column<3>& down) {
        update();
        fmat::Matrix<3,3> lowip; //3 points define a plane
        float heights[3];float arr[6];
        unsigned int idx[NumLegs];
        for (unsigned int i=0; i<NumLegs; i++) {  idx[i] = i; }
        unsigned int legs[3];
        std::string names[3];
        for(unsigned int i=0; i<NumLegs; i++) {
                if(jointMaps[FootFrameOffset+i]==NULL)
                        continue;
                fmat::Column<3> ip_b=jointMaps[FootFrameOffset+i]->getWorldPosition();
                float h = -fmat::dotProduct(ip_b,down);
                h-=BallOfFootRadius; //add the ball's radius
                arr[i]=h;
                //cout<<"arr["<<i<<"] = "<<arr[i]<<endl<<ip_b << endl;
        }
        for(unsigned int i=0; i<NumLegs; i++) {
                 for(unsigned int j=i+1; j<NumLegs; j++) {
                        if (arr[idx[i]] > arr[idx[j]])
                        {
                           std::swap(idx[i], idx[j]);
                        }
               }
        }
        for(unsigned int i=0; i<3; i++) {
                legs[i] = idx[i];
                heights[i] = arr[idx[i]];
                lowip.column(i) = jointMaps[FootFrameOffset+idx[i]]->getWorldPosition();
                names[i]="paw"; names[i]+=(char)('0'+idx[i]);
                //cout<<"heights["<<i<<"] = "<<heights[i]<< endl << lowip.column(i);
        }


        //cout << "Ground plane initial: " << names[0] <<" ("<<heights[0]<<") " << names[1] << " ("<<heights[1]<<") " << names[2] << " ("<<heights[2]<<")"<< endl;

//now check interest points
        for(InterestPointMap::const_iterator it=ips.begin(); it!=ips.end(); ++it) {
                if(jointMaps[it->second.output]==NULL)
                        continue;
                fmat::Column<3> ip_b=jointMaps[it->second.output]->getFullT() * it->second.p;
                float h = -fmat::dotProduct(ip_b,down);
                if(h<heights[0]) {
                        unsigned int leg;
                        if(it->second.output>=LegOffset && it->second.output<LegOffset+NumFrLegJoints)
                                leg = (it->second.output - LegOffset) / JointsPerFrLeg;
                        else if(it->second.output>=LMdLegOffset && it->second.output<LMdLegOffset+NumPosLegJoints)
                                leg = (it->second.output - LMdLegOffset) / JointsPerPosLeg;
                        else if(it->second.output>=FootFrameOffset && it->second.output<FootFrameOffset+NumLegs)
                                leg = (it->second.output - FootFrameOffset);
                        else
                                leg=-1U;

                        if(h<heights[1]) {
                                if(h<heights[2]) {
                                        if(leg==-1U || (legs[1]!=leg && legs[2]!=leg)) {
                                                heights[0]=heights[1];
                                                lowip.column(0)=lowip.column(1);
                                                legs[0]=legs[1];
                                                names[0]=names[1];
                                        }
                                        if(leg==-1U || legs[2]!=leg) {
                                                heights[1]=heights[2];
                                                lowip.column(1)=lowip.column(2);
                                                if(legs[2]!=leg)
                                                        legs[1]=legs[2];
                                                names[1]=names[2];
                                        }

                                        heights[2]=h;
                                        lowip.column(2)=ip_b;
                                        legs[2]=leg;
                                        names[2]=(*it).first;
                                } else {
                                        if(leg!=-1U && legs[2]==leg)
                                                continue;
                                        if(leg==-1U || legs[1]!=leg) {
                                                heights[0]=heights[1];
                                                lowip.column(0)=lowip.column(1);
                                                legs[0]=legs[1];
                                                names[0]=names[1];
                                        }
                                        heights[1]=h;
                                        lowip.column(1)=ip_b;
                                        legs[1]=leg;
                                        names[1]=(*it).first;
                                }
                        } else {
                                if(leg!=-1U && (legs[1]==leg || legs[2]==leg))
                                        continue;
                                heights[0]=h;
                                lowip.column(0)=ip_b;
                                legs[0]=leg;
                                names[0]=(*it).first;
                        }
                }
        }

        //Fit a plane to the remaining 3 feet
        fmat::Column<3> ones(1.f);
        fmat::Column<3> dir;
        try {
                dir = invert(lowip.transpose())*ones;
        } catch(...) {
                std::cout << "Exception during ground plane processing, saving current posture..." << std::flush;
                if(PostureEngine * tpose=dynamic_cast<PostureEngine*>(this)) {
                        tpose->setSaveFormat(true,state);
                        tpose->saveFile("/error.pos");
                } else {
                        PostureEngine pose;
                        pose.takeSnapshot();
                        pose.setWeights(1);
                        pose.setSaveFormat(true,state);
                        pose.saveFile("/error.pos");
                }
                std::cout << "Wrote current sensor info \"error.pos\" on memstick" << std::endl;
                cout << "Ground plane was using " << names[0] <<" ("<<heights[0]<<") " << names[1] << " ("<<heights[1]<<") " << names[2] << " ("<<heights[2]<<")"<< endl;
                cout << lowip;
                throw;
        }
        // todo this doesn't handle plane through the origin, what if plane is z=0 (e.g. [0,0,1,0])
        return PlaneEquation(dir, 1);
}

#else // NO LEGS -- constant ground plane

PlaneEquation Kinematics::calculateGroundPlane() {
	return PlaneEquation(0,0,1,0);
}

PlaneEquation
Kinematics::calculateGroundPlane(const fmat::Column<3>& down) {
	return PlaneEquation(-down, 0); // flat x-y plane through the origin with z pointing up
}

#endif

fmat::Column<4> 
Kinematics::projectToPlane(
			unsigned int j, const fmat::Column<3>& r_j,
			unsigned int b, const PlaneEquation& p_b,
			unsigned int f,
			float objCentroidHeight)
{
	update();
	if((j!=b && (jointMaps[j]==NULL || jointMaps[b]==NULL)) || (f!=b && (jointMaps[f]==NULL || jointMaps[b]==NULL)) )
		return fmat::Column<4>(0.f);
	
	/*! Mathematical implementation:
	 *  
	 *  We'll convert the ray to the plane's reference frame, solve there.
	 *  We find a point on the ray (ro_b) and the direction of the ray (rv_b).
	 *  rv_b does not need to be normalized because we're going to find
	 *  a scaling factor for it, and that factor accounts for current magnitude.
	 *
	 *  Proof, p=plane normal vector, d=plane displacement, r = ray direction, o = ray offset, x = [x y z] coordinates, t = scaling factor */
	 
	/* Non-latex/doxygen version for easier reading in source (UTF-8):
	 p⃑ · x⃑ = d    (definition of plane)
	 x⃑ = r⃑(t) + o⃑    (definition of ray through point)
	 p⃑ · ( r⃑(t)  + o⃑ ) = d
	 p⃑ · r⃑(t) + p⃑ · o⃑ = d
	 t ( p⃑ · r⃑ ) = d - p⃑ · o⃑
	 t = (d - p⃑ · o⃑) / (p⃑ · r⃑)
	 t = dist / align
	 substitute back to find intersection: x⃑ = r⃑(t) + o⃑
	 */
	 
	 /*! @f{eqnarray*} 
	 \vec p \cdot\vec x &=& d \qquad{\rm(Definition\ of\ plane)}\\
	 \vec x &=& t \vec r + \vec o \qquad{\rm(Definition\ of\ ray\ through\ point)}\\
	 \vec p \cdot ( t \vec r  + \vec o ) &=& d \\
	 \vec p \cdot t \vec r + \vec p \cdot \vec o &=& d \\
	 t ( \vec p \cdot \vec r ) &=& d - \vec p \cdot \vec o \\
	 t &=& \frac{d - \vec p \cdot \vec o}{\vec p \cdot \vec r} = \frac{dist}{align} \\
	 && {\rm (substitute\ back\ to\ find\ } \vec x = t \vec r + \vec o{\rm )}
	 @f}
	 */
	fmat::Column<3> ro_b, rv_b;
	if(j==b)
		rv_b=r_j;
	else {
		fmat::Transform tr = jointMaps[j]->getT(*jointMaps[b]);
		ro_b = tr.translation();
		rv_b = tr.rotation() * r_j;
	}
	//std::cout << "ro_b: " << ro_b << "  rv_b: " << rv_b << std::endl;
	
	/*! Find distance from the ray offset (ro_b) and the closest point on the plane. */
	float dist = p_b.getDisplacement() - fmat::dotProduct(p_b.getDirection(),ro_b);
	/*! Object height is applied along the plane normal toward the ray origin
	 *  (we assume the ray source is "above" ground) */
	dist += signbit(dist) ? objCentroidHeight : -objCentroidHeight;
	/*! Find scaling factor by projecting ray vector (rv_b) onto plane normal. */
	float align = fmat::dotProduct(p_b.getDirection(),rv_b);
	//std::cout << "dist " << dist << "  align " << align << std::endl;
	
	/*! Intersection point will be rv_b*dist/align + ro_b, but need to watch out
	 *  for case where align==0 (rv_b and plane are parallel, no intersection) */
	fmat::Column<4> hit;
	if(std::abs(align)>numeric_limits<float>::epsilon())
		hit = fmat::pack(rv_b*(dist/align)+ro_b,1.f);
	else if(align!=0 && dist!=0 && signbit(align)!=signbit(dist))
		hit = fmat::pack(-rv_b,std::abs(align));
	else
		hit = fmat::pack(rv_b,std::abs(align));
	//std::cout << "hit " << hit << std::endl;
	
	return (f==b) ? hit : jointMaps[b]->getT(*jointMaps[f]) * hit;
}

fmat::Column<4>
Kinematics::projectToGround(const VisionObjectEvent& visObj, const PlaneEquation& gndPlane, float objCentroidHeight) {
#ifndef TGT_HAS_CAMERA
	return fmat::Column<4>();
#else
	fmat::Column<3> imgRay_c;
	config->vision.computeRay(visObj.getCenterX(),visObj.getCenterY(), imgRay_c[0],imgRay_c[1],imgRay_c[2]);
	return projectToPlane(CameraFrameOffset, imgRay_c, BaseFrameOffset, calculateGroundPlane(), BaseFrameOffset, objCentroidHeight);
#endif
}


void
Kinematics::update() const {
	if(lastUpdateTime == state->lastSensorUpdateTime)
		 return;
	for(unsigned int j=0; j<NumOutputs; j++) {
		if(jointMaps[j]!=NULL)
			jointMaps[j]->setQ(state->outputs[j]);
	}
	lastUpdateTime = state->lastSensorUpdateTime;
}


/*! @file
 * @brief 
 * @author ejt (Creator)
 */

