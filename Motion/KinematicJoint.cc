#include "KinematicJoint.h"
#include "IKSolver.h"
#include "Shared/RobotInfo.h"
#include <cmath>
#include <limits>

using namespace std;

const char* KinematicJoint::jointTypeNames[3] = { "revolute", "prismatic", NULL };
INSTANTIATE_NAMEDENUMERATION_STATICS(KinematicJoint::JointType_t);

PLIST_CLONE_IMP(LinkComponent,new LinkComponent(*this));

//! for each node, if it's an array, start loading the subtree via recursive instantiation, otherwise load the KinematicJoint
bool KinematicJointLoader::loadXMLNode(size_t index, xmlNode* val, const std::string& comment) {
	if(xNodeHasName(val,"array")) {
		if(index==0) {
			xmlChar* path=xGetNodePath(val);
			std::cerr << "WARNING: loading kinematic file " << xNodeGetURL(val) << ":" << xmlGetLineNo(val) << " (" << path << "), encountered branch without prior link" << std::endl;
			xmlFree(path);
		}
		if(skipToElement(xNodeGetChildren(val))!=NULL) { // make sure there's something to load...
			KinematicJoint * kj = new KinematicJoint;
			KinematicJointLoader(*kj,val);
			parent->addBranch(kj);
		} else {
			xmlChar* path=xGetNodePath(val);
			std::cerr << "WARNING: loading kinematic file " << xNodeGetURL(val) << ":" << xmlGetLineNo(val) << " (" << path << "), skipping empty array" << std::endl;
			xmlFree(path);
		}
	} else {
		if(!plist::ArrayOf<KinematicJoint>::loadXMLNode(index==0 ? 0 : size(),val,comment))
			return false;
		if(index>0) {
			parent->addBranch(&back());
			parent = &back();
			myRef.clear();
		}
	}
	return true;
}



//! saves the array of joints, prepending human-readable help text as a comment if this is a root XML node
void KinematicJointSaver::saveXML(xmlNode* node) const {
	if(node==NULL)
		return;
	if(xNodeGetChildren(node)==NULL && (xNodeGetParent(node)==NULL || xNodeGetParent(xNodeGetParent(node))==NULL || xNodeHasName(xNodeGetParent(node),"plist"))) {
		std::string indentStr=getIndentationPrefix(node);
		std::string jointTypes;
		const char ** jtName = KinematicJoint::jointTypeNames;
		while(*jtName!=NULL && (*jtName)[0]!='\0') {
			if(jointTypes.size()>0)
				jointTypes+=" | ";
			jointTypes+=*jtName++;
		}
		
		std::string headerComment="\n"
		"##################################################################\n"
		"##################   Kinematics Configuration   ##################\n"
		"##################################################################\n"
		"\n"
		"This is an XML-based format using the Property List (plist) layout.\n"
		"\n"
		"Each joint is defined by a <dict> element with the keys listed below.\n"
		"A branch in the chain is denoted by an <array> containing the\n"
		"joints of the sub-chain.\n"
		"\n"
		"  JointType: Indicates the type of motion produced by the joint\n"
		"    One of: "+jointTypes+"\n"
		"\n"
		"  Denavit-Hartenberg parameters: (here in order of application)\n"
		"    d: Displacement along the previous joint's z axis\n"
		"    θ: Rotation about the previous joint's z axis (theta, U+03B8)\n"
		"    r: Displacement from prev. z along the current joint's x axis\n"
		"    α: Rotation about the current joint's x axis (alpha, U+03B1)\n"
		"  In other words, θ and d align the previous joint's x axis with\n"
		"  this joint's x axis, and then a displacement of r (radius of\n"
		"  rotation) along this x defines the current joint's origin.\n"
		"  α then defines the current joint's z axis (the axis of actuation).\n"
		"\n"
		"  qOffset: An additional parameter which shifts the final reference\n"
		"    frame to the physical joint's 0 position.  This is a rotation\n"
		"    about the joint's z axis for revolute joints, or translation\n"
		"    along z for prismatic.\n"
		"  Min: The minimum acceptable joint value for inverse kinematics\n"
		"  Max: The maximum acceptable joint value for inverse kinematics\n"
		"    Inverse kinematics ignores this joint if Min==Max (immobile).\n"
		"\n"
		"  Model: for 3D graphics, the name of the OGRE mesh file to render\n"
		"    for the link following the joint. (drop the \".mesh\" suffix)\n"
		"  Material: for 3D graphics, the name of the material to apply to\n"
		"    the model, or blank to use the model's defaults\n"
		"\n"
		"All distances are in millimeters.  Angles are radians, unless a\n"
		"'unit' attribute is specified, or a '°' is suffixed.  You can\n"
		"also specify radians as multiples of Pi, e.g. 'π/2'.\n";
		xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
		xmlAddChild(node,xmlNewComment((const xmlChar*)headerComment.c_str()));
		xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+indentStr).c_str()));
	}
	plist::Array::saveXML(node);
}

void KinematicJointSaver::init(const std::set<KinematicJoint*>& joints, xmlNode* node) {
	const std::set<KinematicJoint*>* branches = &joints;
	while(branches->size()>0) {
		if(branches->size()==1) {
			addEntry(**branches->begin());
			branches = &(*branches->begin())->branches;
		} else {
			for(std::set<KinematicJoint*>::const_iterator it=branches->begin(); it!=branches->end(); ++it)
				addEntry(new KinematicJointSaver(**it));
			break;
		}
	}
	if(node!=NULL)
		saveXML(node);
}

KinematicJoint::~KinematicJoint() {
	removeSelfListener();
	clearBranches();
	delete branchListeners;
	branchListeners=NULL;
	delete ik;
	ik=NULL;
}

void KinematicJoint::addBranchListener(BranchListener* l) const {
	if(l!=NULL) {
		if(branchListeners==NULL)
			branchListeners=new std::set<BranchListener*>;
		branchListeners->insert(l);
	}
}

void KinematicJoint::removeBranchListener(BranchListener* l) const {
	if(branchListeners==NULL)
		return;
	std::set<BranchListener*>::iterator it=branchListeners->find(l);
	if(it!=branchListeners->end()) {
		branchListeners->erase(it);
		if(branchListeners->empty()) {
			delete branchListeners;
			branchListeners=NULL;
		}
	}
}

bool KinematicJoint::isBranchListener(BranchListener* l) const {
	if(l==NULL)
		return false;
	if(branchListeners==NULL)
		return false;
	return ( branchListeners->count(l) > 0 );
}

IKSolver& KinematicJoint::getIK() const {
	if(ik!=NULL)
		return *ik;
	ik = IKSolver::getRegistry().create(ikSolver);
	if(ik==NULL) {
		ik = IKSolver::getRegistry().create("");
		if(ik==NULL)
			throw std::runtime_error("Could not create any IKSolvers");
		std::cerr << "Warning: could not create any IKSolver '" << ikSolver << "' for " << outputOffset.get() << ", falling back on default generic solver" << std::endl;
	}
	ikSolver.addPrimitiveListener(const_cast<KinematicJoint*>(this));
	return *ik;
}

PLIST_CLONE_IMP(KinematicJoint,new KinematicJoint(*this));

void KinematicJoint::fireBranchAdded(KinematicJoint& val) {
	if(branchListeners==NULL)
		return;
	// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
	std::set<BranchListener*> pls=*branchListeners;
	for(std::set<BranchListener*>::const_iterator it=pls.begin(); branchListeners!=NULL && it!=pls.end(); ++it) {
		// make sure current listener hasn't been removed
		if(branchListeners->count(*it)>0)
			(*it)->kinematicJointBranchAdded(*this,val);
	}
}

void KinematicJoint::fireBranchRemoved(KinematicJoint& val) {
	if(branchListeners==NULL)
		return;
	// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
	std::set<BranchListener*> pls=*branchListeners;
	for(std::set<BranchListener*>::const_iterator it=pls.begin(); branchListeners!=NULL && it!=pls.end(); ++it) {
		// make sure current listener hasn't been removed
		if(branchListeners->count(*it)>0)
			(*it)->kinematicJointBranchRemoved(*this,val);
	}
}

void KinematicJoint::fireReconfigured() {
	if(branchListeners==NULL)
		return;
	// copy primitive listeners so we aren't screwed if a listener is removed during processing, particularly if it's the *last* listener
	std::set<BranchListener*> pls=*branchListeners;
	for(std::set<BranchListener*>::const_iterator it=pls.begin(); branchListeners!=NULL && it!=pls.end(); ++it) {
		// make sure current listener hasn't been removed
		if(branchListeners->count(*it)>0)
			(*it)->kinematicJointReconfigured(*this);
	}
}

/*! returning false here can cause IK to invert solutions... for better stability,
    only return false if a joint limit is "significantly" exceeded. */
bool KinematicJoint::tryQ(float x) {
	const float EPSILON = numeric_limits<float>::epsilon()*100;
	if(x>qmax) {
		if(q!=qmax) {
			q=qmax;
			updateTq();
		}
		return (x-EPSILON < qmax);
	} else if(x<qmin) {
		if(q!=qmin) {
			q=qmin;
			updateTq();
		}
		return (x+EPSILON > qmin);
	} else {
		if(x!=q) {
			q=x;
			updateTq();
		}
		return true;
	}
}

void KinematicJoint::zeroChildrenQ() {
	if(qmin!=qmax)
		setQ(0);
	for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
		(*it)->zeroChildrenQ();
}

void KinematicJoint::zeroAncestorQ() {
	for(KinematicJoint* p = this; p!=NULL; p=p->getParent())
		if(p->qmin!=p->qmax)
			p->setQ(0);
}


fmat::Transform KinematicJoint::getFullT() const {
	fmat::Transform t;
	getFullT(t,NULL);
	return t;
}

fmat::Transform KinematicJoint::getT(const KinematicJoint& j) const {
	if(&j==this)
		return fmat::Transform::identity();
	fmat::Transform t;
	const KinematicJoint* myAnc=parent;
	const KinematicJoint* jAnc=j.parent;
	while(myAnc!=NULL || jAnc!=NULL) {
		if(myAnc!=NULL) {
			if(myAnc==&j) {
				getFullT(t,&j);
				return t;
			}
			myAnc=myAnc->parent;
		}
		if(jAnc!=NULL) {
			if(jAnc==this) {
				j.getFullT(t,this);
				return t.rigidInverse();
			}
			jAnc=jAnc->parent;
		}
	}
	getFullT(t,NULL);
	return j.getFullInvT() * t;
}

void KinematicJoint::sumCenterOfMass(fmat::Column<3>& cOfM, float& totalMass) const {
	fmat::Column<4> com = sumCenterOfMass();
	cOfM = fmat::SubVector<3>(com);
	if(com[3]>std::numeric_limits<fmat::fmatReal>::epsilon())
		cOfM/=com[3];
	totalMass=com[3];
}

fmat::Column<4> KinematicJoint::sumCenterOfMass() const {	
	fmat::Column<4> com = sumLinkCenterOfMass();
	for(branch_iterator it=branches.begin(); it!=branches.end(); ++it)
		com += (*it)->getTq() * (*it)->sumCenterOfMass();
	return com;
}

void LinkComponent::sumLinkCenterOfMass(fmat::Column<3>& cOfM, float& totalMass) const {
	fmat::Column<4> com = sumLinkCenterOfMass();
	cOfM = fmat::SubVector<3>(com);
	if(com[3]>std::numeric_limits<fmat::fmatReal>::epsilon())
		cOfM/=com[3];
	totalMass=com[3];
}

fmat::Column<4> KinematicJoint::sumLinkCenterOfMass() const {
	fmat::Column<4> com = getMassVector();
	for(component_iterator it=components.begin(); it!=components.end(); ++it)
		com += (*it)->getMassVector();
	return com;
}

bool KinematicJoint::hasMass() const {
	if(mass > 0)
		return true;
	for(component_iterator it=components.begin(); it!=components.end(); ++it)
		if((*it)->mass > 0)
			return true;
	return false;
}

fmat::Column<6> KinematicJoint::getJointJacobian(const fmat::Column<3>& p) const {
	fmat::Column<6> j;
	if(outputOffset>=NumOutputs)
		return j;
	fmat::Transform t = fmat::Transform::identity();
	getFullT(t,NULL);
	//std::cout << "Joint " << depth << "\nTq\n" << Tq << "Tfull\n" << t << std::endl;
	switch(jointType) {
		case REVOLUTE: {
			fmat::Column<3> z(t.column(2));
			fmat::Column<3> v(t.column(3));
			v = p - v;
			fmat::SubVector<3>(j,0) = fmat::crossProduct(z,v);
			fmat::SubVector<3>(j,3) = &t.column(2)[0];
		} break;
		case PRISMATIC: {
			j=0.f;
			fmat::SubVector<3>(j,0) = &t.column(2)[0];
		} break;
	}
	return j;
}

fmat::Column<3> KinematicJoint::getParentPosition() const {
	fmat::Column<3> p = -Tq.column(3);
	return fmat::pack(fmat::dotProduct(Tq.column(0),p), fmat::dotProduct(Tq.column(1),p), fmat::dotProduct(Tq.column(2),p));	
}

fmat::Quaternion KinematicJoint::getWorldQuaternion() const {
	fmat::Quaternion quat;
	getQuaternion(quat,NULL);
	return quat;
}

fmat::Quaternion KinematicJoint::getQuaternion(const KinematicJoint& j) const {
	if(&j==this)
		return fmat::Quaternion::identity();
	fmat::Quaternion quat;
	const KinematicJoint* myAnc=parent;
	const KinematicJoint* jAnc=j.parent;
	while(myAnc!=NULL || jAnc!=NULL) {
		if(myAnc!=NULL) {
			if(myAnc==&j) {
				getQuaternion(quat,&j);
				return quat;
			}
			myAnc=myAnc->parent;
		}
		if(jAnc!=NULL) {
			if(jAnc==this) {
				j.getQuaternion(quat,this);
				return quat.inverse();
			}
			jAnc=jAnc->parent;
		}
	}
	getQuaternion(quat,NULL);
	fmat::Quaternion jq;
	j.getQuaternion(jq,NULL);
	return jq.inverse() * quat;
}

void KinematicJoint::loadXML(xmlNode* node) {
	removeSelfListener();
	clearBranches();
	if(xNodeHasName(node,"array")) {
		try {
			KinematicJointLoader(*this,node);
		} catch(...) {
			addSelfListener();
			throw;
		}
	} else {
		try {
			LinkComponent::loadXML(node);
		} catch(...) {
			addSelfListener();
			throw;
		}
	}
	addSelfListener();
}

void KinematicJoint::saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const {
	if(node==NULL)
		return;
	if(xNodeGetParent(node)==NULL || xNodeGetParent(xNodeGetParent(node))==NULL || xNodeHasName(xNodeGetParent(node),"plist")) {
		KinematicJointSaver(*this,node);
	} else {
		if(xNodeGetChildren(node)==NULL && seen.size()==0) {
			// a bit of customization to put things in a more friendly order (not alphabetical)
			std::string indentStr=getIndentationPrefix(node);
			size_t longestKeyLen = getLongestKeyLen(NULL,1);
			doSaveXMLNode(seen, node, "Name", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "JointType", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "d", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "θ", indentStr, longestKeyLen); // theta
			doSaveXMLNode(seen, node, "r", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "α", indentStr, longestKeyLen); // alpha
			doSaveXMLNode(seen, node, "qOffset", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "Min", indentStr, longestKeyLen);
			doSaveXMLNode(seen, node, "Max", indentStr, longestKeyLen);
			std::string parentIndent;
			if(indentStr.size()>=perIndent().size())
				parentIndent=indentStr.substr(perIndent().size());
			xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+parentIndent).c_str()));
		}
		// save anything else we didn't already cover
		LinkComponent::saveXML(node,onlyOverwrite,seen);
	}
}

void KinematicJoint::plistValueChanged(const plist::PrimitiveBase& pl) {
	updateTo();
	if(&pl==&jointType) {
		switch(*jointType) {
			case REVOLUTE: {
				qOffset.setFormat(plist::Angle::FORMAT_SAME);
				qmin.setFormat(plist::Angle::FORMAT_SAME);
				qmax.setFormat(plist::Angle::FORMAT_SAME);
			} break;
			case PRISMATIC: {
				qOffset.setFormat(plist::Angle::FORMAT_NONE);
				qmin.setFormat(plist::Angle::FORMAT_NONE);
				qmax.setFormat(plist::Angle::FORMAT_NONE);
			} break;
		}
	} else if(&pl==&ikSolver) {
		delete ik;
		ik=NULL;
		ikSolver.removePrimitiveListener(this);
	}
	fireReconfigured();
}

void KinematicJoint::addSelfListener() {
	jointType.addPrimitiveListener(this);
	theta.addPrimitiveListener(this);
	d.addPrimitiveListener(this);
	alpha.addPrimitiveListener(this);
	r.addPrimitiveListener(this);
	qOffset.addPrimitiveListener(this);
	plistValueChanged(jointType); // update values, may have changed
}

void KinematicJoint::removeSelfListener() {
	jointType.removePrimitiveListener(this);
	theta.removePrimitiveListener(this);
	d.removePrimitiveListener(this);
	alpha.removePrimitiveListener(this);
	r.removePrimitiveListener(this);
	qOffset.removePrimitiveListener(this);
}

void KinematicJoint::updateTo() {
	fmat::fmatReal ct = std::cos(static_cast<fmat::fmatReal>(theta));
	fmat::fmatReal st = std::sin(static_cast<fmat::fmatReal>(theta));
	fmat::fmatReal ca = std::cos(static_cast<fmat::fmatReal>(alpha));
	fmat::fmatReal sa = std::sin(static_cast<fmat::fmatReal>(alpha));
	// no final q:
	// fmat is column-major, transpose this array...
	/*fmat::fmatReal mat[16] = {
		ct, -st*ca, st*sa, r*ct,
		st, ct*ca, -ct*sa, r*st,
		0, sa, ca, d,
		0, 0, 0, 1
	};*/
	fmat::fmatReal mat[16] = {
		ct, st, 0, 
		-st*ca, ct*ca, sa, 
		st*sa, -ct*sa, ca, 
		r*ct, r*st, d
	};
	To = mat;
	// with final q:
	/*fmat::fmatReal mat[16] = {
	 cq*ct + sq*(-st*ca), -sq*ct + cq*(-st*ca), st*sa, d*ct,
	 cq*st + sq*(ct*ca), -sq*st + cq*(ct*ca), -ct*sa, d*sa,
	 sq*sa, cq*sa, ca, a,
	 0, 0, 0, 1
	 };*/
	Tq = mat; updateTq();
}

void KinematicJoint::updateTq() {
	const fmat::fmatReal qv = static_cast<fmat::fmatReal>(q+qOffset);
	switch(jointType) {
		case REVOLUTE: {
			const fmat::fmatReal cq = std::cos(qv);
			const fmat::fmatReal sq = std::sin(qv);
			const fmat::fmatReal t11=To(0,0), t12=To(0,1), t21=To(1,0), t22=To(1,1), t32=To(2,1);
			Tq(0,0) = cq*t11 + sq*t12;
			Tq(1,0) = cq*t21 + sq*t22;
			Tq(2,0) = /*t31 is 0*/ sq*t32;
			Tq(0,1) = -sq*t11 + cq*t12;
			Tq(1,1) = -sq*t21 + cq*t22;
			Tq(2,1) = /*t31 is 0*/ cq*t32;
		} break;
		case PRISMATIC: {
			Tq(0,3) = To(0,2)*qv + To(0,3);
			Tq(1,3) = To(1,2)*qv + To(1,3);
			Tq(2,3) = To(2,2)*qv + To(2,3);
		} break;
	}
}

void LinkComponent::getModelTransform(fmat::Transform& tr) const {
	tr.rotation() = fmat::Quaternion::fromAxis(modelRotation).toMatrix();
	tr.translation().importFrom(modelOffset);
}

void LinkComponent::getCollisionModelTransform(fmat::Transform& tr) const {
	tr.rotation() = fmat::Quaternion::fromAxis(collisionModelRotation).toMatrix();
	tr.translation().importFrom(collisionModelOffset);
}

KinematicJoint* LinkComponent::getNextMobileAncestor() const {
	KinematicJoint* joint = parent;
	if (!joint) return NULL;
	while (joint->isMobile() == false) {
		joint = joint->getParent();
		if (!joint) return NULL;
	}
	return joint;
}

PlannerObstacle2D* LinkComponent::getObstacle(const fmat::Transform& worldT) const {
	if(collisionModel.empty())
		return NULL;
	bool asCirc=false;
	if(collisionModel=="Sphere") {
		asCirc = true;
	} else if(collisionModel=="Cylinder") {
		fmat::Quaternion q = fmat::Quaternion::fromAxis(collisionModelRotation);
		fmat::Matrix<3,3> rot = worldT.rotation() * q.toMatrix();
		if(std::abs(rot(2,2))>0.9) // this is just a heuristic, could be better
			asCirc=true;
	}
	if(asCirc) {
		RectangularObstacle ro;
		getOwnBB2D(worldT, ro);
		return new EllipticalObstacle(ro.getCenter(), ro.getWidth()/2, ro.getHeight()/2, ro.getOrientation());
	} else {
		RectangularObstacle * ro = new RectangularObstacle;
		getOwnBB2D(worldT, *ro);
		return ro;
	}
}

void LinkComponent::getObstacles(const fmat::Transform& worldT, HierarchicalObstacle& obs, bool /*recurse*/) const {
	PlannerObstacle2D * o = getObstacle(worldT);
	if(o!=NULL)
		obs.add(o);
}

void KinematicJoint::getObstacles(const fmat::Transform& worldT, HierarchicalObstacle& obs, bool recurse) const {
	fmat::Transform tr = worldT * getTq();
	LinkComponent::getObstacles(tr,obs,recurse);
	for(component_iterator it = components.begin(); it!=components.end(); ++it) {
		(*it)->getObstacles(tr,obs,recurse);
	}
	if(recurse) {
		for(branch_iterator it = branches.begin(); it!=branches.end(); ++it) {
			(*it)->getObstacles(tr, obs, recurse);
		}
	}
}

void LinkComponent::dirtyBB() {
	bbDirty=true;
	if(parent!=NULL)
		static_cast<LinkComponent&>(*parent).dirtyBB(); // silly cast because compiler complains about dirtyBB being protected otherwise :-P
}

void LinkComponent::computeOwnAABB(BoundingBox3D& bb) const {
	if(collisionModel.size()==0) {
		// no collision shape, no bounding box
		bb.clear();
		return;
	}
	fmat::Column<3> ex; // will set to extent (half dim)
	ex.importFrom(collisionModelScale);
	ex/=2; // for primitive shapes, scale is dim, if we ever support polygon collision models, this will need to be smarter
	fmat::Quaternion q = fmat::Quaternion::fromAxis(collisionModelRotation);
	
	// this leverages rotational symmetry, only needs to expand positive dimensions of one face
	fmat::Column<3> b = fmat::abs(ex);
	ex[0]=-ex[0]; b.maximize(fmat::abs(q*ex));
	ex[1]=-ex[1]; b.maximize(fmat::abs(q*ex));
	ex[0]=-ex[0]; b.maximize(fmat::abs(q*ex));
	
	// now apply offset	
	fmat::Column<3> off; off.importFrom(collisionModelOffset);
	bb.min = off - b;
	bb.max = off + b;
}

void LinkComponent::updateBB() const {
	computeOwnAABB(boundingBox);
	bbDirty=false;
}

void KinematicJoint::updateBB() const {
	LinkComponent::updateBB(); // clears bbDirty
	for(component_iterator it = components.begin(); it!=components.end(); ++it) {
		if((*it)->collisionModel.size()==0)
			continue;
		boundingBox.expand((*it)->getAABB());
	}
}

bool LinkComponent::getOwnBB2D(const fmat::Transform& worldT, RectangularObstacle& ro) const {
	if (collisionModel.empty())
		return false;
	fmat::Transform obT;
	getCollisionModelTransform(obT);
	fmat::Transform fullT = worldT * obT;
	computeBB2D(fullT, ro, collisionModelScale.exportTo<fmat::Column<3> >());
	return true;
}

bool LinkComponent::getOwnBB3D(const fmat::Transform& worldT, BoxObstacle& bo) const {
	if (collisionModel.empty())
		return false;
	fmat::Transform obT;
	getCollisionModelTransform(obT);
	fmat::Transform fullT = worldT * obT;
	bo.reset(fullT.translation(),
			 collisionModelScale.exportTo<fmat::Column<3> >()/2,
			 fmat::SubMatrix<3,3,const fmat::fmatReal>(fullT.rotation()));
	return true;
}

bool LinkComponent::getBB2D(const fmat::Transform& worldT, RectangularObstacle& ro) const {
	const BoundingBox3D& bb = getAABB();
	if (bb.empty())
		return false;
	fmat::Transform fullT = worldT;
	fullT.translation()+=bb.getCenter();
	computeBB2D(fullT, ro, bb.getDimensions());
	return true;
}

bool LinkComponent::getBB3D(const fmat::Transform& worldT, BoxObstacle& bo) const {
	const BoundingBox3D& bb = getAABB();
	if (bb.empty())
		return false;
	fmat::Transform fullT = worldT;
	fullT.translation()+=bb.getCenter();
	bo.reset(fullT.translation(),
			 bb.getDimensions()/2,
			 fmat::SubMatrix<3,3,const fmat::fmatReal>(fullT.rotation()));
	return true;
}

void LinkComponent::computeBB2D(const fmat::Transform& fullT, RectangularObstacle& ro, const fmat::Column<3>& obD) {
	// assume projection along Z (could tweak worldT to compensate otherwise)
	// Find longest dimension in XY plane, will use that to align bounding box
	// (is this necessarily optimal fit?  Good enough...)
	
	// columns are the directions of each collision model axis, projected on XY:
	fmat::Matrix<2,3> projR(fullT.rotation());
	
	// now find projected dimension of each axis:
	fmat::Column<3> prD;
	for(size_t i=0; i<3; ++i)
		prD[i] = std::abs(projR.column(i).norm() * obD[i]);
	// abs() only because we don't trust user not to use negative dimensions...
	
	// find maximum dimension, set primary to its axis
	fmat::Column<2> primary = projR.column(0);
	if(prD[2] > prD[1]) {
		if(prD[2] > prD[0]) {
			// if picking z axis, and if x is second dominant, use normal instead for
			// more stable corner ordering when projecting down-axis using square dimensions
			if(prD[0] > prD[1]) {
				if(fullT(2,1)>0) {
					primary = fmat::normalRight(projR.column(2));
				} else {
					primary = fmat::normalLeft(projR.column(2));
				}
			} else {
				primary = projR.column(2);
			}
			// however, above does introduce some extra flip-flopping on
			// some diagonal projections, could just use the axis directly:
			//primary = projR.column(2);
		}
	} else if(prD[1] > prD[0]) {
		// picking y axis, actually use normal to maintain corner ordering
		// fixes corner numerical instability projecting down-axis with square dimensions
		if(prD[0] > prD[2]) {
			if(fullT(2,2)>0) {
				primary = fmat::normalRight(projR.column(1));
			} else {
				primary = fmat::normalLeft(projR.column(1));
			}
		} else {
			if(fullT(2,0)>0) {
				primary = fmat::normalLeft(projR.column(1));
			} else {
				primary = fmat::normalRight(projR.column(1));
			}
		}
		// however, above does introduce some extra flip-flopping on
		// some diagonal projections, could just use the axis directly:
		//primary = projR.column(1);
	}
	fmat::fmatReal primaryNorm = primary.norm();
	if(primaryNorm==0) {
		// projecting down a line or empty collision model
		ro.reset(fmat::SubVector<2,const fmat::fmatReal>(fullT.translation()), fmat::pack(0,0), 0);
		return;
	}
	primary/=primaryNorm;
	
	// construct a (inverse) rotation matrix from this vector
	fmat::Matrix<2,2> boxR;
	boxR(0,0) = primary[0]; boxR(0,1) = primary[1];
	boxR(1,0) =-primary[1]; boxR(1,1) = primary[0];
	// projection of XY components
	fmat::Matrix<2,2> boxProj = boxR * fmat::SubMatrix<2,2>(projR);
	// offset due to Z component
	fmat::Column<2> off = boxR * projR.column(2) * obD[2];
	
	// because of rotational symmetry, only need to test extents of one face
	// (errr, I think so anyway?  Also think it doesn't matter which face?)
	fmat::Column<2> xyD(obD);
	fmat::Column<2> dim = fmat::abs(boxProj*xyD + off);
	xyD[0]=-xyD[0];
	dim.maximize(fmat::abs(boxProj*xyD + off));
	xyD[1]=-xyD[1];
	dim.maximize(fmat::abs(boxProj*xyD + off));
	xyD[0]=-xyD[0];
	dim.maximize(fmat::abs(boxProj*xyD + off));
	
	ro.reset(fmat::SubVector<2,const fmat::fmatReal>(fullT.translation()), dim/2, boxR.transpose());
}

KinematicJoint* KinematicJoint::addBranch(KinematicJoint* b) {
	if(b==NULL)
		throw std::runtime_error("Attempted to add NULL branch to KinematicJoint");
	if(b==this)
		throw std::runtime_error("Attempted to add KinematicJoint as its own branch (no loops!)");
	if(!branches.insert(b).second)
		return b; // already added
	if(b->parent!=NULL)
		throw std::runtime_error("KinematicJoint was told to add a branch which already has a parent.  Pass a clone instead (no loops)");
	if(b->depth>0) // shouldn't happen if we didn't have a parent
		throw std::runtime_error("KinematicJoint was told to add a branch which has a non-zero depth (but doesn't have a parent!?!?  Something's broken.)");
	b->parent=this;
	b->updateDepth();
	fireBranchAdded(*b);
	return b;
}

KinematicJoint* KinematicJoint::removeBranch(KinematicJoint* b) {
	if(branches.erase(b)==0)
		return NULL;
	b->parent=NULL;
	b->depth=0;
	fireBranchRemoved(*b);
	return b;
}

void KinematicJoint::clearBranches() {
	if(branchListeners==NULL) {
		for(std::set<KinematicJoint*>::const_iterator it=branches.begin(); it!=branches.end(); ++it)
			delete *it;
		branches.clear();
	} else {
		while(branches.size()>0)
			delete removeBranch(*branches.begin());
	}
}

KinematicJoint* KinematicJoint::cloneBranch() const {
	KinematicJoint * leaf=NULL, *cur = NULL;
	const KinematicJoint * next=this;
	while(next!=NULL) {
		KinematicJoint * tmp = new KinematicJoint;
		tmp->shallowCopy(*next);
		if(cur!=NULL)
			tmp->addBranch(cur);
		else
			leaf = tmp;
		cur=tmp;
		next = next->parent;
	}
	return leaf;
}

/*! @file
 * @brief Implements KinematicJoint, which manages parameters defining the position and type of motion produced by an actuator (i.e. forward kinematics)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
