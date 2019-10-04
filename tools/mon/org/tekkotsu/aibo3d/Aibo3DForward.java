package org.tekkotsu.aibo3d;

import javax.media.j3d.*;
import javax.vecmath.*;
import java.util.*;
import java.lang.*;

import com.sun.j3d.loaders.Scene;

public class Aibo3DForward {
	Scene _scene;
	Hashtable _namedObjects;
	boolean lock;
	
	AiboJoint head;
	AiboJoint neck;
	AiboJoint jaw;
	AiboJoint tail;
	AiboJoint thigh_fl;
	AiboJoint knee_fl;
	AiboJoint thigh_fr;
	AiboJoint knee_fr;
	AiboJoint thigh_bl;
	AiboJoint knee_bl;
	AiboJoint thigh_br;
	AiboJoint knee_br;
	List aiboJoints;
	
	public Aibo3DForward(String model, Scene scene) {
		_scene=scene;
		_namedObjects=scene.getNamedObjects();
		lock=false;
		
		makeSceneTransformable();
		if(model.equals("ERS-210"))
			initERS210();
		else if(model.equals("ERS-7")) {
			 initERS7();
		}
	}
	
	public List getAiboJoints() {
		return aiboJoints;    
	}
	
	void makeSceneTransformable() {
		for (Enumeration e=_namedObjects.keys(); e.hasMoreElements();)
			makeObjectTransformable(e.nextElement());
	}
	
	void makeObjectTransformable(Object key) {
		TransformGroup objtg=getTG(key);
		objtg.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
		objtg.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
	}
	
	void printNamedObjects() {
		for (Enumeration e=_namedObjects.keys(); e.hasMoreElements();)
			System.out.println(e.nextElement());
	}
	
	void printNamedObjectTransformationMatrices() {
		for (Enumeration e=_namedObjects.keys(); e.hasMoreElements();) {
			System.out.println(e.nextElement()+": ");
			printNamedObjectTransformationMatrix(e.nextElement());
		}
	}
	
	void printNamedObjectTransformationMatrix(Object key) {
		TransformGroup objtg=getTG(key);
		Transform3D t3d=new Transform3D();
		objtg.getTransform(t3d);
		System.out.println(t3d);
	}
	
	public TransformGroup getTG(Object key) {
		return (TransformGroup)_namedObjects.get(key);
	}
	
	TransformGroup flipNormals(TransformGroup tg) {
		return flipNormals(tg,false);
	}
	
	TransformGroup flipNormals(TransformGroup tg, boolean highlight) {
		for(Enumeration e=tg.getAllChildren(); e.hasMoreElements();) {
			Node n=(Node)e.nextElement();
			if(n instanceof TransformGroup) {
				//flipNormals((TransformGroup)n); // don't recurse
			} else {
				if(n instanceof Shape3D) {
					Appearance app=((Shape3D)n).getAppearance();
					PolygonAttributes papp=app.getPolygonAttributes();
					if(papp==null)
						app.setPolygonAttributes(papp=new PolygonAttributes());
					papp.setBackFaceNormalFlip(!papp.getBackFaceNormalFlip());
					papp.setCullFace(PolygonAttributes.CULL_FRONT);
					if(highlight)
						app.getMaterial().setDiffuseColor(1.f,0,0);
				} else {
					System.out.println("unhandled in flip: "+n.getClass());
				}
			}
		}
		return tg;
	}
		
	
	
	void initERS210 () {
		neck=new AiboJoint(getTG("objects-ERS-210/neck.lwo"), "neck",
			-46.0f,   85.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		head=new AiboJoint(getTG("objects-ERS-210/head.lwo"), "head",
			0.0f,     0.0f,
			-92.6f,   92.6f,
			-32.0f,   32.0f);
		jaw=new AiboJoint(getTG("objects-ERS-210/jaw.lwo"), "jaw",
			0.0f,     50.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		tail=new AiboJoint(getTG("objects-ERS-210/tail2.lwo"), "tail",
			-25.0f,   25.0f,
			-25.0f,   25.0f,
			0.0f,     0.0f);
		thigh_fl=new AiboJoint(getTG("objects-ERS-210/leg-f-up-l.lwo"), "thigh_fl",
			-120.0f,  120.0f,
			0.0f,     0.0f,
			-15.0f,   92.0f);
		thigh_fr=new AiboJoint(getTG("objects-ERS-210/leg-f-up-r.lwo"), "thigh_fr",
			-120.0f,  120.0f,
			0.0f,     0.0f,
			-92.0f,   15.0f);
		thigh_bl=new AiboJoint(getTG("objects-ERS-210/leg-b-up-l.lwo"), "thigh_bl",
			-120.0f,  120.0f,
			0.0f,     0.0f,
			-15.0f,   92.0f);
		thigh_br=new AiboJoint(getTG("objects-ERS-210/leg-b-up-r.lwo"), "thigh_br",
			-120.0f,  120.0f,
			0.0f,     0.0f,
			-92.0f,   15.0f);
		knee_fl=new AiboJoint(getTG("objects-ERS-210/leg-f-low-l.lwo"), "knee_fl",
			-150.0f,  30.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_fl.setRotationalPreOffset(35*(float)Math.PI/180,0.f,0.f);
		knee_fr=new AiboJoint(getTG("objects-ERS-210/leg-f-low-r.lwo"), "knee_fr",
			-150.0f,  30.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_fr.setRotationalPreOffset(35*(float)Math.PI/180,0.f,0.f);
		knee_bl=new AiboJoint(getTG("objects-ERS-210/leg-b-low-l.lwo"), "knee_bl",
			-30.0f,   150.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_bl.setRotationalPreOffset(-35*(float)Math.PI/180,0.f,0.f);
		knee_br=new AiboJoint(getTG("objects-ERS-210/leg-b-low-r.lwo"), "knee_br",
			-30.0f,   150.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_br.setRotationalPreOffset(-35*(float)Math.PI/180,0.f,0.f);
		aiboJoints=new ArrayList(20);
		
		aiboJoints.add(head);
		aiboJoints.add(neck);
		aiboJoints.add(jaw);
		aiboJoints.add(tail);
		aiboJoints.add(thigh_fl);
		aiboJoints.add(knee_fl);
		aiboJoints.add(thigh_fr);
		aiboJoints.add(knee_fr);
		aiboJoints.add(thigh_bl);
		aiboJoints.add(knee_bl);
		aiboJoints.add(thigh_br);
		aiboJoints.add(knee_br);
	}

	void initERS7 () {
		flipNormals(getTG("objects-ERS-7/body01.lwo")); // main body
		flipNormals(getTG("objects-ERS-7/body02.lwo")); // chest IR
		flipNormals(getTG("objects-ERS-7/body03.lwo")); // throat plate
		flipNormals(getTG("objects-ERS-7/body04.lwo")); // back
		//flipNormals(getTG("objects-ERS-7/body05.lwo"),true); // back buttons
		flipNormals(getTG("objects-ERS-7/body06.lwo")); //underbelly
		
		//flipNormals(getTG("objects-ERS-7/chops.lwo"),true); //mouth
		
		//flipNormals(getTG("objects-ERS-7/head01.lwo"),true); // chin
		//flipNormals(getTG("objects-ERS-7/head02.lwo"),true); // camera
		//flipNormals(getTG("objects-ERS-7/head03.lwo"),true); // face
		//flipNormals(getTG("objects-ERS-7/head04.lwo"),true); // face panel LEDs
		//flipNormals(getTG("objects-ERS-7/head05.lwo"),true); // right ear mount
		//flipNormals(getTG("objects-ERS-7/head06.lwo"),true); // left ear mount

		flipNormals(getTG("objects-ERS-7/Lear01.lwo")); // left microphone
		//flipNormals(getTG("objects-ERS-7/Lear02.lwo"),true); // left ear

		flipNormals(getTG("objects-ERS-7/LforefootA.lwo")); // left front shoulder
		//flipNormals(getTG("objects-ERS-7/LforefootB.lwo"),true); // left front thigh
		//flipNormals(getTG("objects-ERS-7/LforefootC.lwo"),true); // left front shin
		//flipNormals(getTG("objects-ERS-7/LforefootD.lwo"),true); // left front foot
		
		flipNormals(getTG("objects-ERS-7/LhindfootA.lwo")); // left hind shoulder
		//flipNormals(getTG("objects-ERS-7/LhindfootB.lwo"),true); // left hind thigh
		//flipNormals(getTG("objects-ERS-7/LhindfootC.lwo"),true); // left hind shin
		//flipNormals(getTG("objects-ERS-7/LhindfootD.lwo"),true); // left hind foot
		
		//flipNormals(getTG("objects-ERS-7/neck.lwo"),true); //neck
		
		//flipNormals(getTG("objects-ERS-7/Rear01.lwo")); // right microphone
		flipNormals(getTG("objects-ERS-7/Rear02.lwo")); // right ear
		
		//flipNormals(getTG("objects-ERS-7/RforefootA.lwo")); // right front shoulder
		flipNormals(getTG("objects-ERS-7/RforefootB.lwo")); // right front thigh
		flipNormals(getTG("objects-ERS-7/RforefootC.lwo")); // right front shin
		flipNormals(getTG("objects-ERS-7/RforefootD.lwo")); // right front foot
		
		//flipNormals(getTG("objects-ERS-7/RhindfootA.lwo")); // right hind shoulder
		flipNormals(getTG("objects-ERS-7/RhindfootB.lwo")); // right hind thigh
		flipNormals(getTG("objects-ERS-7/RhindfootC.lwo")); // right hind shin
		flipNormals(getTG("objects-ERS-7/RhindfootD.lwo")); // right hind foot
		
		flipNormals(getTG("objects-ERS-7/tail01.lwo")); // tail "bone"
		flipNormals(getTG("objects-ERS-7/tail02.lwo")); // tail itself
		
		neck=new AiboJoint(getTG("objects-ERS-7/neck.lwo"), "neck",
			-80.0f,   3.f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		neck.setRotationalPreOffset(0.f,-(float)Math.PI,0.f);
		neck.setRotationalPostOffset(0.f,(float)Math.PI,0.f);
		head=new AiboJoint(getTG("objects-ERS-7/head01.lwo"), "head",
			0.0f,      0.0f,
			-93.0f,   93.0f,
			-20.0f,    50.0f);
		head.setRotationalPreOffset(0.f,-(float)Math.PI/2,0.f);
		head.setRotationalPostOffset(0.f,(float)Math.PI/2,0.f);
		jaw=new AiboJoint(getTG("objects-ERS-7/chops.lwo"), "jaw",
			0.0f,     55.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		tail=new AiboJoint(getTG("objects-ERS-7/tail01.lwo"), "tail",
			0.0f,   63.0f,
			0.0f,     0.0f,
			-60.0f,   60.0f);
		thigh_fl=new AiboJoint(getTG("objects-ERS-7/LforefootB.lwo"), "thigh_fl",
			-120.0f,  135.0f,
			0.0f,     0.0f,
			-15.0f,   93.0f);
		thigh_fr=new AiboJoint(getTG("objects-ERS-7/RforefootB.lwo"), "thigh_fr",
			-120.0f,  135.0f,
			0.0f,     0.0f,
			-93.0f,   15.0f);
		thigh_bl=new AiboJoint(getTG("objects-ERS-7/LhindfootB.lwo"), "thigh_bl",
			-135.0f,  120.0f,
			0.0f,     0.0f,
			-15.0f,   93.0f);
		thigh_br=new AiboJoint(getTG("objects-ERS-7/RhindfootB.lwo"), "thigh_br",
			-135.0f,  120.0f,
			0.0f,     0.0f,
			-93.0f,   15.0f);
		knee_fl=new AiboJoint(getTG("objects-ERS-7/LforefootC.lwo"), "knee_fl",
			-127.0f,  30.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_fl.setRotationalPreOffset(30*(float)Math.PI/180,0.f,0.f);
		knee_fr=new AiboJoint(getTG("objects-ERS-7/RforefootC.lwo"), "knee_fr",
			-127.0f,  30.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_fr.setRotationalPreOffset(30*(float)Math.PI/180,0.f,0.f);
		knee_bl=new AiboJoint(getTG("objects-ERS-7/LhindfootC.lwo"), "knee_bl",
			-30.0f,   127.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_bl.setRotationalPreOffset(-30*(float)Math.PI/180,0.f,0.f);
		knee_br=new AiboJoint(getTG("objects-ERS-7/RhindfootC.lwo"), "knee_br",
			-30.0f,   127.0f,
			0.0f,     0.0f,
			0.0f,     0.0f);
		knee_br.setRotationalPreOffset(-30*(float)Math.PI/180,0.f,0.f);
		aiboJoints=new ArrayList(20);
		
		aiboJoints.add(head);
		aiboJoints.add(neck);
		aiboJoints.add(jaw);
		aiboJoints.add(tail);
		aiboJoints.add(thigh_fl);
		aiboJoints.add(knee_fl);
		aiboJoints.add(thigh_fr);
		aiboJoints.add(knee_fr);
		aiboJoints.add(thigh_bl);
		aiboJoints.add(knee_bl);
		aiboJoints.add(thigh_br);
		aiboJoints.add(knee_br);
	}
}
