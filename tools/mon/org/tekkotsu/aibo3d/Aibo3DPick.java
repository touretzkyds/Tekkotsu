package org.tekkotsu.aibo3d;

import java.awt.*;
import java.awt.event.*;
import javax.media.j3d.*;
import java.io.*;
import javax.vecmath.*;
import java.util.*;
import java.lang.*;
import java.util.prefs.Preferences;

import com.sun.j3d.loaders.lw3d.Lw3dLoader;
import com.sun.j3d.loaders.Loader;
import com.sun.j3d.loaders.Scene;
import com.sun.j3d.utils.applet.MainFrame;
import com.sun.j3d.utils.universe.SimpleUniverse;
import com.sun.j3d.utils.behaviors.mouse.*;
import com.sun.j3d.utils.picking.behaviors.*;
import com.sun.j3d.utils.picking.*;

import javax.swing.JFrame;
import org.tekkotsu.mon.WorldStateJointsListener;
import org.tekkotsu.mon.Joints;

public class Aibo3DPick extends Frame implements WorldStateJointsListener.UpdatedListener {
  Canvas3D canvas3d;
  Scene scene;
  Aibo3DForward forward;
  PickMoveBehavior picker;
  BranchGroup branchGroup;
  SimpleUniverse universe;
  BranchGroup root;
  static Preferences prefs = Preferences.userNodeForPackage(Aibo3DPick.class);
	//Thread mirror;
	WorldStateJointsListener wsjl;
	WorldStateJointsWriter wsjw;
  
  public static void main(String args[]) {
		if(args.length<1) {
			System.out.println("Usage: java Aibo3DPick host [port]");
			System.out.println("       if port is not specified, it defaults to 10051");
			System.exit(2);
		}
		int port=10051;
		if(args.length>1)
			port=Integer.parseInt(args[1]);
    new Aibo3DPick(args[0],port,null);
  }

  public Aibo3DPick(String host, int port, String args[]) {
    super("Aibo 3D");
    setSize(600,400);
    setLocation(prefs.getInt("Aibo3DPick.location.x",50),prefs.getInt("Aibo3DPick.location.y",50));

    setup3DCanvas();
    loadLW3D("ERS-210.lws");
    forward=new Aibo3DForward("ERS-210",scene);
    showScene();

    picker.createPicker();
    wsjl=WorldStateJointsListener.getCommonInstance(host);
		wsjl.addUpdatedListener(this);
		wsjw=new WorldStateJointsWriter(host, port);

		//mirror=new mirrorThread(this);
		//mirror.start();
    addWindowListener(new CloseAdapter(this));
  }

	public void worldStateUpdated(WorldStateJointsListener mc) {
		mirrorListener(mc);
	}


	class mirrorThread extends Thread {
		Aibo3DPick gui;
		mirrorThread(Aibo3DPick gui) { this.gui=gui; }
		public void run() { 
			//wait for connection so we know what the current positions are
			while(!gui.wsjl.hasData()) {
				try { Thread.sleep(100); } catch (Exception e) {break;}
			}
			gui.mirrorListener(wsjl);
			while (true) {
				if (gui.forward.lock) gui.mirrorWriter(wsjw);
				else gui.mirrorListener(wsjl);
				try { Thread.sleep(27); } catch (Exception e) {break;}
			}
		}
	}

	class CloseAdapter extends WindowAdapter {
		Aibo3DPick gui;
		CloseAdapter(Aibo3DPick gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	public void close() {
    prefs.putInt("Aibo3DPick.location.x",getLocation().x+getInsets().left);
    prefs.putInt("Aibo3DPick.location.y",getLocation().y+getInsets().top);
		wsjl.removeUpdatedListener(this);
		if(wsjw!=null)
			wsjw.kill();
		//mirror.interrupt();
		dispose();
	}

  void mirrorListener (WorldStateJointsListener wsj) {
    if (wsj.isConnected()) {
      //if (wsj.hasData()) {
			Joints j=wsj.getData();
			float[] p=j.positions;
			forward.knee_fl.setX(-p[2]);
			forward.thigh_fl.set(-p[0],0.0f,p[1]);
			forward.knee_fr.setX(-p[5]);
			forward.thigh_fr.set(-p[3],0.0f,-p[4]);
			forward.knee_bl.setX(p[8]);
			forward.thigh_bl.set(p[6],0.0f,p[7]);
			forward.knee_br.setX(p[11]);
			forward.thigh_br.set(p[9],0.0f,-p[10]);
			forward.neck.setX(-p[12]);
			forward.head.set(0.0f,p[13],p[14]);
			forward.tail.set(p[15],p[16],0.0f);
			forward.jaw.setX(-p[17]);
			//}
    }
  }

  void mirrorWriter(WorldStateJointsWriter wsj) {
    if (wsj.isConnected()) {
      float[] p=new float[18];
      p[0]=-forward.thigh_fl.x;
      p[1]=forward.thigh_fl.z;
      p[2]=-forward.knee_fl.x;
      p[3]=-forward.thigh_fr.x;
      p[4]=-forward.thigh_fr.z;
      p[5]=-forward.knee_fr.x;
      p[6]=forward.thigh_bl.x;
      p[7]=forward.thigh_bl.z;
      p[8]=forward.knee_bl.x;
      p[9]=forward.thigh_br.x;
      p[10]=-forward.thigh_br.z;
      p[11]=forward.knee_br.x;
      p[12]=-forward.neck.x;
      p[13]=forward.head.y;
      p[14]=forward.head.z;
      p[15]=forward.tail.x;
      p[16]=forward.tail.y;
      p[17]=-forward.jaw.x;
      wsj.write(p);
    }
  }

  void testLimits(AiboJoint j) {
    float step;
    step=(j.maxX-j.minX)/100;
    for (float x=j.minX; x<j.maxX; x+=step) { j.setX(x); sleep(10); }
    for (float x=j.maxX; x>j.minX; x-=step) { j.setX(x); sleep(10); }
    j.setX(0.0f);
    step=(j.maxY-j.minY)/100;
    for (float y=j.minY; y<j.maxY; y+=step) { j.setY(y); sleep(10); }
    for (float y=j.maxY; y>j.minY; y-=step) { j.setY(y); sleep(10); }
    j.setY(0.0f);
    step=(j.maxZ-j.minZ)/100;
    for (float z=j.minZ; z<j.maxZ; z+=step) { j.setZ(z); sleep(10); }
    for (float z=j.maxZ; z>j.minZ; z-=step) { j.setZ(z); sleep(10); }
    j.setZ(0.0f);
  }

  void testLimits(AiboJoint j, int n) {
    for (int i=0; i<n; i++) {
      testLimits(j);
    }
  }

  void sleep(long ms) {
    try { Thread.sleep(ms); } catch (Exception e) {}
  }

  void loadLW3D(String filename) {
    Loader loader=new Lw3dLoader(Loader.LOAD_ALL);
    try {
      scene=loader.load(filename);
    } catch (Exception ex) {
      System.out.println("error loading "+filename);
      System.exit(1);
    }
  }

  void setup3DCanvas() {
    canvas3d=new Canvas3D(SimpleUniverse.getPreferredConfiguration());
    add("Center",canvas3d);
    setVisible(true);
  }

  void showScene() {
    GraphicsConfiguration config =
      SimpleUniverse.getPreferredConfiguration();

    universe = new SimpleUniverse(canvas3d);
    universe.getViewingPlatform().setNominalViewingTransform();

    branchGroup = createSceneGraph(scene);
    universe.addBranchGraph(branchGroup);
  }

  BranchGroup createSceneGraph(Scene scene) {
    root = new BranchGroup();
    TransformGroup transformGroup;

    transformGroup = new TransformGroup();
    transformGroup.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
    transformGroup.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);

    Hashtable namedObjects = scene.getNamedObjects( );

    // recursively set the user data here
    // so we can find our objects when they are picked
    
    java.util.Enumeration enumValues = namedObjects.elements( );
    java.util.Enumeration enumKeys = namedObjects.keys( );
    if( enumValues != null )
    {
      while( enumValues.hasMoreElements( ) != false )
      {
        Object value = enumValues.nextElement( );
        Object key = enumKeys.nextElement( );

        recursiveSetUserData( value, key );
      }
    }


    Bounds bounds=new BoundingSphere(new Point3d(0.0,0.0,0.0), 1.0);
/*
    MouseRotate rotator = new MouseRotate(transformGroup);
    rotator.setSchedulingBounds(bounds);
    root.addChild(rotator);

    MouseTranslate translator = new MouseTranslate(transformGroup);
    translator.setSchedulingBounds(bounds);
    root.addChild(translator);
*/
    MouseZoom zoomer = new MouseZoom(transformGroup);
    zoomer.setSchedulingBounds(bounds);
    root.addChild(zoomer); 

    picker=new PickMoveBehavior (canvas3d, scene.getSceneGroup(), scene,
                                 forward, transformGroup);
    picker.setSchedulingBounds(bounds);
    root.addChild(picker);

    if (scene.getSceneGroup() != null) {
      transformGroup.addChild(scene.getSceneGroup());
      root.addChild(transformGroup);
    }

    moveLights(scene.getSceneGroup(), root);
//    recursivePrintGroup(root);

    return root;
  }

  void moveLights(Group from, Group to) {
    from.setCapability(Group.ALLOW_CHILDREN_READ);

    java.util.Enumeration enumKids = from.getAllChildren();
    while( enumKids.hasMoreElements( ) != false ) {
      Object o = enumKids.nextElement();
      if (o instanceof Group) {
        moveLights((Group)o, to);
      } else if (o instanceof DirectionalLight || o instanceof PointLight) {
        from.setCapability(Group.ALLOW_CHILDREN_WRITE);
        to.setCapability(Group.ALLOW_CHILDREN_READ);
        to.setCapability(Group.ALLOW_CHILDREN_WRITE);
        from.removeChild((Node)o);
        to.addChild((Node)o);
      }
    }

  }

  void recursivePrintGroup(Object sgo) {
    recursivePrintGroup(sgo,"");
  }

  void recursivePrintGroup(Object sgo, String indent) {
    if( sgo instanceof SceneGraphObject != false )
    {
      SceneGraphObject sg = (SceneGraphObject) sgo;
                                                                                
      // recursively process group
      if( sg instanceof Group )
      {
        Group g = (Group) sg;
        g.setCapability(Group.ALLOW_CHILDREN_READ);
                                                                                
        // recurse on child nodes
        java.util.Enumeration enumKids = g.getAllChildren( );
                                                                                
        while( enumKids.hasMoreElements( ) != false )
          recursivePrintGroup( enumKids.nextElement( ), indent+"  ");
      }
      else if ( sg instanceof Node || sg instanceof NodeComponent)
      {
        System.out.println(indent+sg);
      }
    }
  
  }

  void recursiveSetUserData( Object value, Object key )
  {
    if( value instanceof SceneGraphObject != false )
    {
      // set the user data for the item
      SceneGraphObject sg = (SceneGraphObject) value;
      sg.setUserData( key );
                                                                                
      // recursively process group
      if( sg instanceof Group )
      {
        Group g = (Group) sg;
        g.setCapability(Group.ALLOW_CHILDREN_READ);
                                                                                
        // recurse on child nodes
        java.util.Enumeration enumKids = g.getAllChildren( );
                                                                                
        while( enumKids.hasMoreElements( ) != false )
          recursiveSetUserData( enumKids.nextElement( ), key );
      }
      else if ( sg instanceof Shape3D || sg instanceof Morph )
      {
        PickTool.setCapabilities( (Node) sg, PickTool.INTERSECT_FULL );
      }
    }
  }

}
