package org.tekkotsu.aibo3d;

import java.awt.*;
import javax.swing.*;
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

import org.tekkotsu.mon.Joints;
import org.tekkotsu.mon.Listener;
import org.tekkotsu.mon.WorldStateJointsListener;

public class Aibo3D extends JFrame implements Listener.ConnectionListener, WorldStateJointsListener.UpdatedListener {
	static Preferences prefs = Preferences.userNodeForPackage(Aibo3D.class);
	Scene scene;
	Canvas3D canvas3d;
	SimpleUniverse universe;
	BranchGroup rootgroup;
	Aibo3DForward forward;
	WorldStateJointsListener wsj;
	TransformGroup viewpoint;
	boolean firstUpdate;
	JLabel disconnectMsg; 
	String model;
	
	public static void main(String args[]) {
		if(args.length<1) {
			System.out.println("Usage: java Aibo3D host [port]");
			System.out.println("       if port is not specified, it defaults to 10031");
			System.exit(2);
		}
		int port=WorldStateJointsListener.defPort;
		if(args.length>1)
			port=Integer.parseInt(args[1]);
		new Aibo3D(args[0],port,null);
	}
	
	public Aibo3D(String host, int port, String args[]) {
		super("Aibo 3D");
		getContentPane().setLayout(new BorderLayout());
		setSize(prefs.getInt("Aibo3D.size.width",500),prefs.getInt("Aibo3D.size.height",500));
		setLocation(prefs.getInt("Aibo3D.location.x",50), prefs.getInt("Aibo3D.location.y",50));
		
		disconnectMsg=new JLabel("Disconnected. trying to reconnect...",SwingConstants.CENTER);
		add(disconnectMsg,BorderLayout.CENTER);
		addWindowListener(new WindowAdapter() { // cleans up properly
			public void windowClosing(WindowEvent e) {
				close();
			}
		});
		
		wsj=WorldStateJointsListener.getCommonInstance(host,port);
		firstUpdate=true;
		wsj.addConnectionListener(this);
		wsj.addUpdatedListener(this);
		
		setVisible(true);
		
		/*
		BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
		String lastS="";
		ArrayList lastC=null;
		while(true) {
			try {
				String s=in.readLine();
				if(model!=null) {
					if(lastC!=null)
						highlight(lastS,lastC);
					lastC=highlight(s,new Color3f(1,0,0));
					lastS=s;
				}
			} catch(Exception e) {
				e.printStackTrace();
				break;
			}
		}
		 */
		
		//    testLimits(forward.thigh_bl,50);
	}
	
	void highlight(String s, ArrayList c) {
		TransformGroup tg=forward.getTG("objects-"+model+"/"+s+".lwo");
		if(tg==null) {
			System.out.println("invalid: "+s);
			return;
		}
		for(Enumeration e=tg.getAllChildren(); e.hasMoreElements();) {
			Node n=(Node)e.nextElement();
			if(n instanceof TransformGroup) {
				//flipNormals((TransformGroup)n); // don't recurse
			} else {
				if(n instanceof Shape3D) {
					Appearance app=((Shape3D)n).getAppearance();
					app.getMaterial().setDiffuseColor((Color3f)c.get(0));
					c.remove(0);
				} else {
					System.out.println("unhandled in flip: "+n.getClass());
				}
			}
		}
	}
	ArrayList highlight(String s, Color3f c) {
		TransformGroup tg=forward.getTG("objects-"+model+"/"+s+".lwo");
		if(tg==null) {
			System.out.println("invalid: "+s);
			return null;
		}
		ArrayList l=new ArrayList();
		for(Enumeration e=tg.getAllChildren(); e.hasMoreElements();) {
			Node n=(Node)e.nextElement();
			if(n instanceof TransformGroup) {
				//flipNormals((TransformGroup)n); // don't recurse
			} else {
				if(n instanceof Shape3D) {
					Appearance app=((Shape3D)n).getAppearance();
					Color3f ans=new Color3f();
					app.getMaterial().getDiffuseColor(ans);
					l.add(ans);
					app.getMaterial().setDiffuseColor(1.f,0,0);
				} else {
					System.out.println("unhandled in flip: "+n.getClass());
				}
			}
		}
		return l;
	}

	public void onConnected() {
		firstUpdate=true;
	}
	public void onDisconnected() {
		firstUpdate=true;
		getContentPane().removeAll();
		add(disconnectMsg,BorderLayout.CENTER);
		((JComponent)getContentPane()).revalidate();
		model=null;
	}
	
	public void worldStateUpdated(WorldStateJointsListener mc) {
		Joints j=wsj.getData();
		if(firstUpdate) {
			getContentPane().removeAll();
			firstUpdate=false;
			setup3DCanvas();
			loadLW3D(j.model+".lws");
			model=j.model;
			forward=new Aibo3DForward(model,scene);
			createSceneGraph(scene);
			((JComponent)getContentPane()).revalidate();
			canvas3d.getView().setFrontClipDistance(0.025);
			recursivePrintGroup(scene);
		}
		float[] p=j.positions;
		forward.knee_fl.setX(-p[2]);
		forward.thigh_fl.set(-p[0],0.0f,p[1]);
		forward.knee_fr.setX(-p[5]);
		forward.thigh_fr.set(-p[3],0.0f,-p[4]);
		forward.knee_bl.setX(p[8]);
		forward.thigh_bl.set(p[6],0.0f,p[7]);
		forward.knee_br.setX(p[11]);
		forward.thigh_br.set(p[9],0.0f,-p[10]);
		forward.neck.setX(p[12]);
		forward.head.set(0.0f,p[13],p[14]);
		if(model.equals("ERS-7")) {
			forward.tail.set(p[15],0.0f,p[16]);
		} else {
			forward.tail.set(p[15],p[16],0.0f);
		}
		forward.jaw.setX(-p[17]);
	}
	
	
	public void close() {
		prefs.putInt("Aibo3D.location.x",getLocation().x/*+getInsets().left*/);
		prefs.putInt("Aibo3D.location.y",getLocation().y/*+getInsets().top*/);
		prefs.putInt("Aibo3D.size.width",getSize().width);
		prefs.putInt("Aibo3D.size.height",getSize().height);
		float[] trans = new float[16];
		Transform3D t3d=new Transform3D();
		if(viewpoint!=null) {
			viewpoint.getTransform(t3d);
			t3d.get(trans);
			for(int i=0; i<4; ++i)
				for(int j=0; j<4; ++j)
					prefs.putFloat("Aibo3D.viewpoint."+i+"."+j,trans[i*4+j]);
		}
		wsj.removeUpdatedListener(this);
		wsj.removeConnectionListener(this);
		dispose();
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
		add(canvas3d,BorderLayout.CENTER);
		universe = new SimpleUniverse(canvas3d);
		universe.getViewingPlatform().setNominalViewingTransform();
	}
	
	
	void createSceneGraph(Scene scene) {
		rootgroup = new BranchGroup();
		
		TransformGroup trans = viewpoint = new TransformGroup();
		trans.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
		trans.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		
		Transform3D t3d=new Transform3D();
		float[] preftrans = new float[16];
		for(int i=0; i<4; ++i)
			for(int j=0; j<4; ++j)
				preftrans[i*4+j]=prefs.getFloat("Aibo3D.viewpoint."+i+"."+j,i==j?1:0);
		t3d.set(preftrans);
		viewpoint.setTransform(t3d);
		
		BoundingSphere bounds = new BoundingSphere(new Point3d(), 100.0);
		
		MouseRotate rotator = new MouseRotate(trans);
		rotator.setSchedulingBounds(bounds);
		rootgroup.addChild(rotator);
		
		MouseTranslate translator = new MouseTranslate(trans);
		translator.setSchedulingBounds(bounds);
		rootgroup.addChild(translator);
		
		MouseZoom zoomer = new MouseZoom(trans);
		zoomer.setSchedulingBounds(bounds);
		rootgroup.addChild(zoomer); 
		
		if (scene.getSceneGroup() != null) {
			trans.addChild(scene.getSceneGroup());
			rootgroup.addChild(trans);
		}
		setFlags(rootgroup);
		moveLights(scene.getSceneGroup(), rootgroup);
		universe.addBranchGraph(rootgroup);
	}
	
	void setFlags(Group g) {
		g.setCapability(Group.ALLOW_CHILDREN_READ);
		//g.setCapability(Group.ALLOW_CHILDREN_WRITE);
		for(Enumeration e=g.getAllChildren(); e.hasMoreElements(); ) {
			Object o=e.nextElement();
			if(o instanceof Group) {
				setFlags((Group)o);
			} else if(o instanceof Shape3D) {
				((Shape3D)o).setCapability(Shape3D.ALLOW_APPEARANCE_READ);
				//((Shape3D)o).setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);
				//((Shape3D)o).getAppearance().setCapability(Appearance.ALLOW_POLYGON_ATTRIBUTES_READ);
				//((Shape3D)o).getAppearance().setCapability(Appearance.ALLOW_POLYGON_ATTRIBUTES_WRITE);
				//((Shape3D)o).getAppearance().getPolygonAttributes().setCapability(PolygonAttributes.ALLOW_CULL_FACE_READ);
				//((Shape3D)o).getAppearance().getPolygonAttributes().setCapability(PolygonAttributes.ALLOW_CULL_FACE_WRITE);
				((Shape3D)o).getAppearance().setCapability(Appearance.ALLOW_MATERIAL_READ);
				((Shape3D)o).getAppearance().getMaterial().setCapability(Material.ALLOW_COMPONENT_READ);
				((Shape3D)o).getAppearance().getMaterial().setCapability(Material.ALLOW_COMPONENT_WRITE);
			}
		}
	}
	
	void moveLights(Group from, Group to) {
		//from.setCapability(Group.ALLOW_CHILDREN_READ);
		
		java.util.Enumeration enumKids = from.getAllChildren();
		while( enumKids.hasMoreElements( ) != false ) {
			Object o = enumKids.nextElement();
			if (o instanceof Group) {
				moveLights((Group)o, to);
			} else if (o instanceof DirectionalLight || o instanceof PointLight) {
				//from.setCapability(Group.ALLOW_CHILDREN_WRITE);
				//to.setCapability(Group.ALLOW_CHILDREN_READ);
				//to.setCapability(Group.ALLOW_CHILDREN_WRITE);
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
				//g.setCapability(Group.ALLOW_CHILDREN_READ);
				
				// recurse on child nodes
				java.util.Enumeration enumKids = g.getAllChildren( );
				
				System.out.println(indent+sg.getClass()+" {");
				while( enumKids.hasMoreElements( ) != false )
					recursivePrintGroup( enumKids.nextElement( ), indent+"  ");
				System.out.println(indent+"}");
			}
			else if ( sg instanceof Node || sg instanceof NodeComponent)
			{
				System.out.println(indent+sg);
			} else {
				System.out.println(indent+sg.getClass());
			}
		}
		
	}
	
}
