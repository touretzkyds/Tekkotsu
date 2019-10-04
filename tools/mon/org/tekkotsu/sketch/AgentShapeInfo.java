package org.tekkotsu.sketch;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.BasicStroke;
import java.awt.Rectangle;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.util.*;
import javax.media.j3d.*;
import com.sun.j3d.utils.geometry.Cone;
import javax.vecmath.*;

public class AgentShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"agent.png");
    float orientation; // orientation of agent
    float offsetX, offsetY, halfDimsX, halfDimsY;
    
    public AgentShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			  float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			  float _orientation, float _offsetX, float _offsetY, float _halfDimsX, float _halfDimsY) {
	super(_gui, _id, _parentId, _name, _color, _centroidx,_centroidy, _centroidz, _obstacle, _landmark);
	orientation = _orientation;
	offsetX = _offsetX;
	offsetY = _offsetY;
	halfDimsX = _halfDimsX;
	halfDimsY = _halfDimsY;
    }
    
    // returns left-most coordinate of object (want some buffer)
    public float getLeft() {return centroidx-halfDimsX+offsetX;}
    // returns right-most coordinate of object
    public float getRight() {return centroidx+halfDimsX+offsetX;}
    // returns top-most coordinate of object
    public float getTop() {return centroidy-halfDimsY+offsetY;}
    // returns bottom-most coordinate of object
    public float getBottom() {return centroidy+halfDimsY+offsetY;}
    
    public String toString() {
	return (super.toString() + " (x=" + centroidx + ", y=" + centroidy + ", theta=" + orientation + ")");
    }	
    
    public Icon getIcon() { return icon; }
    public void renderTo(Graphics2D graphics, float scaling) {
	//		graphics.drawOval((int)(getCentroidX()+1),
	//	(int)(getCentroidY()+1), 5, 5);
	int [] coordsX, coordsY;
	coordsX = new int[3];
	coordsY = new int[3];
	
	//represent the robot as a triangle which has base length of 10cm, height of 20cm
	//top
	coordsX[0] = (int) (centroidx + 100 * Math.cos(orientation));
	coordsY[0] = (int) (centroidy + 100 * Math.sin(orientation));
	
	//left
	coordsX[1] = (int) (centroidx + 120 * Math.cos(-2.618+orientation));
	coordsY[1] = (int) (centroidy + 120 * Math.sin(-2.618+orientation));
	
	//right
	coordsX[2] = (int) (centroidx + 120 * Math.cos(2.618+orientation));
	coordsY[2] = (int) (centroidy + 120 * Math.sin(2.618+orientation));
	graphics.setStroke(new BasicStroke(1.0f/scaling));
	graphics.drawPolygon(coordsX,coordsY, 3);
	
	//draw a point at the exact agent location
	Rectangle bounds = graphics.getClipBounds();
	int radius;
	if (bounds != null)
	    radius= java.lang.Math.max(4,java.lang.Math.max(bounds.width,bounds.height)/100);
	else
	    radius = 4;
	graphics.fillOval((int)centroidx-radius/2, (int)centroidy-radius/2, radius, radius);
    }
    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        Cone agentCone = new Cone(50.0f/scaling, 200.0f/scaling);
        agentCone.setAppearance(ap);
        TransformGroup newTrans = new TransformGroup();
        Transform3D t3d = new Transform3D();
        t3d.rotZ(orientation - ((90 * Math.PI)/180));
        t3d.setTranslation(new Vector3f(centroidx/scaling, centroidy/scaling, 50.0f/scaling));
        newTrans.setTransform(t3d);
        newTrans.addChild(agentCone);
        vinfoBranch.addChild(newTrans);
        vinfoGroup.addChild(vinfoBranch);
    }
    
    public float getOrientation() { return orientation; }
}

