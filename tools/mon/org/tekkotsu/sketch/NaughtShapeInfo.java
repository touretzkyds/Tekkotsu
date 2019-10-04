package org.tekkotsu.sketch;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Graphics;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.BasicStroke;
import javax.media.j3d.*;
import javax.vecmath.*;
import com.sun.j3d.utils.geometry.Cylinder;

// stores info for a NaughtShape
public class NaughtShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"naught.png");
    static Icon inv_icon = new ImageIcon(icon_path+"naughtinv.png");
	
    float height;
    float radius;
    Quat4f q;
	
    // returns left-most coordinate of object
    public float getLeft() { return centroidx-radius; }
    // returns right-most coordinate of object
    public float getRight() { return centroidx+radius; }
    // returns top-most coordinate of object
    public float getTop() { return centroidy-radius; }
    // returns bottom-most coordinate of object
    public float getBottom() { return centroidy+radius; }

    public NaughtShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			     float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			   float _height, float _radius) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	height=_height;
	radius=_radius;
	q = new Quat4f((float)Math.sqrt(2.0), (float)Math.sqrt(2.0), 0, 0);
    }
	
    public String toString() {
        String _center = "center=(" + centroidx + " " + centroidy + " " + centroidz + ")";
        return super.toString() + " " + _center + " radius=" + radius + " height=" + height;
    }
	
    public Icon getIcon() { 
        if (inverted)
            return inv_icon;
        else
            return icon; 
    }
	
    public void renderTo(Graphics2D graphics, float scaling) {
        graphics.setStroke(new BasicStroke(1.0f/scaling));
	graphics.draw(new Ellipse2D.Float(getCentroidX()-radius, getCentroidY()-radius, radius*2, radius*2));
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        Cylinder naught = new Cylinder(radius/scaling, height/scaling, ap);
        TransformGroup cylTG = new TransformGroup();
        Transform3D cylT3d = new Transform3D(q, new Vector3f(getCentroidX()/scaling, getCentroidY()/scaling, getCentroidZ()/scaling), 1);
        cylTG.setTransform(cylT3d);
        cylTG.addChild(naught);
        vinfoBranch.addChild(cylTG);
        vinfoGroup.addChild(vinfoBranch);
    }
}