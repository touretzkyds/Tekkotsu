package org.tekkotsu.sketch;

import java.awt.Graphics2D;
import java.awt.Graphics;
import java.awt.Color;
import java.awt.Font;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.BasicStroke;
import javax.media.j3d.*;
import javax.vecmath.*;
import com.sun.j3d.utils.geometry.Sphere;

// stores info for a LineShape
public class PointShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"point.png");
    static Icon inv_icon = new ImageIcon(icon_path+"pointinv.png");
   
    public PointShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			  float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
    }

    // returns left-most coordinate of object
    public float getLeft() { return centroidx; }
    // returns right-most coordinate of object
    public float getRight() { return centroidx; }
    // returns top-most coordinate of object
    public float getTop() { return centroidy; }
    // returns bottom-most coordinate of object
    public float getBottom() { return centroidy; }
   
    public String toString() {
	String _point = "point=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	return (super.toString() + " " + _point); }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	// TODO: to use constant size pixels, get the graphics' AffineTransform,
	// apply it to [1,0,0] and scale by the inverse
	float radius = Math.max(2.0f, 2/scaling);
	graphics.fill(new Ellipse2D.Float(centroidx-radius, centroidy-radius, radius*2, radius*2));
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        System.out.println("Rendering 3D"+id);    
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        final float radius = 0.015f;
        Sphere point = new Sphere(radius, ap);
        TransformGroup pointTG = new TransformGroup();
        Transform3D pointT3d = new Transform3D();
        pointT3d.setTranslation(new Vector3f(centroidx/scaling, centroidy/scaling, centroidz/scaling + 0));
        pointTG.setTransform(pointT3d);
        pointTG.addChild(point);
        vinfoBranch.addChild(pointTG);
        vinfoGroup.addChild(vinfoBranch);
    }

}

