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
import com.sun.j3d.utils.geometry.Cone;
import javax.vecmath.*;

// stores info for a LocalizationParticleShape
public class LocalizationParticleShapeInfo extends ShapeInfo {

    static Icon icon = new ImageIcon(icon_path+"locpart.png");
    static Icon inv_icon = new ImageIcon(icon_path+"locpartinv.png");

    public LocalizationParticleShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
					 float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
					 float _orient, float _weight) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	orientation = _orient;
	weight = _weight;

    }
	
    float orientation;
    float weight;

    // returns left-most coordinate of object
    public float getLeft() { return centroidx; }
    // returns right-most coordinate of object
    public float getRight() { return centroidx; }
    // returns top-most coordinate of object
    public float getTop() { return centroidy; }
    // returns bottom-most coordinate of object
    public float getBottom() { return centroidy; }
    
    public String toString() {
	String _particle = "wt=" + weight + " (" + centroidx + ", " + centroidy + ") th=" + orientation;
	return super.toString() + " " + _particle; }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	Rectangle bounds = graphics.getClipBounds();
	int radius;
	if (bounds != null)
	    radius= java.lang.Math.max(4,java.lang.Math.max(bounds.width,bounds.height)/100);  // was max(4,...)
	else
	    radius = 4;  // was 4
	graphics.setStroke(new BasicStroke(6.0f));  // was 1.0f
	graphics.fillOval((int)centroidx-radius/2, (int)centroidy-radius/2, radius, radius);
	float dx = (float)java.lang.Math.cos(orientation)*radius*2; // was *3
	float dy = (float)java.lang.Math.sin(orientation)*radius*2; // was *3
	graphics.drawLine((int)centroidx,(int)centroidy,(int)(centroidx+dx),(int)(centroidy+dy));
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        Cone particle = new Cone(0.01f, 0.03f);
        particle.setAppearance(ap);
        TransformGroup newTrans = new TransformGroup();
        Transform3D t3d = new Transform3D();
        t3d.rotZ(orientation - ((90 * Math.PI)/180));
        t3d.setTranslation(new Vector3f(centroidx/scaling, centroidy/scaling, 0.01f));
        newTrans.setTransform(t3d);
        newTrans.addChild(particle);
        vinfoBranch.addChild(newTrans);
        vinfoGroup.addChild(vinfoBranch);
    }
}