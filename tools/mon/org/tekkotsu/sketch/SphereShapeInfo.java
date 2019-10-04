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

// stores info for a EllipseShape
// note that ellipse center is same as centroid
public class SphereShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon("org/tekkotsu/sketch/icons/ellipse.png");
    static Icon inv_icon = new ImageIcon("org/tekkotsu/sketch/icons/ellipseinv.png");
    float radius; // length of semimajor axes

    public SphereShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			   float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			   float _radius) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
		radius = _radius;
	}

	// returns left-most coordinate of object
	public float getLeft() { return centroidx-radius; }
	// returns right-most coordinate of object
	public float getRight() { return centroidx+radius; }
	// returns top-most coordinate of object
	public float getTop() { return centroidy-radius; }
	// returns bottom-most coordinate of object
	public float getBottom() { return centroidy+radius; }


	public String toString() {
		return (super.toString() + " center=(" + centroidx + "," + centroidy + "," + centroidz + ")"
			+ ", radius=" + radius);
	}

	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    return icon; 
	}

    public void renderTo(Graphics2D graphics, float scaling) {
	graphics.setStroke(new BasicStroke(1.0f));
	graphics.drawOval((int)(getCentroidX()-radius+1), 
			  (int)(getCentroidY()-radius+1),
			  (int)(radius*2), (int)(radius*2));
    }

	public float getRadius() { return radius; }
}

