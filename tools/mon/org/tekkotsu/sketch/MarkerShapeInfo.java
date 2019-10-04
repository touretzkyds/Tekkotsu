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

// stores info for a MarkerShape
public class MarkerShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"marker.png");
    static Icon inv_icon = new ImageIcon(icon_path+"markerinv.png");
    
    public String type;
    public String description;

    public MarkerShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			   float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			   String _type, String _description)
    {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	type = _type;
	description = _description;
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
	return super.toString() + " " + description + ": " + type;
    }

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
	    radius= java.lang.Math.max(bounds.width,bounds.height)/30;
	else
	    radius = 10;
	
	graphics.setStroke(new BasicStroke(1.0f/scaling));
	graphics.drawOval((int)centroidx-radius/2, (int)centroidy-radius/2, radius, radius);
	graphics.fillArc((int)centroidx-radius/2, (int)centroidy-radius/2, radius, radius, 0, 90);
	graphics.fillArc((int)centroidx-radius/2, (int)centroidy-radius/2, radius, radius, -180, 90);
    }

}

