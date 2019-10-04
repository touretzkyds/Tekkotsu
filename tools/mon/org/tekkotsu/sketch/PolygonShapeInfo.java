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
import com.sun.j3d.utils.geometry.Box;
import com.sun.j3d.utils.geometry.Sphere;

// stores info for a PolygonShape
public class PolygonShapeInfo extends ShapeInfo {
	static Icon icon = new ImageIcon(icon_path+"poly.png");
	static Icon inv_icon = new ImageIcon(icon_path+"polyinv.png");
	// x/y coordinates of endpoints, with variances
    int num_vertices;
    float[][] vertices;
    boolean end1_valid, end2_valid;
    
    public PolygonShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			    float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			    int _num_vertices, float[][] _vertices,
			    boolean _end1_valid, boolean _end2_valid) {			
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	num_vertices = _num_vertices;
	vertices = _vertices;
	end1_valid = _end1_valid;
	end2_valid = _end2_valid;
    }
	
    // returns left-most coordinate of object
    public float getLeft() { 
	float min_x = vertices[0][0];
	for (int i = 1; i < num_vertices; i++)
	    if (min_x > vertices[i][0])
		min_x = vertices[i][0];
	return min_x; 
    }
    // returns right-most coordinate of object
    public float getRight() {
	float max_x = vertices[0][0];
	for (int i = 1; i < num_vertices; i++)
	    if (max_x < vertices[i][0])
		max_x = vertices[i][0];
	return max_x; 
    }
    // returns top-most coordinate of object
    public float getTop() { 
	float min_y = vertices[0][1];
	for (int i = 1; i < num_vertices; i++)
	    if (min_y > vertices[i][1])
		min_y = vertices[i][1];
	return min_y;     
    }
    // returns bottom-most coordinate of object
    public float getBottom() { 
	float max_y = vertices[0][1];
	for (int i = 1; i < num_vertices; i++)
	    if (max_y < vertices[i][1])
		max_y = vertices[i][1];
	return max_y; 
    }
    public String toString() {
	String vtx_coords = "(" + vertices[0][0] + "," + vertices[0][1] + ")";
	for (int i = 1; i < num_vertices; i++)
	    vtx_coords += "->(" + vertices[i][0] + "," + vertices[i][1] + ")";
	return (super.toString() + " " +  vtx_coords);
    }    
    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	return icon; 
    }
    
    public void renderTo(Graphics2D graphics, float scaling) {
	graphics.setStroke(new BasicStroke(1.0f/scaling));
	for (int i = 0; i < num_vertices-1; i++)
	    graphics.draw(new Line2D.Float(vertices[i][0],vertices[i][1],vertices[i+1][0],vertices[i+1][1]));

	final int radius = (int) (10.0/scaling);
	if (end1_valid)
	    graphics.fillOval(Math.round(vertices[0][0]-radius/2),
			      Math.round(vertices[0][1]-radius/2), radius, radius);

	if (end2_valid)
	    graphics.fillOval(Math.round(vertices[num_vertices-1][0]-radius/2),
			      Math.round(vertices[num_vertices-1][1]-radius/2), radius, radius);
    }
    
    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        for (int i = 0; i < num_vertices - 1; i++)
        {
            float e1x = vertices[i][0];
            float e2x = vertices[i + 1][0];
            float e1y = vertices[i][1];
            float e2y = vertices[i + 1][1];
            float distance = (float)Math.sqrt(Math.pow(e2x - e1x, 2) + Math.pow(e2y - e1y, 2));
            Box lineBox = new Box(distance/2000, 0.005f, 0.008f, ap);
            float rot = (float)Math.atan2((e2y - e1y), (e2x - e1x));
            float midPointx = (e2x + e1x) / 2;
            float midPointy = (e2y + e1y) / 2;
            TransformGroup lineTrans = new TransformGroup();
            Transform3D l3d = new Transform3D();
            l3d.rotZ(rot);
            l3d.setTranslation(new Vector3f(midPointx / scaling, midPointy / scaling, 0.008f));
            lineTrans.setTransform(l3d);
            lineTrans.addChild(lineBox);
            vinfoBranch.addChild(lineTrans);
        }
        final float radius = 0.03f;
        if (end1_valid) {
            Sphere sph1 = new Sphere(radius, ap);
            TransformGroup sphTrans1 = new TransformGroup();
            Transform3D sph3d1 = new Transform3D();
            sph3d1.setTranslation(new Vector3f(vertices[0][0]/scaling, vertices[0][1]/scaling, 0));
            sphTrans1.setTransform(sph3d1);
            sphTrans1.addChild(sph1);
            vinfoBranch.addChild(sphTrans1);
        }
        if (end2_valid) {
            Sphere sph2 = new Sphere(radius, ap);
            TransformGroup sphTrans2 = new TransformGroup();
            Transform3D sph3d2 = new Transform3D();
            sph3d2.setTranslation(new Vector3f(vertices[num_vertices - 1][0]/scaling, vertices[num_vertices - 1][1]/scaling, 0));
            sphTrans2.setTransform(sph3d2);
            sphTrans2.addChild(sph2);
            vinfoBranch.addChild(sphTrans2);
        }
        vinfoGroup.addChild(vinfoBranch);
    }

    public float[] getFirstVertex() {
	return vertices[0];
    }
    
    public float getIdX() { return vertices[0][0]; }
    public float getIdY() { return vertices[0][1]; }
}