package org.tekkotsu.sketch;
import com.sun.j3d.utils.universe.*;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.geom.*;
import javax.media.j3d.*;
import com.sun.j3d.utils.geometry.Sphere;

import javax.vecmath.*;

// stores info for a Shape, to use as UserObject for DefaultMutableTreeNode

public class ShapeInfo extends SketchOrShapeInfo {
    float centroidx, centroidy, centroidz;
    boolean obstacle;
    boolean landmark;
    boolean useDefaultSelection, useDefaultInversion;
    final static String icon_path = "org/tekkotsu/sketch/icons/";
    
    public ShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
		     float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark) {
	super(_gui, _id, _parentId, _name, _color);
	centroidx = _centroidx;
	centroidy = _centroidy;
	centroidz = _centroidz;
	obstacle = _obstacle;
	landmark = _landmark;
	useDefaultSelection = true;
	useDefaultInversion = true;
    }


    // if no specific renderer defined, just plot a point at centroid
    public void renderTo(Graphics2D graphics, float scaling) {
	graphics.setColor(color);
	graphics.drawOval((int)centroidx,(int)centroidy,5,5);
    }

    public void renderTo(Graphics graphics, Rectangle2D.Double r) {
	double xscale = r.width;
	double yscale = r.height;
	graphics.setColor(color);
	graphics.drawOval((int)(centroidx*xscale+r.x),(int)(centroidy*yscale+r.y),(int)(5*xscale),(int)(5*yscale));	
    }
    
    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        System.out.println("Rendering 3D"+id);       
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
        final float radius = 0.015f;
        Sphere sph = new Sphere(radius, ap);
        TransformGroup newTrans = new TransformGroup();
        Transform3D t3d = new Transform3D();
        t3d.setTranslation(new Vector3f(centroidx/scaling, centroidy/scaling, 0));
        newTrans.setTransform(t3d);
        newTrans.addChild(sph);
        vinfoBranch.addChild(newTrans);
        vinfoGroup.addChild(vinfoBranch);
    }

    public float getCentroidX() { return centroidx; }
    public float getCentroidY() { return centroidy; }
    public float getCentroidZ() { return centroidz; }

    // Specific call to get the coordinates of where to display the shape's Id
    // defaults to the centroid, override for other behavior
    public float getIdX() { return centroidx; }
    public float getIdY() { return centroidy; }

    public void setUseDefaultSelection(boolean useSelection) { useDefaultSelection = useSelection; }
    public boolean getUseDefaultSelection() { return useDefaultSelection; }
    public void setUseDefaultInversion(boolean useInversion) { useDefaultInversion = useInversion; }
    public boolean getUseDefaultInversion() { return useDefaultInversion; }
}