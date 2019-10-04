package org.tekkotsu.sketch;

import org.tekkotsu.mon.*;

import javax.swing.*;
import javax.imageio.ImageIO;
import javax.swing.SwingConstants;
import javax.swing.JLabel;
import java.awt.Color;
import java.awt.image.BufferedImage;
import java.awt.Component;
import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.FontMetrics;
import java.awt.Cursor;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.RenderingHints;
import javax.swing.JFrame;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowFocusListener;
import java.awt.image.IndexColorModel;
import java.lang.Enum;
import java.util.prefs.Preferences;
import java.io.File;
import java.io.*;
import javax.swing.tree.*;


public class SketchPanel extends VisionPanel 
    implements ActionListener, WindowFocusListener
{
    SketchGUI gui; // reference back to owning SketchGUI
    AffineTransform atrans;
    AffineTransform resultAtrans;

    int imageWidth, imageHeight;
    float scaling = 1;
    float margin; // leave extra space around sketch area for local and world maps
    protected TreePath[] paths;

    boolean drawText;

    Box buttonBox = null;
    Component buttonStrut;

    static Preferences prefs;
     

    // coordinate bounds for the view of the SketchPanel
    float leftBound=0, rightBound=176, topBound=0, bottomBound=144;
     
    protected SketchPanel(SketchGUI _gui, VisionListener listener, SketchGUI.Space _space, 
                           Preferences _prefs, int _imageWidth, int _imageHeight) {
         //         super(listener, _gui, _isCam);
         super(listener, _gui, _space);
         gui = _gui;
         space = _space;
         scaling = 1;
         margin = (space == SketchGUI.Space.cam) ? 1.0f : 1.2f; // leave extra room for local and world
         prefs = _prefs;
         drawText = true;
         mouseX = -1;
         mouseY = -1;
         imageWidth = _imageWidth;
         imageHeight = _imageHeight;
         rightBound = imageWidth;
         bottomBound = imageHeight;
    }        
        
    public void visionUpdated(VisionListener listener) {
         super.visionUpdated(listener);
    }
     
    protected void drawImage(Graphics _g, BufferedImage img, int x, int y, int w, int h) {
        Graphics2D g2d = (Graphics2D)_g;
        AffineTransform origtrans = g2d.getTransform();
        scaling = 1;
        // right now sketches start at (0,0) but this will change someday for worldspace
        // April '06: x and y are probably always 0 and shouldn't be passed in
        int imgLeft = 0; int imgTop = 0;
        if (img != null) {
            if(space == SketchGUI.Space.cam) {
                scaling = java.lang.Math.min(w/(rightBound-leftBound),
                                             h/(bottomBound-topBound));
                atrans = new AffineTransform(scaling, 0, 0, scaling, 
                                             x - leftBound * scaling, 
                                             y - topBound * scaling);
            } else {
                scaling = java.lang.Math.min(w/(bottomBound-topBound),
                                             h/(rightBound-leftBound));
                atrans = new AffineTransform(0, -scaling, -scaling, 0,
                                             bottomBound*scaling, rightBound*scaling);
            }
            // System.out.println("SketchPanel::img width/height = "+img.getWidth() + " " + img.getHeight());
            // System.out.println("Bounds = "+leftBound+" "+rightBound+" "+topBound+" "+bottomBound);
            //             System.out.println("SketchPanel::scaling = "+scaling);

             g2d.transform(atrans);
             resultAtrans = atrans;
             if ( gui.sketchCount > 0 )
                 g2d.drawImage(img,imgLeft,imgTop,img.getWidth(),img.getHeight(),null);
             // add crosshairs if requested
             if (space == SketchGUI.Space.cam && crosshairsEnabled) {
                 g2d.setColor(Color.WHITE);
                 g2d.setXORMode(Color.BLACK);
                 g2d.setStroke(new BasicStroke(0.5f));
                 g2d.drawLine(imgLeft+img.getWidth()/2,imgTop,imgLeft+img.getWidth()/2,imgTop+img.getHeight());
                 g2d.drawLine(imgLeft,imgTop+img.getHeight()/2, imgLeft+img.getWidth(), imgTop+img.getHeight()/2);
                 g2d.setPaintMode();
             }
             
         } else {  // no sketches or shapes available
             g2d.setColor(getBackground());
             g2d.fillRect(x,y,w,h);
             FontMetrics fmet=g2d.getFontMetrics();
             String msg="No image";
             int strw=fmet.stringWidth(msg);
             int strh=fmet.getHeight();
             g2d.setColor(getForeground());
             g2d.drawString(msg,(getSize().width-strw)/2,(getSize().height-strh)/2+fmet.getAscent());
	}
        g2d.transform(gui.Tmat);
	drawFeatures(g2d);

	// undo the transform so we can write world/local space text that isn't flipped
	g2d.setTransform(origtrans);

         // draw image bounds and mouse coordinates
         if (img != null && drawText) {
             FontMetrics fmet=g2d.getFontMetrics();
             if(space == SketchGUI.Space.cam) {
                 // draw bounds and mouse position in cam coordinates
                 g2d.setColor(getForeground());
                 String msg="("+String.valueOf((int)leftBound)+","+String.valueOf((int)topBound)+")";
                 int strw=fmet.stringWidth(msg);
                 g2d.drawString(msg,x,y+fmet.getAscent());
                 msg="("+String.valueOf((int)rightBound)+","+String.valueOf((int)bottomBound)+")";
                 strw=fmet.stringWidth(msg);
                 g2d.drawString(msg,x+w-strw,y+h-fmet.getDescent());
                 if (mouseX >= 0 && mouseY >= 0) {
                     int camX = java.lang.Math.round(mouseX/scaling+leftBound);
                     int camY = java.lang.Math.round(mouseY/scaling+topBound);
                     g2d.setColor(Color.BLUE);  // was YELLOW
                     g2d.drawString("("+camX+","+camY+")", x, y+h-fmet.getDescent());
                     g2d.setColor(Color.YELLOW);
                     g2d.drawString("("+camX+","+camY+")", x+w-strw-5, y+fmet.getAscent());
                     g2d.setColor(getForeground());
                 }
             } else {
                 // draw bounds and mouse position in world coordinates
                 g2d.setXORMode(Color.GRAY);
                 g2d.setColor(getForeground());
                 String msg="("+String.valueOf((int)rightBound)+","+String.valueOf((int)bottomBound)+")";
                 int strw=fmet.stringWidth(msg);
                 g2d.drawString(msg,x,y+fmet.getAscent());
                 msg="("+String.valueOf((int)leftBound)+","+String.valueOf((int)topBound)+")";
                 strw=fmet.stringWidth(msg);
                 g2d.drawString(msg,x+(bottomBound-topBound)*scaling-strw,y+(rightBound-leftBound)*scaling-fmet.getDescent());
                 g2d.setPaintMode();
                 if (mouseX >= 0 && mouseY >= 0) {
                     int worldX = java.lang.Math.round(rightBound-mouseY/scaling);
                     int worldY = java.lang.Math.round(bottomBound-mouseX/scaling);
                     g2d.setColor(Color.BLUE);
                     g2d.drawString("(" + worldX + "," + worldY + ")", x, y+h-fmet.getDescent());
                     g2d.setColor(getForeground());
                 }
             }
             if (idEnabled) {
                 for (int path_i = 0; path_i < paths.length; path_i++) {
                     DefaultMutableTreeNode node = (DefaultMutableTreeNode)(paths[path_i].getLastPathComponent());
                     if (node == null) break;
                     
                     if((node.getUserObject() instanceof SketchOrShapeInfo)) {
                         SketchOrShapeInfo vinfo = (SketchOrShapeInfo)(node.getUserObject());
                         g2d.setColor(ColorConverter.convertColor(vinfo.getColor()));
                         float[] id_coords = new float[2];
                         if (vinfo instanceof ShapeInfo) {
                             id_coords[0] = ((ShapeInfo)vinfo).getIdX();
                             id_coords[1] = ((ShapeInfo)vinfo).getIdY();
                         }
                         /*if (vinfo instanceof EllipseShapeInfo) { // draw id at the centroid
                           id_coords[0] = ((EllipseShapeInfo)vinfo).getCentroidX();
                           id_coords[1] =  ((EllipseShapeInfo)vinfo).getCentroidY();
                           }
                           if (vinfo instanceof PointShapeInfo) { // draw id at the centroid
                             id_coords[0] = ((PointShapeInfo)vinfo).getCentroidX();
                             id_coords[1] =  ((PointShapeInfo)vinfo).getCentroidY();
                         }
                         else if (vinfo instanceof SphereShapeInfo) { // draw id at the centroid
                             id_coords[0] = ((SphereShapeInfo)vinfo).getCentroidX();
                             id_coords[1] =  ((SphereShapeInfo)vinfo).getCentroidY();
                         }
                         else if (vinfo instanceof BlobShapeInfo) { // draw id at the centroid
                             id_coords[0] = ((BlobShapeInfo)vinfo).getCentroidX();
                             id_coords[1] =  ((BlobShapeInfo)vinfo).getCentroidY();
                         }
                         else if(vinfo instanceof LineShapeInfo) { // draw id at the first end point
                             id_coords[0] = ((LineShapeInfo)vinfo).getE1X();
                             id_coords[1] = ((LineShapeInfo)vinfo).getE1Y();
                         }
                         else if(vinfo instanceof PolygonShapeInfo) { // draw id next to the first vertex
                             id_coords[0] = ((PolygonShapeInfo)vinfo).getFirstVertex()[0];
                             id_coords[1] = ((PolygonShapeInfo)vinfo).getFirstVertex()[1];
                         }
                         else if(vinfo instanceof BrickShapeInfo) { // draw id at the centroid
                         id_coords[0] = ((BrickShapeInfo)vinfo).getCentroidX();
                         id_coords[1] = ((BrickShapeInfo)vinfo).getCentroidY();
                         }*/
                             //if (isCam)
                         //                         g2d.transform(AffineTransform.getScaleInstance(.25,.25));
                         //                         System.out.print(id_coords[0] + ", " + id_coords[1] + " => ");
                         gui.Tmat.transform(id_coords, 0, id_coords, 0, 1);
                         if(space == SketchGUI.Space.cam)
                             atrans.transform(id_coords, 0, id_coords, 0, 1);//change this, or no id for cam
                         else
                             atrans.transform(id_coords, 0, id_coords, 0, 1);
                         g2d.drawString(Integer.toString(vinfo.getId()),(int)id_coords[0], (int)id_coords[1]);
                         /*
                         try { g2d.transform(gui.Tmat.createInverse());
                         } catch (java.awt.geom.NoninvertibleTransformException nte) {
                             System.out.println("Error occured trying to inverse Tmat");
                         }
                         */
                     }
                     g2d.setPaintMode();
                 }
             }
         }
     }
     
     public void makeSketchFrame(SketchPanel sketchPanel, String title) {
         JFrame sketchFrame = new JFrame(title);
         Box sketchBox = Box.createVerticalBox();
         buttonBox = Box.createHorizontalBox();
         buttonBox.setBackground(Color.red);
         JButton cloneButton = new JButton("Clone");
         cloneButton.addActionListener(sketchPanel);
         cloneButton.setActionCommand("clone");
         cloneButton.setEnabled(true);
         buttonBox.add(cloneButton);
         
         buttonBox.add(Box.createHorizontalStrut(10));

         JButton saveButton = new JButton("Save Image");
         saveButton.setAlignmentX(0.5f);
         saveButton.addActionListener(sketchPanel);
         saveButton.setActionCommand("saveimg");
         saveButton.setEnabled(true);
         saveButton.setToolTipText("Saves sketch to a file - use .jpg or .png extension to choose format;");
         buttonBox.add(saveButton);

         buttonBox.add(Box.createHorizontalStrut(10));
        
         if(space == SketchGUI.Space.cam) {
                 //         if (space == 1) { // only show crosshairs for the local model
             JCheckBox xhairsBox = new JCheckBox("Crosshairs", false);
             xhairsBox.addActionListener(sketchPanel);
             xhairsBox.setActionCommand("xhairs");
             xhairsBox.setEnabled(true);
             buttonBox.setVisible(windowHasFocus);
             buttonBox.add(xhairsBox);
         }

         JCheckBox idBox = new JCheckBox("ID", false);
         idBox.addActionListener(sketchPanel);
         idBox.setActionCommand("id");
         idBox.setEnabled(true);
         buttonBox.setVisible(windowHasFocus);
         buttonBox.add(idBox);


         Box bigButtonBox = Box.createHorizontalBox();

         bigButtonBox.add(buttonBox);
         buttonStrut = Box.createVerticalStrut((int)cloneButton.getPreferredSize().getHeight());
         buttonStrut.setVisible(true);
         bigButtonBox.add(buttonStrut);
         sketchBox.add(bigButtonBox);
         sketchBox.add(sketchPanel);
         sketchFrame.getContentPane().add(sketchBox);
         sketchFrame.setCursor(new Cursor(Cursor.CROSSHAIR_CURSOR));
         sketchFrame.pack();
         sketchFrame.addWindowFocusListener(sketchPanel);
         sketchFrame.setVisible(true);
     }

     public void actionPerformed(ActionEvent e) {
         if (e.getActionCommand().compareTo("clone")==0) {
             SketchPanel sketchPanel=new SketchPanel(gui, _listener, space, prefs, 
                                                     imageWidth, imageHeight);
             sketchPanel.setMinimumSize(new Dimension(imageWidth/2, imageHeight/2));
             sketchPanel.setPreferredSize(new Dimension(imageWidth*2, imageHeight*2));
             sketchPanel.setLockAspectRatio(true);
             ++gui.panelCount;
             sketchPanel.makeSketchFrame(sketchPanel, 
                                         gui.panelTitle+" "+gui.panelCount+": "+gui.host);
             sketchPanel.setBounds(leftBound, rightBound, topBound, bottomBound);
             sketchPanel.imageUpdated(_image,paths);                
         } 
         else if(e.getActionCommand().compareTo("saveimg")==0) {
             File cursavepath = new File(prefs.get("cursavepath",""));
             JFileChooser dia=new JFileChooser(cursavepath);
             dia.setDialogTitle("Save Image...");
             Component cur=this;
             while(cur.getParent()!=null)
                 cur=cur.getParent();
             if(dia.showSaveDialog(cur)==JFileChooser.APPROVE_OPTION) {
                 prefs.put("cursavepath",dia.getCurrentDirectory().getPath());
                 String base=dia.getSelectedFile().getName();
                 String format;
                 if(base.lastIndexOf('.')==-1) {
                     format="png";
                 } else {
                     int i=base.lastIndexOf(".");
                     format=base.substring(i+1);
                     base=base.substring(0,i);
                 }
                 try {
                     FileOutputStream fileout=new FileOutputStream(dia.getSelectedFile().getParent()+File.separator+base+"."+format);
                     //ImageIO.write(sketchPanel.getListener().getImage(),format,fileout);
                     ImageIO.write(getSaveImage(),format,fileout);
                 } catch(IOException ex) {}
             }
         }
         else if (e.getActionCommand().compareTo("xhairs") == 0) {
             crosshairsEnabled = ((JCheckBox)(e.getSource())).isSelected();
             repaint();
         }
         else if (e.getActionCommand().compareTo("id") == 0) {
             idEnabled = ((JCheckBox)(e.getSource())).isSelected();
             repaint();
         }         
     }

    // stretches the view of the SketchPanel to accomodate the specified object 
    public void scaleToSketchOrShape(SketchOrShapeInfo oinfo) {
	Point2D.Float topleft = new Point2D.Float();
	topleft.setLocation(oinfo.getLeft(), oinfo.getTop());
	Point2D.Float bottomright = new Point2D.Float();
	bottomright.setLocation(oinfo.getRight(), oinfo.getBottom());
	Point2D.Float tl = new Point2D.Float();
	gui.Tmat.transform(topleft,tl);
	topBound = (float)Math.min(topBound, margin*tl.getY());
	leftBound = (float)Math.min(leftBound, margin*tl.getX());
	Point2D.Float br = new Point2D.Float();
	gui.Tmat.transform(bottomright,br);
	bottomBound = (float)Math.max(bottomBound, margin*br.getY());
	rightBound = (float)Math.max(rightBound, margin*br.getX());
    }

    public void imageUpdated(BufferedImage sketchImage, TreePath[] newPaths) {
        _image = sketchImage;
        paths = newPaths;
        repaint();
    }
    
     public void loseFocus() {
         windowHasFocus = false;
         buttonBox.setVisible(false);
         buttonStrut.setVisible(true);
         repaint();
     }

     public void windowGainedFocus(WindowEvent e) {
         gui.setCurrentSketchPanel(this,paths);
         windowHasFocus = true;
         if (buttonBox != null) {
             buttonBox.setVisible(true);
             buttonStrut.setVisible(true);
         }
         repaint();
     }

     // Don't use the standard lose focus, because it only actually should lose 
     // concept of focus when a different sketchpanel gains focus
     public void windowLostFocus(WindowEvent e) {}

     public BufferedImage getSaveImage() {
         int width=0, height=0;
         //if (isCam) {
         System.out.println("space= " + space);
         //         if(space == SketchGUI.Space.cam) {
         if(space == SketchGUI.Space.cam) {
             //         if (space==1) {
             width = (int)(rightBound-leftBound);
             height = (int)(bottomBound-topBound);
         } else {
             width = (int)(bottomBound-topBound);
             height = (int)(rightBound-leftBound);
         }
         BufferedImage saveimg = new BufferedImage(width,height,_image.getType());
         boolean oldxhairs = crosshairsEnabled;
         boolean olddrawtext = drawText;
         crosshairsEnabled = false;
         drawText = false;
         Graphics g = saveimg.createGraphics();
         g.setColor(Color.WHITE);
         g.fillRect(0,0,width,height);
         drawImage(g,_image,0,0,width,height);
         //         drawFeatures((Graphics2D)g);
         crosshairsEnabled = oldxhairs;
         drawText = olddrawtext;
         return saveimg;
     }

     public void drawFeatures(Graphics2D g) {
        if (paths == null) return;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,RenderingHints.VALUE_STROKE_PURE);

        for (int path_i = 0; path_i < paths.length; path_i++) {
             DefaultMutableTreeNode node = (DefaultMutableTreeNode)(paths[path_i].getLastPathComponent());
             if (node == null) return;
            
             // check for dummy node; it's used for "camspace" or "worldspace" folder
             if((node.getUserObject() instanceof SketchOrShapeInfo)) {
                 SketchOrShapeInfo vinfo = (SketchOrShapeInfo)(node.getUserObject());
                 if (vinfo instanceof ShapeInfo) {
                     g.setColor(ColorConverter.convertColor(vinfo.getColor()));
                     vinfo.renderTo(g, scaling);
                 }
             }
         }
     }

     public void setBounds(float newLeft, float newRight, float newTop, float newBottom) {
         leftBound = newLeft;
         rightBound = newRight;
         topBound = newTop;
         bottomBound = newBottom;
     }
}
