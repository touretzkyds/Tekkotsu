package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import java.util.ArrayList;
import static java.lang.Math.*;

public class PointPick extends JComponent implements MouseListener, MouseMotionListener {
	
	static public void main(String s[]) {
		/*JFrame frame=new JFrame("Point Pick Test");
		 frame.setSize(new Dimension(300, 300)); 
		 PointPick pp=new PointPick(true, false);
		 frame.getContentPane().setLayout(new BorderLayout());
		 frame.getContentPane().add(pp,BorderLayout.CENTER);
		 frame.addWindowListener(new WindowAdapter() {
		 public void windowClosing(WindowEvent e) { System.exit(0); }
		 });
		 frame.setVisible(true);*/
	}
	
	float x=0;
	float y=0;
	float x_orig = 0.0f;
	float y_orig = 0.0f;
	float scale = 1;
	float gx=0f;
	float gy=0f;
	int dotSize=8;
	float tracksize=14;
	float armJointCoord[][];
	float gripperAngles[];
	boolean armflag = false;
	ArrayList<PointPickedListener> listeners = new ArrayList<PointPickedListener>();
	ArrayList<Point2D.Float> goodPts = new ArrayList<Point2D.Float>();
	boolean isCirc=true;
	boolean gripper=false;
	boolean mouseDown = false;
	
	public interface PointPickedListener {
		public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp);
	}
	
	public void addPointPickedListener(PointPickedListener ppl) {
		listeners.add(ppl);
	}
	
	public void removePointPickedListener(PointPickedListener ppl) {
		listeners.remove(listeners.indexOf(ppl));
	}
	
	public void firePointPicked(Point2D.Float p, MouseEvent e) {
		for(int i=0; i<listeners.size(); i++)
			(listeners.get(i)).pointPicked(p,e,this);
	}
	
	public PointPick(boolean circ) {
		init(circ,false);
	}
	
	public PointPick(boolean circ, boolean hasArm) {		
		init(circ,hasArm);
	}
	
	public PointPick(boolean circ, boolean hasArm, float armJtCrd[][]) {
		armJointCoord = armJtCrd;
		init(circ,hasArm);
	}
	
	protected void init(boolean circ, boolean hasArm) {
		armflag = hasArm;
		
		addMouseListener(this);
		addMouseMotionListener(this);
		
		isCirc = circ;
		setOpaque(!circ);
		repaint();
	}
	
	public void paint(Graphics graphics) {
		super.paint(graphics);
		int w=getWidth()-1;
		int h=getHeight()-1;
		
		// background
		graphics.setColor(Color.WHITE);
		graphics.fillRect(0, 0, w, h);
		
		// border
		graphics.setColor(Color.BLACK);
		graphics.drawRect(0, 0, w, h);
		
		// paint on successful points
		if (goodPts.size() > 0) {
			graphics.setColor(isEnabled() ? Color.GREEN : new Color(135,255,135));
			for (int i = 0; i < goodPts.size(); i++)
				graphics.fillOval(modelToScreenX(goodPts.get(i).x),
								  modelToScreenY(goodPts.get(i).y), 3, 3);
		}
		
		// draw grid
		graphics.setColor(Color.LIGHT_GRAY);
		if (isCirc) {
			// circles
			graphics.drawOval(w/8, h/8, 3*w/4, 3*h/4);
			graphics.drawOval(w/4, h/4, w/2, h/2);
			graphics.drawOval(3*w/8, 3*h/8, w/4, h/4);
		}
		else {
			// rectangles
			graphics.drawRect(w/8, h/8, 3*w/4, 3*h/4);
			graphics.drawRect(w/4, h/4, w/2, h/2);
			graphics.drawRect(3*w/8, 3*h/8, w/4, h/4);
		}
		
		// draw axes
		graphics.setColor(isEnabled()?Color.BLACK:Color.GRAY);
		graphics.drawLine(0,w/2,h,w/2);
		graphics.drawLine(h/2,0,h/2,w);
		
		// draw tracker
		if (mouseDown) {
			graphics.setColor(Color.GREEN);
			graphics.fillOval(modelToScreenX(x) - w/25,
							  modelToScreenY(y) - h/25,
							  w/13,
							  h/13);
			
			graphics.setColor(Color.WHITE);
			graphics.fillOval(modelToScreenX(x) - w/40,
							  modelToScreenY(y) - h/40,
							  w/20,
							  h/20);
		}
		
		// draw dots
		if (isEnabled()) {
			graphics.setColor(Color.RED);
			graphics.fillOval(modelToScreenX(x) - (int)dotSize/2, modelToScreenY(y) - dotSize/2, dotSize, dotSize); // red
			
			if (armflag) {
				graphics.setColor(Color.GREEN);
				graphics.fillOval(modelToScreenX(gx) - dotSize/2, modelToScreenY(gy) - dotSize/2, dotSize, dotSize); // green
				graphics.setColor(Color.BLACK);
				graphics.drawOval(modelToScreenX(gx) - dotSize/2, modelToScreenY(gy) - dotSize/2, dotSize, dotSize); // green
			}
		}
		
		// arm stuff
		if (armflag) {
			// Blue links, or gray if disabled
			graphics.setColor(isEnabled()?Color.BLUE:Color.GRAY);
			
			// Draw the arm's links
			for(int counter = 0; counter < armJointCoord.length; counter ++){
				if (counter == 0) {
					graphics.drawLine(modelToScreenX(0f),
									  modelToScreenY(0f),
									  modelToScreenX(armJointCoord[counter][0]),
									  modelToScreenY(armJointCoord[counter][1]));
				}
				else {
					graphics.drawLine(modelToScreenX(armJointCoord[counter-1][0]),
									  modelToScreenY(armJointCoord[counter-1][1]),
									  modelToScreenX(armJointCoord[counter][0]),
									  modelToScreenY(armJointCoord[counter][1]));
				}
			}
			// if we have a gripper, deal with that.
			if (gripper) {
				float length, rotationAngle, originX, originY;
				
				length = 0.12f;
				int lastCoord = armJointCoord.length - 1;
				originX = (lastCoord < 1) ? 0.0f : armJointCoord[lastCoord-1][0];
				originY = (lastCoord < 1) ? 0.0f : armJointCoord[lastCoord-1][1];
				rotationAngle = (float)atan2(originY - armJointCoord[lastCoord][1], originX - armJointCoord[lastCoord][0]);
				AffineTransform rotation = AffineTransform.getRotateInstance(rotationAngle);
				AffineTransform translation = AffineTransform.getTranslateInstance(armJointCoord[lastCoord][0], armJointCoord[lastCoord][1]);
				
				// make a gripper at the origin
				Point2D extendLeft = new Point2D.Float(0, -2*length/3);
				Point2D extendRight = new Point2D.Float(0, 2*length/3);
				
				Point2D leftFinger = new Point2D.Float((float)((length * (float)cos(PI-gripperAngles[0])) + extendLeft.getX()), (float)((length * (float)sin(PI-gripperAngles[0])) + extendLeft.getY()));
				Point2D rightFinger = new Point2D.Float((float)((length * (float)cos(PI-gripperAngles[1])) + extendRight.getX()), (float)((length * (float)sin(PI-gripperAngles[1])) + extendRight.getY()));
				
				// rotation to get it into place
				extendLeft = rotation.transform(extendLeft, null);
				extendRight = rotation.transform(extendRight, null);
				leftFinger = rotation.transform(leftFinger, null);
				rightFinger = rotation.transform(rightFinger, null);
				
				// shift over to the proper position
				extendLeft = translation.transform(extendLeft, null);
				extendRight = translation.transform(extendRight, null);
				leftFinger = translation.transform(leftFinger, null);
				rightFinger = translation.transform(rightFinger, null);
				
				graphics.drawLine(modelToScreenX(armJointCoord[lastCoord][0]),
								  modelToScreenY(armJointCoord[lastCoord][1]),
								  modelToScreenX((float)extendLeft.getX()),
								  modelToScreenY((float)extendLeft.getY()));
				graphics.drawLine(modelToScreenX(armJointCoord[lastCoord][0]),
								  modelToScreenY(armJointCoord[lastCoord][1]),
								  modelToScreenX((float)extendRight.getX()),
								  modelToScreenY((float)extendRight.getY()));
				// from bases to tip of fingers
				graphics.drawLine(modelToScreenX((float)extendLeft.getX()),
								  modelToScreenY((float)extendLeft.getY()),
								  modelToScreenX((float)leftFinger.getX()),
								  modelToScreenY((float)leftFinger.getY()));
				graphics.drawLine(modelToScreenX((float)extendRight.getX()),
								  modelToScreenY((float)extendRight.getY()),
								  modelToScreenX((float)rightFinger.getX()),
								  modelToScreenY((float)rightFinger.getY()));
			}
		}
	}
	// new data members
	public void setOrigPoint(float x, float y){
		this.x_orig = x;
		this.y_orig = y; 
	}
	public void setScale(float s) {
		this.scale = s;
	}
	// accessor methods... for what? I guess good object-oriented design.
	public float getOrigPointX() {
		return this.x_orig;
	}
	public float getOrigPointY() {
		return this.y_orig;
	}
	public float getScale() {
		return this.scale;
	}
	
	public void setBounds(int x, int y, int w, int h) {
		if(w>h)
			w=h;
		if(h>w)
			h=w;
		super.setBounds(x,y,w,h);
	}
	
	/* Mouse Events */
	
	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent e) {}
	public void mousePressed(MouseEvent e) {
		if (isEnabled()) {
			Point2D.Float mp=screenToModel(e.getPoint());
			mouseDown = true;
			doSetPoint(mp);
			firePointPicked(mp,e);
			repaint();
		}
	}
	public void mouseReleased(MouseEvent e) {
		if (isEnabled()) {
			mouseDown = false;
			repaint();
		}
	}
	public void mouseExited(MouseEvent e) {}
	
	public void mouseMoved(MouseEvent e) {}
	public void mouseDragged(MouseEvent e) {
		if(isEnabled()) {
			Point2D.Float mp=screenToModel(e.getPoint());
			doSetPoint(mp);
			firePointPicked(mp,e);
			repaint();
		}
	}
	
	/* End mouse events */
	
	public Point2D.Float getPoint() { return new Point2D.Float(x,y); }
	public float getXValue() { return x; }
	public float getYValue() { return y; }
	
	public void setPoint(float x, float y) {
		doSetPoint(x,y);
		firePointPicked(new Point2D.Float(x,y),null);
	}
	
	public void doSetArm(float armJtCrd[][]){
		armJointCoord = armJtCrd;
		repaint();
	}
	
	public void doSetGripper(float fgrJtAgs[]) {
		gripperAngles = fgrJtAgs;
		repaint();
	}
	
	public void setHasGripper(boolean g) {
		gripper = g;
		gripperAngles = new float[2];
	}
	
	public boolean hasGripper() {
		return gripper;
	}
	
	protected Point2D.Float screenToModel(Point p) {
		return screenToModel(p.x,p.y);
	}
	protected Point2D.Float screenToModel(int x, int y) {
		float fx=(x/(float)getWidth()*2 - 1 - x_orig)/scale;
		float fy=(1-y/(float)getHeight()*2 - y_orig)/scale;
		if(isCirc) {
			if(fx*fx+fy*fy>1) {
				double a=atan2(fy,fx);
				fx=(float)cos(a);
				fy=(float)sin(a);
			}
		}
		else {
			if(fx>1)
				fx=1;
			else if(fx<-1)
				fx=-1;
			if(fy>1)
				fy=1;
			else if(fy<-1)
				fy=-1;
		}
		return new Point2D.Float(fx,fy);
	}
	protected Point modelToScreen(Point2D.Float p) {
		return modelToScreen(p.x,p.y);
	}
	protected Point modelToScreen(float x, float y) {
		return new Point(modelToScreenX(x),modelToScreenY(y));
	}
	
	protected int modelToScreenX(float x) {
		return (int)(getWidth()*(1+((x*scale)+x_orig))/2);
	}
	
	protected int modelToScreenY(float y) {
		return (int)(getHeight()*(1-((y*scale)+y_orig))/2);
	}
	
	public void paintPoint(float _x, float _y) {
		Point2D.Float p = new Point2D.Float(_x,_y);
		goodPts.add(p);
	}
	
	public void clearReachablePoints() {
		goodPts.clear();
	}
	
	public void doSetPoint(Point2D.Float p) {
		doSetPoint(p.x,p.y);
	}
	public void doSetPoint(float x, float y) {
		this.x=x;
		this.y=y;
	}
	
	public void doSetGPoint(Point2D.Float p) {
		doSetGPoint(p.x,p.y);
	}
	
	public void doSetGPoint(float x, float y) {
		this.gx=x;
		this.gy=y;
	}
}
