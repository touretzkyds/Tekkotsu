import javax.swing.*;
import javax.swing.undo.UndoableEditSupport;
import javax.swing.undo.AbstractUndoableEdit;
import javax.swing.event.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;

public class ImageSelectionManager extends JComponent implements MouseListener, MouseMotionListener, KeyListener
{
	final static float selectionStrokeWidth=1.5f;
	Color fillColor;
	Color strokeColor;
	UndoableEditSupport undoListeners;
	
	class AddEdit extends AbstractUndoableEdit {
		ImageSelectionManager src;
		Area addition;
		Area previous;
		AddEdit(ImageSelectionManager src, Area previous, Area addition) {
			super();
			this.src=src;
			this.previous=(Area)previous.clone();
			this.addition=addition;
		}
		public void redo() {
			super.redo();
			src.selection=(Area)previous.clone();
			src.selection.add(addition);
			src.repaint();
			src.fireSelectionChanged();
		}
		public void undo() {
			super.undo();
			src.selection=(Area)previous.clone();
			src.repaint();
			src.fireSelectionChanged();
		}
	}
	
	class SubtractEdit extends AbstractUndoableEdit {
		ImageSelectionManager src;
		Area removal;
		Area previous;
		SubtractEdit(ImageSelectionManager src, Area previous, Area removal) {
			super();
			this.src=src;
			this.previous=(Area)previous.clone();
			this.removal=removal;
		}
		public void redo() {
			super.redo();
			src.selection=(Area)previous.clone();
			src.selection.subtract(removal);
			src.repaint();
			src.fireSelectionChanged();
		}
		public void undo() {
			super.undo();
			src.selection=(Area)previous.clone();
			src.repaint();
			src.fireSelectionChanged();
		}
	}
	
	class ReplaceEdit extends AbstractUndoableEdit {
		ImageSelectionManager src;
		Area next;
		Area previous;
		ReplaceEdit(ImageSelectionManager src, Area previous, Area next) {
			super();
			this.src=src;
			this.previous=(Area)previous.clone();
			this.next=(next==null) ? null : (Area)next.clone();
		}
		public void redo() {
			super.redo();
			if(next==null) {
				src.clear();
			} else {
				src.selection=(Area)next.clone();
				src.repaint();
				src.fireSelectionChanged();
			}
		}
		public void undo() {
			super.undo();
			src.selection=(Area)previous.clone();
			src.repaint();
			src.fireSelectionChanged();
		}
	}
	
	Area selection; //the current full selection -- may contain disjoint areas; coordinates are stored in the range 0-1 (window size independent)
	
	GeneralPath selInProgress; //a path in the process of being drawn (null if mouse is not down); coordinates are stored in the range 0-1 (window size independent)
	Point lastPos; // the screen X position of the last mouse event, so we know where the polyInProgress end is at
	boolean curIsAdd; //true if the current selection in progress is a 'union' operation; if not curIsSubtract as well, is 'replace'
	boolean curIsSubtract; //true if the current selection in progress is a 'subtract' operation; if not curIsAdd as well, is 'replace'
	ReplaceEdit replaceInProgress; //non-null after the current selection has been replaced (only happens once mouse is dragged without modifiers)
	
	public ImageSelectionManager() {
		init(this);
	}
	
	public ImageSelectionManager(Component clickSrc) {
		init(clickSrc);
	}
	
	protected void init(Component clickSrc) {
		fillColor=new Color(255,255,255,96);
		strokeColor=Color.WHITE;
		selection=new Area();
		selInProgress=null;
		listeners=new Vector();
		clickSrc.addMouseListener(this);
		clickSrc.addMouseMotionListener(this);
		clickSrc.setCursor(new Cursor(Cursor.CROSSHAIR_CURSOR));
		clickSrc.addKeyListener(this);
		undoListeners=new UndoableEditSupport();
		replaceInProgress=null;
	}
	
	public void addUndoableEditListener(UndoableEditListener l) { undoListeners.addUndoableEditListener(l); }
	public void removeUndoableEditListener(UndoableEditListener l) { undoListeners.removeUndoableEditListener(l); }
	
	public void setFillColor(Color c) {
		fillColor=c;
		repaintSelection();
	}
	
	public void setStrokeColor(Color c) {
		strokeColor=c;
		repaintSelection();
	}
	
	public void invertColors() {
		fillColor=invert(fillColor);
		strokeColor=invert(strokeColor);
		repaintSelection();
	}
	
	static protected Color invert(Color c) {
		return new Color(255-c.getRed(),255-c.getGreen(),255-c.getBlue(),c.getAlpha());
	}
	
	protected void repaintSelection() {
		repaint(getShapeBounds(selection));
		if(selInProgress!=null)
			repaint(getShapeBounds(selInProgress));
	}
	
	public void clear() {
		if(!selection.isEmpty()) {
			Rectangle r=getShapeBounds(selection);
			undoListeners.postEdit(new ReplaceEdit(this,selection,null));
			selection.reset();
			repaint(r);
			fireSelectionChanged();
		}
	}
	
	public Area getSelectedArea() { return (Area)selection.clone(); }
	public void setSelectedArea(Area selection) {
		if(selection==null)
			this.selection.reset();
		else
			this.selection=(Area)selection.clone();
		repaint();
		fireSelectionChanged();
	}
	
	public Area getSelectionInProgress() {
		if(selInProgress==null) {
			return getSelectedArea();
		} else {
			if(curIsSubtract) {
				Area all=(Area)selection.clone();
				all.subtract(new Area(selInProgress));
				return all;
			} else {
				Area all=new Area(selInProgress);
				all.add(selection);
				return all;
			}
		}
	}
	
	//******* LISTENER MANAGEMENT ********
	Vector listeners; //a vector of UndoableEditListeners, to be notified whenever selection is changed
	boolean firing; //true if in the process of notifying SelectionListeners
	interface SelectionListener extends EventListener {
		public void selectionInProgress(ImageSelectionManager src);
		public void selectionChanged(ImageSelectionManager src);
	}
	
	public void addSelectionListener(SelectionListener l) { listeners.add(l); }
	public void removeSelectionListener(SelectionListener l) { listeners.remove(l); }
	public void fireSelectionChanged() {
		firing=true;
		for(int i=0; i<listeners.size() && firing; i++)
			((SelectionListener)listeners.get(i)).selectionChanged(this);
		firing=false;
	}
	public void fireSelectionInProgress() {
		firing=true;
		for(int i=0; i<listeners.size() && firing; i++)
			((SelectionListener)listeners.get(i)).selectionInProgress(this);
		firing=false;
	}
	
	//******* GRAPHICS ********
	public void paintComponent(Graphics g) {
		Graphics2D g2d=(Graphics2D) g;
		AffineTransform origTrans=g2d.getTransform();
		g2d.transform(AffineTransform.getScaleInstance(getWidth()-1,getHeight()-1));
			
		float strokeWidth=2.f/(getWidth()+getHeight()); //average of width and height so 1 pixel stroke on screen
		g2d.setStroke(new BasicStroke(selectionStrokeWidth*strokeWidth));

		if(selInProgress!=null) {
			Area inside=(Area)selection.clone();
			inside.intersect(new Area(selInProgress));
			g2d.setPaint(curIsSubtract?Color.RED:strokeColor);
			g2d.draw(inside);
			
			Area outside=(Area)selection.clone();
			outside.subtract(new Area(selInProgress));
			g2d.setPaint(fillColor);
			g2d.fill(outside);
			g2d.setPaint(strokeColor);
			g2d.draw(outside);
			
			if(!curIsSubtract) {
				g2d.setPaint(fillColor);
				g2d.fill(selInProgress);
			}
			g2d.setPaint(strokeColor);
			g2d.draw(selInProgress);
		} else {
			g2d.setPaint(fillColor);
			g2d.fill(selection);
			g2d.setPaint(strokeColor);
			g2d.draw(selection);
		}

		g2d.setTransform(origTrans);
	}

	//******* MOUSE CONTROLS ********
	
	//workaround for bug in OS X: 
	// http://developer.apple.com/releasenotes/Java/JavaMOSX10.3RN/Known/chapter_3_section_5.html
	boolean button1down=false;
	
	//takes a mouse event from a (possibly parent) panel, translates and clips it to this
	Point getEventPoint(MouseEvent e) {
		Point p=e.getPoint();
		for(Component c=this; c!=null && c!=e.getComponent(); c=c.getParent())
			p.translate(-c.getX(),-c.getY());
		if(p.x<0)
			p.x=-1;
		if(p.y<0)
			p.y=-1;
		if(p.x>getWidth())
			p.x=getWidth();
		if(p.y>getHeight())
			p.y=getHeight();
		return p;
	}
	
	Rectangle getShapeBounds(Shape s) {
		AffineTransform trans=AffineTransform.getScaleInstance(getWidth()-1,getHeight()-1);
		Rectangle2D scrnBnds=trans.createTransformedShape(s).getBounds2D();
		return new Rectangle((int)(scrnBnds.getX()-selectionStrokeWidth/2), (int)(scrnBnds.getY()-selectionStrokeWidth/2),(int)(scrnBnds.getWidth()+selectionStrokeWidth+2),(int)(scrnBnds.getHeight()+selectionStrokeWidth+2));
	}
	
	public void mousePressed(MouseEvent e) 
	{
		//System.out.println("press: "+e);
		
		if (e.getButton()!=MouseEvent.BUTTON1){return;}
		button1down=true;
		lastPos=getEventPoint(e);

		if (e.isControlDown() && e.isShiftDown()) {
			//ambiguous, cancel operation
			button1down=false;
		} else {
			selInProgress=new GeneralPath(GeneralPath.WIND_NON_ZERO);
			selInProgress.moveTo(lastPos.x/(float)(getWidth()-1),lastPos.y/(float)(getHeight()-1));
			curIsAdd=curIsSubtract=false;
			replaceInProgress=null;
			if (e.isControlDown()) {
				curIsSubtract=true;
			} else if (e.isShiftDown()) {
				curIsAdd=true;
			}
		}
	}
	
	public void mouseDragged(MouseEvent e) 
	{	  
		//System.out.println("drag: "+e);
		
		//getButton could be 0 if we are forwarding call from mouseMoved to get around OS X bug
		if (!button1down || e.getButton()!=0 && e.getButton()!=MouseEvent.BUTTON1){return;}
		if (selInProgress==null) return;
		
		if(!curIsAdd && !curIsSubtract && replaceInProgress==null) {
			repaint(getShapeBounds(selection));
			replaceInProgress=new ReplaceEdit(this,selection,null);
			selection.reset();
			fireSelectionChanged();
		}
		Point curPoint=getEventPoint(e);
		if (curPoint.distance(lastPos)>0) {
			lastPos=curPoint;
			selInProgress.lineTo(lastPos.x/(float)(getWidth()-1),lastPos.y/(float)(getHeight()-1));
			repaint(getShapeBounds(selInProgress));
			fireSelectionInProgress();
		}
	}
	
	public void mouseReleased(MouseEvent e) 
	{
		//System.out.println("released: "+e);
		
		if (!button1down || e.getButton()!=MouseEvent.BUTTON1) return;
		button1down=false;
		
		if (selInProgress==null) return;
		
		Point curPoint=getEventPoint(e);
		if (curPoint.distance(lastPos)>0) {
			lastPos=curPoint;
			selInProgress.lineTo(lastPos.x/(float)(getWidth()-1),lastPos.y/(float)(getHeight()-1));
		}
		selInProgress.closePath();
		
		int npoints=0;
		for(PathIterator it=selInProgress.getPathIterator(null); !it.isDone(); it.next(), npoints++) {}
		
		if (npoints>=3) 
		{
			if(curIsSubtract) { 
				Area removal=new Area(selInProgress);
				undoListeners.postEdit(new SubtractEdit(this,selection,removal));
				selection.subtract(removal);
			} else { //add or replace
				Area addition=new Area(selInProgress);
				if(replaceInProgress==null) //add
					undoListeners.postEdit(new AddEdit(this,selection,addition));
				else { //replace
					replaceInProgress.next=addition;
					undoListeners.postEdit(replaceInProgress);
					replaceInProgress=null;
				}
				selection.add(addition);
			}
		}
		
		repaint();
		selInProgress=null;
		fireSelectionChanged();
		
		//		segImage.reSegment(makeTmap(), getAverageColors());
	}
	
	public void mouseMoved(MouseEvent e)
	{
		//workaround for bug with OS X
		// http://developer.apple.com/releasenotes/Java/JavaMOSX10.3RN/Known/chapter_3_section_5.html
		//System.out.println("getButton()="+e.getButton()+"  button1down="+button1down+"  isControlDown()="+e.isControlDown());
		if(e.getButton()==0 && button1down && e.isControlDown()) {
			mouseDragged(e);
			return;
		}
	}
	
	public void mouseClicked(MouseEvent e){}
	public void mouseExited(MouseEvent e){}
	public void mouseEntered(MouseEvent e){}
	public void updateLocation(MouseEvent e){}

	public void keyPressed(KeyEvent e) {
		if((e.getModifiersEx()&(InputEvent.META_DOWN_MASK|InputEvent.CTRL_DOWN_MASK))!=0) {
			switch(e.getKeyCode()) {
				case KeyEvent.VK_A: {
					Area newsel=new Area(new Rectangle(-1,-1,3,3));
					undoListeners.postEdit(new ReplaceEdit(this,selection,newsel));
					selection=newsel;
					break;
				}
				case KeyEvent.VK_D: {
					undoListeners.postEdit(new ReplaceEdit(this,selection,null));
					selection.reset();
					break;
				}
				default:
					return;
			}
			repaint();
			fireSelectionChanged();
		}
	}
	public void keyReleased(KeyEvent e) {} 
	public void keyTyped(KeyEvent e) {}
}
