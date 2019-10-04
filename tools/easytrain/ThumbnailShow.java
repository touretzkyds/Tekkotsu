/** @file ThumbnailShow.java
 *  @brief 
 *
 *  Frame for showing the Thumbnail objects. each Thumbnail contains a 
 *  small image of one of the images the user loaded.  4 per row horizontally.
 *
 *	@author editted by: Eric Durback
 *  @bug No known bugs.
 */

import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.awt.geom.*;
import java.util.prefs.Preferences;
import javax.swing.undo.UndoableEditSupport;
import javax.swing.undo.AbstractUndoableEdit;

public class ThumbnailShow extends JFrame implements KeyListener, ComponentListener, FocusListener,  ActionListener
{
	public static final String CURIMG_PROPERTY="CurrentImage";
	JScrollPane rootScroll;	
	JScrollBar scrollbar;
		
	int curimg;
	int numImages;
	int downAmount;
	int columns,rows;
	ArrayList thumbs = new ArrayList();
	JList colorlist;
	Container thumbList;
	DefaultListModel list;
	ImageData imgData;

	Container root;
	JPanel thumbRoot;
	
	static Preferences prefs = Preferences.userNodeForPackage(ThumbnailShow.class);
 
	UndoableEditSupport undoListeners;
	
	class ImageChangeEdit extends AbstractUndoableEdit {
		ThumbnailShow src;
		int previous, next;
		ImageChangeEdit(ThumbnailShow src, int previous, int next) {
			super();
			this.src=src;
			this.previous=previous;
			this.next=next;
		}
		public void redo() {
			super.redo();
			src.changeThumb(src.curimg,next);
		}
		public void undo() {
			super.undo();
			src.changeThumb(src.curimg,previous);
		}
		public boolean isSignificant() {
			return false;
		}
	}
	public void addUndoableEditListener(UndoableEditListener l) { undoListeners.addUndoableEditListener(l); }
	public void removeUndoableEditListener(UndoableEditListener l) { undoListeners.removeUndoableEditListener(l); }

	public void focusGained(FocusEvent e) {} 

	public void focusLost(FocusEvent e) {}

	public ThumbnailShow(ImageData imgData) 
	{
	  	
		this.imgData=imgData;
		numImages = imgData.getNumImages();
		
		setBackground(Color.GRAY);
		
		if(numImages < 3)
		{
			setSize(240, 150);
			columns = 2;
			rows = 1;
		}
		else if(numImages < 5)
		{		    
			setSize(450, 180);
			columns = 2;
			rows = 2;
		}
		else if(numImages<10)
		{			
			setSize(450, 300);
			columns = 3;
			rows = (numImages-1) / columns + 1;
		}
		else
		{
			setSize(450, 400);
			columns = 4;
			rows = numImages / columns + 1;
		}
		/*if( numImages == 5 || numImages == 6 || numImages == 9 || numImages == 12)
		{
			downAmount = 3;
		}
		else if(numImages == 3 || numImages ==4)
		{
			downAmount = 2;
		}
		else
		{
			downAmount = 4;
		}*/
		
		
		
		setTitle("Image Thumbnails");
		setLocation(prefs.getInt("ThumbnailShow.location.x",50),prefs.getInt("ThumbnailShow.location.y",50));
		
		addKeyListener(this);
		addComponentListener(this);
		addFocusListener(this); 
		undoListeners=new UndoableEditSupport();
		
		GridLayout g = new GridLayout();
		
		g.setColumns(columns);
		
		//g.setRows(numImages/4 + 1);
		g.setRows(rows);
		
		root = this.getContentPane();
		
		thumbRoot = new JPanel();
		thumbRoot.setMaximumSize(new Dimension(300,300));
		thumbRoot.setLayout(g);
		thumbRoot.setBackground(Color.GRAY);
		
		
		Thumbnail temp;
		for(int i=0; i<numImages; i++)
		{
			temp = new Thumbnail(imgData,i);
			
			temp.addActionListener(this);
			temp.addKeyListener(this);
			temp.setSelected(true);
			thumbRoot.add(temp);
			thumbs.add(temp);		
		}
		
		
		downAmount = g.getColumns();
		
		rootScroll = new JScrollPane(thumbRoot,JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		                    JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
		root.add(rootScroll,BorderLayout.CENTER);
	
		
		((Thumbnail)thumbs.get(0)).setSelected(false);

		rootScroll.setLayout(new ScrollPaneLayout());
	}
	
	public int getCurrentImage() {
		return curimg;
	}

	public void actionPerformed(ActionEvent e)
	{
		
		
		for(int i =0; i<numImages; i++)
		{
			if (e.getSource()==thumbs.get(i) ) 
			{			
				
				undoListeners.postEdit(new ImageChangeEdit(this, curimg, i));
				changeThumb(curimg,i);
				return; //only one should match	
			}
			
		}
	}
	
	public void changeThumb(int last, int pressed)
	{
		if (last<0) last+=numImages;
		if (pressed<0) pressed+=numImages;
		if (last>=numImages) last-=numImages;
		if (pressed>=numImages) pressed-=numImages;
		    
		
		((Thumbnail)thumbs.get(last)).setSelected(true);
		((Thumbnail)thumbs.get(pressed)).setSelected(false);
		curimg = pressed;	

		firePropertyChange(CURIMG_PROPERTY,last,pressed);
	}
	
	public void keyPressed(KeyEvent e) 
	{
		int move;
		
		if (e.getKeyCode()==KeyEvent.VK_LEFT ||
		    e.getKeyCode()==KeyEvent.VK_PAGE_UP ||
		    e.getKeyCode()==KeyEvent.VK_KP_LEFT) 
		{
			curimg--;
		  
			if (curimg<0) curimg+=numImages;
			
			undoListeners.postEdit(new ImageChangeEdit(this, curimg+1, curimg));
			changeThumb(curimg+1,curimg);	
		} 
		else if (e.getKeyCode()==KeyEvent.VK_RIGHT ||
		    e.getKeyCode()==KeyEvent.VK_PAGE_DOWN ||
		    e.getKeyCode()==KeyEvent.VK_KP_RIGHT) 
		{
			curimg++;
			
			if (curimg>=numImages) curimg-=numImages;
		    
			undoListeners.postEdit(new ImageChangeEdit(this, curimg-1, curimg));
		    changeThumb(curimg-1,curimg);
		} else if (e.getKeyCode()==KeyEvent.VK_DOWN ||
		    e.getKeyCode()==KeyEvent.VK_KP_DOWN)
		{
			move = getDownMove();
			curimg+=move;
		    
			while(curimg>=numImages)
				curimg-=numImages;
		    
			undoListeners.postEdit(new ImageChangeEdit(this, curimg-move, curimg));
			changeThumb(curimg-move,curimg);
		} else if (e.getKeyCode()==KeyEvent.VK_UP ||
		    e.getKeyCode()==KeyEvent.VK_KP_UP)
		{
			move = getUpMove();
			curimg-=move;
		  
			while(curimg<0)
				curimg+=numImages;

			undoListeners.postEdit(new ImageChangeEdit(this, curimg+move, curimg));
			changeThumb(curimg+move,curimg);
		}
	}
	
	public int getDownMove() //change to down and up
	{
		int empty = getEmptySpots();
		
		if(numImages < 3)
		{
			return  1;
		}
		else if(lastColumnEndRow())
		{
			if(lastRow())
			{
				return 1;
				
			}
			else
			{
				return (columns - empty+1);
			}
		}
		else if(aboveEmptySpot())//case where not a image underneath
		{
			return (2 * columns) - empty + 1;
		}
		else if(lastRow())
		{
			return (columns-empty+1);
		}		
		else
		{
			return columns ;
		}
			
			
	}
	
	
	public int getUpMove() //change to down and up
	{
		int empty = getEmptySpots();
		
		if(numImages < 3)
		{
			return 1;
		}
		
		else if(topLeft())
		{
			if(empty == 0)//full grid
			{
				return 1;
			}
			else
			{
				return  columns - empty +1;
			}
		}
		else if(topRight())
		{
			if(empty == 0)
			{
				return columns + 1;
			}
			else if(belowEmpty())
			{
				return (2*columns) - empty + 1;
			}
			else
			{
				return columns - empty + 1;
			}
		}		
		else if(curimg < columns)
		{
			return columns -empty + 1;
		}	
		else
		{
			return columns ;
		}
			
			
	}
	
	public boolean topLeft()
	{
		return (curimg == 0);
	}
	
	public boolean topRight()
	{
		return (curimg == columns-1);
	}
	
	public boolean belowEmpty()
	{
		return (getEmptySpots() > columns - curimg);
	}
	
	public boolean lastColumnEndRow()
	{
		return (curimg != 0 && (((curimg+1) % columns) == 0) && (numImages - curimg <= columns));	
	}
	
	public int getEmptySpots()
	{
		return (columns * rows) % numImages;
	}
	
	public boolean lastRow()
	{
		return(curimg / columns == rows-1); 
	}
	
	public boolean aboveEmptySpot()
	{
		return(getEmptySpots() > 0 && (curimg / columns) +1== rows -1  //2nd to last row
		   && ((rows-1)*columns) - curimg <= getEmptySpots());			
	}	
	
	public void keyReleased(KeyEvent e) { }
	public void keyTyped(KeyEvent e) { }

	public void componentResized(ComponentEvent e) 
	{            	
		repaint();
		
	}

	public void componentHidden(ComponentEvent e) { }
	public void componentMoved(ComponentEvent e)
	{ 
	
		prefs.putInt("ThumbnailShow.location.x",getLocation().x);
		prefs.putInt("ThumbnailShow.location.y",getLocation().y);

	}
	public void componentShown(ComponentEvent e) { }

	
	
  
}
