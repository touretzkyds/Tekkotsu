import java.awt.image.BufferedImage;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Container;
import java.awt.BorderLayout;
import java.awt.event.*;
import javax.swing.Box;
import javax.swing.JFrame;
import java.util.prefs.Preferences;

public class ImageViewer extends JFrame implements ComponentListener, KeyListener {
	ImagePanel disp;

	ImageViewer() { super("Image Viewer"); init(0); }
	ImageViewer(String title) { super(title); init(0); }
	ImageViewer(String title, int inset) { super(title); init(inset); }

	public void setImage(BufferedImage img) { disp.setImage(img); correctSize(); }
	public BufferedImage getImage() { return disp.getImage(); }
	public ImagePanel getDisplay() { return disp; }

	protected void init(int inset) {
		disp = new ImagePanel();
		disp.setOpaque(true);
		if(inset==0) {
			setContentPane(disp);
		} else {
			Container root=getContentPane();
			root.setLayout(new BorderLayout());
			root.add(Box.createVerticalStrut(inset),BorderLayout.NORTH);
			root.add(Box.createVerticalStrut(inset),BorderLayout.SOUTH);
			root.add(Box.createHorizontalStrut(inset),BorderLayout.EAST);
			root.add(Box.createHorizontalStrut(inset),BorderLayout.WEST);
			root.add(disp,BorderLayout.CENTER);
		}
		setBackground(Color.BLACK);
		disp.setPreferredSize(new Dimension(100,100));
		pack();
		addComponentListener(this);
		addKeyListener(this);
	}
	
	public void setBackground(Color c) {
		super.setBackground(c);
		if(disp!=null)
			disp.setBackground(c);
	}
	
	protected void correctSize() {
		int imgW=getImage().getWidth();
		int imgH=getImage().getHeight();
		float imgAspect=imgW/(float)imgH;
		int dispW=disp.getWidth();
		int dispH=disp.getHeight();
		float dispAspect=dispW/(float)dispH;
		int corrW=dispW,corrH=dispH;
		if(imgAspect>dispAspect) {
			//too narrow, bring up the bottom
			corrH=Math.round(dispW/imgAspect);
		} else if(imgAspect<dispAspect) {
			corrW=Math.round(dispH*imgAspect);
		}
		if(corrW!=dispW || corrH!=dispH) {
			disp.setPreferredSize(new Dimension(corrW,corrH));
			pack();
		}
	}
	
	public void componentResized(ComponentEvent e) { correctSize(); }
	public void componentHidden(ComponentEvent e) { }
	public void componentMoved(ComponentEvent e) { }
	public void componentShown(ComponentEvent e) { }

	public void keyPressed(KeyEvent e) {}
	public void keyReleased(KeyEvent e) {} 
	public void keyTyped(KeyEvent e) {
		//System.out.println(getName()+" keyTyped: "+e.getKeyChar());
		Dimension prev=disp.getSize();
		//System.out.println("Previous: "+prev);
		switch(e.getKeyChar()) {
			case '=': {
				disp.setPreferredSize(new Dimension(disp.getImage().getWidth(),disp.getImage().getHeight()));
				break;
			}
			case '+': {
				double xs;
				if(disp.getWidth()>=disp.getImage().getWidth()) {
					xs=disp.getWidth()/disp.getImage().getWidth()+1;
				} else {
					xs=disp.getImage().getWidth()/disp.getWidth();
					if((int)(disp.getImage().getWidth()/xs)==disp.getWidth())
						xs--;
					xs=1/xs;
				}
				//System.out.println("xs="+xs);
				disp.setPreferredSize(new Dimension((int)(disp.getImage().getWidth()*xs),(int)(disp.getImage().getHeight()*xs)));
				break;
			}
			case '-': {
				float xs;
				if(disp.getWidth()>disp.getImage().getWidth()) {
					xs=disp.getWidth()/disp.getImage().getWidth();
					if((int)(disp.getImage().getWidth()*xs)==disp.getWidth())
						xs--;
				} else {
					xs=disp.getImage().getWidth()/disp.getWidth()+1;
					xs=1/xs;
				}
				//System.out.println("xs="+xs);
				disp.setPreferredSize(new Dimension((int)(disp.getImage().getWidth()*xs),(int)(disp.getImage().getHeight()*xs)));
				break;
			}
			default:
				return;
		}
		//System.out.println("New: "+disp.getPreferredSize());
		setSize(getSize().width+disp.getPreferredSize().width-prev.width,getSize().height+disp.getPreferredSize().height-prev.height);
		//setSize(1,1);
		//invalidate();
		//pack();
	}
}