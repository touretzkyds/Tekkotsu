package org.tekkotsu.mon;

import org.tekkotsu.sketch.*;

import javax.swing.JPanel;
import java.awt.image.BufferedImage;
import java.awt.Graphics;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import javax.swing.JFrame;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import java.awt.Rectangle;
import java.awt.geom.*;
import java.util.Vector;
import java.awt.event.*;
import javax.swing.Box;
import javax.swing.JButton;
import java.awt.image.IndexColorModel;
import java.awt.BasicStroke;
import java.awt.Graphics2D;

public class VisionPanel extends JPanel implements VisionUpdatedListener, 
                                                   MouseListener,
						   MouseMotionListener
{
    boolean usingSketchGUI=false;

    protected boolean windowHasFocus;
    protected boolean crosshairsEnabled=false;
    protected boolean idEnabled=false;
    
    protected BufferedImage _image;
    protected VisionListener _listener;
    boolean lockAspect=false;
    float tgtAspect=-1;

    SketchGUI gui;
    protected SketchGUI.Space space;
    //    public int space;

    int drawX, drawY, drawHeight, drawWidth;

    protected int mouseX=-1, mouseY=-1;

	public static void usage() {
		System.out.println("Usage: java VisionPanel host [port|raw|rle] [udp|tcp]");
		System.out.println("       if port is not specified, it defaults to:");
		System.out.println("       "+VisionListener.defRawPort+" for raw");
		System.out.println("       "+VisionListener.defRLEPort+" for RLE.");
		System.out.println("       if transport protocol is not specified, it defaults to UDP");
		System.exit(2);
	}

	public static void main(String s[]) {
		int port=-1;
		if(s.length<2)
			usage();
		String[] args=new String[s.length-1];
		for(int i=0; i<args.length; i++)
			args[i]=s[i+1];

		JFrame frame=new JFrame("TekkotsuMon: Vision");
		frame.setBackground(Color.black);
		//frame.getContentPane().setLayout(new FlowLayout());
		frame.setSize(new Dimension(VisionListener.DEFAULT_WIDTH*2, VisionListener.DEFAULT_HEIGHT*2)); 
		VisionPanel vision=new VisionPanel(s[0],args);
		frame.getContentPane().add(vision);
		frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) { System.exit(0); } });
		//frame.show();
		frame.setVisible(true);
	}

	public void setConvertRGB(boolean b) { _listener.setConvertRGB(b); }
	public boolean getConvertRGB() { return _listener.getConvertRGB(); }

    //	public VisionPanel(VisionListener listener, SketchGUI _gui, int _space) {
        public VisionPanel(VisionListener listener, SketchGUI _gui, SketchGUI.Space _space) {
	    super();
		usingSketchGUI = true;
	    init(listener);
	    gui = _gui;
	    space = _space;
	}
	
	public VisionPanel(String host, String[] args) {
		super();
		boolean useUDP=true;
		int port=0;
		for(int i=0; i<args.length; i++) {
			if(args[i].toUpperCase().compareTo("RLE")==0) {
				port=VisionListener.defRLEPort;
			} else if(args[i].toUpperCase().compareTo("RAW")==0) {
				port=VisionListener.defRawPort;
			} else if(args[i].toUpperCase().compareTo("DEPTH")==0) {
				port=VisionListener.defDepthPort;
			} else if(args[i].toUpperCase().compareTo("REG")==0) {
				port=VisionListener.defRegionPort;
			} else if(args[i].toUpperCase().compareTo("UDP")==0) {
				useUDP=true;
			} else if(args[i].toUpperCase().compareTo("TCP")==0) {
				useUDP=false;
			} else if(args[i].length()>0) {
				port=Integer.parseInt(args[i]);
			}
		}
		if(port==0) {
			System.err.println("VisionPanel port unspecified or 0 - check arguments");
			System.exit(2);
		}
		
		if(useUDP)
			init(new UDPVisionListener(host,port));
		else
			init(new TCPVisionListener(host,port));
	}	
	
	public VisionPanel(String host, int port, String[] args) {
		super();
		boolean useUDP=true;
		for(int i=0; i<args.length; i++) {
			if(args[i].toUpperCase().compareTo("UDP")==0) {
				useUDP=true;
			} else if(args[i].toUpperCase().compareTo("TCP")==0) {
				useUDP=false;
			}
		}
		System.out.println("Connecting to port "+port+" with udp=="+useUDP);
		if(useUDP)
			init(new UDPVisionListener(host,port));
		else
			init(new TCPVisionListener(host,port));
	}
	
	public void init(VisionListener listener) {
	    setBackground(Color.BLACK);
	    setForeground(Color.WHITE);
	    setOpaque(!lockAspect);
	    addMouseMotionListener(this);
	    addMouseListener(this);
	    windowHasFocus = true;
	    crosshairsEnabled = false;
	    _listener=listener;
		_listener.addListener(this);
	}

   
	public VisionListener getListener() { return _listener; }
	public void close() { _listener.close();	}
	public void kill() { _listener.kill(); }
	public void open() { _listener.needConnection(); }
	public void visionUpdated(VisionListener l) { repaint(); }
	public void sensorsUpdated(VisionListener l) {}
	
	public void setLockAspectRatio(boolean b) {
		if(b!=lockAspect) {
			lockAspect=b;
			setOpaque(!lockAspect);
			repaint();
		}
	}
	
	public boolean getLockAspectRatio() { return lockAspect; }

	public void setAspectRatio(float asp) {
		if(asp<=0)
			tgtAspect=-1;
		else
			tgtAspect=asp;
		if(getLockAspectRatio())
			repaint();
	}
	
	public float getAspectRatio() { return tgtAspect; }

	public void setLockAspectRatio(boolean b, float asp) {
		setLockAspectRatio(b);
		setAspectRatio(asp);
	}
	
	public void paint(Graphics graphics) {
		if (!usingSketchGUI) { _image=_listener.getImage(); }
		super.paint(graphics);
		Dimension sz=getSize();
		drawX = 0;
		drawY = 0;
		drawWidth = sz.width;
		drawHeight = sz.height;
		// Scale image to fit the window size while maintaining aspect ratio.
		// Center the image in the unused space if window width or height too large.
		// Note: this only makes sense for cam space.
		if (_image != null && getLockAspectRatio() && (!usingSketchGUI || space==SketchGUI.Space.cam)) {
			float curasp=sz.width/(float)sz.height;
			float tgtasp=getAspectRatio();
			if(tgtasp<0)
				tgtasp=_image.getWidth()/(float)_image.getHeight();
			if(curasp>tgtasp) {
				drawWidth = (int)(sz.height*tgtasp);
				drawX = (sz.width-drawWidth)/2;
			} else if(curasp<tgtasp) {
				drawHeight = (int)(sz.width/tgtasp);
				drawY = (sz.height-drawHeight)/2;
			} else {
			}
		}
		drawImage(graphics, _image, drawX, drawY, drawWidth, drawHeight);
		
		// If requested, draw crosshairs for RawCam, SegCam.
		// Crosshairs for SketchGUI are handled in SketchPanel.java.
		if (crosshairsEnabled && !usingSketchGUI)
		{
			graphics.setXORMode(Color.GRAY);
			graphics.setColor(Color.WHITE);
			((Graphics2D)graphics).setStroke(new BasicStroke(1.0f));
			graphics.drawLine(drawX+drawWidth/2,drawY, drawX+drawWidth/2, drawY+drawHeight);
			graphics.drawLine(drawX, drawY+drawHeight/2, drawX+drawWidth, drawY+drawHeight/2);
			graphics.setPaintMode();
		}
	}
	
	protected void drawImage(Graphics g, BufferedImage img, int x, int y, int w, int h) {
		if(img!=null) {
			synchronized(img) {
				g.drawImage(img,x,y,w,h,null);
			}
		} else {
			g.setColor(getBackground());
			g.fillRect(x,y,w,h);
			FontMetrics tmp=g.getFontMetrics();
			String msg="No image";
			int strw=tmp.stringWidth(msg);
			int strh=tmp.getHeight();
			g.setColor(getForeground());
			g.drawString(msg,(getSize().width-strw)/2,(getSize().height-strh)/2+tmp.getAscent());
		}
	}
	
    public void mouseDragged(MouseEvent e){}

    public void mouseMoved(MouseEvent e)
    {
	// Update the mouse position in the corner
	mouseX = e.getX() - drawX;
	// mouseX *= _listener.DEFAULT_WIDTH*1.0/drawWidth;
	mouseY = e.getY()-drawY;
	// mouseY *= _listener.DEFAULT_HEIGHT*1.0/drawHeight;
	repaint();
    }
    
    public void mousePressed(MouseEvent e){}

    public void mouseClicked(MouseEvent e){}

    public void mouseReleased(MouseEvent e){}

    public void mouseEntered(MouseEvent e){}

    public void mouseExited(MouseEvent e)
    {
	mouseX = -1;
	mouseY = -1;
	repaint();
    }

}
