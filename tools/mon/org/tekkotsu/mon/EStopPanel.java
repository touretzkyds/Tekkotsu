package org.tekkotsu.mon;

import javax.swing.JPanel;
import javax.swing.JFrame;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.InputEvent;

public class EStopPanel extends JPanel implements EStopListener.UpdatedListener, Listener.ConnectionListener {
	EStopListener comm;
	int mode;
	static final int STOPPED_MODE=0;
	static final int NOTSTOPPED_MODE=1;
	static final int DISABLED_MODE=2;
	static final int MAX_MODE=3;
	Shape shape[] = new Shape[MAX_MODE];
	BasicStroke stroke[] = new BasicStroke[MAX_MODE];
	Paint fill[] = new Color[MAX_MODE];
	Paint line[] = new Color[MAX_MODE];
	Paint textFill[] = new Color[MAX_MODE];
	String text[] = new String[MAX_MODE];
	float textSize[] = new float[MAX_MODE];
	static final int res=1000;
	
	static public void main(String s[]) {
		int port=EStopListener.defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new JFrame("TekkotsuMon: EStop");
		frame.setSize(new Dimension(200, 200)); 
		EStopPanel estop=new EStopPanel(new EStopListener(s[0],port));
		frame.getContentPane().add(estop);
		frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) { System.exit(0); } });
		frame.setVisible(true);
	}

	public static void usage() {
		System.out.println("Usage: java EStopPanel host [port]");
		System.out.println("			 if port is not specified, it defaults to "+EStopListener.defPort);
		System.exit(2);
	}

	public EStopPanel(EStopListener comm) {
		super();
		init(comm,false);
	}
	
	public EStopPanel(EStopListener comm, boolean isStatus) {
		super();
		init(comm,isStatus);
	}
	
	protected void init(EStopListener comm, boolean isStatus) {
		this.comm=comm;
		mode=comm.isConnected()?(comm.getEStop()?STOPPED_MODE:NOTSTOPPED_MODE):DISABLED_MODE;
		setEnabled(mode!=DISABLED_MODE);
		comm.addUpdatedListener(this);
		
		if(!isStatus) {
			shape[NOTSTOPPED_MODE]=makeStopSign();
			stroke[NOTSTOPPED_MODE]=new BasicStroke(res/30);
			fill[NOTSTOPPED_MODE]=Color.RED;
			line[NOTSTOPPED_MODE]=Color.BLACK;
			text[NOTSTOPPED_MODE]="STOP";
			textFill[NOTSTOPPED_MODE]=Color.WHITE;
			textSize[NOTSTOPPED_MODE]=3.125f;

			shape[STOPPED_MODE]=new Ellipse2D.Float(0,0,res,res);
			stroke[STOPPED_MODE]=new BasicStroke(res/30);
			fill[STOPPED_MODE]=Color.GREEN;
			line[STOPPED_MODE]=Color.BLACK;
			text[STOPPED_MODE]="GO";
			textFill[STOPPED_MODE]=Color.BLACK;
			textSize[STOPPED_MODE]=2.5f;
		} else {
			shape[STOPPED_MODE]=makeStopSign();
			stroke[STOPPED_MODE]=new BasicStroke(res/30);
			fill[STOPPED_MODE]=Color.RED;
			line[STOPPED_MODE]=Color.BLACK;
			text[STOPPED_MODE]="STOPPED";
			textFill[STOPPED_MODE]=Color.WHITE;
			textSize[STOPPED_MODE]=5.5f;
			
			shape[NOTSTOPPED_MODE]=new Ellipse2D.Float(0,0,res,res);
			stroke[NOTSTOPPED_MODE]=new BasicStroke(res/30);
			fill[NOTSTOPPED_MODE]=Color.GREEN;
			line[NOTSTOPPED_MODE]=Color.BLACK;
			text[NOTSTOPPED_MODE]="RUNNING";
			textFill[NOTSTOPPED_MODE]=Color.BLACK;
			textSize[NOTSTOPPED_MODE]=5.5f;
		}
			
		shape[DISABLED_MODE]=new Rectangle2D.Float(res/8,res/8,res*3/4,res*3/4);
		stroke[DISABLED_MODE]=new BasicStroke(0);
		fill[DISABLED_MODE]=null;
		line[DISABLED_MODE]=null;
		text[DISABLED_MODE]="><";
		textFill[DISABLED_MODE]=Color.BLACK;
		textSize[DISABLED_MODE]=1.5f;
		
		setToolTipText("Toggle Emergency Stop; alt-click to open new window");
		enableEvents(AWTEvent.MOUSE_EVENT_MASK);
	}

	protected Dimension dim;
	public void setMyDim(Dimension d) { dim=d; }
	public Dimension getMyDim() { return dim; }
	
	public Dimension getPreferredSize() { return (dim==null) ? super.getPreferredSize() : dim; }
	public Dimension getMinimumSize() { return (dim==null) ? super.getPreferredSize() : dim; }
	public Dimension getMaximumSize() { return (dim==null) ? super.getPreferredSize() : dim; }

	protected Shape makeStopSign() {
		int x[] = new int[10];
		int y[] = new int[10];
		double l=res*Math.tan(22.5/180.0*Math.PI);
		x[0]=res/2;
		y[0]=0;
		x[1]=(int)((res-l)/2+l);
		y[1]=0;
		x[2]=res;
		y[2]=(int)((res-l)/2);
		x[3]=res;
		y[3]=x[1];
		x[4]=x[1];
		y[4]=res;
		x[5]=y[2];
		y[5]=res;
		x[6]=0;
		y[6]=y[3];
		x[7]=0;
		y[7]=y[2];
		x[8]=y[2];
		y[8]=y[0];
		x[9]=x[0];
		y[9]=y[0];
		return new Polygon(x,y,10);
	}
	
	public void close() {
		comm.close();
		remove();
	}
	
	public void remove() {
		comm.removeUpdatedListener(this);
	}
	
	public void open() {
		comm.addUpdatedListener(this);
	}

	public void estopUpdated(EStopListener l) {
		if(l.isConnected()) {
			if(l.getEStop())
				mode=STOPPED_MODE;
			else
				mode=NOTSTOPPED_MODE;
		} else
			mode=DISABLED_MODE;
		setEnabled(l.isConnected());
		repaint();
	}
	
	public void onConnected() {
		estopUpdated(comm);
	}
	public void onDisconnected() {
		estopUpdated(comm);
	}

	public void processMouseEvent(MouseEvent e) {
		if(e.getID()==MouseEvent.MOUSE_RELEASED) {
			if((e.getModifiersEx()&InputEvent.ALT_DOWN_MASK)!=0) {
				EStopGUI es=new EStopGUI(comm);
			} else {
				if(isEnabled()) {
					comm.toggleEStop();
				} else {
					close();
					open();
				}
			}
		}
		super.processMouseEvent(e);
	}

	public void paint(Graphics graphics) {
		super.paint(graphics);
		Graphics2D g=(Graphics2D)graphics;
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
		int w=getWidth();
		int h=getHeight();
		int cons=w>h?h:w;
		int ex=(w>h?w:h)-cons;
		double scale=(cons-1)/(double)(res+stroke[mode].getLineWidth());
		if(w>h)
			g.translate(ex/2.0,0);
		else
			g.translate(0,ex/2.0);
		g.translate(stroke[mode].getLineWidth()/2*scale,stroke[mode].getLineWidth()/2*scale);
		g.scale(scale,scale);
		if(fill[mode]!=null) {
			g.setPaint(fill[mode]);
			g.fill(shape[mode]);
		}
		if(line[mode]!=null) {
			g.setPaint(line[mode]);
			g.setStroke(stroke[mode]);
			g.draw(shape[mode]);
		}
		if(textFill[mode]!=null && cons/textSize[mode]>7) {
			g.setPaint(textFill[mode]);
			g.setFont(new Font("Arial",Font.BOLD,(int)(res/textSize[mode])));
			FontMetrics fm=g.getFontMetrics();
			float fw=(float)fm.getStringBounds(text[mode],g).getWidth();
			float fa=fm.getLineMetrics(text[mode],g).getAscent();
			float fd=fm.getLineMetrics(text[mode],g).getDescent();
			g.drawString(text[mode],(res-fw)/2.0f,(res-fd+fa)/2.0f);
		}
	}
}
