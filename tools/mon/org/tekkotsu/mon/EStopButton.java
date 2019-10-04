package org.tekkotsu.mon;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.ImageIcon;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.InputEvent;

public class EStopButton extends JButton implements EStopListener.UpdatedListener {
	EStopListener comm;
	int mode;
	static final int STOPPED_MODE=0;
	static final int NOTSTOPPED_MODE=1;
	static final int DISABLED_MODE=2;
	static final int MAX_MODE=3;
	final static ImageIcon goIcon = new ImageIcon("images/go.gif");
	final static ImageIcon stopIcon = new ImageIcon("images/stop.gif");
	final static ImageIcon disIcon = new ImageIcon("images/offline.gif");
	Dimension dim;

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
		EStopButton estop=new EStopButton(new EStopListener(s[0],port));
		frame.getContentPane().add(estop);
		frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) { System.exit(0); } });
		frame.setVisible(true);
	}

	public static void usage() {
		System.out.println("Usage: java EStopButton host [port]");
		System.out.println("       if port is not specified, it defaults to "+EStopListener.defPort);
		System.exit(2);
	}

	public EStopButton(EStopListener comm) {
		super();
		
		this.comm=comm;
		mode=comm.isConnected()?(comm.getEStop()?STOPPED_MODE:NOTSTOPPED_MODE):DISABLED_MODE;
		setEnabled(mode!=DISABLED_MODE);
		comm.addUpdatedListener(this);
		
		setToolTipText("Toggle Emergency Stop; middle-click (or alt-click) to open into new window");
		enableEvents(AWTEvent.MOUSE_EVENT_MASK);

		updateStatus();
	}

	public void close() {
		comm.kill();
		remove();
	}
	
	public void remove() {
		comm.removeUpdatedListener(this);
	}
	
	public void open() {
		comm.addUpdatedListener(this);
		comm.startThread();
	}

	public void setMyDim(Dimension d) { dim=d; }
	public Dimension getMyDim() { return dim; }

	public Dimension getPreferredSize() { return (dim==null) ? super.getPreferredSize() : dim; }
	public Dimension getMinimumSize() { return (dim==null) ? super.getPreferredSize() : dim; }
	public Dimension getMaximumSize() { return (dim==null) ? super.getPreferredSize() : dim; }
	
	public void estopUpdated(EStopListener l) {
		if(l.isConnected()) {
			if(l.getEStop())
				mode=STOPPED_MODE;
			else
				mode=NOTSTOPPED_MODE;
		} else
			mode=DISABLED_MODE;
		updateStatus();
		setEnabled(l.isConnected());
	}

	public void updateStatus() {
		if(mode==STOPPED_MODE) {
			setText("Go");
			setIcon(goIcon);
		}
		if(mode==NOTSTOPPED_MODE) {
			setText("Stop");
			setIcon(stopIcon);
		}
		if(mode==DISABLED_MODE) {
			setText("");
			setIcon(disIcon);
		}
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
}
