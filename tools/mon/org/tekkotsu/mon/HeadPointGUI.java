package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;

public class HeadPointGUI extends JFrame implements PointPick.PointPickedListener, ChangeListener, ActionListener, MouseListener, HeadPointListener.HeadPointUpdatedListener {
	static int defPort=10052;
	PointPick pp;
	JSlider tslide;
	JSlider pslide;
	JSlider rslide;
	JButton stopBut;
	JRadioButton vertNodBut;
	JRadioButton vertTiltBut;
	boolean vertButFake=false;
	JCheckBox resetOnRelease;
	JLabel status;
	JButton reconnectBut;
	HeadPointListener comm;
	static int slidermax=10000;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(HeadPointGUI.class);

	static public void main(String s[]) {
		int port=defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new HeadPointGUI(s[0],port,args);
		/*		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) { System.exit(0); }
			});*/
	}
	
	public static void usage() {
		System.out.println("Usage: java HeadPointGUI host [port]");
		System.out.println("       if port is not specified, it defaults to: "+defPort);
		System.exit(2);
	}
		
	public HeadPointGUI(String host, int port, String args[]) {
		super("TekkotsuMon: Head Pointer Control");
		pack();
		comm=new HeadPointListener(host,port);
		comm.addHeadPointUpdatedListener(this);
		int offx=getGraphicsConfiguration().getBounds().x;
		int offy=getGraphicsConfiguration().getBounds().y;
		setLocation(prefs.getInt("HeadPointGUI.location.x",50),prefs.getInt("HeadPointGUI.location.y",50));
		setVisible(true);
	}

	public void close() {
		prefs.putInt("HeadPointGUI.location.x",getLocation().x);
		prefs.putInt("HeadPointGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("HeadPointGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("HeadPointGUI.location.y",getLocation().y+getInsets().top);
		comm.kill();
		dispose();
	}
	
	class CloseHeadPointAdapter extends WindowAdapter {
		HeadPointGUI gui;
		CloseHeadPointAdapter(HeadPointGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp) {
		boolean isBut2=(e.getModifiersEx()&MouseEvent.BUTTON3_DOWN_MASK)==MouseEvent.BUTTON3_DOWN_MASK;
		if(!vertButFake && isBut2) {
			if(vertTiltBut.isSelected())
				vertNodBut.setSelected(true);
			else
				vertTiltBut.setSelected(true);
			vertButFake=isBut2;
		}
		if(vertNodBut.isSelected())
			rslide.setValue((int)(slidermax*p.y));
		if(vertTiltBut.isSelected())
			tslide.setValue((int)(slidermax*p.y));
		pslide.setValue((int)(slidermax*p.x));
	}

	public void headPointUpdated(HeadPointListener comm) {
		if(status!=null) {
			pp.setEnabled(comm._isConnected);
			tslide.setEnabled(comm._isConnected);
			pslide.setEnabled(comm._isConnected);
			rslide.setEnabled(comm._isConnected);
			stopBut.setEnabled(comm._isConnected);
			if(comm._isConnected)
				status.setText("Connected.");
			else
				status.setText("Reconnecting...");
		}
	}

	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent e) {}
	public void mouseExited(MouseEvent e) {}
	public void mousePressed(MouseEvent e) {
		boolean isBut2=(e.getModifiersEx()&MouseEvent.BUTTON3_DOWN_MASK)==MouseEvent.BUTTON3_DOWN_MASK;
		if(!vertButFake && isBut2) {
			if(vertTiltBut.isSelected())
				vertNodBut.setSelected(true);
			else
				vertTiltBut.setSelected(true);
			vertButFake=isBut2;
			updatePP();
		}
	}
	public void mouseReleased(MouseEvent e) {
		boolean isBut1=(e.getModifiersEx()&MouseEvent.BUTTON1_DOWN_MASK)==MouseEvent.BUTTON1_DOWN_MASK;
		boolean isBut2=(e.getModifiersEx()&MouseEvent.BUTTON3_DOWN_MASK)==MouseEvent.BUTTON3_DOWN_MASK;
		if(vertButFake && !isBut2) {
			if(vertTiltBut.isSelected())
				vertNodBut.setSelected(true);
			else
				vertTiltBut.setSelected(true);
			vertButFake=isBut2;
			updatePP();
		}
		if(!isBut1 && !isBut2) {
			if(resetOnRelease.isSelected())
				stopBut.doClick();
		}
	}

	public void stateChanged(ChangeEvent e) {
		if(e.getSource()==tslide) {
			comm.sendCommand("t",tslide.getValue()/(float)slidermax);
			if(vertTiltBut.isSelected())
				pp.doSetPoint(pp.getXValue(),tslide.getValue()/(float)slidermax);
		} else if(e.getSource()==pslide) {
			comm.sendCommand("p",-pslide.getValue()/(float)slidermax);
			pp.doSetPoint(pslide.getValue()/(float)slidermax,pp.getYValue());
		} else if(e.getSource()==rslide) {
			// Rotation is both fast and sensitive, so we'll exponentiate it to
			// drag out the low end without sacrificing the high end
			comm.sendCommand("r",rslide.getValue()/(float)slidermax);
			float tmp=pp.getYValue();
			if(vertNodBut.isSelected())
				pp.doSetPoint(pp.getXValue(),rslide.getValue()/(float)slidermax);
		}
	}

	public void actionPerformed(ActionEvent e) {
		if(e.getSource()==stopBut) {
			tslide.setValue(0);
			pslide.setValue(0);
			rslide.setValue(0);
		} else if(e.getSource()==vertNodBut) {
			updatePP();
		} else if(e.getSource()==vertTiltBut) {
			updatePP();
		} else if(e.getSource()==reconnectBut) {
			int port=comm._port;
			String addr=comm._host;
			comm.kill();
			comm.removeHeadPointUpdatedListener(this);
			comm = new HeadPointListener(comm._host,comm._port);
			comm.addHeadPointUpdatedListener(this);
		}
	}
	
	public void updatePP() {
		float y=0;
		if(vertTiltBut.isSelected())
			y=tslide.getValue()/(float)slidermax;
		else if(vertNodBut.isSelected())
			y=rslide.getValue()/(float)slidermax;
		pp.doSetPoint(pslide.getValue()/(float)slidermax,y);
	}

	public void frameInit() {
		super.frameInit();
		
		int strutsize=10;
		int sepsize=5;
		getContentPane().setLayout(new BorderLayout());
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.EAST);
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.WEST);
		getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.NORTH);
		JPanel p=new JPanel(new SquareRightLayout());
		p.setLayout(new SquareRightLayout());
		pp=new PointPick(false, false);
		pp.addPointPickedListener(this);
		pp.addMouseListener(this);
		p.add(pp,SquareRightLayout.SQUARE);
		Box tmp=Box.createHorizontalBox();
		tmp.add(Box.createHorizontalStrut(strutsize));
		JSeparator sep;
		sep=new JSeparator(SwingConstants.VERTICAL);
		sep.setMaximumSize(new Dimension(sepsize,slidermax));
		tmp.add(sep);
		tmp.add(Box.createHorizontalStrut(strutsize));
		{
			Box tmp2=Box.createVerticalBox();
			tmp2.add(Box.createVerticalGlue());
			int labwidth=45;
			tmp2.add(new JLabel("Tilt:"));
			{
				Box tmp3=Box.createHorizontalBox();
				tslide=new JSlider(-slidermax,slidermax,0);
				tslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Down");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(tslide);
				lab=new JLabel("Up");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.LEFT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				//tmp3.add(new JButton("Zero"));
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			tmp2.add(new JLabel("Pan:"));
			{
				Box tmp3=Box.createHorizontalBox();
				pslide=new JSlider(-slidermax,slidermax,0);
				pslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Left");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(pslide);
				lab=new JLabel("Right");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.LEFT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				//tmp3.add(new JButton("Zero"));
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			tmp2.add(new JLabel("Nod:"));
			{
				Box tmp3=Box.createHorizontalBox();
				rslide=new JSlider(-slidermax,slidermax,0);
				rslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Down");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(rslide);
				lab=new JLabel("Up");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.LEFT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				//tmp3.add(new JButton("Zero"));
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			{
				Box tmp3=Box.createHorizontalBox();
				tmp3.add(Box.createHorizontalGlue());
				stopBut=new JButton("Center");
				stopBut.addActionListener(this);
				rootPane.setDefaultButton(stopBut);
				tmp3.add(stopBut);
				tmp3.add(Box.createHorizontalGlue());
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			ButtonGroup bg = new ButtonGroup();
			vertTiltBut=new JRadioButton("Vertical is Tilt");
			vertTiltBut.addActionListener(this);
			bg.add(vertTiltBut);
			tmp2.add(vertTiltBut);
			vertNodBut=new JRadioButton("Vertical is Nod");
			vertNodBut.addActionListener(this);
			bg.add(vertNodBut);
			tmp2.add(vertNodBut);
			vertNodBut.setSelected(true);
			tmp2.add(Box.createVerticalStrut(strutsize));
			tmp2.add(resetOnRelease=new JCheckBox("Center on release")); 
			tmp2.add(Box.createVerticalGlue());
			tmp.add(tmp2);
//			Dimension d=tmp2.getMinimumSize();
//			System.out.println(d);
//			pp.setSize(d);
//			pp.setMinimumSize(d);
//			pp.setPreferredSize(d);
//			pp.setMaximumSize(d);
		}
		p.add(tmp,SquareRightLayout.RIGHT);
		getContentPane().add(p,BorderLayout.CENTER);
		{
			Box tmp2=Box.createHorizontalBox();
			tmp2.add(Box.createHorizontalStrut(strutsize));
			{
				Box tmp3=Box.createVerticalBox();
				tmp3.add(Box.createVerticalStrut(strutsize));
				tmp3.add(new JSeparator());
				tmp3.add(Box.createVerticalStrut(strutsize-sepsize));
				{
					Box tmp4=Box.createHorizontalBox();
					tmp4.add(status=new JLabel("Connecting..."));
					tmp4.add(Box.createHorizontalGlue());
					reconnectBut=new JButton(carrows);
					reconnectBut.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
					reconnectBut.addActionListener(this);
					reconnectBut.setToolTipText("Drop current connection and try again.");
					tmp4.add(reconnectBut);
					tmp3.add(tmp4);
				}
				tmp3.add(Box.createVerticalStrut(strutsize));
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createHorizontalStrut(strutsize));
			getContentPane().add(tmp2,BorderLayout.SOUTH);
		}

		pp.setEnabled(false);
		tslide.setEnabled(false);
		pslide.setEnabled(false);
		rslide.setEnabled(false);
		stopBut.setEnabled(false);
		addWindowListener(new CloseHeadPointAdapter(this));
	}
}
