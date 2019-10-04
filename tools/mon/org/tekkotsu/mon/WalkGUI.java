package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;

public class WalkGUI extends JFrame implements PointPick.PointPickedListener, ChangeListener, ActionListener, MouseListener, MechaController.MechaUpdatedListener {
	static int defPort=10050;
	PointPick pp;
	JSlider xslide;
	JSlider yslide;
	JSlider aslide;
	JButton stopBut;
	JRadioButton horizRotateBut;
	JRadioButton horizStrafeBut;
	boolean horizButFake=false;
	JCheckBox resetOnRelease;
	JLabel status;
	JButton reconnectBut;
	MechaController comm;
	static int slidermax=10000;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(WalkGUI.class);

	static public void main(String s[]) {
		int port=defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new WalkGUI(s[0],port,args);
		/*		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) { frame.dispose(); }
			});*/
	}
	
	public static void usage() {
		System.out.println("Usage: java WalkGUI host [port]");
		System.out.println("       if port is not specified, it defaults to: "+defPort);
		System.exit(2);
	}
		
	public WalkGUI(String host, int port, String args[]) {
		super("TekkotsuMon: Walk Remote Control");
		pack();
		comm=new MechaController(host,port);
		comm.addMechaUpdatedListener(this);
		setLocation(prefs.getInt("WalkGUI.location.x",50),prefs.getInt("WalkGUI.location.y",50));
		setVisible(true);
	}

	public void close() {
		prefs.putInt("WalkGUI.location.x",getLocation().x);
		prefs.putInt("WalkGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("WalkGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("WalkGUI.location.y",getLocation().y+getInsets().top);
		comm.kill();
		dispose();
	}
	
	class CloseWalkAdapter extends WindowAdapter {
		WalkGUI gui;
		CloseWalkAdapter(WalkGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp) {
		boolean isBut2=(e.getModifiersEx()&MouseEvent.BUTTON3_DOWN_MASK)==MouseEvent.BUTTON3_DOWN_MASK;
		if(!horizButFake && isBut2) {
			if(horizStrafeBut.isSelected())
				horizRotateBut.setSelected(true);
			else
				horizStrafeBut.setSelected(true);
			horizButFake=isBut2;
		}
		if(horizRotateBut.isSelected())
			aslide.setValue((int)(slidermax*p.x));
		if(horizStrafeBut.isSelected())
			yslide.setValue((int)(slidermax*p.x));
		xslide.setValue((int)(slidermax*p.y));
	}

	public void mechaUpdated(MechaController comm) {
		if(status!=null) {
			pp.setEnabled(comm._isConnected);
			xslide.setEnabled(comm._isConnected);
			yslide.setEnabled(comm._isConnected);
			aslide.setEnabled(comm._isConnected);
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
		if(!horizButFake && isBut2) {
			if(horizStrafeBut.isSelected())
				horizRotateBut.setSelected(true);
			else
				horizStrafeBut.setSelected(true);
			horizButFake=isBut2;
			updatePP();
		}
	}
	public void mouseReleased(MouseEvent e) {
		boolean isBut1=(e.getModifiersEx()&MouseEvent.BUTTON1_DOWN_MASK)==MouseEvent.BUTTON1_DOWN_MASK;
		boolean isBut2=(e.getModifiersEx()&MouseEvent.BUTTON3_DOWN_MASK)==MouseEvent.BUTTON3_DOWN_MASK;
		if(horizButFake && !isBut2) {
			if(horizStrafeBut.isSelected())
				horizRotateBut.setSelected(true);
			else
				horizStrafeBut.setSelected(true);
			horizButFake=isBut2;
			updatePP();
		}
		if(!isBut1 && !isBut2) {
			if(resetOnRelease.isSelected())
				stopBut.doClick();
		}
	}

	public void stateChanged(ChangeEvent e) {
		if(e.getSource()==xslide) {
			comm.sendCommand("f",xslide.getValue()/(float)slidermax);
			pp.doSetPoint(pp.getXValue(),xslide.getValue()/(float)slidermax);
		} else if(e.getSource()==yslide) {
			comm.sendCommand("s",yslide.getValue()/(float)-slidermax);
			if(horizStrafeBut.isSelected())
				pp.doSetPoint(yslide.getValue()/(float)slidermax,pp.getYValue());
		} else if(e.getSource()==aslide) {
			// Rotation is both fast and sensitive, so we'll exponentiate it to
			// drag out the low end without sacrificing the high end
			float aval=aslide.getValue()/(float)slidermax;
			aval*=(aval<0?aval:-aval);
			comm.sendCommand("r",aval);
			float tmp=pp.getYValue();
			if(horizRotateBut.isSelected())
				pp.doSetPoint(aslide.getValue()/(float)slidermax,pp.getYValue());
		}
	}

	public void actionPerformed(ActionEvent e) {
		if(e.getSource()==stopBut) {
			xslide.setValue(0);
			yslide.setValue(0);
			aslide.setValue(0);
		} else if(e.getSource()==horizRotateBut) {
			updatePP();
		} else if(e.getSource()==horizStrafeBut) {
			updatePP();
		} else if(e.getSource()==reconnectBut) {
			int port=comm._port;
			String addr=comm._host;
			comm.kill();
			comm.removeMechaUpdatedListener(this);
			comm = new MechaController(comm._host,comm._port);
			comm.addMechaUpdatedListener(this);
		}
	}
	
	public void updatePP() {
		float x=0;
		if(horizStrafeBut.isSelected())
			x=yslide.getValue()/(float)slidermax;
		else if(horizRotateBut.isSelected())
			x=aslide.getValue()/(float)slidermax;
		pp.doSetPoint(x,xslide.getValue()/(float)slidermax);
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
		pp=new PointPick(false);
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
			tmp2.add(new JLabel("Forward:"));
			{
				Box tmp3=Box.createHorizontalBox();
				xslide=new JSlider(-slidermax,slidermax,0);
				xslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Aft");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(xslide);
				lab=new JLabel("Fore");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.LEFT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				//tmp3.add(new JButton("Zero"));
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			tmp2.add(new JLabel("Strafe:"));
			{
				Box tmp3=Box.createHorizontalBox();
				yslide=new JSlider(-slidermax,slidermax,0);
				yslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Left");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(yslide);
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
			tmp2.add(new JLabel("Rotate:"));
			{
				Box tmp3=Box.createHorizontalBox();
				aslide=new JSlider(-slidermax,slidermax,0);
				aslide.addChangeListener(this);
				JLabel lab;
				lab=new JLabel("Counter");
				lab.setFont(lab.getFont().deriveFont(lab.getFont().getSize2D()-2));
				lab.setHorizontalAlignment(SwingConstants.RIGHT);
				lab.setPreferredSize(new Dimension(labwidth,lab.getFont().getSize()));
				tmp3.add(lab);
				tmp3.add(aslide);
				lab=new JLabel("Clock");
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
				stopBut=new JButton("Stop!");
				stopBut.addActionListener(this);
				rootPane.setDefaultButton(stopBut);
				tmp3.add(stopBut);
				tmp3.add(Box.createHorizontalGlue());
				tmp3.setAlignmentX(0);
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createVerticalStrut(strutsize));
			ButtonGroup bg = new ButtonGroup();
			horizRotateBut=new JRadioButton("Horizontal is Rotate");
			horizRotateBut.addActionListener(this);
			horizRotateBut.setSelected(true);
			bg.add(horizRotateBut);
			tmp2.add(horizRotateBut);
			horizStrafeBut=new JRadioButton("Horizontal is Strafe");
			horizStrafeBut.addActionListener(this);
			bg.add(horizStrafeBut);
			tmp2.add(horizStrafeBut);
			tmp2.add(Box.createVerticalStrut(strutsize));
			tmp2.add(resetOnRelease=new JCheckBox("Reset on release")); 
			resetOnRelease.setSelected(true);
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
		xslide.setEnabled(false);
		yslide.setEnabled(false);
		aslide.setEnabled(false);
		stopBut.setEnabled(false);
		addWindowListener(new CloseWalkAdapter(this));
	}
}
