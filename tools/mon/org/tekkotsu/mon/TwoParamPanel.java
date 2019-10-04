package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;
import java.util.Iterator;

public class TwoParamPanel extends JPanel implements PointPick.PointPickedListener, ChangeListener, ActionListener, MouseListener, ParamListener.CommListener {
	PointPick pp;
	JPanel sliderPane;
	JPanel draggerPane;
	JButton stopBut;
	JComboBox xsel;
	JComboBox ysel;
	JCheckBox resetOnRelease;
	ParamListener comm;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(TwoParamPanel.class);
	static final int strutsize=10;
	static final int sepsize=5;

	static public void main(String s[]) {
		int port=ParamListener.defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=null;
		if(s.length>2) {
			args=new String[s.length-2];
			for(int i=2; i<s.length; i++)
				args[i-2]=s[i];
		}
		JFrame frame=new JFrame("TekkotsuMon: TwoParamPanel");
		frame.getContentPane().setLayout(new BoxLayout(frame.getContentPane(),BoxLayout.Y_AXIS));
		TwoParamPanel tpp=new TwoParamPanel(new ParamListener(s[0],port),args);
		frame.getContentPane().add(tpp);
		{
			Box tmp=Box.createHorizontalBox();
			tmp.add(Box.createHorizontalStrut(strutsize));
			{
				Box tmp2=Box.createVerticalBox();
				tmp2.add(Box.createVerticalStrut(strutsize));
				JSeparator jsep=new JSeparator();
				jsep.setMaximumSize(new Dimension(jsep.getMaximumSize().width,jsep.getPreferredSize().height));
				tmp2.add(jsep);
				tmp2.add(Box.createVerticalStrut(sepsize));
				{
					Box tmp3=Box.createHorizontalBox();
					tmp3.add(Box.createHorizontalGlue());
					JButton reconnectBut=new JButton(carrows);
					reconnectBut.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
					reconnectBut.setMaximumSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
					reconnectBut.setActionCommand("reconnect");
					reconnectBut.addActionListener(tpp);
					reconnectBut.setToolTipText("Drop current connection and try again.");
					tmp3.add(reconnectBut);
					tmp2.add(tmp3);
				}					
				tmp2.add(Box.createVerticalStrut(strutsize));
				tmp.add(tmp2);
			}
			tmp.add(Box.createHorizontalStrut(strutsize));
			frame.getContentPane().add(tmp);
		}
		
		frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) { System.exit(0); }
			});
		
		frame.pack();
		frame.setVisible(true);
	}
	
	public static void usage() {
		System.out.println("Usage: java TwoParamPanel host [port]");
		System.out.println("       if port is not specified, it defaults to: "+ParamListener.defPort);
		System.exit(2);
	}
		
	public TwoParamPanel(ParamListener pl, String args[]) {
		super();
		comm=pl;
		if(args!=null)
			for(int i=0; i<args.length; i++)
				if(comm.getParam(args[i])==null)
					comm.setParam(args[i],"0");
		setLayout(new BoxLayout(this,BoxLayout.Y_AXIS));
		{
			Box tmp=Box.createHorizontalBox();
			{
				Box tmp2=Box.createVerticalBox();
				JLabel label=new JLabel("Y axis:");
				tmp2.add(label);
				ysel=new JComboBox(comm.getParams().toArray());
				ysel.setMaximumSize(ysel.getMinimumSize());
				ysel.setAlignmentX(0);
				if(args!=null && args.length>1)
					ysel.setSelectedItem(args[1]);
				else if(comm.getParams().size()>1)
					ysel.setSelectedIndex(1);
				tmp2.add(ysel);
				tmp.add(tmp2);
			}
			pp=new PointPick(false);
			pp.addPointPickedListener(this);
			pp.addMouseListener(this);
			pp.setMinimumSize(new Dimension(100,100));
			pp.setPreferredSize(new Dimension(300,300));
			tmp.add(pp);
			add(tmp);
		}
		{
			Box tmp=Box.createVerticalBox();
			tmp.add(Box.createVerticalStrut(sepsize));
			{
				Box tmp2=Box.createHorizontalBox();
				tmp2.add(Box.createHorizontalGlue());
				tmp2.add(new JLabel("X axis:  "));
				xsel=new JComboBox(comm.getParams().toArray());
				xsel.setMaximumSize(xsel.getMinimumSize());
				if(args!=null && args.length>0)
					xsel.setSelectedItem(args[0]);
				else if(comm.getParams().size()>0)
					xsel.setSelectedIndex(0);
				tmp2.add(xsel);
				tmp2.add(Box.createHorizontalGlue());
				tmp.add(tmp2);
			}
			tmp.add(Box.createVerticalStrut(sepsize));
			{
				Box tmp2=Box.createHorizontalBox();
				tmp2.add(Box.createHorizontalGlue());
				stopBut=new JButton("Center");
				stopBut.addActionListener(this);
				tmp2.add(stopBut);
				tmp2.add(Box.createHorizontalGlue());
				tmp.add(tmp2);
			}
			tmp.add(Box.createVerticalStrut(sepsize));
			{
				Box tmp2=Box.createHorizontalBox();
				tmp2.add(Box.createHorizontalGlue());
				tmp2.add(resetOnRelease=new JCheckBox("Center on release"));
				tmp2.add(Box.createHorizontalGlue());
				tmp.add(tmp2);
			}
			add(tmp);
		}
		pp.setEnabled(comm._isConnected);
		stopBut.setEnabled(comm._isConnected);
		comm.addCommListener(this);
	}

	public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp) {
		comm.setParam((String)xsel.getSelectedItem(),String.valueOf(p.x));
		comm.setParam((String)ysel.getSelectedItem(),String.valueOf(p.y));
	}

	public void commUpdated(ParamListener comm) {
		if(pp.isEnabled()!=comm._isConnected || comm.getParams().size()!=xsel.getItemCount()) {
			Object x=xsel.getSelectedItem();
			xsel.removeAllItems();
			for(Iterator it=comm.getParams().iterator(); it.hasNext(); xsel.addItem(it.next())) {}
			xsel.setMaximumSize(xsel.getMinimumSize());
			xsel.setSelectedItem(x);

			Object y=ysel.getSelectedItem();
			ysel.removeAllItems();
			for(Iterator it=comm.getParams().iterator(); it.hasNext(); ysel.addItem(it.next())) {}
			ysel.setMaximumSize(ysel.getMinimumSize());
			ysel.setSelectedItem(y);

			pp.setEnabled(comm._isConnected);
			stopBut.setEnabled(comm._isConnected);
		}
	}

	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent e) {}
	public void mouseExited(MouseEvent e) {}
	public void mousePressed(MouseEvent e) {}
	public void mouseReleased(MouseEvent e) {
		if(resetOnRelease.isSelected())
			stopBut.doClick();
	}

	public void stateChanged(ChangeEvent e) {
		/*
		if(e.getSource()==tslide) {
			comm.sendCommand("t",tslide.getValue()/(float)slidermax);
			pp.doSetPoint(pp.getXValue(),tslide.getValue()/(float)slidermax);
		} else if(e.getSource()==pslide) {
			comm.sendCommand("p",-pslide.getValue()/(float)slidermax);
			if(horizPanBut.isSelected())
				pp.doSetPoint(pslide.getValue()/(float)slidermax,pp.getYValue());
		} else if(e.getSource()==rslide) {
			// Rotation is both fast and sensitive, so we'll exponentiate it to
			// drag out the low end without sacrificing the high end
			comm.sendCommand("r",rslide.getValue()/(float)slidermax);
			float tmp=pp.getYValue();
			if(horizRollBut.isSelected())
				pp.doSetPoint(rslide.getValue()/(float)slidermax,pp.getYValue());
		}
		*/
	}

	public void actionPerformed(ActionEvent e) {
		if(e.getSource()==stopBut) {
			pp.doSetPoint(0,0);
			comm.setParam((String)xsel.getSelectedItem(),"0");
			comm.setParam((String)ysel.getSelectedItem(),"0");
		} else if(e.getActionCommand().equals("reconnect")) {
			int port=comm._port;
			String addr=comm._host;
			comm.close();
			comm.startThread();
		}
	}
}
