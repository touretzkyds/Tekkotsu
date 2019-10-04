package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;

public class RoverGUI extends JFrame implements PointPick.PointPickedListener, ChangeListener, ActionListener, MouseListener, RoverListener.RoverUpdatedListener {
	static int defPort=10056;
	
	PointPick gripper;
	PointPick camera;
	
	JSlider gripperSlider;
	JSlider gripperAngleSlider;
	JSlider perspectiveSlider;
	JSlider gripperHeightSlider;
	JSlider cameraDistanceSlider;
	
	JButton flattenButton;
	JCheckBox autoPerspectiveCheckBox;
	JCheckBox autoAngleCheckBox;
	JCheckBox autoTrackCheckBox;
	JCheckBox maxDistCheckBox;
	
	

	JLabel status;
	JButton reconnectBut;
	
	RoverListener comm;
	static int slidermax=1000;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(RoverGUI.class);

	static public void main(String s[]) {
		int port=defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new RoverGUI(s[0],port,args);
		/*		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) { System.exit(0); }
			});*/
	}
	
	public static void usage() {
		System.out.println("Usage: java RoverGUI host [port]");
		System.out.println("       if port is not specified, it defaults to: "+defPort);
		System.exit(2);
	}
		
	public RoverGUI(String host, int port, String args[]) {
		super("TekkotsuMon: Rover Manipulator Control");
		pack();
		comm=new RoverListener(host,port);
		RoverUpdated(comm);
		comm.addRoverUpdatedListener(this);
		int offx=getGraphicsConfiguration().getBounds().x;
		int offy=getGraphicsConfiguration().getBounds().y;
		setLocation(prefs.getInt("RoverGUI.location.x",50),prefs.getInt("RoverGUI.location.y",50));
		setVisible(true);
	}

	public void close() {
		prefs.putInt("RoverGUI.location.x",getLocation().x);
		prefs.putInt("RoverGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("RoverGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("RoverGUI.location.y",getLocation().y+getInsets().top);
		comm.kill();
		dispose();
	}
	
	class CloseRoverAdapter extends WindowAdapter {
		RoverGUI gui;
		CloseRoverAdapter(RoverGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	public void RoverUpdated(RoverListener comm) {
		if(status!=null) {
			gripper.setEnabled(comm._isConnected);
			camera.setEnabled(comm._isConnected && !autoTrackCheckBox.isSelected());
			
			gripperSlider.setEnabled(comm._isConnected);
			gripperAngleSlider.setEnabled(comm._isConnected && !autoAngleCheckBox.isSelected());
			perspectiveSlider.setEnabled(comm._isConnected && !autoPerspectiveCheckBox.isSelected());
			gripperHeightSlider.setEnabled(comm._isConnected);
			cameraDistanceSlider.setEnabled(comm._isConnected && !maxDistCheckBox.isSelected());
			
			flattenButton.setEnabled(comm._isConnected);
			autoPerspectiveCheckBox.setEnabled(comm._isConnected);
			autoAngleCheckBox.setEnabled(comm._isConnected);
			autoTrackCheckBox.setEnabled(comm._isConnected);
			maxDistCheckBox.setEnabled(comm._isConnected);
			
			if(comm._isConnected) {
				comm.sendCommand("autoPerspective "+(autoPerspectiveCheckBox.isSelected()?1:0));
				comm.sendCommand("autoAngle "+(autoAngleCheckBox.isSelected()?1:0));
				comm.sendCommand("autoTrack "+(autoTrackCheckBox.isSelected()?1:0));
				comm.sendCommand("maxDist "+(maxDistCheckBox.isSelected()?1:0));

				comm.sendCommand("gripper "+gripperSlider.getValue()/(float)slidermax);
				comm.sendCommand("gripperAngle "+gripperAngleSlider.getValue()/(float)slidermax*Math.PI);
				comm.sendCommand("perspective "+perspectiveSlider.getValue()/(float)slidermax*Math.PI/2);
				comm.sendCommand("gripperHeight "+gripperHeightSlider.getValue()/(float)slidermax);
				comm.sendCommand("cameraDistance "+cameraDistanceSlider.getValue()/(float)slidermax);

				Point2D.Float p=gripper.getPoint();
				comm.sendCommand("tgt "+p.x+" "+p.y);
				p=camera.getPoint();
				comm.sendCommand("look "+p.x+" "+p.y);
				
				status.setText("Connected.");
			} else
				status.setText("Reconnecting...");
		}
	}

	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent e) {}
	public void mouseExited(MouseEvent e) {}
	public void mousePressed(MouseEvent e) {}
	public void mouseReleased(MouseEvent e) {}

	public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp) {
		if(pp==gripper) {
			comm.sendCommand("tgt "+p.x+" "+p.y);
			if(autoTrackCheckBox.isSelected())
				camera.setPoint(p.x,p.y);
		} else if(pp==camera) {
			comm.sendCommand("look "+p.x+" "+p.y);
		}
	}

	public void stateChanged(ChangeEvent e) {
		if(e.getSource()==gripperSlider) {
			comm.sendCommand("gripper "+gripperSlider.getValue()/(float)slidermax);
		} else if(e.getSource()==gripperAngleSlider) {
			comm.sendCommand("gripperAngle "+gripperAngleSlider.getValue()/(float)slidermax*Math.PI);
		} else if(e.getSource()==perspectiveSlider) {
			comm.sendCommand("perspective "+perspectiveSlider.getValue()/(float)slidermax*Math.PI/2);
		} else if(e.getSource()==gripperHeightSlider) {
			comm.sendCommand("gripperHeight "+gripperHeightSlider.getValue()/(float)slidermax);
		} else if(e.getSource()==cameraDistanceSlider) {
			comm.sendCommand("cameraDistance "+cameraDistanceSlider.getValue()/(float)slidermax);
		}
	}

	public void actionPerformed(ActionEvent e) {
		if(e.getSource()==flattenButton) {
			gripperHeightSlider.setValue(0);
		} else if(e.getSource()==autoPerspectiveCheckBox) {
			perspectiveSlider.setEnabled(!autoPerspectiveCheckBox.isSelected());
			comm.sendCommand("autoPerspective "+(autoPerspectiveCheckBox.isSelected()?1:0));
		} else if(e.getSource()==autoAngleCheckBox) {
			gripperAngleSlider.setEnabled(!autoAngleCheckBox.isSelected());
			comm.sendCommand("autoAngle "+(autoAngleCheckBox.isSelected()?1:0));
		} else if(e.getSource()==autoTrackCheckBox) {
			camera.setEnabled(!autoTrackCheckBox.isSelected());
			comm.sendCommand("autoTrack "+(autoTrackCheckBox.isSelected()?1:0));
			Point2D.Float p=gripper.getPoint();
			camera.setPoint(p.x,p.y);
		} else if(e.getSource()==maxDistCheckBox) {
			cameraDistanceSlider.setEnabled(!maxDistCheckBox.isSelected());
			comm.sendCommand("maxDist "+(maxDistCheckBox.isSelected()?1:0));
		} else if(e.getSource()==reconnectBut) {
			int port=comm._port;
			String addr=comm._host;
			comm.kill();
			comm.removeRoverUpdatedListener(this);
			comm = new RoverListener(comm._host,comm._port);
			comm.addRoverUpdatedListener(this);
		}
	}
	
	public void frameInit() {
		super.frameInit();
		
		int strutsize=10;
		int topstrut=30;
		int sepsize=5;
		getContentPane().setLayout(new BorderLayout());
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.EAST);
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.WEST);
		getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.NORTH);
		
		JPanel p=new JPanel();
		p.setLayout(new GridLayout(1,2,50,strutsize));
		
		// GRIPPER
		{
			JPanel tmp=new JPanel();
			tmp.setLayout(new BorderLayout());
			{
				Box tmp2=Box.createVerticalBox();
				tmp2.add(Box.createVerticalStrut(topstrut));
				tmp2.add(new JLabel("Height:"));
				gripperHeightSlider = new JSlider(SwingConstants.VERTICAL,-slidermax/4,slidermax,0);
				gripperHeightSlider.setPreferredSize(new Dimension(0,0));
				gripperHeightSlider.addChangeListener(this);
				tmp2.add(gripperHeightSlider);
				flattenButton=new JButton("Flatten");
				flattenButton.addActionListener(this);
				tmp2.add(flattenButton);
				tmp.add(tmp2,BorderLayout.WEST);
			}
			{
				Box tmp2=Box.createVerticalBox();
				{
					Box tmp3=Box.createHorizontalBox();
					tmp3.add(new JLabel("Gripper Position:"));
					tmp3.add(Box.createVerticalStrut(topstrut));
					tmp2.add(tmp3);
				}
				gripper=new PointPick(false);
				gripper.addPointPickedListener(this);
				Dimension d=new Dimension(225,225);
				gripper.setPreferredSize(d);
				tmp2.add(gripper);
				tmp.add(tmp2,BorderLayout.CENTER);
			}
			p.add(tmp);
		}
		
		// CAMERA
		{
			JPanel tmp=new JPanel();
			tmp.setLayout(new BorderLayout());
			{
				Box tmp2=Box.createVerticalBox();
				tmp2.add(Box.createVerticalStrut(topstrut));
				tmp2.add(new JLabel("Distance:"));
				maxDistCheckBox = new JCheckBox("Max Dist");
				maxDistCheckBox.setSelected(true);
				maxDistCheckBox.addActionListener(this);
				tmp2.add(maxDistCheckBox);
				cameraDistanceSlider = new JSlider(SwingConstants.VERTICAL,0,slidermax,0);
				cameraDistanceSlider.addChangeListener(this);
				cameraDistanceSlider.setEnabled(false);
				tmp2.add(cameraDistanceSlider);
				tmp.add(tmp2,BorderLayout.EAST);
			}
			{
				Box tmp2=Box.createVerticalBox();
				{
					Box tmp3=Box.createHorizontalBox();
					tmp3.add(new JLabel("Camera Position:"));
					tmp3.add(Box.createHorizontalGlue());
					tmp3.add(Box.createVerticalStrut(topstrut));
					autoTrackCheckBox=new JCheckBox("Track Gripper");
					autoTrackCheckBox.addActionListener(this);
					autoTrackCheckBox.setSelected(true);
					tmp3.add(autoTrackCheckBox);
					tmp2.add(tmp3);
				}
				camera=new PointPick(false);
				camera.addPointPickedListener(this);
				Dimension d=new Dimension(225,225);
				camera.setPreferredSize(d);
				tmp2.add(camera);
				tmp.add(tmp2,BorderLayout.CENTER);
			}
			p.add(tmp);
		}
		getContentPane().add(p,BorderLayout.CENTER);
		{
			Box tmp2=Box.createHorizontalBox();
			tmp2.add(Box.createHorizontalStrut(strutsize));
			{
				Box tmp3=Box.createVerticalBox();
				{
					JPanel tmp3b = new JPanel(new GridLayout(1,2,50,strutsize));
					
					// GRIPPER
					Box tmpL=Box.createHorizontalBox();
					{
						Box tmp4=Box.createVerticalBox();
						gripperAngleSlider=new JSlider(-slidermax,slidermax,0);
						gripperAngleSlider.addChangeListener(this);
						tmp4.add(gripperAngleSlider);
						{
							Box tmp5=Box.createHorizontalBox();
							JLabel tl;
							tmp5.add(tl=new JLabel("Face Left"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(new JLabel("Angle"));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(tl=new JLabel("Face Right"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp4.add(tmp5);
						}
						gripperSlider=new JSlider(0,slidermax,slidermax);
						gripperSlider.addChangeListener(this);
						tmp4.add(gripperSlider);
						{
							Box tmp5=Box.createHorizontalBox();
							JLabel tl;
							tmp5.add(tl=new JLabel("Close"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(new JLabel("Gripper"));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(tl=new JLabel("Open"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp4.add(tmp5);
						}
						tmpL.add(tmp4);
					}
					{
						Box tmp4=Box.createVerticalBox();
						autoAngleCheckBox=new JCheckBox("Auto");
						autoAngleCheckBox.addActionListener(this);
						autoAngleCheckBox.setSelected(true);
						tmp4.add(autoAngleCheckBox);
						tmp4.add(Box.createVerticalGlue());
						tmpL.add(tmp4);
					}
					tmp3b.add(tmpL);
					
					// CAMERA
					Box tmpR=Box.createHorizontalBox();
					{
						Box tmp4=Box.createVerticalBox();
						perspectiveSlider=new JSlider(-slidermax,0,-slidermax);
						perspectiveSlider.addChangeListener(this);
						tmp4.add(perspectiveSlider);
						{
							Box tmp5=Box.createHorizontalBox();
							JLabel tl;
							tmp5.add(tl=new JLabel("Overhead"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(new JLabel("Angle"));
							tmp5.add(Box.createHorizontalGlue());
							tmp5.add(tl=new JLabel("Parallel"));
							tl.setFont(tl.getFont().deriveFont(tl.getFont().getSize2D()*.8f));
							tmp4.add(tmp5);
						}
						tmp4.add(Box.createVerticalGlue());
						tmpR.add(tmp4);
					}
					{
						Box tmp4=Box.createVerticalBox();
						autoPerspectiveCheckBox=new JCheckBox("Auto");
						autoPerspectiveCheckBox.addActionListener(this);
						autoPerspectiveCheckBox.setSelected(true);
						tmp4.add(autoPerspectiveCheckBox);
						tmp4.add(Box.createVerticalGlue());
						tmpR.add(tmp4);
					}
					tmp3b.add(tmpR);
					
					tmp3.add(tmp3b);
				}
				
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

		gripper.setEnabled(false);
		camera.setEnabled(false);
		addWindowListener(new CloseRoverAdapter(this));
	}
}
