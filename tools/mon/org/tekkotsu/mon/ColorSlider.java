package org.tekkotsu.mon;

import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;

import org.tekkotsu.mon.ControllerGUI;
import org.tekkotsu.mon.ColorConverter;

public class ColorSlider extends JFrame implements ChangeListener, ActionListener {

    //functionality for the accessibility button's box
    //Code by Bruce Hill and James Criscuolo, Spring 2012

    static Preferences prefs = Preferences.userNodeForPackage(ColorSlider.class);
    private ControllerGUI controllerGUI;
    private JSlider r,g,b;
    private JButton buttonS,buttonP,buttonD,buttonT;

    public ColorSlider(ControllerGUI _controllerGUI) {
        super();
        controllerGUI = _controllerGUI;
        int[] defaults = ColorConverter.getSliders();
    
        Container content = getContentPane();
        
        Box box = Box.createVerticalBox();
        
        Box visionTypes = Box.createHorizontalBox();
        
        buttonS = new JButton("Standard");
		buttonS.addActionListener(this);
		buttonS.setToolTipText("Change vision type to standard");
		buttonS.setActionCommand("Standard");
		buttonS.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL);
		visionTypes.add(buttonS);
		
		buttonP = new JButton("Protanope");
		buttonP.addActionListener(this);
		buttonP.setToolTipText("Change vision type to protanope");
		buttonP.setActionCommand("Protanope");
		buttonP.setEnabled(ColorConverter.visionType != ColorConverter.PROTANOPE);
		visionTypes.add(buttonP);
		
		buttonD = new JButton("Deuteranope");
		buttonD.addActionListener(this);
		buttonD.setToolTipText("Change vision type to deuteranope");
		buttonD.setActionCommand("Deuteranope");
		buttonD.setEnabled(ColorConverter.visionType != ColorConverter.DEUTERANOPE);
		visionTypes.add(buttonD);
		
		buttonT = new JButton("Tritanope");
		buttonT.addActionListener(this);
		buttonT.setToolTipText("Change vision type to tritanope");
		buttonT.setActionCommand("Tritanope");
		buttonT.setEnabled(ColorConverter.visionType != ColorConverter.TRITANOPE);
		visionTypes.add(buttonT);
		
		
		box.add(visionTypes);
        
        r = new JSlider(0,400,defaults[0]);
        r.setBorder(BorderFactory.createTitledBorder("Red"));
		r.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.PROTANOPE);
        box.add(r);//, BorderLayout.NORTH);
        r.addChangeListener(this);
        
        g = new JSlider(0,400,defaults[1]);
        g.setBorder(BorderFactory.createTitledBorder("Green"));
		g.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.DEUTERANOPE);
        box.add(g);//, BorderLayout.CENTER);
        g.addChangeListener(this);
        
        b = new JSlider(0,400,defaults[2]);
        b.setBorder(BorderFactory.createTitledBorder("Blue"));
		b.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.TRITANOPE);
        box.add(b);//, BorderLayout.SOUTH);
        b.addChangeListener(this);
        
        content.add(box,BorderLayout.NORTH);
        
        
        pack();
        setVisible(true);
    }

    private void refresh() {
        buttonS.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL);
	buttonP.setEnabled(ColorConverter.visionType != ColorConverter.PROTANOPE);
	buttonD.setEnabled(ColorConverter.visionType != ColorConverter.DEUTERANOPE);
	buttonT.setEnabled(ColorConverter.visionType != ColorConverter.TRITANOPE);
		
	r.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.PROTANOPE);
	g.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.DEUTERANOPE);
	b.setEnabled(ColorConverter.visionType != ColorConverter.NORMAL && ColorConverter.visionType != ColorConverter.TRITANOPE);

	int[] sliderPositions = ColorConverter.getSliders();
	prefs.putInt("ColorConverter.visionType",ColorConverter.visionType);
        prefs.putInt("ColorConverter.sliderVals[0]",sliderPositions[0]);
        prefs.putInt("ColorConverter.sliderVals[1]",sliderPositions[1]);
        prefs.putInt("ColorConverter.sliderVals[2]",sliderPositions[2]);
		
        if (controllerGUI.cameraSkGUI != null)
            controllerGUI.cameraSkGUI.refreshColors();
        if (controllerGUI.localSkGUI != null)
            controllerGUI.localSkGUI.refreshColors();
        if (controllerGUI.worldSkGUI != null)
            controllerGUI.worldSkGUI.refreshColors();
    }
    
    public void actionPerformed(ActionEvent e) {
        ColorConverter.hasChanged = true;
        if (e.getActionCommand().equals("Standard")) {
            ColorConverter.visionType = ColorConverter.NORMAL;
        } else if (e.getActionCommand().equals("Protanope")) {
            ColorConverter.visionType = ColorConverter.PROTANOPE;
        } else if (e.getActionCommand().equals("Deuteranope")) {
            ColorConverter.visionType = ColorConverter.DEUTERANOPE;
        } else if (e.getActionCommand().equals("Tritanope")) {
            ColorConverter.visionType = ColorConverter.TRITANOPE;
        }
        refresh();
    }

    
    public void stateChanged(ChangeEvent e) {
        ColorConverter.hasChanged = true;
        ColorConverter.setSliders((int)r.getValue(), (int)g.getValue(), (int)b.getValue());
        refresh();
    }
    
    public void close() {
        dispose();
    }

}


