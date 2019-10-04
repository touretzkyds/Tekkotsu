/*
 * ArmGUI is the java interface that interacts with the C++ side of Tekkotsu.
 * Commands that it can receive:
 
 * Connected - Receive from ArmListener to inform that a connection
 * has been established between ArmListener and ArmControllerBehavior
 
 * Commands that it can send:
 
 * ("z", [cmdno],[placeholder- default = 0.0f]) - Inform ArmControllerBehavior that is ready to receive the
 * parameters of the arm it is working with. Parameters currently include:
 - NOJ -> Number of joints on the arm
 - armJointName -> The name of the joint
 - armJointMax -> The maximum limit of a joint
 - armJointMin -> The minimum limit of a joint
 - armStartValue -> The current value of the joint [It is only updated at the start so that we can update the sliders accordingly.
*/

package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.prefs.Preferences;
import java.util.StringTokenizer;
import java.util.concurrent.Semaphore;
import java.lang.Math;
import java.text.DecimalFormat;

public class ArmGUI extends JFrame implements PointPick.PointPickedListener, ChangeListener, 
					      ActionListener, MouseListener, ArmListener.ArmUpdatedListener {
    static int defPort=10054; // default port to communicate 
	
    // Point Pickers
    PointPick horizontalPP;    
    PointPick verticalPP;
	
    // Either horizontalPP or verticalPP
    PointPick gripperPP;
	
    // Sliders
    JSlider armJointSlider[];
    JSlider speedSlider;
    JSlider gripperSlider;
	
    // Radio Buttons
    JRadioButton radButton;
    JRadioButton degreeButton;
	
    // Buttons
    JButton centerButton;
    JButton relaxButton;
    JButton reconnectButton;
	
    // Labels
    JLabel armJointStatus[];
    JLabel speedLabel;
    JLabel status;
	
    // Combo box (orientation)
    JComboBox orientation;
    String orientationNames[];
    int primes[];
    int orientationChoices;
	
    String armJointName[];
    String armConfig[];
	
    float armJointMax[];
    float armJointMin[];
    float armStartValue[];
    float horizontalCoord[][];
    float verticalCoord[][]; 
    float gripperAngles[];
	
    int NOJ = 0;
    int planeConfig = 0;
    int wristOrientJoint = -1;  // joint that controls wrist orientation, if any
    int firstFingerJoint = -1; // joint number of the first finger, if any

    // whether the arm has a gripper
    boolean hasGripper;
	
    // whether or not we've already created the frame, in case the controller re-loads
    boolean frameCreated = false;
	
    Semaphore cmdmutex = new Semaphore(1);
    int cmdno = 0;
    Semaphore lastcmdnomutex = new Semaphore(1);
    int lastcmdno[];
	
    ArmListener comm;
	
    float flagdeg = 1.0f; // A flag to determine if the interface should output rad or degree. Value of 1 stands for rad.
    DecimalFormat myFormatter = new DecimalFormat("#0.00");
	
    static int sliderMax = 10000;
    final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
    static Preferences prefs = Preferences.userNodeForPackage(ArmGUI.class);
	
    public void main(String s[]) {
	int port=defPort;
	if (s.length<1)
	    usage();
	if (s.length>1)
	    port=Integer.parseInt(s[1]);
	String[] args=new String[s.length-1];
	for (int i=0; i<s.length-1; i++)
	    args[i-1]=s[i];
	// Have to initialize all the variable before calling JFrame;
	JFrame frame=new ArmGUI(s[0],port,args);
    }
	
    public static void usage() {
	System.out.println("Usage: java ArmGUI host [port]");
	System.out.println("       if port is not specified, it defaults to: "+defPort);
	System.exit(2);
    }
    // Constructor
    public ArmGUI(String host, int port, String args[]) {
	super("TekkotsuMon: Arm Control");
	comm = new ArmListener(host,port);  // Create a commucation instance
	comm.addArmUpdatedListener(this); // Send to the ArmListener object an ArmGUI to establish the commucation
		
	orientationNames = new String[] {"Side Grip", "Overhead Grip", "Unconstrained Grip"};
	primes = new int[] {2, 3, 5};
    }
	
    public void close() {
	prefs.putInt("ArmGUI.location.x",getLocation().x);
	prefs.putInt("ArmGUI.location.y",getLocation().y);
	comm.kill();
	dispose();
    }
	
    class CloseArmAdapter extends WindowAdapter {
	ArmGUI gui;
	CloseArmAdapter(ArmGUI gui) {this.gui=gui;}
		
	public void windowClosing(WindowEvent e) {
	    gui.close();
	}
    }
	
    /*
     * Handle the sending of the commands to Tekkotsu
     * when there is a change in the point in PointPick
     */
    public void pointPicked(Point2D.Float p, MouseEvent e, PointPick pp) {
	try {
	    cmdmutex.acquire();
	    lastcmdnomutex.acquire();
			
	    /*
	     * Check to see if the arm is relaxed
	     * If yes, unrelax it before continuing
	     */
			
	    if ((relaxButton.getText()).equals("Unrelax")) {
		comm.sendCommand("u", cmdno++, 0.0f);  // CMD_unrelax
		relaxButton.setText("Relax");
	    }
			
	    /*
	     * Send the coordinate given by the point picker
	     */
			
	    // Choose between which plane to use
	    if (this.horizontalPP.equals(pp)) {
		comm.sendCommand("x", cmdno++, p.x, p.y, 1.0);
	    }
	    else if (this.verticalPP.equals(pp)) {
		comm.sendCommand("x", cmdno++, p.x, p.y, 2.0);
	    }
			
	    lastcmdnomutex.release();
	    cmdmutex.release();
	}
	catch(Exception epp) {
	    System.out.println("ArmGUI exception in pointPicked: "+e);
	}
    }
	
    public void ArmUpdated(ArmListener comm) {
	if (status != null) {
	    horizontalPP.setEnabled(comm._isConnected);
	    verticalPP.setEnabled(comm._isConnected);
	    for (int counter = 0; counter < NOJ; counter ++) {
		armJointSlider[counter].setEnabled(comm._isConnected);
	    }
			
	    speedSlider.setEnabled(comm._isConnected);
	    if (hasGripper)  {
		gripperSlider.setEnabled(comm._isConnected);
	    }
			
	    centerButton.setEnabled(comm._isConnected);
	    radButton.setEnabled(comm._isConnected); 
	    degreeButton.setEnabled(comm._isConnected); 
	    relaxButton.setEnabled(comm._isConnected);
	    orientation.setEnabled(comm._isConnected);
			
	    try {
		cmdno++;
		lastcmdnomutex.release();
		cmdmutex.release();
	    }	
	    catch(Exception e) {
		System.out.println("ArmGUI exception in ArmUpdated: "+e);
	    }
			
	    if (comm._isConnected)
		status.setText("Connected.");
	    else
		status.setText("Reconnecting...");
	} 
    }
	
    /* Given an angle in degrees or radians, truncate it appropriately. */
    public String truncateAngle(float trunk) {
	return myFormatter.format(trunk*flagdeg);
    }
	
    /* Maps a normalized value val, where 0 <= val <= 1, to the range (min, max) */
    public float map(float val, float min, float max) {
	return val * min + (1.0f - val) * max;
    }
	
    /* Determines whether the Horizontal or Vertical plane
     * should display the gripper. Depends on the
     * orientation of the wrist rotator, if such a joint exists. */
    private void determineGripperPointPicker() {
	if (!hasGripper || horizontalPP == null || verticalPP == null)
	    return;
		
	float wristrot =  (wristOrientJoint == -1) ? 0.5f : armJointSlider[wristOrientJoint].getValue()/(float)sliderMax;
	// System.out.println("determineGripperPointPicker: wristrot=" + wristrot);
	if ((wristrot >= 0.25 && wristrot <= 0.85) || (wristrot <= -0.25 && wristrot >= -0.85)) {
	    gripperPP = horizontalPP;
	    gripperPP.setHasGripper(true);
	    verticalPP.setHasGripper(false);
	}
	else {
	    gripperPP = verticalPP;
	    gripperPP.setHasGripper(true);
	    horizontalPP.setHasGripper(false);
	}
	sendGripper();
    }
	
    private void sendGripper() {
	float wristrot = (wristOrientJoint == -1) ? 0 : armJointSlider[wristOrientJoint].getValue()/(float)sliderMax;
	float gripperAnglesTemp[] = new float[2];
	if (wristrot < -.25 || wristrot > .75) {
	    gripperAnglesTemp[0] = 2*(float)Math.PI-gripperAngles[1];
	    gripperAnglesTemp[1] = 2*(float)Math.PI-gripperAngles[0];
	}
	else
	    gripperAnglesTemp = gripperAngles;
	gripperPP.doSetGripper(gripperAnglesTemp);
    }
	
    /*
     * Handle all the commands that were recieved from ArmControllerBehavior
     * through ArmListener
     */
    public void ArmUpdated(String msg) {
	// System.out.println("ArmGUI got: "+msg);
	String temp = "(nothing)";
	try {
	    cmdmutex.acquire();
	    lastcmdnomutex.acquire();
	}
	catch(Exception e) {
	    System.out.println("ArmGUI exception in ArmUpdated: "+e+" in command '" + temp + "': " + e.toString());
	}
	StringTokenizer st = new StringTokenizer(msg);
	temp = st.nextToken().toString();
	/*
	 * After ArmListener has established a connection with ArmControllerBehavior
	 * it will pass control to ArmGUI which will in turn attempt to obtain the
	 * parameters of the arm that it is working on from ArmControllerBehavior
	 */
	if (temp.equals("Connected")) {
	    comm.sendCommand("z", cmdno++, 0.0f); // CMD_connect
	}
	/*
	 * Recieve the number of joints for the arm and initialize the variables
	 */
	else if (temp.equals("NOJ")) {
	    NOJ = Integer.parseInt(st.nextToken());
	    planeConfig = Integer.parseInt(st.nextToken()); // Set the plane configuration
	    orientationChoices = Integer.parseInt(st.nextToken()); // Set the orientation configuration
	    System.out.println("ArmGUI NOJ=" + NOJ + " planeConfig=" + planeConfig + " orientationChoices=" + orientationChoices);
	    armJointName = new String[NOJ];
	    lastcmdno = new int[NOJ];
	    armJointMin = new float[NOJ];
	    armJointMax = new float[NOJ];
	    armStartValue = new float[NOJ];
	    armConfig = new String[NOJ];
	    for (int counter = 0; counter < NOJ; counter++)
		lastcmdno[counter] = 0;
	}
	/*
	 * Receive the arm configuration
	 */
	else if (temp.equals("Config")) {
	    int i = Integer.parseInt(st.nextToken());
	    armConfig[i] = String.valueOf(st.nextToken());
	    System.out.println("ArmGUI Config: joint " + i + " config '" + armConfig[i] + "'");
	}
	/*
	 * Receive the joint parameters
	 */
	else if (temp.equals("JointParam")) {
	    int curJoint = Integer.parseInt(st.nextToken());
	    armJointName[curJoint] = st.nextToken();
	    armJointMax[curJoint] = Float.parseFloat(st.nextToken());
	    armJointMin[curJoint] = Float.parseFloat(st.nextToken());
	    armStartValue[curJoint] = Float.parseFloat(st.nextToken());
	    System.out.println("ArmGUI JointParam: joint " + curJoint + " (" + armJointName[curJoint] +
			       ") max=" + armJointMax[curJoint] + " min=" + armJointMin[curJoint] +
			       " start=" + armStartValue[curJoint]);
	    /*
	     * once all the joint values are received, create the interface.
	     */
	    if (curJoint+1 == NOJ) {
		if (!frameCreated) {
		    createFrame();
		    frameCreated = true;
		    if (hasGripper)
			gripperSlider.setValue(-armJointSlider[NOJ-1].getValue());
		}
					
		// have to remove action listener, otherwise errors are generated
		orientation.removeActionListener(this);
		orientation.removeAllItems();
					
		// re-add everything
		for (int i = 0; i < primes.length; i++)
		    if (orientationChoices % primes[i] == 0)
			orientation.addItem(orientationNames[i]);
					
		// now we can listen again
		orientation.addActionListener(this);
		orientation.setSelectedItem(0);
	    }
	}
	else if (temp.equals("JointTypes")) {
	    int hor = Integer.parseInt(st.nextToken());
	    int ver = Integer.parseInt(st.nextToken());
	    int fingers = Integer.parseInt(st.nextToken());

	    if (NOJ > hor+ver+fingers)
		wristOrientJoint = hor + ver;
				
	    System.out.println("ArmGUI JointTypes: #yaw=" + hor + " #pitch=" + ver +
			       " #fingers=" + fingers + ".  wristOrientJoint is " + wristOrientJoint + ".");

	    if (fingers > 0) {
		hasGripper = true;
		firstFingerJoint = NOJ - fingers;
		gripperAngles = new float[2];
		gripperAngles[0] = gripperAngles[1] = 0;
	    }
	    else
		hasGripper = false;
	    // Initialize the Coordinates for the planes being used
	    if (hor > 0) {
		horizontalCoord = new float[hor][2];
		for (int i =0; i < horizontalCoord.length; i++) {
		    horizontalCoord[i][0] = 0;
		    horizontalCoord[i][1] = 0;
		}
	    }
	    if (ver > 0) {
		verticalCoord = new float[ver][2];
		for (int i =0; i < verticalCoord.length; i++) {
		    verticalCoord[i][0] = 0;
		    verticalCoord[i][1] = 0;
		}
	    }
	}
	else if (temp.equals("CoordH")) {
	    // Set the segments for horizontal plane
	    int tempJoint = Integer.parseInt(st.nextToken());
				
	    horizontalCoord[tempJoint][0] = Float.parseFloat(st.nextToken());
	    horizontalCoord[tempJoint][1] = Float.parseFloat(st.nextToken());
				
	    horizontalPP.doSetArm(horizontalCoord);
	    if (hasGripper)
		sendGripper();
	}
	else if (temp.equals("CoordV")) {
	    // Set the segments for the vertical plane
	    int t = Integer.parseInt(st.nextToken());
				
	    verticalCoord[t][0] = Float.parseFloat(st.nextToken());
	    verticalCoord[t][1] = Float.parseFloat(st.nextToken());
				
	    verticalPP.doSetArm(verticalCoord);
	    if (hasGripper)
		sendGripper();
	}
	else if (temp.equals("Value")) {
	    // Receive-> Value [comno] [jointNum] [jointValue]
	    int curcmdno = Integer.parseInt(st.nextToken());
	    int jointNum = Integer.parseInt(st.nextToken());
	    float jointValue = Float.parseFloat(st.nextToken());
	    if (curcmdno > lastcmdno[jointNum]  || curcmdno == -1) {
		// Very Ugly
		armJointSlider[jointNum].removeChangeListener(this);
		armJointSlider[jointNum].setValue((int)( ((jointValue*2)-1)*sliderMax ));
		armJointStatus[jointNum].setText( armJointName[jointNum] + ": " + truncateAngle(map(jointValue,armJointMin[jointNum],armJointMax[jointNum])) );
		armJointSlider[jointNum].addChangeListener(this);
		comm.sendCommand(String.valueOf((char)jointNum), curcmdno, jointValue);
		lastcmdno[jointNum] = curcmdno;
	    }
	    if (jointNum >= firstFingerJoint) {
		gripperAngles[jointNum-firstFingerJoint] = jointValue;
		sendGripper();
	    }
	}
	else if (temp.equals("Gripper")) {
	    // Receive-> Value [comno] [jointNum] [jointValue]
	    int curcmdno = Integer.parseInt(st.nextToken());
	    int jointNum = Integer.parseInt(st.nextToken());
	    float jointValue = Float.parseFloat(st.nextToken());
	    if (curcmdno > lastcmdno[jointNum]) {
		// Very Ugly
		armJointSlider[jointNum].removeChangeListener(this);
		armJointSlider[jointNum].setValue((int)( ((jointValue*2)-1)*sliderMax ));
		armJointStatus[jointNum].setText( armJointName[jointNum] + ": " + truncateAngle(map(jointValue, armJointMin[jointNum], armJointMax[jointNum])) );
		armJointSlider[jointNum].addChangeListener(this);
		comm.sendCommand(String.valueOf((char)(jointNum + 30)), curcmdno, jointValue);
		lastcmdno[jointNum] = curcmdno;
	    }
	    gripperAngles[jointNum-firstFingerJoint] = -(jointValue * (armJointMax[jointNum] - armJointMin[jointNum])) + armJointMax[jointNum];
	    sendGripper();
	}
	else if (temp.equals("PP")) {
	    // Receive-> Value [comno] [jointNum] [jointValue]
	    int curcmdno = Integer.parseInt(st.nextToken());
	    int jointNum = Integer.parseInt(st.nextToken());
	    float jointValue = Float.parseFloat(st.nextToken());
	    if (curcmdno > lastcmdno[jointNum]) {
		// Very Ugly
		armJointSlider[jointNum].removeChangeListener(this);
		armJointSlider[jointNum].setValue((int)( ((jointValue*2)-1)*sliderMax ));
		armJointStatus[jointNum].setText( armJointName[jointNum] + ": " + truncateAngle(map(jointValue, armJointMin[jointNum], armJointMax[jointNum])) );
		armJointSlider[jointNum].addChangeListener(this);
		//comm.sendCommand(String.valueOf((char)(jointNum + 30)), curcmdno, jointValue);
		lastcmdno[jointNum] = curcmdno;
	    }
	}
	else if (temp.equals("SuccessH")) {
	    float x = Float.parseFloat(st.nextToken());
	    float y = Float.parseFloat(st.nextToken());
	    horizontalPP.paintPoint(x, y);
	}
	else if (temp.equals("SuccessV")) {
	    float x = Float.parseFloat(st.nextToken());
	    float y = Float.parseFloat(st.nextToken());
	    verticalPP.paintPoint(x, y);
	}
	else if (temp.equals("ClearPtsH")) {
	    if (horizontalPP != null)
		horizontalPP.clearReachablePoints();
	}
	else if (temp.equals("ClearPtsV")) {
	    if (verticalPP != null)
		verticalPP.clearReachablePoints();
	}
	else if (temp.equals("RepaintH")) {
	    if (horizontalPP != null)
		horizontalPP.repaint();
	}
	else if (temp.equals("RepaintV")) {
	    if (verticalPP != null)
		verticalPP.repaint();
	}
	else if (temp.equals("RedValH")) {
	    float rx = Float.parseFloat(st.nextToken());
	    float ry = Float.parseFloat(st.nextToken());
	    horizontalPP.doSetPoint(rx, ry);
	}
	else if (temp.equals("RedValV")) {
	    float rx = Float.parseFloat(st.nextToken());
	    float ry = Float.parseFloat(st.nextToken());
	    verticalPP.doSetPoint(rx, ry);
	}
	else if (temp.equals("GreenValH")) {
	    float gx = Float.parseFloat(st.nextToken());
	    float gy = Float.parseFloat(st.nextToken());
	    horizontalPP.doSetGPoint(gx, gy);
	}
	else if (temp.equals("GreenValV")) {
	    float gx = Float.parseFloat(st.nextToken());
	    float gy = Float.parseFloat(st.nextToken());
	    verticalPP.doSetGPoint(gx, gy);
	}
	else {
	    System.out.println("Unrecognized arm command '"+msg+"'");
	}
	lastcmdnomutex.release();
	cmdmutex.release();
    }

    /* These currently do nothing */
    public void mouseClicked(MouseEvent e) {}
    public void mouseEntered(MouseEvent e) {}
    public void mouseExited(MouseEvent e) {}
    public void mousePressed(MouseEvent e) {}
    public void mouseReleased(MouseEvent e) {}
	
    /*
     * Track which slider bar change and take the appropriate actions
     * If the arm is relaxed, it will unrelax the arm first
     */
    public void stateChanged(ChangeEvent e) {
	try {
	    cmdmutex.acquire();
	    lastcmdnomutex.acquire();
	} catch (Exception error) {}

	/*
	 * Change the speed of the arm
	 */ 
	if (e.getSource() == speedSlider) {
	    float newAngle = speedSlider.getValue() / (float)sliderMax + 0.1f;
	    comm.sendCommand("y", cmdno++, newAngle);
	    if (flagdeg == 1.0f) 
		speedLabel.setText("Speed: " + truncateAngle(newAngle) + " rad/s.");
	    else 
		speedLabel.setText("Speed: " + truncateAngle(newAngle) + " deg/s.");
	} /* Change the value of the relax button */
	else if (e.getSource() == gripperSlider) {
	    if  ((relaxButton.getText()).equals("Unrelax")) {
		comm.sendCommand("u", cmdno++, 0.0f);
		relaxButton.setText("Relax");
	    }
				
	    comm.sendCommand("w", cmdno++, gripperSlider.getValue() / (float)sliderMax);
	}
	else {
	    if ((relaxButton.getText()).equals("Unrelax")) {
		comm.sendCommand("u", cmdno++, 0.0f);
		relaxButton.setText("Relax");
	    }
	    for (int counter = 0; counter < NOJ; counter ++) {
		if (e.getSource() == armJointSlider[counter]) {
		    lastcmdno[counter] = cmdno;
		    float normVal = ( (armJointSlider[counter].getValue()/(float)sliderMax) + 1 ) / 2;
		    comm.sendCommand(String.valueOf((char)counter), cmdno++, normVal);
		    armJointStatus[counter].setText( armJointName[counter] + ": " + truncateAngle(map(normVal, armJointMin[counter], armJointMax[counter])) );
		}
	    }
	    if (hasGripper && wristOrientJoint != -1 && e.getSource() == armJointSlider[wristOrientJoint]) {
		determineGripperPointPicker();
	    }
	}
			
	lastcmdnomutex.release();
	cmdmutex.release();
    }
	
    public void actionPerformed(ActionEvent e) {
	if (e.getSource() == centerButton) {
	    for (int counter = 0; counter < NOJ; counter++) {
		armJointSlider[counter].setValue((int)(((armJointMax[counter] / (armJointMax[counter] - armJointMin[counter])) - 0.5)* 2 * sliderMax));
		armJointStatus[counter].setText(armJointName[counter] + ": 0.00");
	    }
			
	    if (hasGripper)  {
		gripperSlider.removeChangeListener(this);
		gripperSlider.setValue(-armJointSlider[NOJ-1].getValue());
		gripperSlider.addChangeListener(this);
	    }
	} 
	else if (e.getSource() == relaxButton) {
	    if  ((relaxButton.getText()).equals("Relax")) {
		comm.sendCommand("v", cmdno++, 0.0f);
		relaxButton.setText("Unrelax");
	    }
	    else {
		comm.sendCommand("u", cmdno++, 0.0f);
		relaxButton.setText("Relax");
	    }
	}
		
	else if (e.getSource() == reconnectButton) {
	    int port=comm._port;
	    String addr=comm._host;
	    comm.kill();
	    comm.removeArmUpdatedListener(this);
	    comm = new ArmListener(comm._host,comm._port);
	    comm.addArmUpdatedListener(this);
	}
	else if (e.getSource() == radButton) {
	    flagdeg = 1.0f;
	    for (int counter = 0; counter < NOJ; counter ++) {
		float normVal = ( (armJointSlider[counter].getValue()/(float)sliderMax) + 1 ) / 2;
		// Val = %*Min + (1 - %)*Max
		armJointStatus[counter].setText( armJointName[counter] + ": " + truncateAngle(map(normVal, armJointMin[counter], armJointMax[counter])) );
	    }
	    speedLabel.setText("Speed: " + truncateAngle(speedSlider.getValue() / (float)sliderMax) + " rad/s.");
	}
	else if (e.getSource() == degreeButton) {
	    flagdeg = 180.0f / (float)(Math.PI);
	    for (int counter = 0; counter < NOJ; counter ++) {
		float normVal = ( (armJointSlider[counter].getValue()/(float)sliderMax) + 1 ) / 2;
		// Val = %*Min + (1 - %)*Max
		armJointStatus[counter].setText( armJointName[counter] + ": " + truncateAngle(map(normVal, armJointMin[counter], armJointMax[counter])) );
	    }
	    speedLabel.setText("Speed: " + truncateAngle(speedSlider.getValue() / (float)sliderMax ) + " deg/s.");
	}
	else if (e.getSource() == orientation) {
	    for (int i = 0; i < primes.length; i++)
		if (orientation.getSelectedItem().equals(orientationNames[i]))
		    comm.sendCommand("o", cmdno++, i);
	}
    }
	
    // Create the GUI interface
    public void createFrame() {
	setMinimumSize(new Dimension(400, 300));
		
	armJointSlider = new JSlider[NOJ];
	armJointStatus = new JLabel[NOJ];
		
	// some commonly used dimensions
	int strutSize = 15; // Size of the window
	int sepSize = 1; // Size of separators
	int labWidth = 45; // Width of text labels
	Dimension ppDims = new Dimension(200, 200); // Size of Point Pickers
	JLabel dummy = new JLabel("");
	Font font = dummy.getFont();
		
	getContentPane().setLayout(new BorderLayout());
	getContentPane().add(Box.createHorizontalStrut(strutSize),BorderLayout.LINE_END);
	getContentPane().add(Box.createHorizontalStrut(strutSize),BorderLayout.LINE_START);
	getContentPane().add(Box.createVerticalStrut(strutSize),BorderLayout.PAGE_START);
		
	// Main Panel
	JPanel mainPanel = new JPanel(new SquareRightLayout());
	mainPanel.setLayout(new SquareRightLayout());
		
	// First Point Picker
	horizontalPP = new PointPick(false, true, horizontalCoord);
	horizontalPP.addPointPickedListener(this);
	horizontalPP.addMouseListener(this);
		
	// Second Point Picker
	verticalPP = new PointPick(false, true, verticalCoord);
	verticalPP.addPointPickedListener(this);
	verticalPP.addMouseListener(this);
	// The vertical arm will have its origin on the left side
	verticalPP.setOrigPoint(-0.98f, 0.0f);
		
	// ========== LEFT SIDE ==========
		
	JPanel leftSide = new JPanel(new GridLayout(2,1));
		
	// ====== HORIZONTAL AND VERTICAL POINT PICKERS =====
		
	JPanel pointPanes = new JPanel(new BorderLayout());
		
	// Horizontal and Vertical
	if (planeConfig == 0) {
	    Box horizontal = Box.createVerticalBox();
	    horizontal.setAlignmentX(Component.CENTER_ALIGNMENT);
			
	    horizontal.add(new JLabel("Horizontal Plane"));
	    horizontal.add(horizontalPP);
	    horizontal.setPreferredSize(ppDims);
	    pointPanes.add(horizontal, BorderLayout.LINE_START);
			
	    Box vertical = Box.createVerticalBox();
	    vertical.setAlignmentX(Component.CENTER_ALIGNMENT);
			
	    vertical.add(new JLabel("Vertical Plane"));
	    vertical.add(verticalPP);
	    vertical.setPreferredSize(ppDims);
	    pointPanes.add(vertical, BorderLayout.LINE_END);
	}
	// Just Horizontal
	else if (planeConfig == 1) {
	    JLabel hor = new JLabel("Horizontal Plane");
			
	    hor.setAlignmentX(Component.CENTER_ALIGNMENT);
	    horizontalPP.setAlignmentX(Component.CENTER_ALIGNMENT);
	    pointPanes.setAlignmentX(Component.CENTER_ALIGNMENT);
			
	    pointPanes.add(hor, BorderLayout.PAGE_START);
	    pointPanes.add(horizontalPP, BorderLayout.CENTER);
	}
	// Just Vertical
	else {
	    JLabel ver = new JLabel("Vertical Plane");
			
	    ver.setAlignmentX(Component.CENTER_ALIGNMENT);
	    verticalPP.setAlignmentX(Component.CENTER_ALIGNMENT);
	    pointPanes.setAlignmentX(Component.CENTER_ALIGNMENT);
			
	    pointPanes.add(ver, BorderLayout.PAGE_START);
	    pointPanes.add(verticalPP, BorderLayout.CENTER);
	}
	leftSide.add(pointPanes);
		
	// ===== SLIDERS (SPEED / GRIPPER) =====
		
	JPanel sliders = new JPanel(new GridBagLayout());
	GridBagConstraints c = new GridBagConstraints();
		
	c.gridy = 0;
	c.gridx = 0;
		
	speedLabel = new JLabel("Speed: 0.40 rad/s.");
	sliders.add(speedLabel, c);
		
	c.gridx = 1;
		
	JLabel lab = new JLabel("Slower");
	lab.setFont(font.deriveFont(font.getSize2D()-2));
	lab.setHorizontalAlignment(SwingConstants.RIGHT);
	lab.setPreferredSize(new Dimension(labWidth,font.getSize()));
	sliders.add(lab, c);
		
	c.gridx = 2;
		
	speedSlider = new JSlider(1,sliderMax,4000);
	speedSlider.addChangeListener(this);
	speedSlider.setMinimumSize(new Dimension(150,30));
	sliders.add(speedSlider, c);
		
	c.gridx = 3;
		
	lab = new JLabel("Faster");
	lab.setFont(font.deriveFont(font.getSize2D()-2));
	lab.setHorizontalAlignment(SwingConstants.LEFT);
	lab.setPreferredSize(new Dimension(labWidth,font.getSize()));
	sliders.add(lab, c);
		
	// === GRIPPER SLIDER ===
		
	if (hasGripper)  {
	    c.gridy = 1;
	    c.gridx = 0;
			
	    lab = new JLabel("Gripper");
	    sliders.add(lab, c);
			
	    c.gridx = 1;
			
	    lab = new JLabel("Closed");
	    lab.setFont(font.deriveFont(font.getSize2D()-2));
	    lab.setHorizontalAlignment(SwingConstants.RIGHT);
	    lab.setPreferredSize(new Dimension(labWidth,font.getSize()));
	    sliders.add(lab, c);
			
	    c.gridx = 2;
			
	    gripperSlider=new JSlider(-sliderMax,sliderMax,0);
	    gripperSlider.addChangeListener(this);
	    gripperSlider.setMinimumSize(new Dimension(150,30));
	    sliders.add(gripperSlider, c);
			
	    c.gridx = 3;
			
	    lab=new JLabel("Open");
	    lab.setFont(font.deriveFont(font.getSize2D()-2));
	    lab.setHorizontalAlignment(SwingConstants.LEFT);
	    lab.setPreferredSize(new Dimension(labWidth,font.getSize()));
	    sliders.add(lab, c);
	}
	leftSide.add(sliders);
	mainPanel.add(leftSide, SquareRightLayout.SQUARE);
		
	// ========== RIGHT SIDE ==========
		
	Box rightSide = Box.createHorizontalBox();
		
	rightSide.add(Box.createHorizontalStrut(strutSize));
	JSeparator sep = new JSeparator(SwingConstants.VERTICAL);
	rightSide.add(sep);
	rightSide.add(Box.createHorizontalStrut(strutSize));
		
	// ===== BUTTONS =====
		
	// declare buttons
	radButton = new JRadioButton("Radian");
	radButton.setSelected(true);
	radButton.addActionListener(this);
		
	degreeButton = new JRadioButton("Degrees");
	degreeButton.addActionListener(this);
		
	// group buttons
	ButtonGroup group = new ButtonGroup();
	group.add(radButton);
	group.add(degreeButton);
		
	// add buttons to layout
	Box buttonBox = Box.createHorizontalBox();
	buttonBox.add(radButton);
	buttonBox.add(degreeButton);
	buttonBox.setAlignmentX(0);
		
	// box to contain rad/deg, sliders, and center/relax
	Box jointControls = Box.createVerticalBox();
	jointControls.add(buttonBox);
	jointControls.add(Box.createVerticalStrut(strutSize));
		
	// ===== ORIENTATION SELECT =====
		
	orientation = new JComboBox();
	orientation.addActionListener(this);
	orientation.setPreferredSize(new Dimension(175, 25));
		
	// need to enclose in box for proper formatting
	Box orientationBox = Box.createHorizontalBox();
	orientationBox.add(new JLabel("Grip Orientation: "));
	orientationBox.add(orientation);
	orientationBox.setAlignmentX(0);
	jointControls.add(orientationBox);
	jointControls.add(Box.createVerticalStrut(strutSize));
		
	// ===== JOINT ANGLE SLIDERS =====
		
	for (int counter = 0; counter < NOJ; counter ++) {
	    // first, add status label
	    armJointStatus[counter] = new JLabel(armJointName[counter] + ": " + truncateAngle(map(armStartValue[counter], armJointMin[counter], armJointMax[counter])) );
	    jointControls.add(armJointStatus[counter]);
			
	    // add slider for each joint
	    Box joint = Box.createHorizontalBox();
	    int val = (int)((2*armStartValue[counter]-1)*sliderMax);
	    // we must be inside the limits of the slider
	    if (val > sliderMax)
		val = sliderMax;
	    if (val < -sliderMax)
		val = -sliderMax;
			
	    // initialize the slider itself
	    armJointSlider[counter] = new JSlider(-sliderMax, sliderMax, val);
	    armJointSlider[counter].addChangeListener(this);
			
	    // based on whether the joint is vertical/horizontal, we set labels appropriately.
	    JLabel leftLabel, rightLabel;
	    if (armConfig[counter].equals("V")) {
		leftLabel = new JLabel("Up");
		rightLabel = new JLabel("Down");
	    }
	    else {
		leftLabel = new JLabel("Left");
		rightLabel = new JLabel("Right");
	    }
			
	    // Format labels appropriately, add them
	    leftLabel.setFont(font.deriveFont(font.getSize2D()-2));
	    leftLabel.setHorizontalAlignment(SwingConstants.RIGHT);
	    leftLabel.setPreferredSize(new Dimension(labWidth,font.getSize()));
	    joint.add(leftLabel);
			
	    joint.add(armJointSlider[counter]);
			
	    rightLabel.setFont(font.deriveFont(font.getSize2D()-2));
	    rightLabel.setHorizontalAlignment(SwingConstants.LEFT);
	    rightLabel.setPreferredSize(new Dimension(labWidth,font.getSize()));
	    joint.add(rightLabel);
			
	    joint.setAlignmentX(0);
	    jointControls.add(joint);
			
	    jointControls.add(Box.createVerticalStrut(strutSize));
	}
		
	// ===== CENTER/RELAX =====
		
	Box bottomButtons = Box.createHorizontalBox();
	bottomButtons.add(Box.createHorizontalGlue());
		
	centerButton = new JButton("Center");
	centerButton.addActionListener(this);
		
	relaxButton = new JButton("Relax");
	relaxButton.addActionListener(this);
	rootPane.setDefaultButton(relaxButton);
		
	bottomButtons.add(centerButton);
	bottomButtons.add(relaxButton);
	bottomButtons.add(Box.createHorizontalGlue());
	bottomButtons.setAlignmentX(0);
		
	jointControls.add(bottomButtons);
	jointControls.add(Box.createVerticalGlue());
		
	// add all of the controls now to rightSide
	rightSide.add(jointControls);
		
	mainPanel.add(rightSide,SquareRightLayout.RIGHT);
	getContentPane().add(mainPanel,BorderLayout.CENTER);
		
	// ========== BOTTOM ==========
		
	{
	    Box tmp2=Box.createHorizontalBox();
	    tmp2.add(Box.createHorizontalStrut(strutSize));
	    {
		Box tmp3=Box.createVerticalBox();
		tmp3.add(Box.createVerticalStrut(strutSize));
		tmp3.add(new JSeparator());
		tmp3.add(Box.createVerticalStrut(strutSize-sepSize));
		{
		    Box tmp4=Box.createHorizontalBox();
					
		    tmp4.add(status=new JLabel("Connecting..."));
		    tmp4.add(Box.createHorizontalGlue());
		    reconnectButton=new JButton(carrows);
		    reconnectButton.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
		    reconnectButton.addActionListener(this);
		    reconnectButton.setToolTipText("Drop current connection and try again.");
		    tmp4.add(reconnectButton);
		    tmp3.add(tmp4);
		}
		tmp3.add(Box.createVerticalStrut(strutSize));
		tmp2.add(tmp3);
	    }
	    tmp2.add(Box.createHorizontalStrut(strutSize));
	    getContentPane().add(tmp2,BorderLayout.SOUTH);
	}
		
	String warningMsg = "Servo";
	boolean comma = false;
		
	for (int counter = 0; counter < NOJ; counter++) {
	    if (armStartValue[counter] > 1) {
		if (comma) warningMsg = warningMsg + ",";
		warningMsg += " " + counter;
		comma = true;
	    }
	    else if (armStartValue[counter] < -1) {
		if (comma) warningMsg = warningMsg + ",";
		warningMsg += " " + counter;
		comma = true;
		armStartValue[counter] = Math.max(-1, Math.min(1, armStartValue[counter]));
	    }
	}
		
	if (warningMsg.length() > 5) {
	    warningMsg += " is out of servo limits.";
	    JOptionPane.showMessageDialog(null, warningMsg, "Servo Out of Range", JOptionPane.WARNING_MESSAGE);
	}
		
	if (hasGripper)
	    determineGripperPointPicker();
		
	addWindowListener(new CloseArmAdapter(this));
	comm.ArmStatusUpdate();
	pack();
		
	int offx = getGraphicsConfiguration().getBounds().x;
	int offy = getGraphicsConfiguration().getBounds().y;
	setLocation(prefs.getInt("ArmGUI.location.x",200),prefs.getInt("ArmGUI.location.y",200));
	setVisible(true);
    }
}
