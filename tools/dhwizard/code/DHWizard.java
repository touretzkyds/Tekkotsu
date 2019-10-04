import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSpinner;
import javax.swing.JSplitPane;
import javax.swing.UIManager;
import javax.swing.event.*;
import javax.swing.*;

import javax.swing.JTree;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.TreeSelectionModel;
import javax.swing.tree.*;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;

import java.io.IOException;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.*;

import java.io.*;
import java.net.*;

import java.util.HashMap;
import java.util.Map;


public class DHWizard extends JPanel implements TreeSelectionListener, ChangeListener, ActionListener {
	private JPanel infoPanel;
	//	private JTextField nameField;
	private JSpinner dSpinner, thetaSpinner, rSpinner,alphaSpinner , qSpinner, massSpinner, minSpinner, maxSpinner;
	private JSpinner xScaleSpinner, yScaleSpinner, zScaleSpinner;
	private JSlider qSlider;
	private JComboBox typeBox;
	private JButton saveButton, addButton, removeButton, clearButton, renameButton;
	private JTextField meshText;
	private JTree tree;
	private static boolean DEBUG = false;
	private static String kinFileName;
	private static String serverName;
	private static boolean connectToServer;
	protected DefaultMutableTreeNode rootNode;
	protected DefaultTreeModel treeModel;
	protected MirageWriter mirageOut;
	private static int robotID = 1;
	private static boolean sendUpdates = true;
	private static DHWizard wizard;

	//Optionally play with line styles.  Possible values are
	//"Angled" (the default), "Horizontal", and "None".
    private static boolean playWithLineStyle = false;
    private static String lineStyle = "Horizontal";
   
    private static Map<String,Integer> qValues = new HashMap<String,Integer>();

    //Optionally set the look and feel.
    private static boolean useSystemLookAndFeel = false;

    public DHWizard(JointNode node) {
        super(new GridLayout(1,0));

        //Create the nodes.
        rootNode = createTree(node);
	treeModel = new DefaultTreeModel(rootNode);
        //Create a tree that allows one selection at a time.
        tree = new JTree(treeModel);
        tree.getSelectionModel().setSelectionMode
        (TreeSelectionModel.SINGLE_TREE_SELECTION);

        //Listen for when the selection changes.
        tree.addTreeSelectionListener(this);

        if (playWithLineStyle) {
            System.out.println("line style = " + lineStyle);
            tree.putClientProperty("JTree.lineStyle", lineStyle);
        }

        //Create the scroll pane and add the tree to it. 
        JScrollPane treeView = new JScrollPane(tree);


        //Create the HTML viewing pane.
        infoPanel = new JPanel(new GridLayout(0,2));

	dSpinner = new JSpinner(new SpinnerNumberModel(
							0.0, null, null, 0.2));
	dSpinner.addChangeListener(this);
	JLabel dLabel = new JLabel("d");
	infoPanel.add(dLabel);
	infoPanel.add(dSpinner);

	thetaSpinner = new JSpinner(new SpinnerNumberModel(
							0, -3.15, 3.15, 0.2));
	thetaSpinner.addChangeListener(this);
	JLabel thetaLabel = new JLabel("theta");
	infoPanel.add(thetaLabel);
	infoPanel.add(thetaSpinner);

	rSpinner = new JSpinner(new SpinnerNumberModel(
							0.0, null, null, 0.2));
	rSpinner.addChangeListener(this);
	JLabel rLabel = new JLabel("r");
	infoPanel.add(rLabel);
	infoPanel.add(rSpinner);

	alphaSpinner = new JSpinner(new SpinnerNumberModel(
							0, -3.15, 3.15, 0.2));
	alphaSpinner.addChangeListener(this);
	JLabel alphaLabel = new JLabel("alpha");
	infoPanel.add(alphaLabel);
	infoPanel.add(alphaSpinner);

	qSpinner = new JSpinner(new SpinnerNumberModel(0.0,null,null,.2));

	qSpinner.addChangeListener(this);
	JLabel qLabel = new JLabel("qOffset");
	infoPanel.add(qLabel);
	infoPanel.add(qSpinner);
		
	minSpinner = new JSpinner(new SpinnerNumberModel(0.0,null,null,.2));
	minSpinner.addChangeListener(this);
	JLabel minLabel = new JLabel("min");
	infoPanel.add(minLabel);
	infoPanel.add(minSpinner);
	
	maxSpinner = new JSpinner(new SpinnerNumberModel(0.0,null,null,.2));
	maxSpinner.addChangeListener(this);
	JLabel maxLabel = new JLabel("max");
	infoPanel.add(maxLabel);
	infoPanel.add(maxSpinner);
		
	massSpinner = new JSpinner(new SpinnerNumberModel(0.0,0.0,null,.1));
	massSpinner.addChangeListener(this);
	JLabel massLabel = new JLabel("mass");
	infoPanel.add(massLabel);
	infoPanel.add(massSpinner);

    qSlider = new JSlider(-180,180,0);
	JLabel qSlideLabel = new JLabel("q (angle)");
	qSlider.addChangeListener(this);
	infoPanel.add(qSlideLabel);
	infoPanel.add(qSlider);

	String [] types = {"revolute", "prismatic"};
	typeBox = new JComboBox (types);
	typeBox.addActionListener(this);
	JLabel typeLabel = new JLabel("type");
	infoPanel.add(typeLabel);
	infoPanel.add(typeBox);
	
	meshText = new JTextField(40);
	JLabel meshLabel = new JLabel("mesh");
	infoPanel.add(meshLabel);
	infoPanel.add(meshText);

	xScaleSpinner = new JSpinner(new SpinnerNumberModel(1.0,0.0,null,.1));
	xScaleSpinner.addChangeListener(this);
	JLabel xScaleLabel = new JLabel("model scale X");
	infoPanel.add(xScaleLabel);
	infoPanel.add(xScaleSpinner);
	
	yScaleSpinner = new JSpinner(new SpinnerNumberModel(1.0,0.0,null,.1));
	yScaleSpinner.addChangeListener(this);
	JLabel yScaleLabel = new JLabel("model scale Y");
	infoPanel.add(yScaleLabel);
	infoPanel.add(yScaleSpinner);

	zScaleSpinner = new JSpinner(new SpinnerNumberModel(1.0,0.0,null,.1));
	zScaleSpinner.addChangeListener(this);
	JLabel zScaleLabel = new JLabel("model scale Z");
	infoPanel.add(zScaleLabel);
	infoPanel.add(zScaleSpinner);

	infoPanel.add(new JLabel(" "));
	infoPanel.add(new JLabel(" "));
		
	addButton = new JButton("add");
	addButton.setActionCommand("add");
	addButton.addActionListener(this);
	infoPanel.add(addButton);

	renameButton = new JButton("rename");
	renameButton.setActionCommand("rename");
	renameButton.addActionListener(this);
	infoPanel.add(renameButton);
        
	removeButton = new JButton("remove");
	removeButton.setActionCommand("remove");
	removeButton.addActionListener(this);
	infoPanel.add(removeButton);
	
	clearButton = new JButton("clear");
	clearButton.setActionCommand("clear");
	clearButton.addActionListener(this);
	infoPanel.add(clearButton);
	
	//Add the scroll panes to a split pane.
        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
        splitPane.setLeftComponent(treeView);
        //splitPane.setRightComponent(infoView);
        splitPane.setRightComponent(infoPanel);

        Dimension minimumSize = new Dimension(100, 50);
        //infoView.setMinimumSize(minimumSize);
        infoPanel.setMinimumSize(minimumSize);
        treeView.setMinimumSize(minimumSize);
        splitPane.setDividerLocation(300); 
        splitPane.setPreferredSize(new Dimension(500, 370));


        //Add the split pane to this panel.
        add(splitPane);
	
	//Try to establish connection with Mirage if it is running
	try{
		mirageOut = new MirageWriter("DHwizard", serverName);
		connectToServer = true;
	} catch (UnknownHostException e) {
		System.out.println("Could not connect to mirage");
		connectToServer = false;
	} catch (IOException e) {
		System.out.println("Could not connect to mirage");
		connectToServer = false;
	}
	//Transmit the initial robot
	if (connectToServer) {
		mirageOut.sendOpeningMessage((JointNode)(rootNode.getUserObject()));
	}
    }

    /** Required by TreeSelectionListener interface.
  	Triggers when a node is selected in the tree
     */
    public void valueChanged(TreeSelectionEvent e) {
	TreePath prevPath = e.getOldLeadSelectionPath();
	if (prevPath != null){
		DefaultMutableTreeNode prevNode = (DefaultMutableTreeNode)prevPath.getLastPathComponent();
		JointNode prevJoint = (JointNode)prevNode.getUserObject();
		prevJoint.model = meshText.getText();
	}

        DefaultMutableTreeNode node = (DefaultMutableTreeNode)
                           tree.getLastSelectedPathComponent();

        if (node == null) return;
	
	resetRobot();

        JointNode joint = (JointNode)node.getUserObject();
	if (DEBUG) {
		System.out.println(joint.name + " selected.");
	}
	displayJointInfo(joint);
    }

    /** Triggers when a joint node is modified*/
    public void stateChanged(ChangeEvent e) {
	DefaultMutableTreeNode node = (DefaultMutableTreeNode)
                           tree.getLastSelectedPathComponent();

        if (node == null) return;

        JointNode joint = (JointNode)node.getUserObject();

		       if (e.getSource() == dSpinner) {
			joint.d = ((Double)dSpinner.getValue()).doubleValue();
		} else if (e.getSource() == thetaSpinner) {
			joint.theta = ((Double)thetaSpinner.getValue()).doubleValue();
		} else if (e.getSource() == rSpinner) {
			joint.r = ((Double)rSpinner.getValue()).doubleValue();
		} else if (e.getSource() == alphaSpinner) {
			joint.alpha = ((Double)alphaSpinner.getValue()).doubleValue();
		} else if (e.getSource() == qSpinner) {
			joint.qOffset = ((Double)qSpinner.getValue()).doubleValue();
		} else if (e.getSource() == massSpinner){
			joint.mass = ((Double)massSpinner.getValue()).doubleValue();
		} else if (e.getSource() == minSpinner) {
			joint.min = ((Double)minSpinner.getValue()).doubleValue();
		} else if (e.getSource() == maxSpinner) {
			joint.max = ((Double)maxSpinner.getValue()).doubleValue();
		} else if (e.getSource() == qSlider) {
            qValues.put(joint.name,(Integer)qSlider.getValue());
		} else if (e.getSource() == xScaleSpinner){
			joint.scale[0] = ((Double)xScaleSpinner.getValue()).doubleValue();
		} else if (e.getSource() == yScaleSpinner){
			joint.scale[1] = ((Double)yScaleSpinner.getValue()).doubleValue();
		} else if (e.getSource() == zScaleSpinner){
			joint.scale[2] = ((Double)zScaleSpinner.getValue()).doubleValue();
		} else {
			//System.out.println("Source: " + e.getSource());
		}
		
		//Transmit updated joint to Mirage
		if (connectToServer){
			if (e.getSource() == qSlider)
				mirageOut.sendUpdateMessage(joint.name, qSlider.getValue());
			else
				resetRobot();
		}
        else
            resetRobot();
    }

	/** Resend the robot to Mirage when its joint structure is modified */
	private void resetRobot() {
        if (!sendUpdates) {
		if (DEBUG)
		    System.out.println("Reset disabled");
            return;
        } else if (connectToServer){
		if (DEBUG)
			System.out.println("Robot Reset");
		//we've edited the structure of the chain
		mirageOut.sendCloseMessage(); // This kills the Socket
		//try {
		//	Thread.sleep(100);
		//}catch (InterruptedException ex) {
			
		//}
		try {
			mirageOut = new MirageWriter("DHwizard"+(robotID++)%30, serverName);
			mirageOut.sendOpeningMessage((JointNode)(rootNode.getUserObject()));
			try {
				Thread.sleep(100);
			}catch (InterruptedException ex) {}
			mirageOut.sendAllAngles(qValues);
		} catch (UnknownHostException e){
			System.out.println("Connection lost");
			System.exit(1);
		} catch (IOException e) {
			System.out.println("Couldn't get I/O for :19785");
			System.exit(1);
		}
	}
	}

	/** Triggered when a button or menu item is selected*/
	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("save")) {
			//Save with the original filename
			saveFile(kinFileName);
		} else if (e.getActionCommand().equals("comboBoxChanged")) {
        		DefaultMutableTreeNode node = (DefaultMutableTreeNode)
			tree.getLastSelectedPathComponent();

        		if (node == null) return;

        		JointNode joint = (JointNode)node.getUserObject();
			joint.type = (String)typeBox.getSelectedItem();
		}
		else if (e.getActionCommand().equals("add")) {			
        		DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();
        		//Use root node as parent if no node selected
			if (node == null) node = rootNode;
			JointNode joint = (JointNode)node.getUserObject();
			String name = JOptionPane.showInputDialog("Enter new joint name:");
			//Create a standard revolute joint
			if (name != null){
				JointNode newJoint = new JointNode(name);
				newJoint.type = "revolute";
				newJoint.model = "HandEye/Link1";
				newJoint.min = -3.15;
				newJoint.max = 3.15;
				//Add new node to joint layout
				joint.addChild(newJoint);
				DefaultMutableTreeNode nodeToAdd = new DefaultMutableTreeNode(newJoint);

				//Add new node to tree GUI display
				treeModel.insertNodeInto(nodeToAdd, node, node.getChildCount());
				tree.scrollPathToVisible(new TreePath(nodeToAdd.getPath()));
			resetRobot();
			}
		} else if (e.getActionCommand().equals("remove")){
        		DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();
			if (node == null) return;
			JointNode joint = (JointNode)node.getUserObject();
			DefaultMutableTreeNode parent = (DefaultMutableTreeNode)(node.getParent());
			if (parent != null){
				JointNode parentJoint = (JointNode)parent.getUserObject();	
				treeModel.removeNodeFromParent(node);
				parentJoint.removeChild(joint);
			}
			resetRobot();
		} else if (e.getActionCommand().equals("clear")){
			JointNode newRoot = new JointNode("BaseFrame");
			newRoot.type = "revolute";
			newRoot.model = "ReferenceFrame";
        		rootNode = createTree(newRoot);
			treeModel = new DefaultTreeModel(rootNode);
			tree.setModel(treeModel);
		} else if (e.getActionCommand().equals("rename")){
        		DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();
			if (node == null) node = rootNode;
			
			String newName = JOptionPane.showInputDialog("Enter new joint name: ");
			JointNode joint = (JointNode)node.getUserObject();
			if (newName != null){
				joint.name = newName;
				((DefaultTreeModel)tree.getModel()).reload();
			}
			resetRobot();
		} else if (e.getActionCommand().equals("save as")){
            		//Pass the current directory to the constructor so it starts there
            		JFileChooser c = new JFileChooser(new File("."));
            		int status = c.showSaveDialog(DHWizard.this);
            		
			if (status == JFileChooser.APPROVE_OPTION) {
                		String filename = c.getCurrentDirectory().toString()+"/";
                		filename += c.getSelectedFile().getName();
			    	saveFile(filename);
            		}
            		else if (status == JFileChooser.CANCEL_OPTION){
                		System.out.println("Save as canceled.");
            		}	
		} else if (e.getActionCommand().equals("quit")){
            		System.exit(0);
		} else if (e.getActionCommand().equals("open")){
            		JFileChooser c = new JFileChooser(new File("."));
            		int status = c.showOpenDialog(DHWizard.this);
            		
			if (status == JFileChooser.APPROVE_OPTION) {
                		String filename = c.getCurrentDirectory().toString()+"/";
                		filename += c.getSelectedFile().getName();
			    	kinFileName = filename;
                		createAndShowGUI();
            		}
            		else if (status == JFileChooser.CANCEL_OPTION){
                		System.out.println("Open canceled.");
            		}	
		} else {
			System.err.println("I can't handle the action: "+e.getActionCommand());
		}
	}

    //Saves the current joint configuration in a file with name filename
    private void saveFile(String filename){
        System.out.println("Saving\n");
	try {
		FileWriter f = new FileWriter(filename);
		BufferedWriter out = new BufferedWriter(f);
		//Be sure to write XML doctype to enable future parsing
		out.write("<?xml version=\"1.0\"?>\n" +
			"<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n" +
			"<plist version=\"1.0\"><array>\n");
		out.write(((JointNode)rootNode.getUserObject()).toXMLString());
		out.write("</array>\n</plist>\n");
		out.close();
	} catch (Exception x) {
	}
    }

    //Set all input elements to display the current current values for the selected joint
    private void displayJointInfo(JointNode joint) {
	    sendUpdates = false;
	    dSpinner.setValue(new Double(joint.d));
	    thetaSpinner.setValue(new Double(joint.theta));
	    rSpinner.setValue(new Double(joint.r));
	    alphaSpinner.setValue(new Double(joint.alpha));
	    qSpinner.setValue(new Double(joint.qOffset));
	    massSpinner.setValue(new Double(joint.mass));
	    minSpinner.setValue(new Double(joint.min));
	    maxSpinner.setValue(new Double(joint.max));
	    if (qValues.containsKey(joint.name)) {
		    qSlider.setValue(qValues.get(joint.name));
	    } else
		    qSlider.setValue(0);
	    qSlider.setMinimum((int)(joint.min * 180 / Math.PI));
	    qSlider.setMaximum((int)(joint.max * 180 / Math.PI));
	    typeBox.setSelectedItem(new String(joint.type));

	    meshText.setText(joint.model);
	    xScaleSpinner.setValue(new Double(joint.scale[0]));
	    yScaleSpinner.setValue(new Double(joint.scale[1]));
	    zScaleSpinner.setValue(new Double(joint.scale[2]));

	    sendUpdates = true;
    }

    //Builds the tree structure in the GUI from the JointNode tree object
    private DefaultMutableTreeNode createTree(JointNode node) {
	    DefaultMutableTreeNode result;

	    result = new DefaultMutableTreeNode(node);
	    int size = node.children.size();
	    for (int i = 0; i < size; i++) {
		    result.add(createTree((JointNode)node.children.get(i)));
	    }

	    return result;
    }
        
    /**
     * Create the GUI and show it.  For thread safety,
     * this method should be invoked from the
     * event dispatch thread.
     */
    private static void createAndShowGUI() {
        if (useSystemLookAndFeel) {
            try {
                UIManager.setLookAndFeel(
                    UIManager.getSystemLookAndFeelClassName());
            } catch (Exception e) {
                System.err.println("Couldn't use system look and feel.");
            }
        }
	JointNode base;
        System.out.println("showing");
	if (kinFileName != null){
		KinParser k = new KinParser();
		base = k.parse(kinFileName);
	} else {
		kinFileName = "DHout.kin";
		base = new JointNode("BaseFrame");
		base.type = "revolute";
		base.model = "ReferenceFrame";
	}

        //Create and set up the window.
        JFrame frame = new JFrame("DHWizard");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        wizard = new DHWizard(base);
        //Add content to the window.
        frame.add(wizard);

        JMenuBar menuBar;
        JMenu menu;
        JMenuItem menuItem;
        menuBar = new JMenuBar();
        menu = new JMenu("File");
        //Open
        menuItem = new JMenuItem("Open");
        menuItem.setActionCommand("open");
        menuItem.addActionListener(wizard);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, java.awt.Toolkit.getDefaultToolkit().getMenuShortcutKeyMask()));
        menu.add(menuItem);
        //Save
        menuItem = new JMenuItem("Save");
		menuItem.setActionCommand("save");
		menuItem.addActionListener(wizard);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, java.awt.Toolkit.getDefaultToolkit().getMenuShortcutKeyMask()));
        menu.add(menuItem);
        //Save As
        menuItem = new JMenuItem("Save As");
        menuItem.setActionCommand("save as");
        menuItem.addActionListener(wizard);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, java.awt.Event.SHIFT_MASK | java.awt.Toolkit.getDefaultToolkit().getMenuShortcutKeyMask()));
        menu.add(menuItem);

		// Mac application menu already has a "Quit", others, add it explicitly
	    if (System.getProperty("os.name").toLowerCase().indexOf("mac") == -1) {
	        //Quit
	        menuItem = new JMenuItem("Quit");
	        menuItem.setActionCommand("quit");
	        menuItem.addActionListener(wizard);
			menuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Q, java.awt.Toolkit.getDefaultToolkit().getMenuShortcutKeyMask()));
	        menu.add(menuItem);
		}
		
        menuBar.add(menu);
        frame.setJMenuBar(menuBar);

        //Display the window.
        frame.pack();
        frame.setVisible(true);
    }

    public static void main(String[] args) {
        //Schedule a job for the event dispatch thread:
        //creating and showing this application's GUI.
		if (args.length == 0)
			kinFileName = null;
		else
			kinFileName = args[0];

		if (args.length > 1) {
			serverName = args[1];
		} else {
			serverName = "localhost";
		}

        javax.swing.SwingUtilities.invokeLater(new GUIRunner());
    }

    public static class GUIRunner implements Runnable {
            public void run() {
                createAndShowGUI();
            }
    }
}
