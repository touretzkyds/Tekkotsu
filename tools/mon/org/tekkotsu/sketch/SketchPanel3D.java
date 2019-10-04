package org.tekkotsu.sketch;

//import org.tekkotsu.mon.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.tree.*;
import java.awt.event.*;
import javax.media.j3d.*;
import javax.vecmath.*;
import java.util.*;
import java.lang.*;
import com.sun.j3d.utils.universe.*;
import com.sun.j3d.utils.behaviors.vp.*;
import com.sun.j3d.utils.behaviors.keyboard.KeyNavigatorBehavior;
import com.sun.j3d.utils.geometry.Box;
import com.sun.j3d.utils.geometry.Cylinder;
import com.sun.j3d.utils.geometry.Cone;

public class SketchPanel3D extends JPanel
{
    JFrame frame;
    private static final int BOUNDSIZE = 500; // world bound size
    private static final float scaling = 1000.0f; // world scaling in millimiters
    Canvas3D canvas3d;
    SimpleUniverse universe;
    BranchGroup rootGroup;
    BranchGroup mainGroup;
    BranchGroup vinfoGroup;
    BranchGroup floorGroup;
    Appearance fap;
    Material fmat;
    float fcx;
    float fcy;

    TransformGroup viewTG = new TransformGroup();
    Transform3D topViewT3d = new Transform3D();
    Transform3D defaultViewT3d = new Transform3D();
    Boolean viewIsSet = false;

    BoundingSphere worldBounds;
    TransformGroup rootTrans;
    protected TreePath[] paths;
    float margin = 0.8f;
    float leftBound = 0, rightBound = 0, topBound = 0, bottomBound = 0;
    float prevleftBound = 0, prevrightBound = 0, prevtopBound = 0, prevbottomBound = 0;

    public SketchPanel3D() {
        frame = new JFrame("3D SketchPanel");
        frame.setBackground(Color.black);
        frame.getContentPane().setLayout(new BorderLayout());
        frame.setSize(600, 600);
        frame.setLocation(50, 50);
        setup3DCanvas();
        frame.add(canvas3d, BorderLayout.CENTER);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.setVisible(true);
    }

    public void setup3DCanvas() {
        // Create a Canvas3D with prefered config
        canvas3d = new Canvas3D(SimpleUniverse.getPreferredConfiguration());
        add(canvas3d,BorderLayout.CENTER);
        // Initiate the universe
        universe = new SimpleUniverse(canvas3d);
        // Initiate the scene. This function initiate and fill the rootGroup data structure
        //with objects of all kinds.
        createSceneGraph();
        // Set the viewers point of view to the center of the universe
        universe.getViewingPlatform().setNominalViewingTransform();
        viewTG = universe.getViewingPlatform().getViewPlatformTransform();
        universe.addBranchGraph(rootGroup);
    }

    public void createSceneGraph() {
        // Initiate and set the correct capabilities of the root level group and contained groups.
        rootGroup = new BranchGroup();
        rootGroup.setCapability(BranchGroup.ALLOW_CHILDREN_EXTEND);
        rootGroup.setCapability(BranchGroup.ALLOW_CHILDREN_WRITE);

        mainGroup = new BranchGroup();
        mainGroup.setCapability(BranchGroup.ALLOW_CHILDREN_EXTEND);
        mainGroup.setCapability(BranchGroup.ALLOW_CHILDREN_WRITE);

        floorGroup = new BranchGroup();
        floorGroup.setCapability(BranchGroup.ALLOW_CHILDREN_EXTEND);
        floorGroup.setCapability(BranchGroup.ALLOW_CHILDREN_WRITE);

        vinfoGroup = new BranchGroup();
        vinfoGroup.setCapability(BranchGroup.ALLOW_CHILDREN_EXTEND);
        vinfoGroup.setCapability(BranchGroup.ALLOW_CHILDREN_WRITE);

        // Initiate the world bounds. Used by the abient light and the orbit controls
        worldBounds = new BoundingSphere(new Point3d(0,0,0), BOUNDSIZE);
        // Initiate tier 0 transform group. This transform group holds every object including
        //lights and shapes of all types and is used to create translations and rotations of
        //the whole system.
        rootTrans = new TransformGroup();
        rootTrans.setCapability(TransformGroup.ALLOW_CHILDREN_EXTEND);
        rootTrans.setCapability(TransformGroup.ALLOW_CHILDREN_WRITE);

        createLight();
	createBackground();
        // Mouse and keyboard controlls;
        orbitControls();

        // Initiate floor Appearance and rotate the world PI/2
        fap = new Appearance();
        fmat = new Material();
        fmat.setAmbientColor(new Color3f(0.7f, 0.7f, 0.7f));
        fmat.setDiffuseColor(new Color3f(0.75f, 0.75f, 0.75f));
        fmat.setSpecularColor(new Color3f(0.80f, 0.80f, 0.80f));
        fap.setMaterial(fmat);

        Transform3D rot = new Transform3D();
        rot.rotZ((90 * Math.PI)/180);
        rootTrans.setTransform(rot);

        // Construct the tree
        mainGroup.addChild(vinfoGroup);
        mainGroup.addChild(floorGroup);
        rootTrans.addChild(mainGroup);
        rootGroup.addChild(rootTrans);
        rootGroup.compile();
    }

    public void createBackground() {
        Background back = new Background();
        back.setApplicationBounds(worldBounds);
        back.setColor(0.17f, 0.65f, 0.92f);
        rootGroup.addChild(back);
    }

    public void rescale() {
        // creates a rescaled floor
        if (floorGroup.numChildren() != 0)
            floorGroup.removeAllChildren();
        BranchGroup floorBranch = new BranchGroup();
        floorBranch.setCapability(BranchGroup.ALLOW_DETACH);
        TransformGroup floorTrans = new TransformGroup();
        Transform3D f3d = new Transform3D();
        fcx = (leftBound + (rightBound - leftBound) / 2) / scaling;
        fcy = (bottomBound + (topBound - bottomBound) / 2) / scaling;
        f3d.setTranslation(new Vector3f(fcx, fcy, -0.05f));
        float width = -((leftBound - rightBound) / scaling);
        float length = -((topBound - bottomBound) / scaling);
        Box floor = new Box(width, length, 0.05f, fap);
        floorTrans.setTransform(f3d);
        floorTrans.addChild(floor);
        floorBranch.addChild(floorTrans);
        floorGroup.addChild(floorBranch);

        // check if view was already calculated
        if (!viewIsSet)
            calculateViews(fcx, fcy);

        leftBound = topBound = rightBound = bottomBound = 0;
    }

    public void calculateViews(float x, float y) {
        // calculate views
        topViewT3d = new Transform3D();
        defaultViewT3d = new Transform3D();
        Vector3f viewDistance = new Vector3f();
        float viewScale = 2.5f;
        Boolean isWider = false;
        //check which dimension is larger and adjust our viewer's hight accordingly.
        //Since our world is rotated by 90 degrees use the secondary dimensions.
        if ((prevleftBound - prevrightBound) > (prevtopBound - prevbottomBound)) {
            viewDistance.setZ(-viewScale * (prevtopBound - prevbottomBound) / scaling); // top - bottom
            isWider = false;
        }
        else {
            viewDistance.setZ(-viewScale * (prevleftBound - prevrightBound) / scaling); //left - rig
            isWider = true;
        }
        viewDistance.setY(x);
        viewDistance.setX(-y);
        topViewT3d.setTranslation(viewDistance);

        // For the default view we need to give some perspective. Move the viewer downwards
        //in viewer's coordinate (-y) by an entire floor size and then rotate upward so that
        //we can see the whole floor in perspective.
        Vector3f viewDistance2 = new Vector3f(viewDistance);
        if (isWider) {
            viewDistance2.setY((prevleftBound - prevrightBound) / scaling);
        }
        else {
            viewDistance2.setY((prevtopBound - prevbottomBound) / scaling);
        }
        viewDistance2.setZ(viewDistance2.getZ() * 0.8f);
        float rot = (float)Math.atan2(viewDistance2.getY(), viewDistance2.getZ());
        defaultViewT3d.rotX(-rot);
        defaultViewT3d.setTranslation(viewDistance2);
        viewIsSet = true;
        setView(defaultViewT3d);
    }

    public void setView(Transform3D view) {
     // Set the view so that we can see everything with some perspective
        viewTG.setTransform(view);
    }

    public void scaleToSketchOrShape(SketchOrShapeInfo oinfo) {
        prevleftBound = leftBound = Math.min(leftBound, margin*oinfo.getLeft());
        prevrightBound = rightBound = Math.max(rightBound, margin*oinfo.getRight());
        prevtopBound = topBound = Math.min(topBound, margin*oinfo.getTop());
        prevbottomBound = bottomBound = Math.max(bottomBound, margin*oinfo.getBottom());
    }

    public void imageUpdated(TreePath[] newPaths) {
        paths = newPaths;
        render();
    }

    public void render() {
        vinfoGroup.removeAllChildren();
	if (paths==null) return;
        Color color = new Color(0, 0, 0); // initialize the color to black
        Color3f ambientColor = new Color3f(Color.GRAY);
        Color3f emissiveColor = new Color3f(Color.BLACK);
        Appearance ap = new Appearance();
        ap.setMaterial(new Material(ambientColor, emissiveColor, new Color3f(color), new Color3f(color.brighter()), 5f));
        for (int i = 0; i < paths.length; i++) {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode)(paths[i].getLastPathComponent());
            if (node == null) return;
            if((node.getUserObject() instanceof SketchOrShapeInfo)) {
                SketchOrShapeInfo vinfo = (SketchOrShapeInfo)(node.getUserObject());
                if (vinfo instanceof ShapeInfo) {
                    if ( ! vinfo.getColor().equals(color) ) {
                        ap = new Appearance();
                        color = vinfo.getColor();
                        ap.setMaterial(new Material(ambientColor, emissiveColor, new Color3f(color), new Color3f(color.brighter()) , 5f));
                    }
		    vinfo.renderTo3D(vinfoGroup, ap, scaling);
                }
            }
        }
    }

    public void createLight() {
        Color3f white = new Color3f(0.9f, 0.9f, 0.9f);
        AmbientLight ambientLightNode = new AmbientLight(new Color3f(0.5f, 0.5f, 0.5f));
        ambientLightNode.setInfluencingBounds(worldBounds);
        rootTrans.addChild(ambientLightNode);
        BoundingSphere lightBound = new BoundingSphere(new Point3d(2.0, 2.0, 3.0), 20);
        Vector3f lightDirection = new Vector3f(1.0f, -1.0f, -2.0f);
        DirectionalLight light = new DirectionalLight(white, lightDirection);
        light.setInfluencingBounds(lightBound);
        mainGroup.addChild(light);
    }

    public void orbitControls() {
        OrbitBehavior orbit = new OrbitBehavior(canvas3d);
        orbit.setZoomFactor(0.09);
        orbit.setSchedulingBounds(worldBounds);
        ViewingPlatform vp = universe.getViewingPlatform();
        vp.setViewPlatformBehavior(orbit);


        TransformGroup viewTG = universe.getViewingPlatform().getViewPlatformTransform();
        KeyBoardNavigatorBehavior keyboardBh = new KeyBoardNavigatorBehavior(this);
        keyboardBh.setSchedulingBounds(worldBounds);
        rootGroup.addChild(keyboardBh);
    }

    public void close() {
        frame.dispose();
    }

    // This class implements WASD controls similar to Mirage
    public class KeyBoardNavigatorBehavior extends Behavior {
        SketchPanel3D sketch3d;
        private static final float rotRate = (float) Math.PI / 150;
        private static final float translationRate = 0.05f;
        Boolean isAlt = false;
        WakeupOnAWTEvent keyWake = new WakeupOnAWTEvent(KeyEvent.KEY_PRESSED);

        Transform3D viewT3d = new Transform3D();
        Transform3D rotT3d = new Transform3D();
        Matrix3f rotMatrix = new Matrix3f();
        Vector3f transVector = new Vector3f();
        Vector3f transTemp = new Vector3f();

        KeyBoardNavigatorBehavior(SketchPanel3D _sketch3d) {
            sketch3d = _sketch3d;
        }

        public void initialize() {
            wakeupOn(keyWake);
        }

        public void processStimulus(Enumeration criteria) {
            WakeupCriterion wakeUpEvent;
            AWTEvent[] events;

            while(criteria.hasMoreElements()) {
                wakeUpEvent = (WakeupCriterion)criteria.nextElement();

                if (wakeUpEvent instanceof WakeupOnAWTEvent) {
                    events = ((WakeupOnAWTEvent)wakeUpEvent).getAWTEvent();
                    for (AWTEvent event : events) {
                        if (event instanceof KeyEvent) {
                            KeyEvent keyEvent = (KeyEvent)event;
                            if (keyEvent.isAltDown()) {
                                isAlt = true;
                            }
                            else {
                                isAlt = false;
                            }
                            sketch3d.viewTG.getTransform(viewT3d);
                            int keyCode = keyEvent.getKeyCode();
                            int keyId = keyEvent.getID();
                            switch (keyCode) {
                            case KeyEvent.VK_R: // reset to default view
                                sketch3d.viewIsSet = false;
                                sketch3d.calculateViews(sketch3d.fcx, fcy);
                                sketch3d.setView(sketch3d.defaultViewT3d);
                                break;
                            case KeyEvent.VK_T: // reset to topdown view
                                sketch3d.viewIsSet = false;
                                sketch3d.calculateViews(sketch3d.fcx, fcy);
                                sketch3d.setView(sketch3d.topViewT3d);
                                break;
                            case KeyEvent.VK_D: // strafe right
                                viewT3d.get(rotMatrix, transVector);

                                rotMatrix.transform(new Vector3f(translationRate, 0.0f, 0.0f), transTemp);
                                transVector.add(transTemp);
                                viewT3d.setTranslation(transVector);
                                break;
                            case KeyEvent.VK_A: // strafe left
                                viewT3d.get(rotMatrix, transVector);

                                rotMatrix.transform(new Vector3f(-translationRate, 0.0f, 0.0f), transTemp);
                                transVector.add(transTemp);
                                viewT3d.setTranslation(transVector);
                                break;
                            case KeyEvent.VK_W: // go forward
                                viewT3d.get(rotMatrix, transVector);

                                rotMatrix.transform(new Vector3f(0.0f, 0.0f, -translationRate), transTemp);
                                transVector.add(transTemp);
                                viewT3d.setTranslation(transVector);
                                break;
                            case KeyEvent.VK_S: // go backward
                                viewT3d.get(rotMatrix, transVector);

                                rotMatrix.transform(new Vector3f(0.0f, 0.0f, translationRate), transTemp);
                                transVector.add(transTemp);
                                viewT3d.setTranslation(transVector);
                                break;
                            case KeyEvent.VK_RIGHT: // rotate in z or y
                                if (isAlt) {
                                    rotT3d.rotZ(-rotRate);
                                }
                                else {
                                    rotT3d.rotY(-rotRate);
                                }
                                viewT3d.mul(rotT3d);
                                break;
                            case KeyEvent.VK_LEFT: // rotate in z or y
                                if (isAlt) {
                                    rotT3d.rotZ(rotRate);
                                }
                                else {
                                    rotT3d.rotY(rotRate);
                                }
                                viewT3d.mul(rotT3d);
                                break;
                            case KeyEvent.VK_UP: // rotate in x
                                rotT3d.rotX(rotRate);
                                viewT3d.mul(rotT3d);
                                break;
                            case KeyEvent.VK_DOWN: // rotate in x
                                rotT3d.rotX(-rotRate);
                                viewT3d.mul(rotT3d);
                                break;
                            default:
                            }
                            if (keyCode != KeyEvent.VK_R && keyCode != KeyEvent.VK_T)
                                sketch3d.viewTG.setTransform(viewT3d);
                        }
                    }
                }
            }
            wakeupOn(keyWake);
        }
    }
}