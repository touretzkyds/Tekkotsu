package org.tekkotsu.aibo3d;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import javax.media.j3d.*;
import javax.vecmath.*;
import javax.media.j3d.*;
import com.sun.j3d.utils.picking.*;
import com.sun.j3d.loaders.Scene;

public class PickMoveBehavior extends Behavior {
  Canvas3D canvas3d;
  BranchGroup branchGroup;
  Scene scene;
  AiboPicker picker;
  PickCanvas pickCanvas;
  WakeupCriterion[] mouseEvents;
  WakeupOr mouseCriterion;
  String currentLink;
  int currentDragButton;
  Vector3d currentPos;
  Aibo3DForward forward;

  TransformGroup viewTransformGroup;
  Transform3D viewTransformX;
  Transform3D viewTransformY;
  Transform3D currTransform;
  Vector3d viewTranslation;

  int x, y, dx, dy, x_last, y_last;
  
  public PickMoveBehavior (Canvas3D canvas3d, BranchGroup branchGroup,
      Scene scene, Aibo3DForward forward, TransformGroup viewTransformGroup) {
    this.canvas3d=canvas3d;
    this.branchGroup=branchGroup;
    this.scene=scene;
    this.forward=forward;
    this.viewTransformGroup=viewTransformGroup;
    viewTransformX=new Transform3D();
    viewTransformY=new Transform3D();
    currTransform=new Transform3D();
    viewTranslation=new Vector3d();
  }

  public void initialize() {
    pickCanvas = new PickCanvas (canvas3d, branchGroup);
    pickCanvas.setMode(PickTool.GEOMETRY_INTERSECT_INFO);
    pickCanvas.setTolerance(0.0f); // extra bit of speed

    mouseEvents = new WakeupCriterion[3];
    mouseEvents[0] = new WakeupOnAWTEvent(MouseEvent.MOUSE_DRAGGED);
    mouseEvents[1] = new WakeupOnAWTEvent(MouseEvent.MOUSE_PRESSED);
    mouseEvents[2] = new WakeupOnAWTEvent(MouseEvent.MOUSE_RELEASED);
    mouseCriterion = new WakeupOr(mouseEvents);
    wakeupOn(mouseCriterion);
  }

  public void createPicker() {
    picker=new AiboPicker(scene);
  }

  void selectLink(MouseEvent event) {
    if (currentLink!=null) return;
    if (picker==null) return;
    forward.lock=true;

    pickCanvas.setShapeLocation (event);
    PickResult result = pickCanvas.pickClosest();
    if (result!=null) {
      currentLink=picker.pick(result.getObject());
      x = x_last = event.getX();
      y = y_last = event.getY();
      currentDragButton = event.getButton();

      if (currentLink==AiboPicker.leg_fl ||
          currentLink==AiboPicker.foot_fl ||
          currentLink==AiboPicker.knee_fl) {
        currentPos=Kinematics.getForward(Kinematics.FRONT_LEFT,
          new LegConfiguration(-forward.thigh_fl.x,
                                forward.thigh_fl.z,
                               -forward.knee_fl.x));
      } else if (currentLink==AiboPicker.leg_fr ||
          currentLink==AiboPicker.foot_fr ||
          currentLink==AiboPicker.knee_fr) {
        currentPos=Kinematics.getForward(Kinematics.FRONT_RIGHT,
          new LegConfiguration(-forward.thigh_fr.x,
                               -forward.thigh_fr.z,
                               -forward.knee_fr.x));

      } else if (currentLink==AiboPicker.leg_bl ||
          currentLink==AiboPicker.foot_bl ||
          currentLink==AiboPicker.knee_bl) {
              currentPos=Kinematics.getForward(Kinematics.BACK_LEFT,
          new LegConfiguration( forward.thigh_bl.x,
                                forward.thigh_bl.z,
                                forward.knee_bl.x));

      } else if (currentLink==AiboPicker.leg_br ||
          currentLink==AiboPicker.foot_br ||
          currentLink==AiboPicker.knee_br) {
        currentPos=Kinematics.getForward(Kinematics.BACK_RIGHT,
          new LegConfiguration( forward.thigh_br.x,
                               -forward.thigh_br.z,
                                forward.knee_br.x));

      }
    }
  }

  void rotateViewTransform() {
    double x_angle, y_angle;
    double x_factor = 0.03, y_factor = 0.03;
    x_angle = dy * y_factor;
    y_angle = dx * x_factor;

    viewTransformX.rotX(x_angle);
    viewTransformY.rotY(y_angle);
    
    viewTransformGroup.getTransform(currTransform);

    Matrix4d mat = new Matrix4d();
    currTransform.get(mat);

    currTransform.setTranslation(new Vector3d(0.0,0.0,0.0));
    currTransform.mul(viewTransformX, currTransform);
    currTransform.mul(viewTransformY, currTransform);

    Vector3d translation = new Vector3d(mat.m03, mat.m13, mat.m23);
    currTransform.setTranslation(translation);

    viewTransformGroup.setTransform(currTransform);
  }

  void translateViewTransform() {
    double x_factor = 0.01, y_factor = 0.01;

    viewTransformGroup.getTransform(currTransform);
    viewTranslation.x = dx*x_factor;
    viewTranslation.y = -dy*y_factor;

    viewTransformX.set(viewTranslation);
    currTransform.mul(viewTransformX, currTransform);
    viewTransformGroup.setTransform(currTransform);
  }

  void dragLink(MouseEvent event) {
    if (currentLink==null) return;

    x = event.getX();
    y = event.getY();

    dx = x - x_last;
    dy = y - y_last;

    if (currentLink==AiboPicker.body) {
      if (currentDragButton==MouseEvent.BUTTON1) {
        rotateViewTransform();
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        translateViewTransform();
      }
    } else if (currentLink==AiboPicker.head) {
      if (currentDragButton==MouseEvent.BUTTON1) { 
        forward.head.setYDiff(dx);
        forward.neck.setXDiff(dy);
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        forward.head.setZDiff(-dx);
      }
    } else if (currentLink==AiboPicker.jaw) {
      forward.jaw.setXDiff(dy);
    } else if (currentLink==AiboPicker.tail) {
      forward.tail.setXDiff(-dy);
      forward.tail.setYDiff(dx);
    } else if (currentLink==AiboPicker.leg_fl) {
      if (currentDragButton==MouseEvent.BUTTON1)
        forward.thigh_fl.setXDiff(dy);
      else if (currentDragButton==MouseEvent.BUTTON3)
        forward.thigh_fl.setZDiff(dx);
    } else if (currentLink==AiboPicker.knee_fl) {
      forward.knee_fl.setXDiff(dy);
    } else if (currentLink==AiboPicker.leg_fr) {
      if (currentDragButton==MouseEvent.BUTTON1)
        forward.thigh_fr.setXDiff(dy);
      else if (currentDragButton==MouseEvent.BUTTON3)
        forward.thigh_fr.setZDiff(dx);
    } else if (currentLink==AiboPicker.knee_fr) {
      forward.knee_fr.setXDiff(dy);
    } else if (currentLink==AiboPicker.leg_bl) {
      if (currentDragButton==MouseEvent.BUTTON1)
        forward.thigh_bl.setXDiff(dy);
      else if (currentDragButton==MouseEvent.BUTTON3)
        forward.thigh_bl.setZDiff(dx);
    } else if (currentLink==AiboPicker.knee_bl) {
      forward.knee_bl.setXDiff(dy);
    } else if (currentLink==AiboPicker.leg_br) {
      if (currentDragButton==MouseEvent.BUTTON1)
        forward.thigh_br.setXDiff(dy);
      else if (currentDragButton==MouseEvent.BUTTON3)
        forward.thigh_br.setZDiff(dx);
    } else if (currentLink==AiboPicker.knee_br) {
      forward.knee_br.setXDiff(dy);
    } else if (currentLink==AiboPicker.foot_fl) {
      if (currentDragButton==MouseEvent.BUTTON1) {
        currentPos.x=currentPos.x-dx*0.3;
        currentPos.z=currentPos.z-dy*0.3;
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        currentPos.y=currentPos.y+dx*0.3;
      }
      LegConfiguration l=Kinematics.getInverse(Kinematics.FRONT_LEFT,
                                               currentPos);
      forward.thigh_fl.setX(-l.rotator);
      forward.thigh_fl.setZ(l.shoulder);
      forward.knee_fl.setX(-l.knee);
    } else if (currentLink==AiboPicker.foot_fr) {
      if (currentDragButton==MouseEvent.BUTTON1) {
        currentPos.x=currentPos.x-dx*0.3;
        currentPos.z=currentPos.z-dy*0.3;
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        currentPos.y=currentPos.y+dx*0.3;
      }
      LegConfiguration l=Kinematics.getInverse(Kinematics.FRONT_RIGHT,
                                               currentPos);
      forward.thigh_fr.setX(-l.rotator);
      forward.thigh_fr.setZ(-l.shoulder);
      forward.knee_fr.setX(-l.knee);
    } else if (currentLink==AiboPicker.foot_bl) {
      if (currentDragButton==MouseEvent.BUTTON1) {
        currentPos.x=currentPos.x-dx*0.3;
        currentPos.z=currentPos.z-dy*0.3;
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        currentPos.y=currentPos.y+dx*0.3;
      }
      LegConfiguration l=Kinematics.getInverse(Kinematics.BACK_LEFT,
                                               currentPos);
      forward.thigh_bl.setX(l.rotator);
      forward.thigh_bl.setZ(l.shoulder);
      forward.knee_bl.setX(l.knee);
    } else if (currentLink==AiboPicker.foot_br) {
      if (currentDragButton==MouseEvent.BUTTON1) {
        currentPos.x=currentPos.x-dx*0.3;
        currentPos.z=currentPos.z-dy*0.3;
      } else if (currentDragButton==MouseEvent.BUTTON3) {
        currentPos.y=currentPos.y+dx*0.3;
      }
      LegConfiguration l=Kinematics.getInverse(Kinematics.BACK_RIGHT,
                                               currentPos);
      forward.thigh_br.setX(l.rotator);
      forward.thigh_br.setZ(-l.shoulder);
      forward.knee_br.setX(l.knee);
    }

    x_last = x;
    y_last = y;
  }

  void unselectLink() {
    currentLink=null;
    forward.lock=false;
  }

  public void processStimulus (Enumeration criteria) {
    WakeupCriterion wakeup;
    AWTEvent[] event;
    int id;

    while (criteria.hasMoreElements()) {
      wakeup=(WakeupCriterion) criteria.nextElement();
      if (wakeup instanceof WakeupOnAWTEvent) {
        event=((WakeupOnAWTEvent)wakeup).getAWTEvent();
        for (int i=0; i<event.length; i++) {
          id=event[i].getID();
          if (id == MouseEvent.MOUSE_DRAGGED) {
            dragLink((MouseEvent)event[i]);
          } else if (id == MouseEvent.MOUSE_PRESSED) {
            selectLink((MouseEvent)event[i]);
          } else if (id == MouseEvent.MOUSE_RELEASED) {
            unselectLink();
          }
        }
      }
    }
    wakeupOn(mouseCriterion);
  }

}
