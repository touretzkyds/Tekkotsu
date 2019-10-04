package org.tekkotsu.aibo3d;

import java.util.*;
import java.lang.*;
import javax.media.j3d.*;

import com.sun.j3d.loaders.Scene;

public class AiboPicker {
  Hashtable _namedObjects;
  Hashtable _shapeToTG;

  public static final String head="head";
  public static final String tail="tail";
  public static final String body="body";
  public static final String foot_fl="foot_fl";
  public static final String foot_fr="foot_fr";
  public static final String foot_bl="foot_bl";
  public static final String foot_br="foot_br";
  public static final String leg_fl="leg_fl";
  public static final String leg_fr="leg_fr";
  public static final String leg_bl="leg_bl";
  public static final String leg_br="leg_br";
  public static final String knee_fl="knee_fl";
  public static final String knee_fr="knee_fr";
  public static final String knee_bl="knee_bl";
  public static final String knee_br="knee_br";
  public static final String jaw="jaw";

  public AiboPicker (Scene scene) {
    _namedObjects=scene.getNamedObjects();
    _shapeToTG=new Hashtable();
    initERS210();
  }

  public String pick(Object shape) {
    return (String)_shapeToTG.get(shape);
  }

  public TransformGroup getTG(Object key) {
    return (TransformGroup)_namedObjects.get(key);
  }

  void addShapesFor(TransformGroup key, String name) {
    Enumeration shapes = key.getAllChildren();
    while (shapes.hasMoreElements()) {
      Object s=shapes.nextElement();
      if (s instanceof Shape3D) {
        _shapeToTG.put(s, name);
      }
    }
  }

  void initERS210() {
    addShapesFor(getTG("objects-ERS-210/neck.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/head.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_g01.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_g02.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_g03.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_r01.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_r02.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/LED_r03.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/ear-l.lwo"), head);
    addShapesFor(getTG("objects-ERS-210/ear-r.lwo"), head);

    addShapesFor(getTG("objects-ERS-210/jaw.lwo"), jaw);
    addShapesFor(getTG("objects-ERS-210/body2.lwo"), body);
    addShapesFor(getTG("objects-ERS-210/tail2.lwo"), tail);
   
    addShapesFor(getTG("objects-ERS-210/foot-b-r.lwo"), foot_br);
    addShapesFor(getTG("objects-ERS-210/leg-b-low-r.lwo"), knee_br);
    addShapesFor(getTG("objects-ERS-210/leg-b-up-r.lwo"), leg_br);

    addShapesFor(getTG("objects-ERS-210/foot-b-l.lwo"), foot_bl);
    addShapesFor(getTG("objects-ERS-210/leg-b-low-l.lwo"), knee_bl);
    addShapesFor(getTG("objects-ERS-210/leg-b-up-l.lwo"), leg_bl);

    addShapesFor(getTG("objects-ERS-210/foot-f-r.lwo"), foot_fr);
    addShapesFor(getTG("objects-ERS-210/leg-f-low-r.lwo"), knee_fr);
    addShapesFor(getTG("objects-ERS-210/leg-f-up-r.lwo"), leg_fr);

    addShapesFor(getTG("objects-ERS-210/foot-f-l.lwo"), foot_fl);
    addShapesFor(getTG("objects-ERS-210/leg-f-low-l.lwo"), knee_fl);
    addShapesFor(getTG("objects-ERS-210/leg-f-up-l.lwo"), leg_fl);
    for (Enumeration e=_shapeToTG.keys(); e.hasMoreElements();) {
      Object o=e.nextElement();
      // System.out.println(o + "->" + _shapeToTG.get(o));
    }
  }
}
