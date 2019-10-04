package org.tekkotsu.mon;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.lang.*;

public class GraphCanvas extends Canvas {
  float ymin, ymax, yrange;
  int lastval;
  LinkedList valQueue=new LinkedList();

  public GraphCanvas (float ymin, float ymax) {
    super();

    this.ymin=ymin;
    this.ymax=ymax;
    this.yrange=ymax-ymin;
    lastval=0;

    setBackground(Color.black);
  }

  public void registerValue(float yval) {
    valQueue.add(new Float(yval));
    repaint();
  }

  public void paint(Graphics g) {
    update(g);
  }

  public void update(Graphics g) {
    if (valQueue.isEmpty()) return;
    Dimension d=getSize();

    g.setColor(Color.red);
    int numadded=valQueue.size();
    int xstart=d.width-numadded;
    g.copyArea(numadded,0,xstart,d.height,-numadded,0);
    g.clearRect(xstart,0,numadded,d.height);
    for (int i=0; i<numadded; i++) {
      float val=((Float)valQueue.removeFirst()).floatValue();
      val=val-ymin;
      val=val/yrange;
      if (val<0.0f || val>1.0f) continue;
      int pos=(int)(d.height*val); 
      g.drawLine(xstart+i, lastval, xstart+i+1, pos);
      lastval=pos;
    }
  }
}
