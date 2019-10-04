package org.tekkotsu.mon;
import javax.swing.*;

public class TestGraph {
  public static void main(String args[]) {
    JFrame frame=new JFrame("graph test");
    GraphCanvas canvas=new GraphCanvas(0.0f, 1.0f);
    frame.setSize(500,300);
    frame.setVisible(true);
    frame.getContentPane().add(canvas);
    while (true) {
      canvas.registerValue((float)Math.random());
      try { Thread.sleep(10); } catch (InterruptedException ex) {}
    }
  }
}
