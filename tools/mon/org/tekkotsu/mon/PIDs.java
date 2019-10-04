package org.tekkotsu.mon;

public class PIDs {
  public int timestamp;
  public float[] P;
  public float[] I;
  public float[] D;

  PIDs() {
    P=new float[18];
    I=new float[18];
    D=new float[18];
  } 
}
