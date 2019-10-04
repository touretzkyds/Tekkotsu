package org.tekkotsu.aibo3d;

public class LegConfiguration {
  public double rotator;
  public double shoulder;
  public double knee;

  public LegConfiguration (double rotator, double shoulder, double knee) {
    this.rotator=rotator;
    this.shoulder=shoulder;
    this.knee=knee;
  }

  public String toString() {
    return "rotator = " + String.valueOf(rotator) + ", " +
           "shoulder = " + String.valueOf(shoulder) + ", " +
           "knee = " + String.valueOf(knee);
  }
}
