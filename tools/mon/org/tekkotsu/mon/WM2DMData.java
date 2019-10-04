package org.tekkotsu.mon;

// Data container for the WorldModel2 spherical depth map

public class WM2DMData {
  public float[] DM_depth;
  public float[] DM_confidence;
  public int[] DM_color;

  WM2DMData() {
    DM_depth = new float[WorldModel2Conf.DM_CELL_COUNT];
    DM_confidence = new float[WorldModel2Conf.DM_CELL_COUNT];
    DM_color = new int[WorldModel2Conf.DM_CELL_COUNT];
  }
}
