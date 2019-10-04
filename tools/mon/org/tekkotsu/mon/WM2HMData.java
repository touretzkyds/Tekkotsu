package org.tekkotsu.mon;

// Data container for the WorldModel2 horizontal height map. Also is used
// for the global height map

public class WM2HMData {
  public float[] HM_height;
  public float[] HM_trav;
  public float[] HM_confidence;
  public int[] HM_color;

  WM2HMData() {
    HM_height = new float[WorldModel2Conf.HM_CELL_COUNT];
    HM_trav = new float[WorldModel2Conf.HM_CELL_COUNT];
    HM_confidence = new float[WorldModel2Conf.HM_CELL_COUNT];
    HM_color = new int[WorldModel2Conf.HM_CELL_COUNT];
  }
}
