package org.tekkotsu.mon;

// Data container for the WorldModel2 FastSLAM subsystem

public class WM2FSData {
  public float[][] FS_particlesPos;
  public float FS_x, FS_y, FS_theta;
  public float[][] FS_landmarkPos;
  public float[][] FS_covariance;

  WM2FSData() {
    FS_particlesPos = new float[WorldModel2Conf.AFS_NUM_PARTICLES][3];
    FS_landmarkPos = new float[WorldModel2Conf.AFS_NUM_LANDMARKS][2];
    FS_covariance = new float[WorldModel2Conf.AFS_NUM_LANDMARKS][4];
  }
}
