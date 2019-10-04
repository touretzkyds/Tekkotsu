package org.tekkotsu.mon;

// Constants regarding the configuration of WorldModel2
//
// YOU MUST CHANGE THE CONSTANTS TO MATCH THOSE IN THE WorldModel2
// CONFIGURATION FILES--otherwise, the amounts of data read and written won't
// match up.

import java.lang.Math;

public class WorldModel2Conf {
  // Array dimensions in pixels
    // from WorldModel2/Maps/Configuration.h
  public static final int ALM_DM_V_SIZE = 51;
  public static final int ALM_DM_H_SIZE = 89;
  public static final int ALM_HM_SIZE = 20; // don't forget that this value
					    // is the *radius* of the height
					    // map in pixels, hence one half
					    // of the edge length
  public static final int AGM_V_SIZE = 100;
  public static final int AGM_H_SIZE = 100;
    // from WorldModel2/FastSLAM/Configuration.h
  public static final int AFS_NUM_LANDMARKS = 12;
  public static final int AFS_NUM_PARTICLES = 400;

  // Array dimensions in measurement units
    // from WorldModel2/Maps/Configuration.h
  public static final double ALM_DM_TOP = R(25.0);
  public static final double ALM_DM_BOTTOM = R(-75.0);
  public static final double ALM_DM_LEFT = R(85.0);
  public static final double ALM_DM_RIGHT = R(-85.0);
  public static final double ALM_HM_RADIUS = 400;
  public static final double AGM_TOP = 2000;
  public static final double AGM_BOTTOM = -2000;
  public static final double AGM_LEFT = -2000;
  public static final double AGM_RIGHT = 2000;

  // These computed constants should not need to change.
  public static final int HM_CELL_COUNT = 4*ALM_HM_SIZE*ALM_HM_SIZE;
  public static final int DM_CELL_COUNT = ALM_DM_V_SIZE*ALM_DM_H_SIZE;
  public static final int GM_CELL_COUNT = AGM_V_SIZE*AGM_H_SIZE;

  // Convenience degree to radian function
  public static final double R(double D) { return D*Math.PI/180.0; }
}
