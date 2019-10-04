package org.tekkotsu.mon;

import java.awt.Color;

public class ColorConverter{

    //Algorithm adapted from the 2007 Jefferson & Harvey paper,
    //"An Interface to Support Color Blind Computer Users."
    //Code by Bruce Hill and James Criscuolo, Spring 2012

    public static final int NORMAL = 0;
    public static final int PROTANOPE = 1;
    public static final int DEUTERANOPE = 2;
    public static final int TRITANOPE = 3;
    private static final double[] LMS_575 = {-0.0057, -0.1389, -3.4235};
    private static final double[] LMS_475 = {-0.9238, -0.6733, -0.2767};
    private static final double[] LMS_485 = {-0.7910, -0.5674, -0.5402};
    private static final double[] LMS_660 = {-1.0267, -2.1137, -5.7533};
    private static double[][] A = {{1.0, 0, 0}, {0, 1.0, 0}, {0, 0, 1.0}};
    private static int[] sliderVals = {0,0,0};
    public static int visionType = NORMAL;
    public static boolean hasChanged = true;
    
    public static void cycleVisionType() {
        visionType = (visionType + 1) % 4;
    }
    
    //Sliders represent two values, each between 0 and 5 with 0.25 increments
    private static double[] convertSliderValues(int sliderPos) {
        double max = 5.0;
        double scale = max/20.0;
        double[] ret = { scale * (sliderPos % 20), scale * (sliderPos / 20) };
        return ret;
    }
    
    public static void setSliders(int r, int g, int b) {
        // store in case we need to retrieve
        sliderVals[0] = r; sliderVals[1] = g; sliderVals[2] = b;
        double[][] slider = {convertSliderValues(r),
                             convertSliderValues(g),
                             convertSliderValues(b)};
        double[][] newA = {{1.0000000000, slider[0][0], slider[0][1]},
                           {slider[1][0], 1.0000000000, slider[1][1]},
                           {slider[2][0], slider[2][1], 1.0000000000}};
        A = newA;
    }
    
    // used to set default positions of the jslider
    public static int[] getSliders() {
        return sliderVals;
    }

    //multiply given rgb vector with 3*3 matrix representing intensity values
    private static double[] LMS(double[] c) {
        double[] ret = {(c[0]*0.1992 + c[1]*0.4112 + c[2]*0.0742), 
                        (c[0]*0.0353 + c[1]*0.2226 + c[2]*0.0574),
                        (c[0]*0.0185 + c[1]*0.1231 + c[2]*1.3550)};
        return ret;
    }
    
    private static double[] inverseLMS(double[] c) {
        double[] ret = {(c[0]*7.4645 + c[1]*-13.8882 + c[2]*0.1796), 
                        (c[0]*-1.1852 + c[1]*6.8053 + c[2]*-0.2234),
                        (c[0]*0.0058 + c[1]*-0.4286 + c[2]*0.7559)};
        return ret;
    }
    
    private static double[] makeDoubles(Color c) {
        double[] ret = {(double)c.getRed()/255.0,
                        (double)c.getGreen()/255.0,
                        (double)c.getBlue()/255.0};
        return ret;
    }

    //convert a byte array which is ordered [r1, b1, g1, r2, b2, g2, ...]
    public static byte[] convertRGBBytes(byte []orig) {
        if (visionType == NORMAL) return orig;
        int j =0, r, g, b;
        byte[] change = new byte[orig.length];
        while(j < orig.length){
            r = 0xFF & (int)orig[j];
            g = 0xFF & (int)orig[j+1];
            b = 0xFF & (int)orig[j+2];
            if(! (r == 128 && g == 128 && b == 128)){
                Color ret = convertColor(new Color(r,g,b));
                change[j] = (byte)ret.getRed();
                change[j+1] = (byte)ret.getGreen();
                change[j+2] = (byte)ret.getBlue();
            }else{
                change[j] = orig[j];
                change[j+1] = orig[j+1];
                change[j+2] = orig[j+2];
            }
            j += 3;
        }
        return change;

    }
    
    // convert a Color object
    public static Color convertColor(Color C) {
        if (visionType == NORMAL) return C;
        double[] Q = LMS(makeDoubles(C));
        double[] cvd = CVD(Q,visionType);
        double[] ret = {(Q[0] + A[0][0]*(cvd[0] - Q[0]) + A[0][1]*(cvd[1] - Q[1]) + A[0][2]*(cvd[2] - Q[2])),
                        (Q[1] + A[1][0]*(cvd[0] - Q[0]) + A[1][1]*(cvd[1] - Q[1]) + A[1][2]*(cvd[2] - Q[2])),
                        (Q[2] + A[2][0]*(cvd[0] - Q[0]) + A[2][1]*(cvd[1] - Q[1]) + A[2][2]*(cvd[2] - Q[2]))};
        
        // cap the colors between 0 and 1
        if (ret[0] < 0)
            ret[0] = 0;
        if (ret[0] > 1.0)
            ret[0] = 1;
        if (ret[1] < 0)
            ret[1] = 0;
        if (ret[1] > 1.0)
            ret[1] = 1;
        if (ret[2] < 0)
            ret[2] = 0;
        if (ret[2] > 1.0)
            ret[2] = 1;
        return new Color((int)(255*ret[0]),(int)(255*ret[1]),(int)(255*ret[2]));
    }
    
    // simulate color vision deficiency
    private static double[] CVD(double[] Q, int visionType){
        double[] E = {1.0, 1.0, 1.0};
        double[] lambda;
        if(visionType == PROTANOPE){
            if(Q[2]/Q[1] < E[2]/E[1])
                lambda = LMS_575;
            else
                lambda = LMS_475;
        }else if (visionType == DEUTERANOPE){
            if(Q[2]/Q[0] < E[2]/E[0])
                lambda = LMS_575;
            else
                lambda = LMS_475;
        }else if (visionType == TRITANOPE){
            if(Q[1]/Q[0] < E[1]/E[0])
                lambda = LMS_660;
            else
                lambda = LMS_485;
        } else {
            return Q;
        }
        double[] A = {0.68273*Math.pow(10,lambda[0]),
             0.35235*Math.pow(10,lambda[1]),
             1.00000*Math.pow(10,lambda[2])};
        double a = E[1] * A[2] - E[2] * A[1];
        double b = E[2] * A[1] - E[1] * A[2];
        double c = E[0] * A[1] - E[1] * A[0];
        switch(visionType){
            case PROTANOPE:
                { double[] ret = {-(b * Q[1] + c * Q[2])/a, Q[1], Q[2]}; return ret; }
            case DEUTERANOPE: 
                { double[] ret = {Q[0], -(a * Q[0] + c * Q[2])/b, Q[2]}; return ret; }
            case TRITANOPE:
                { double[] ret = {Q[0], Q[1], -(a * Q[0] + b * Q[1])/c}; return ret; }
            default: return Q;
        }
        
    }    
}


