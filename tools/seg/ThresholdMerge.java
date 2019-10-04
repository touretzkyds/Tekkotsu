import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;

public class ThresholdMerge {
  public static void main(String args[]) {
    if (args.length<3) {
      System.out.println("Usage: java ThresholdMerge infile1 [infile2 ...] outfile");
			System.out.println("       Each threshold file is copied in turn into the output");
			System.out.println("       Earlier thresholds override later thresholds");
			System.out.println("       You will still need to merge the .col files manually");
      System.exit(2);
    }
    ThresholdMerge filter=new ThresholdMerge(args);
  }

  public ThresholdMerge (String files[]) {

    try {
      FileOutputStream out=new FileOutputStream(files[files.length-1]);

      byte[] tmdata=new byte[65536];
      byte[] mergeddata=new byte[65536];

      for (int i=0; i<mergeddata.length; i++) mergeddata[i]=0;

      for (int i=0; i<files.length-1; i++) {
        System.out.println("Adding "+files[i]+ "...");
        FileInputStream in=new FileInputStream(files[i]);
        in.read(tmdata, 0, 20);
        if (i==0) out.write(tmdata, 0, 20);
        in.read(tmdata);

        for (int j=0; j<mergeddata.length; j++)
          if (mergeddata[j]==0) mergeddata[j]=tmdata[j];
        
        in.close();
      }

      out.write(mergeddata);
      out.close();
    } catch (Exception ex) {
      System.out.println(ex);
    }
  }
}
