import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;

public class Vote {
	public static void main(String args[]) {
		if (args.length<3) {
			System.out.println("usage: java Vote infile1 [infile2 ...] outfile");
			System.out.println("       Each threshold file casts a vote for each classified color.");
			System.out.println("       The output is the 'winning' classification for each.");
			System.out.println("       Hint: pass a threshold multiple times to weight it more heavily.");
			System.out.println("       For this to make any sense, each threshold file must agree on the color indicies");
			System.exit(2);
		}
		Vote filter=new Vote(args);
	}

	final static byte MAX_INDEX=10;

	public Vote (String files[]) {
		try {
			byte[] header=new byte[20];
			byte[][] votes=new byte[65536][MAX_INDEX];
			byte[] tmdata=new byte[65536];


			for (int i=0; i<votes.length; i++)
				for(byte j=0; j<MAX_INDEX; ++j)
					votes[i][j]=0;

			for (int i=0; i<files.length-1; i++) {
				System.out.println("Processing "+files[i]+ "...");
				FileInputStream in=new FileInputStream(files[i]);
				in.read(header);
				in.read(tmdata);
				in.close();
				for (int j=0; j<tmdata.length; j++) {
					//System.out.println(j+": "+tmdata[j]);
					if(tmdata[j]!=0) {
						votes[j][tmdata[j]]++;
					}
				}
			}
			
			for (int i=0; i<tmdata.length; i++) {
				tmdata[i]=0;
				for(byte j=1; j<MAX_INDEX; ++j)
					if(votes[i][j]>votes[i][tmdata[i]])
						tmdata[i]=j;
			}
			FileOutputStream out=new FileOutputStream(files[files.length-1]);
			out.write(header);
			out.write(tmdata);
			out.close();
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}
}
