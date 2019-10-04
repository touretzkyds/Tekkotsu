package org.tekkotsu.mon;

import javax.swing.JOptionPane;

public class ControllerErr {

	public static void main(String args[]) {
		new ControllerErr("",0,args);
	}

	public ControllerErr(String host, int port, String args[]) {
		for(int i=0; i<args.length; i++)
			System.out.println("<"+args[i]+">");
		if(args.length==1)
			JOptionPane.showMessageDialog(null,args[0]);
		else if(args.length>1)
			JOptionPane.showMessageDialog(null,args[1],args[0],JOptionPane.ERROR_MESSAGE);
	}
}
