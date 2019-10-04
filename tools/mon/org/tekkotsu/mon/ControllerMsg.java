package org.tekkotsu.mon;

import javax.swing.JOptionPane;

public class ControllerMsg {

	public static void main(String args[]) {
		new ControllerMsg("",0,args);
	}

	public ControllerMsg(String host, int port, String args[]) {
		for(int i=0; i<args.length; i++)
			System.out.println("<"+args[i]+">");
		if(args.length==1)
			JOptionPane.showMessageDialog(null,args[0]);
		else if(args.length>1)
			JOptionPane.showMessageDialog(null,args[1],args[0],JOptionPane.INFORMATION_MESSAGE);
	}
}
