package org.tekkotsu.mon;


import java.io.*;

public class JointWriter implements JointRequestor{
    JointRelay joints;
    boolean inited=false;
    BufferedWriter outlog;
    int counter=0;float start=-1,lasttime=0;    
    protected boolean init(String ip,int port){
	if(inited)return true;
	joints=new JointRelay(ip,port,this);
	return true;
    }

    public void dataArrival(Joints data){
	try{
	    synchronized(outlog){
		if(start==-1){
		    start=data.timestamp;
		    lasttime=start;
		}
		String ret=""+(data.timestamp-start)+" "+
		    catArray(data.positions)+
		    catArray(data.duties)+
		    catArray(data.sensors)+
		    catArray(data.buttons);
		outlog.write(ret);
		outlog.newLine();
		counter+=ret.length()+1;
		System.out.println(data.timestamp-start+" (+"+(data.timestamp-lasttime)+") msec. "+counter+" bytes ("+((float)counter/(data.timestamp-start))+" bytes/s)");
		lasttime=data.timestamp;
	    }
	}
	catch(IOException e){
	    System.err.println(e);
	}
    }

    public static File findNextLog(String prefix){
	File quick;
	int i=0;
	do{
	    i++;
	    quick=new File(prefix+i+".log");
	}while(quick.exists());
	return quick;
    }

    public JointWriter(){
	String s[]=new String[0];
	launch(s);
    }

    public JointWriter(String s[]){
	launch(s);
    }

    protected void launch(String s[]){
	String ip="172.16.1.1",filename="aibo";
	for(int i=0;i<s.length;i++){
	    if(s[i].equals("-?") || s[i].equals("-h") || s[i].startsWith("--h")){
		System.out.println("valid flags:\n"+
				   "-i<ip address/name of dog>"+
				   "-f<log filename prefix>");
		System.exit(0);
	    }
	    if(s[i].startsWith("-i"))
		ip=s[i].substring(2);
	    else if(s[i].startsWith("-f") || s[i].startsWith("-l"))
		filename=s[i].substring(2);
	}
	File outfile=findNextLog(filename);
	try {
	    outlog=new BufferedWriter(new FileWriter(outfile));
	    System.out.println("Writing to: "+outfile);
	    if(!init(ip,10031)){
		System.err.println("Init failed. Very sad...");
		System.exit(1);
		return;
	    }
	    while(true){
		if(System.in.available()>0){ //keyhit
		     int b=System.in.read();
		     System.out.print("read "+b+": ");
		     if(b=='q'){
			 System.out.println("User aborted.\n");
			 synchronized(outlog){
			     outlog.close();
			     joints.close();
			     System.out.println("Finished writing to: "+outfile);
			     System.exit(0);
			 }
			 break;
		     }
		}
	    }
	}
	catch(IOException e){
	    System.err.println(e);
	}
    }
    public String catArray(float []array){
	String ret="";
	for(int i=0;i<array.length;i++)
	    ret+=array[i]+" ";
	return ret;
    }
}
