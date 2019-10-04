package org.tekkotsu.mon;

import java.io.*;
import java.net.*;
import java.util.*;
import javax.swing.*;
import java.awt.*;
import javax.swing.text.*; 
import java.awt.event.*; 
import java.io.File; 
import java.lang.*;
import java.lang.Object.*;
import javax.sound.sampled.*;
import javax.sound.sampled.spi.*;
//import javax.sound.sampled.spi.AudioFileReader.*;
//import javax.sound.sampled.spi.AudioFileWriter.*;

public class LGClient extends JFrame {
	private static JEditorPane jep;	   
	final static JLabel statusBar = new JLabel(" ");
	final static String tmpPath="/tmp/LGwww-"+System.getProperty("user.name","unknown_user");

	public LGClient(){

	}

	public LGClient(String file){
		super("LGClient");
		setSize(1280,1024); 
		setDefaultCloseOperation(EXIT_ON_CLOSE); 
		JPanel urlPanel = new JPanel(); 
		urlPanel.setLayout(new BorderLayout()); 
		JTextField urlField = new JTextField(file);
		urlPanel.add(new JLabel("Site: "), BorderLayout.WEST); 
		urlPanel.add(urlField, BorderLayout.CENTER); 
		
		jep = new JEditorPane(); 
		jep.setEditable(false); 
		jep.setContentType("text/html");
		 
		try { 
			jep.setPage(file);
		} 
		catch(Exception e) { 
			statusBar.setText("Could not open starting page.  Using a blank."); 
		} 

		JScrollPane jsp = new JScrollPane(jep); 
		getContentPane().add(jsp, BorderLayout.CENTER); 
		getContentPane().add(urlPanel, BorderLayout.NORTH); 
		getContentPane().add(statusBar, BorderLayout.SOUTH); 
	
		// and last but not least, hook up our event handlers 
		urlField.addActionListener(new ActionListener() { 
				public void actionPerformed(ActionEvent ae) { 
					try { 
						jep.setPage(ae.getActionCommand()); 
					} 
					catch(Exception e) { 
						statusBar.setText("Error: " + e.getMessage()); 
					} 
				} 
			}); 
		jep.addHyperlinkListener(new SimpleLinkListener1(jep, urlField, 
														 statusBar));	 
	}
	
	public StringBuffer readLine(InputStream inputS){
		StringBuffer lineReader = new StringBuffer();
		int curr;
		try{
			while((curr = inputS.read()) != '\n'){
				if(curr == -1)
					return null;
				else
					lineReader.append((char)curr);
			}
		}
		catch(IOException e){
			System.exit(-1);
		}
		return lineReader;
	}
	
	static protected class SoundThread extends Thread {
		AudioInputStream audioInputStream = null;
		AudioFormat format = null;
		SourceDataLine auline = null;
		
		public SoundThread(File soundFile) {
	 
			try {
				audioInputStream = AudioSystem.getAudioInputStream(soundFile);
			} catch (UnsupportedAudioFileException e1) {
				e1.printStackTrace();
				return;
			} catch (IOException e1) {
				e1.printStackTrace();
				return;
			}
	 
			format = audioInputStream.getFormat();
			DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);
	 
			try {
				auline = (SourceDataLine) AudioSystem.getLine(info);
				auline.open(format);
			} catch (LineUnavailableException e) {
				e.printStackTrace();
				return;
			} catch (Exception e) {
				e.printStackTrace();
				return;
			}
			start();
		 }
		 public void run() {
			auline.start();
			
			try {
				// buffer 0.1s at a time:
				byte[] abData = new byte[(int)(0.1*format.getFrameRate()*format.getFrameSize())];
				int nBytesRead = audioInputStream.read(abData, 0, abData.length);
				while (nBytesRead!=-1 && !interrupted()) {
					auline.write(abData, 0, nBytesRead);
					nBytesRead = audioInputStream.read(abData, 0, abData.length);
				}
			} catch (IOException e) {
				e.printStackTrace();
				return;
			} finally {
				auline.drain();
				auline.close();
			}
		}
	}

	public static void main(String[] args) throws IOException {
		
		Socket sock = null;
		PrintWriter out = null;
		InputStream in = null;
		boolean test = true;
		InputStream fis=  null;
		SoundThread snd = null;

		try {
			sock = new Socket(args[0], 10100);
			out = new PrintWriter(sock.getOutputStream(), true);
			in = sock.getInputStream();
		} catch (UnknownHostException e) {
			System.err.println("Don't know about host: "+args[0]);
			System.exit(1);
		} catch (IOException e) {
			System.err.println("Couldn't get I/O for the connection to: "+args[0]);
			System.exit(1);
		}
		BufferedReader stdIn = new BufferedReader(new InputStreamReader(System.in));
		BufferedWriter bw = null;
		StringBuffer line;
		(new File(tmpPath)).mkdir();
		LGClient lgc = new LGClient();
		
		while ((line = lgc.readLine(in)) != null) {
			StringTokenizer t = new StringTokenizer(line.toString());
			try{
				if(t.hasMoreTokens()){
					String st = t.nextToken();
					if(st.equals("UPLOAD_HTML")){
						String filename = t.nextToken();
						StringBuffer line2 = null;
						bw = new BufferedWriter(new FileWriter(tmpPath+"/" + filename));
						while(!((line2 = lgc.readLine(in)).toString()).toUpperCase().equals("</HTML>")){
							bw.write(line2.toString(),0,(line2.toString()).length());
							bw.newLine();
						}
						if((line2.toString()).toUpperCase().equals("<HTML>"))
							bw.write(line2.toString(),0,(line2.toString()).length());
						bw.close();
					}
					if(st.equals("UPLOAD_AUDIO")){
						String filename = t.nextToken();
						String newFilename = t.nextToken();
						
						try{
							File f = new File(tmpPath+"/" + newFilename);
							URL url = new URL(filename);
							AudioInputStream ais = AudioSystem.getAudioInputStream(url);
							AudioSystem.write(ais,AudioFileFormat.Type.WAVE,f);
							ais.close();
						}
						catch(UnsupportedAudioFileException e){
							System.err.println("Unsupported Audio File: "+ filename);
							System.exit(1);
						}
						
					}
					if(st.equals("UPLOAD_BINARY")){
						String filename = t.nextToken();
						int bytecount=Integer.parseInt(t.nextToken());
						FileOutputStream os = new FileOutputStream(tmpPath+"/" + filename);
						byte[] bytes = new byte[bytecount];
														
						for(int i=0; i<bytecount; i++) {
							int b = in.read(); 
							bytes[i]= (byte)b;
						}
						os.write(bytes,0,bytecount);
						os.close();
					}
					if(st.equals("DISPLAY")){
						String filename = t.nextToken();
						if (!(filename.startsWith("file:"))) {
							// If it's not a fully qualified url, assume it's a file
							if (filename.startsWith("/")) {
								// Absolute path, so just prepend "file:"
								filename = "file:" + filename;
							}
							else {
								try {
									// assume it's relative to the starting point...
									File f = new File(tmpPath+"/" + filename);
									filename = f.toURI().toString();
								}
								catch (Exception e) {
									filename = "http://www-2.cs.cmu.edu/afs/cs.cmu.edu/project/skinnerbots/LookingGlass/";
								}
							}
						}
						else {
							filename = "http://www-2.cs.cmu.edu/afs/cs.cmu.edu/project/skinnerbots/LookingGlass/";
						}
						try { 
							if(test)
								new LGClient(filename).setVisible(true);
							else {
								jep.setText("<html><head> <title> blank </title> </head> <body> </body> </html>");
								jep.setPage(filename); 
							}
						} 
						catch(Exception e) { 
							statusBar.setText("Could not open starting page.  Using a blank."); 
						}		
						test = false;
					}
					if(st.equals("PLAY")){
						File f = new File(t.nextToken());
						if (!f.exists()) {
							System.err.println("Wave file not found: " + f.toString());
							return;
						}
						snd = new LGClient.SoundThread(f);
					}
					if(st.equals("STOP")){
						File f = new File(t.nextToken());
						if(snd != null) {
							snd.interrupt();
							snd = null;
						} else
							System.out.println("Cannot close file:" + f.toString());
					}
				}
			}
			catch(NumberFormatException e){
				System.out.println("Corrupted data. "+line + " is ignored...");
			} 
		}
		out.close();
		in.close();
		stdIn.close();
		sock.close();
	}
}
