package org.tekkotsu.mon;

import java.io.*;
import java.net.*;

public class AiboComm {
  public TCPListener stderr;
  public TCPListener stdio;
  public TCPListener visionRLE;
  public TCPListener visionRaw;
  public TCPListener visionObjs;
  public TCPListener worldGrid;
  public TCPListener localizer;

  public AiboComm() {
//    stdio=new TextListener(10001);
  }

  public void close() {
    stdio.close();
  }

  public static void sleep(long millis) {
    try { Thread.sleep(millis); } catch (Exception ex) { }
  }
  
  public static void main(String[] args) {
    System.out.println("please instantiate from within Matlab");
    System.exit(1);
  }
}
