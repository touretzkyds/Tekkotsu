package org.tekkotsu.mon;

import java.net.*;
import java.io.*;


public class PrintDuties {
  public static void main(String args[]) {
    if (args.length<1) {
      System.out.println("usage: java PrintDuties ip_addr");
      System.exit(1);
    }
    WorldStateJointsListener wsjl=new WorldStateJointsListener(args[0],10031);
    while (true) {
      if (wsjl.isConnected() && wsjl.hasData()) {
        System.out.print(wsjl.getData().dutiesString());
      } else {
        try { Thread.sleep(10); } catch (Exception ex) { }
      }
    }
  }
}
