package org.tekkotsu.mon;

import java.net.*;
import java.io.*;


public class NetDebug {
  public static void main(String args[]) {
    if (args.length<1) {
      System.out.println("usage: java NetDebug ip_addr");
      System.exit(1);
    }
    NetDebug netdebug=new NetDebug(args[0]);
  }

  public NetDebug(String ip) {
    try {
      Socket s = new Socket(ip, 10011);
      InputStream in = s.getInputStream();
      int i=0;
      while (true) {
        i++;
        if (readInt(in)==2) {
          System.out.println(i);
        }
      }
    } catch (Exception ex) {
      System.out.println(ex);
    }
  }

  int readInt(InputStream in) throws IOException {
    int read=0;
    int last=0;
    byte[] buf=new byte[4];
    while (read<4 && last>=0) { last=in.read(buf,read,4-read); read+=last; }
    if(last<0)
      throw new IOException("my exception");
    return (b2i(buf[3])<<24) | (b2i(buf[2])<<16) |
           (b2i(buf[1])<< 8) | b2i(buf[0]);
  }

  int b2i(byte b) { return (b>=0)?(int)b:((int)b)+256; }
}
