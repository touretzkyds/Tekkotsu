package org.tekkotsu.mon;

import java.util.regex.*;

public class WMvar {
  public String type;
  public String registry;
  public String name;
  public boolean watched;
  public byte[] data;
  public int timestamp;
  public String stringData;
  static final Pattern printable=Pattern.compile("^[\\p{Print}\\p{Space}]*$");
  static final int MAX_STRINGREPRESENTATION_LENGTH=20;

  public WMvar(String type, String name, byte[] data, int timestamp) {
    this.type=type;
    setName(name);
    this.data=data;
    this.timestamp=timestamp;
    this.stringData=null;
    watched=false;
  }

  public void update(WMvar wmvar) {
    this.data=wmvar.data;
    this.stringData=wmvar.stringData;
    stringValueRepresentation();
  }

  void setName(String s) {
    int loc=s.lastIndexOf('.');
    if (loc<0) { registry=null; name=s; return; }
    registry=s.substring(0,loc);
    name=s.substring(loc+1, s.length());
  }

  public String toString() {
    return type+" " + getCompleteName() + " = " + stringValueRepresentation();
  }

  public String getName() {
    return name;
  }
  public String getValue() {
    return stringValueRepresentation();
  }
  public String getType() {
    return type;
  }
  public String getRegistry() {
    return registry;
  }
  public String getCompleteName() {
    if (registry==null) return name;
    else return registry+"."+name;
  }

  public String stringValueRepresentation() {
    if (stringData!=null) return stringData;
    String temp;
    if (data.length>MAX_STRINGREPRESENTATION_LENGTH)
      temp=new String(data, 0, MAX_STRINGREPRESENTATION_LENGTH);
    else
      temp=new String(data);
    Matcher m=printable.matcher(temp);
    if (m.matches()) {
      stringData=temp;
    } else {
      stringData="<binary data>";
    }
    return stringData;
  }
}
