package org.tekkotsu.mon;

//'Joints' is something of a misnomer -- also stores sensor and button information as well
public class Joints {
	public String model;
  public float[] positions;
  public float[] duties;
  public float[] sensors;
  public float[] buttons;
  public long timestamp;
	public long frame;

  Joints() {
		//note: this is old and for the ers2xx, but, eh. doesn't hurt to leave like this
		//each packet holds correct number of items for each category, so these
		//can be reassigned with the correct number of items for the model in question
    positions=new float[18];
    duties=new float[18];
    sensors=new float[10];
    buttons=new float[8];
  } 

  public String toString() {
    return positionsString()+dutiesString()+sensorsString()+buttonsString();
  }

  public String positionsString() {
    String ret="";
    for (int i=0; i<positions.length; i++) ret+=positions[i]+" ";
    return ret+"\n";
  }

  public String dutiesString() {
    String ret="";
    for (int i=0; i<duties.length; i++) ret+=duties[i]+" ";
    return ret+"\n";
  }

  public String sensorsString() {
    String ret="";
    for (int i=0; i<sensors.length; i++) ret+=sensors[i]+" ";
    return ret+"\n";
  }

  public String buttonsString() {
    String ret="";
    for (int i=0; i<buttons.length; i++) ret+=buttons[i]+" ";
    return ret+"\n";
  }

}
