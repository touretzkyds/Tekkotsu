import java.util.ArrayList;

public class JointNode {
	public JointNode() {
		//centerOfMass = new double[3];
                aComponent = false;
		children = new ArrayList();
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	public JointNode(String s) {
		//centerOfMass = new double[3];
		aComponent = false;
		children = new ArrayList();
		name = s;
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	public double theta;
	public double d;
	public double alpha;
	public double r;
	public double qOffset;
	public double min;
	public double max;
	public double mass;
	public double[] scale = new double[3];
	public double[] cscale = new double[3];
	public double[] cOffset = new double[3];	
	public double[] crotation = new double[3];
        public double[] COM = new double[3];
	public String name;
	public String CollisionModel;
	public String model;
	public String material;
	public String iksolver;
	public String type = "revolute";
	public boolean aComponent;

	public ArrayList children;

	private static double radToDeg(double rad) {
		return 180*rad/Math.PI;
	}
	public void setComponent() {
		aComponent = true;
        }
        private boolean isComponent() {
	        return aComponent;
        }

	public String toXMLString() {
		String theta_char="\u03B8";
		String alpha_char="\u03B1";
		String degree_char = "\u00B0";
		String s = "";
		s += "<dict>\n";
		s += "\t<key>JointType</key> <string>"+type+"</string>\n";
		s += "\t<key>"+theta_char+"</key> <real>"+radToDeg(theta)+degree_char+"</real>\n";
		s += "\t<key>d</key> <real>"+d+"</real>\n";
		s += "\t<key>"+alpha_char+"</key> <real>"+radToDeg(alpha)+degree_char+"</real>\n";
		s += "\t<key>r</key> <real>"+r+"</real>\n";
		s += "\t<key>Min</key> <real>"+radToDeg(min)+degree_char+"</real>\n";
		s += "\t<key>Max</key> <real>"+radToDeg(max)+degree_char+"</real>\n";
		s += "\t<key>qOffset</key> <real>" + qOffset + "</real>\n";
		s += "\t<key>Mass</key> <real>"+mass+"</real>\n";
		if (model != null) {
			s += "\t<key>Model</key> <string>"+model+"</string>\n";
			s += "\t<key>ModelScale</key> <array><real>"+scale[0]+"</real><real>"+scale[1]+"</real><real>"+scale[2]+"</real></array>\n";
		}
		else {
			s += "\t<key>Model</key> <string></string>\n";	
		}		

		if (material != null) {
			s += "\t<key>Material</key> <string>"+material+"</string>\n";
		}
		if (iksolver != null) {
			s += "\t<key>IKSolver</key> <string>"+iksolver+"</string>\n";
		}
		if (name != null) {
			s += "\t<key>Name</key> <string>"+name+"</string>\n";
		}
		if (CollisionModel !=null){
			s += "\t<key>CollisionModel</key> <string>"+CollisionModel+"</string>\n";
			s += "\t<key>CollisionModelScale</key> <array><real>"+cscale[0]+"</real><real>"+cscale[1]+"</real><real>"+cscale[2]+"</real></array>\n";
			s += "\t<key>CollisionModelOffset</key> <array><real>"+cOffset[0]+"</real><real>"+cOffset[1]+"</real><real>"+cOffset[2]+"</real></array>\n";			 
                        s += "\t<key>CollisionModelRotation</key> <array><real>"+crotation[0]+"</real><real>"+crotation[1]+"</real><real>"+crotation[2]+"</real></array>\n";
                
                }
		if (COM != null) {
			s += "\t<key>CenterOfMass</key> <array><real>"+COM[0]+"</real><real>"+COM[1]+"</real><real>"+COM[2]+"</real></array>\n";
                }		
		
		s += "</dict>\n";
		int size = children.size();

//		if (size > 1) {
		for (int i = 0; i < size; i++) {
			//if(((JointNode)(children.get(i))).isComponent())
			//	s += "\t<key>Components</key> ";
			s += "<array>\n" + ((JointNode)(children.get(i))).toXMLString() + "</array>\n";
		}


//		}
//		else if (size == 1) {
//			s += ((JointNode)(children.get(0))).toXMLString();
//		}


		return s;
	}

	public String toString() {
		if (name != null) {
			return name;
		} else {
			return "dummy joint";
		}
	}

	public void addChild(JointNode child) {
		children.add(child);
	}
	public void removeChild(JointNode child){
		children.remove(child);
	}
}
