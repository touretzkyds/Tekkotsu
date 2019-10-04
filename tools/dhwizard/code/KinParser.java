 import java.io.File;
import javax.xml.parsers.*;
import org.w3c.dom.*;
import org.w3c.dom.traversal.*;


public class KinParser {

	private TreeWalker walker;

	private int i;

	public KinParser() {}

	public JointNode parse (String fname) {
		File file = new File(fname);
		JointNode j = null;
		try {
			DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
			dbf.setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false);
			
			DocumentBuilder builder = dbf.newDocumentBuilder();
			Document document = builder.parse(file);

    		DocumentTraversal traversal = (DocumentTraversal) document;

	    	walker = traversal.createTreeWalker(
				    document.getDocumentElement(), 
					NodeFilter.SHOW_ELEMENT, null, true);

			i = 0;

			j = parse();

		} catch (Exception e) {
			e.printStackTrace();
		}

		return j;
	}

	public JointNode parse() {
		JointNode j = new JointNode();
		Node n = walker.getCurrentNode();
		if (n.getNodeName().equals("array")) {
			walker.firstChild();
			j = parseArray();
			walker.setCurrentNode(n);
		}
		else if (n.getNodeName().equals("dict")) {
			walker.firstChild();
			j = parseDict();
			walker.setCurrentNode(n);
			while (walker.nextSibling() != null) {
				j.addChild(parse());
			}
		}
		else if (n.getNodeName().equals("plist")) {
			walker.firstChild();
			j = parse();
		}
		return j;
	}

	public JointNode parseArray() {
		JointNode j;
		Node n = walker.getCurrentNode();
		j = parse();
		while (walker.nextSibling() != null) {
			j.addChild(parse());
		}
		walker.setCurrentNode(n);
		return j;
	}

	public JointNode parseDict() {
		JointNode j = new JointNode();
		for (Node key = walker.getCurrentNode(); key != null;
				key = walker.nextSibling()) {
			Node val = walker.nextSibling();
			if (key.getFirstChild() != null &&
				key.getFirstChild().getNodeName().equals("#text") &&
				val.getFirstChild() != null) {
				updateJoint(j,key,val);
			}
		}
		return j;
	}

	private void updateJoint(JointNode j, Node keyNode, Node valNode) {		
		String key = keyNode.getFirstChild().getNodeValue();
		//If the node is a text element, parse it and set the appropriate joint variable
		if (valNode.getFirstChild().getNodeName().equals("#text")){
			String val = valNode.getFirstChild().getNodeValue();
			/*if (!key.matches("^\\S+$") || !val.matches("^\\S+$")){
				return;
			}*/
			if (key.equals("Name")) {
				j.name = val;
			} else if (key.equals("θ")) {
				if (val.indexOf("°") != -1) {
					j.theta = Math.toRadians(Double.parseDouble(
						val.substring(0,val.indexOf("°"))));
				} else {
					j.theta = Double.parseDouble(val);
				}
			} else if (key.equals("r")) {
				j.r = Double.parseDouble(val);
			} else if (key.equals("α")) {
				if (val.indexOf("°") != -1) {
					j.alpha = Math.toRadians(Double.parseDouble(
						val.substring(0, val.indexOf("°"))));
				} else {
					j.alpha = Double.parseDouble(val);
				}
			} else if (key.equals("d")) {
				j.d = Double.parseDouble(val);
			} else if (key.equals("qOffset")) {
				if (val.indexOf("°") != -1) {
					j.qOffset = Math.toRadians(Double.parseDouble(
						val.substring(0, val.indexOf("°"))));
				} else {
					j.qOffset = Double.parseDouble(val);
				}	
			} else if (key.equals("Min")) {
				if (val.indexOf("°") != -1) {
					j.min = Math.toRadians(Double.parseDouble(
						val.substring(0, val.indexOf("°"))));
				} else if(val.indexOf("inf")!=-1 || val.indexOf("∞")!=-1) {
					if(val.indexOf("-")!=-1) {
						j.min = Double.NEGATIVE_INFINITY;
					} else {
						j.min = Double.POSITIVE_INFINITY;
					}
				} else {
					j.min = Double.parseDouble(val);
				}
			} else if (key.equals("Max")) {
				if (val.indexOf("°") != -1) {
					j.max = Math.toRadians(Double.parseDouble(
						val.substring(0, val.indexOf("°"))));
				} else if(val.indexOf("inf")!=-1 || val.indexOf("∞")!=-1) {
					if(val.indexOf("-")!=-1) {
						j.max = Double.NEGATIVE_INFINITY;
					} else {
						j.max = Double.POSITIVE_INFINITY;
					}
				} else {
					j.max = Double.parseDouble(val);
				}
			} else if (key.equals("Mass")) {
				j.mass = Double.parseDouble(val);
			} else if (key.equals("Model")) {
				j.model = val;
			} else if (key.equals("IKSolver")) {
				j.iksolver = val;
			} else if (key.equals("Material")) {
				j.material = val;
			} else if (key.equals("JointType")) {
				j.type = val;
			} else if (key.equals("Components")) {
				JointNode jj = new JointNode();
				jj = parse();
				jj.setComponent();
				j.addChild(jj);
				
			} else if (key.equals("CollisionModel")) {
				j.CollisionModel = val;
			} else if (key.equals("CollisionModelScale")) {
				Node cscaleNode = skipText(valNode.getFirstChild());
				if(cscaleNode!=null) {
					j.cscale[0] = Double.parseDouble(cscaleNode.getFirstChild().getNodeValue());
					cscaleNode = skipText(cscaleNode.getNextSibling());
					j.cscale[1] = Double.parseDouble(cscaleNode.getFirstChild().getNodeValue());
					cscaleNode = skipText(cscaleNode.getNextSibling());
					j.cscale[2] = Double.parseDouble(cscaleNode.getFirstChild().getNodeValue());
				} else {
					j.cscale[0] = j.cscale[1] = j.cscale[2] = Double.parseDouble(val);
				}
			} 
			  else if (key.equals("CollisionModelOffset")) {
				Node cOffsetNode = skipText(valNode.getFirstChild());
			 if(cOffsetNode!=null) {
					j.cOffset[0] = Double.parseDouble(cOffsetNode.getFirstChild().getNodeValue());
					cOffsetNode = skipText(cOffsetNode.getNextSibling());
					j.cOffset[1] = Double.parseDouble(cOffsetNode.getFirstChild().getNodeValue());
					cOffsetNode = skipText(cOffsetNode.getNextSibling());
					j.cOffset[2] = Double.parseDouble(cOffsetNode.getFirstChild().getNodeValue());
				} else {
					j.cOffset[0] = j.cOffset[1] = j.cOffset[2] = Double.parseDouble(val);
				}
			} 
                         else if (key.equals("CollisionModelRotation")) {
                                Node crotationNode = skipText(valNode.getFirstChild());
                         if(crotationNode!=null) {
                                        j.crotation[0] = Double.parseDouble(crotationNode.getFirstChild().getNodeValue());
                                        crotationNode = skipText(crotationNode.getNextSibling());
                                        j.crotation[1] = Double.parseDouble(crotationNode.getFirstChild().getNodeValue());
                                        crotationNode = skipText(crotationNode.getNextSibling());
                                        j.crotation[2] = Double.parseDouble(crotationNode.getFirstChild().getNodeValue());
                                } else {
                                        j.crotation[0] = j.crotation[1] = j.crotation[2] = Double.parseDouble(val);
                                }
                       }
			 else if (key.equals("CenterOfMass")) {
				Node COMNode = skipText(valNode.getFirstChild());
			if(COMNode!=null) {
					j.COM[0] = Double.parseDouble(COMNode.getFirstChild().getNodeValue());
					COMNode = skipText(COMNode.getNextSibling());
					j.COM[1] = Double.parseDouble(COMNode.getFirstChild().getNodeValue());
					COMNode = skipText(COMNode.getNextSibling());
					j.COM[2] = Double.parseDouble(COMNode.getFirstChild().getNodeValue());
				} else {
					j.COM[0] = j.COM[1] = j.COM[2] = Double.parseDouble(val);
				}
			} 
			 else if (key.equals("ModelScale")) {
				Node scaleNode = skipText(valNode.getFirstChild());
				if(scaleNode!=null) {
					j.scale[0] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
					scaleNode = skipText(scaleNode.getNextSibling());
					j.scale[1] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
					scaleNode = skipText(scaleNode.getNextSibling());
					j.scale[2] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
				} else {
					j.scale[0] = j.scale[1] = j.scale[2] = Double.parseDouble(val);
				}
			}
		} else 
			if (key.equals("ModelScale")) {
				Node scaleNode = skipText(valNode.getFirstChild());
				j.scale[0] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
				scaleNode = skipText(scaleNode.getNextSibling());
				j.scale[1] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
				scaleNode = skipText(scaleNode.getNextSibling());
				j.scale[2] = Double.parseDouble(scaleNode.getFirstChild().getNodeValue());
			}
		
	}
	Node skipText(Node n) {
		while(n!=null && n.getNodeName().equals("#text"))
			n = n.getNextSibling();
		return n;
	}
}

