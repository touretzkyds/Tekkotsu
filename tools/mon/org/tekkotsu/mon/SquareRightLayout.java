package org.tekkotsu.mon;

import java.awt.*;

public class SquareRightLayout implements LayoutManager {
	Component right;
	Component square;
	static String SQUARE="square";
	static String RIGHT="right";
	public void addLayoutComponent(String name, Component comp) {
		if(name.compareTo(SQUARE)==0)
			square=comp;
		else if(name.compareTo(RIGHT)==0)
			right=comp;
	}
	public void layoutContainer(Container parent) {
		square.setBounds(0,0,parent.getHeight(),parent.getHeight());		
		right.setBounds(parent.getHeight(),0,parent.getWidth()-parent.getHeight(),parent.getHeight());
	}
	public Dimension minimumLayoutSize(Container parent) {
		Dimension sq=square.getMinimumSize();
		Dimension rt=right.getMinimumSize();
		int minsq=sq.width>sq.height?sq.width:sq.height;
		if(minsq<rt.height)
			minsq=rt.height;
//			int minheight=minsq>rt.height?minsq:rt.height
		return new Dimension(minsq+rt.width,minsq);
	}
	public Dimension preferredLayoutSize(Container parent) {
		Dimension sq=square.getPreferredSize();
		Dimension rt=right.getPreferredSize();
		int prefsq=sq.width>sq.height?sq.width:sq.height;
		if(prefsq<rt.height)
			prefsq=rt.height;
//			int prefheight=minsq>rt.height?minsq:rt.height
		return new Dimension(prefsq+rt.width,prefsq);
	}
	public void removeLayoutComponent(Component comp) {
		if(square==comp)
			square=null;
		else if(right==comp)
			right=null;
	}
}
