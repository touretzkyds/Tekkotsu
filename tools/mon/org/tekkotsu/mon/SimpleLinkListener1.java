package org.tekkotsu.mon;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.text.*;
import javax.swing.text.html.*;


public class SimpleLinkListener1 implements HyperlinkListener {
    private JEditorPane pane;        // The pane we're using to display HTML
    private JTextField urlField;     // An optional text field for showing
    // the current URL being displayed
    private JLabel statusBar;        // An optional label for showing where
    // a link would take you
    public SimpleLinkListener1(JEditorPane jep, JTextField jtf, JLabel jl) {
	pane = jep;
	urlField = jtf;
	statusBar = jl;
    }
    public SimpleLinkListener1(JEditorPane jep) {
    this(jep, null, null);
    }
    public void hyperlinkUpdate(HyperlinkEvent he) {
	HyperlinkEvent.EventType type = he.getEventType();
    if (type == HyperlinkEvent.EventType.ENTERED) {
	// Enter event. Fill in the status bar
	if (statusBar != null) {
	    statusBar.setText(he.getURL().toString());
	}
    }
    else if (type == HyperlinkEvent.EventType.EXITED) {
	// Exit event. Clear the status bar
	if (statusBar != null) {
	    statusBar.setText(" "); // must be a space or it disappears
	}
    }
    else {
	// Jump event. Get the url, and if it's not null, switch to that
	// page in the main editor pane and update the "site url" label.
	if (he instanceof HTMLFrameHyperlinkEvent) {
	    // ahh, frame event, handle this separately
	    HTMLFrameHyperlinkEvent evt = (HTMLFrameHyperlinkEvent)he;
	    HTMLDocument doc = (HTMLDocument)pane.getDocument();
	    doc.processHTMLFrameHyperlinkEvent(evt);
	} else {
	    try {
		pane.setPage(he.getURL());
		if (urlField != null) {
		    urlField.setText(he.getURL().toString());
		}
	    }
	    
	    catch (FileNotFoundException fnfe) {
		pane.setText("Could not open file: <tt>" + he.getURL() +
			     "</tt>.<hr>");
	    }
	    catch (Exception e) {
		e.printStackTrace();
	    }
	}
    }
    }
}

