package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Color;
import java.awt.Font;
import java.awt.Insets;
import java.awt.event.KeyListener;
import java.awt.event.KeyEvent;
import java.awt.event.ComponentListener;
import java.awt.event.ComponentEvent;

/* TODO:
   JTextPane
   History, Autocomplete
*/
public class Terminal implements KeyListener, ComponentListener {
    static final int MAX_CHARS = 30000;
    JFrame _term;
    JTextArea _output;
    JTextField _input;
    JScrollPane _scroll;
    JScrollBar _scroll_bar;
    int _width, _height;
    String _title;
    String _data;

    public static void main(String[] args) {
//      Terminal terminal=new Terminal();
      System.out.println("please instantiate from within Matlab");
      System.exit(1);
    }

    public Terminal() {
      _height=400;
      _width=600;
      _title="AiboMon";
      _data="";

      createTerm();
    }

    public void close() {
      _term.setVisible(false);
    }

    public boolean wasClosed() {
      return !_term.isVisible();
    }

    void createTerm() {
      _term=new JFrame("AiboMon");
      _term.setBackground(Color.darkGray);
      _term.getContentPane().setLayout(new BorderLayout());

      _output=new JTextArea();
      _output.setBackground(Color.black);
      _output.setForeground(Color.lightGray);
      _output.setFont(new Font("Monospaced",Font.PLAIN, 12));
      _output.setLineWrap(true);
      _output.setEditable(false);
      _scroll=new JScrollPane(_output,JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                      JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
      _term.getContentPane().add(_scroll, BorderLayout.NORTH);

      _input=new JTextField();
      _input.setBackground(Color.black);
      _input.setForeground(Color.white);
      _input.setFont(new Font("Monospaced",Font.PLAIN,12));
      _term.getContentPane().add(_input, BorderLayout.SOUTH);

      _input.addKeyListener(this);
      _term.addComponentListener(this);
      _term.setVisible(true);
      _term.setSize(new Dimension(_width, _height));
      resize();

      _scroll_bar=_scroll.getVerticalScrollBar();
    }
    
    public void keyPressed(KeyEvent e) {
      switch(e.getKeyCode()) {
        case KeyEvent.VK_ENTER:
          _output.append(_input.getText()+"\n");
          _data=_data+_input.getText()+"\n";
          _input.setText("");
          break;
        case KeyEvent.VK_UP:
          break;
        case KeyEvent.VK_DOWN:
          break;
      }
    }

    public boolean hasData() {
      return _data.length()!=0;
    }

    public String getData() {
      String ret;
      ret=_data;
      _data="";
      return ret;
    }

    public void write(String s) {
      _output.append(s);
      _scroll_bar.setValue(_scroll_bar.getMaximum());
    }

    public void clear() {
      _output.setText("");
    }

    void resize() {
      Dimension dim=_term.getSize();
      Insets ins=_term.getInsets();
      int width=dim.width-ins.left-ins.right;
      int height=dim.height-ins.top-ins.bottom;
      _scroll.setSize(new Dimension(width, height-25));
      _input.setSize(new Dimension(width,25));
    }

    public void componentResized(ComponentEvent e) {
      if (e.getID()==ComponentEvent.COMPONENT_RESIZED) resize();
    }

    public void keyReleased(KeyEvent e) { }
    public void keyTyped(KeyEvent e) { }
    public void componentHidden(ComponentEvent e) { }
    public void componentMoved(ComponentEvent e) { }
    public void componentShown(ComponentEvent e) { }
}

