package org.tekkotsu.mon;

/**
 * A sorter for TableModels. The sorter has a model (conforming to TableModel)
 * and itself implements TableModel. TableSorter does not store or copy
 * the data in the TableModel, instead it maintains an array of
 * integers which it keeps the same size as the number of rows in its
 * model. When the model changes it notifies the sorter that something
 * has changed eg. "rowsAdded" so that its internal array of integers
 * can be reallocated. As requests are made of the sorter (like
 * getValueAt(row, col) it redirects them to its model via the mapping
 * array. That way the TableSorter appears to hold another copy of the table
 * with the rows in a different order. The sorting algorthm used is stable
 * which means that it does not move around rows when its comparison
 * function returns 0 to denote that they are equivalent.
 *
 * @version 1.5 12/17/97
 * @author Philip Milne
 */

// Alok Ladsariya
// (heavily modified)

import java.util.*;
import java.lang.*;
import javax.swing.table.*;
import javax.swing.event.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.table.*;

public class TableSorter extends TableMap {
  int[] forwardIndices;
  int[] reverseIndices;

  public int sortColumn=0;
  public boolean sortOrder=true;

  public TableSorter() {
    forwardIndices=new int[0];
    reverseIndices=new int[0];
  }

  public TableSorter(TableModel model) {
    this();
    setModel(model);
  }

  public TableSorter(TableModel model, int defColumn) {
    this(model);
    sortColumn=defColumn;
  }

  public void setModel(TableModel model) {
    super.setModel(model);
    reallocate();
    sort();
  }

  public void reallocate() {
    int rowCount=model.getRowCount();

    forwardIndices=new int[rowCount];
    reverseIndices=new int[rowCount];
    for (int row=0; row<rowCount; row++) {
      forwardIndices[row]=row;
      reverseIndices[row]=row;
    }
  }

  public void deleteRow(int i) {
    reallocate();
    sort();
  }

  public void insertRow(int i) {
    reallocate();
    sort();
  }

  public void updateRow(int i) {
    sort();
  }

  public final int compareRowsByColumn(int row1, int row2, int column) {
    Class type = model.getColumnClass(column);
    TableModel data = model;

    // Check for nulls.

    Object o1 = data.getValueAt(row1, column);
    Object o2 = data.getValueAt(row2, column); 

    // If both values are null, return 0.
    if (o1 == null && o2 == null) {
      return 0; 
    } else if (o1 == null) { // Define null less than everything. 
      return -1; 
    } else if (o2 == null) { 
      return 1; 
    }

    /*
     * We copy all returned values from the getValue call in case
     * an optimised model is reusing one object to return many
     * values.  The Number subclasses in the JDK are immutable and
     * so will not be used in this way but other subclasses of
     * Number might want to do this to save space and avoid
     * unnecessary heap allocation.
     */

    if (type.getSuperclass() == java.lang.Number.class) {
      Number n1 = (Number)data.getValueAt(row1, column);
      double d1 = n1.doubleValue();
      Number n2 = (Number)data.getValueAt(row2, column);
      double d2 = n2.doubleValue();

      if (d1 < d2) {
        return -1;
      } else if (d1 > d2) {
        return 1;
      } else {
        return 0;
      }
    } else if (type == java.util.Date.class) {
      Date d1 = (Date)data.getValueAt(row1, column);
      long n1 = d1.getTime();
      Date d2 = (Date)data.getValueAt(row2, column);
      long n2 = d2.getTime();

      if (n1 < n2) {
        return -1;
      } else if (n1 > n2) {
        return 1;
      } else {
        return 0;
      }
    } else if (type == String.class) {
      String s1 = (String)data.getValueAt(row1, column);
      String s2    = (String)data.getValueAt(row2, column);
      int result = s1.compareTo(s2);

      if (result < 0) {
        return -1;
      } else if (result > 0) {
        return 1;
      } else {
        return 0;
      }
    } else if (type == Boolean.class) {
      Boolean bool1 = (Boolean)data.getValueAt(row1, column);
      boolean b1 = bool1.booleanValue();
      Boolean bool2 = (Boolean)data.getValueAt(row2, column);
      boolean b2 = bool2.booleanValue();

      if (b1 == b2) {
        return 0;
      } else if (b1) { // Define false < true
        return 1;
      } else {
        return -1;
      }
    } else {
      Object v1 = data.getValueAt(row1, column);
      String s1 = v1.toString();
      Object v2 = data.getValueAt(row2, column);
      String s2 = v2.toString();
      int result = s1.compareTo(s2);

      if (result < 0) {
        return -1;
      } else if (result > 0) {
        return 1;
      } else {
        return 0;
      }
    }
  }

  public final int compare(int row1, int row2) {
    int result = compareRowsByColumn(row1, row2, sortColumn);
    if (result != 0) {
      return sortOrder ? result : -result;
    }
    return 0;
  }

  public void tableChanged(TableModelEvent e) {
    if (e.getType()==TableModelEvent.INSERT) {
      if (e.getFirstRow()==e.getLastRow())
        insertRow(e.getFirstRow());
      else {
        reallocate();
        sort();
        super.tableChanged(new TableModelEvent(this));
        return;
      }
    } else if (e.getType()==TableModelEvent.UPDATE &&
        e.getColumn()==sortColumn) {
/*    Dynamic sorting disabled
      if (e.getFirstRow()==e.getLastRow())
        updateRow(e.getFirstRow());
      else {
        sort();
        super.tableChanged(new TableModelEvent(this));
        return;
      }*/
    } else if (e.getType()==TableModelEvent.DELETE) {
      if (e.getFirstRow()==e.getLastRow())
        deleteRow(e.getFirstRow());
      else {
        reallocate();
        sort();
        super.tableChanged(new TableModelEvent(this));
        return;
      }
    }

    for (int i=e.getFirstRow(); i<=e.getLastRow(); i++) {
      super.tableChanged(new TableModelEvent(this,
                                             reverseIndices[i],
                                             reverseIndices[i],
                                             e.getColumn(),
                                             e.getType()));
    }
  }

  public void checkModel() {
    if (forwardIndices.length != model.getRowCount()) {
      System.err.println("Sorter not informed of a change in model.");
    }
  }

  public void sort() {
    checkModel();
    sort(forwardIndices);
    for (int i=0; i<forwardIndices.length; i++) {
      reverseIndices[forwardIndices[i]]=i;
    }
  }

  // The mapping only affects the contents of the data rows.
  // Pass all requests to these rows through the mapping array: "indexes".

  public Object getValueAt(int aRow, int aColumn) {
    checkModel();
    return model.getValueAt(forwardIndices[aRow], aColumn);
  }

  public void setValueAt(Object aValue, int aRow, int aColumn) {
    checkModel();
    model.setValueAt(aValue, forwardIndices[aRow], aColumn);
  }

  public int translateRow(int aRow) {
    return forwardIndices[aRow];
  }

  // There is no-where else to put this. 
  // Add a mouse listener to the Table to trigger a table sort 
  // when a column heading is clicked in the JTable. 
  public void addMouseListenerToHeaderInTable(JTable table) { 
    final TableSorter sorter = this; 
    final JTable tableView = table; 
    tableView.setColumnSelectionAllowed(false); 
    MouseAdapter listMouseListener = new MouseAdapter() {
      public void mouseClicked(MouseEvent e) {
        TableColumnModel columnModel = tableView.getColumnModel();
        int viewColumn = columnModel.getColumnIndexAtX(e.getX()); 
        int column = tableView.convertColumnIndexToModel(viewColumn); 
        if (e.getClickCount() == 1 && column != -1) {
          if (column==sorter.sortColumn) {
            sorter.sortOrder=!sorter.sortOrder;
          } else {
            sorter.sortColumn=column;
            sorter.sortOrder=true;
          }
          sorter.sort();
        }
      }
    };
    JTableHeader th = tableView.getTableHeader(); 
    th.addMouseListener(listMouseListener); 
  }

  /*
   * Quicksort code from:
   * QSortAlgorithm.java      1.3   29 Feb 1996 James Gosling
   *
   * Copyright (c) 1994-1996 Sun Microsystems, Inc. All Rights Reserved.
   *
   * James Gosling
   * Kevin A. Smith
   * extended with TriMedian and InsertionSort by Denis Ahrens
   * with all the tips from Robert Sedgewick (Algorithms in C++).
   * It uses TriMedian and InsertionSort for lists shorts than 4.
   * <fuhrmann@cs.tu-berlin.de>
   */

  private void QuickSort(int a[], int l, int r)
  {
    int M = 4;
    int i;
    int j;
    int v;

    if ((r-l)>M)
    {
      i = (r+l)/2;
      if (compare(a[l],a[i])>0) swap(a,l,i);     // Tri-Median Methode!
      if (compare(a[l],a[r])>0) swap(a,l,r);
      if (compare(a[i],a[r])>0) swap(a,i,r);

      j = r-1;
      swap(a,i,j);
      i = l;
      v = a[j];
      for(;;)
      {
        while(compare(a[++i],v)<0);
        while(compare(a[--j],v)>0);
        if (j<i) break;
        swap (a,i,j);
      }
      swap(a,i,r-1);
      QuickSort(a,l,j);
      QuickSort(a,i+1,r);
    }
  }

  private void swap(int a[], int i, int j)
  {
    int T;
    T = a[i]; 
    a[i] = a[j];
    a[j] = T;
  }

  private void InsertionSort(int a[], int lo0, int hi0)
  {
    int i;
    int j;
    int v;

    for (i=lo0+1;i<=hi0;i++)
    {
      v = a[i];
      j=i;
      while ((j>lo0) && compare(a[j-1],v)>0)
      {
        a[j] = a[j-1];
        j--;
      }
      a[j] = v;
    }
  }

  public void sort(int a[])
  {
    QuickSort(a, 0, a.length - 1);
    InsertionSort(a,0,a.length-1);
  }
}
