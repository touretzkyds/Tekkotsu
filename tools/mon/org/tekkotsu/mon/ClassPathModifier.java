package org.tekkotsu.mon;

import java.lang.*;
import java.net.*;
import java.io.*;
import java.lang.reflect.*;

public class ClassPathModifier {
  private static final Class[] parameters = new Class[]{URL.class};

  public static void addFile(String s) throws IOException {
    File f = new File(s);
    addFile(f);
  }

  public static void addFile(File f) throws IOException {
    addURL(f.toURI().toURL());
  }

  public static void addURL(URL u) throws IOException {
    URLClassLoader sysloader =
      (URLClassLoader)ClassLoader.getSystemClassLoader();

		Class sysclass = URLClassLoader.class;

    try {
      Method method = sysclass.getDeclaredMethod("addURL",parameters);
      method.setAccessible(true);
      method.invoke(sysloader, new Object[]{ u });
    } catch (Throwable t) {
      throw new IOException("could not add "+u+" to classpath");
		}
  }
}
