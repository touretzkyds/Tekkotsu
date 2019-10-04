/** @file ImageData.java
 *  @brief 
 *
 *  File that contains the functions for the data
 *  contained in each image.
 *
 *	@author editted by: Eric Durback
 *  @bug No known bugs.
 */

import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.util.*;
import java.awt.geom.*;
import java.io.*;
import java.awt.image.Raster;
import javax.imageio.ImageIO;
import javax.imageio.ImageReader;
import javax.imageio.stream.ImageInputStream;
import javax.imageio.stream.MemoryCacheImageInputStream;


public class ImageData extends Frame {
	String[] files;
	int[][] originalImages;
	BufferedImage[] rgbImages;
	float[][] yuvImages;
	float[][] spectrumImages;

	ColorConverter OriginalToRGB;
	ColorConverter YUVToSpectrum;
	ColorConverter OriginalToYUV;
	
	int[] widths;
	int[] heights;
	
	int numImages;
	
	public ImageData(String[] files, ColorConverter toRGB, ColorConverter toYUV, ColorConverter toSpectrum) {
		OriginalToRGB=toRGB;
		OriginalToYUV=toYUV;
		YUVToSpectrum=toSpectrum;
		loadFiles(files);
	}
	
	public void loadFiles(String[] loadfiles) {
		files=loadfiles;
		numImages = files.length;
		originalImages=new int[numImages][];
		rgbImages=new BufferedImage[numImages];
		spectrumImages=new float[numImages][];
		yuvImages=new float[numImages][];
		widths=new int[numImages];
		heights=new int[numImages];
		
		float[] tmp=new float[3];
		
		for(int i=0; i<numImages; i++) {
			//System.out.println("i="+i+" numImages="+numImages+" files="+files.length);
			File f = new File(files[i]);
			if(!f.isFile()) {
				System.out.println("Could not find a File named "+files[i]);
				System.exit(1);
			}
			
			if(f.getName().toLowerCase().endsWith(".jpeg") || f.getName().toLowerCase().endsWith(".jpg")) {
				//jpegs actually store YUV internally, we can access it directly
				try {
					System.out.println(f+" is a jpeg");
					ImageInputStream jpegStream=new MemoryCacheImageInputStream(new FileInputStream(f));
					ImageReader jpegReader=(ImageReader)ImageIO.getImageReadersByFormatName("jpeg").next();
					jpegReader.setInput(jpegStream); 
					
					BufferedImage bi=jpegReader.read(0);
					if(bi==null) {
						System.out.println(f+" is not a valid image file (error during loading).  Skipping...");
						removeFile(i);
						i--;
						continue;
					}
					widths[i]=bi.getWidth();
					heights[i]=bi.getHeight();
					originalImages[i]=bi.getRGB(0,0,widths[i],heights[i],null,0,widths[i]);
					
					Raster decoded=jpegReader.readRaster(0,null);
					if(decoded==null) {
						System.out.println(f+" is not a valid image file (error during loading).  Skipping...");
						removeFile(i);
						i--;
						continue;
					}
					yuvImages[i] = new float[originalImages[i].length*3];
					int off=0;
					for(int y=0; y<heights[i]; y++)
						for(int x=0; x<widths[i]; x++) {
							yuvImages[i][off++]=decoded.getSampleFloat(x,y,0)/255;
							yuvImages[i][off++]=decoded.getSampleFloat(x,y,1)/255;
							yuvImages[i][off++]=decoded.getSampleFloat(x,y,2)/255;
						}

					rgbImages[i]=new BufferedImage(widths[i], heights[i], BufferedImage.TYPE_INT_RGB);
					rgbImages[i].getRaster().setDataElements(0,0,widths[i],heights[i],originalImages[i]);
				} catch(Exception ex) {
					System.out.println(f+" is not a valid image file (error during loading).  Skipping...");
					//e.printStackTrace();
					//System.exit(1);
					removeFile(i);
					i--;
					continue;
				} 
			} else {
				//read as a generic image
				BufferedImage bi=null;
				try {
					bi=ImageIO.read(f);
				} catch (Exception e) {
					System.out.println(f+" is not a valid image file (error during loading).  Skipping...");
					//e.printStackTrace();
					//System.exit(1);
					removeFile(i);
					i--;
					continue;
				}
				if(bi==null) {
					System.out.println(f+" is not a valid image file (error during loading).  Skipping...");
					removeFile(i);
					i--;
					continue;
				}
				widths[i]=bi.getWidth();
				heights[i]=bi.getHeight();
				originalImages[i]=bi.getRGB(0,0,widths[i],heights[i],null,0,widths[i]);
				
				yuvImages[i] = new float[originalImages[i].length*3];
				if(OriginalToYUV==null) {
					int off=0;
					for(int j=0; j<originalImages[i].length; j++) {
						yuvImages[i][off++]=((originalImages[i][j]>>16)&0xFF)/255.f;
						yuvImages[i][off++]=((originalImages[i][j]>>8)&0xFF)/255.f;
						yuvImages[i][off++]=((originalImages[i][j]>>0)&0xFF)/255.f;
					}
				} else {
					int off=0;
					for(int j=0; j<originalImages[i].length; j++) {
						OriginalToYUV.getColor(originalImages[i][j],tmp);
						yuvImages[i][off++]=tmp[0];
						yuvImages[i][off++]=tmp[1];
						yuvImages[i][off++]=tmp[2];						
					}
				}

				rgbImages[i]=new BufferedImage(widths[i], heights[i], BufferedImage.TYPE_INT_RGB);
				if(OriginalToRGB==null) {
					rgbImages[i].getRaster().setDataElements(0,0,widths[i],heights[i],originalImages[i]);
					//rgbImages[i].getRaster().setPixels(0,0,widths[i],heights[i],yuvImages[i]);
				} else {
					int[] rgbData=new int[originalImages[i].length*3];
					for(int j=0; j<originalImages[i].length; j++) {
						OriginalToRGB.getColor(originalImages[i][j],tmp);
						rgbData[j]=Math.round(tmp[0]*255)<<16 | Math.round(tmp[1]*255)<<8 | Math.round(tmp[2]*255);
					}
					rgbImages[i].getRaster().setDataElements(0,0,widths[i],heights[i],rgbData);
				}
			}
			
		}

		changeSpectrum(YUVToSpectrum);
			
	}

	void removeFile(int i) {
		numImages--;

		String[] fs=new String[numImages];
		int[][] ois=new int[numImages][];
		BufferedImage[] ris=new BufferedImage[numImages];
		float[][] sis=new float[numImages][];
		float[][] yis=new float[numImages][];
		int[] ws=new int[numImages];
		int[] hs=new int[numImages];
		for(int j=0; j<i; j++) {
			fs[j]=files[j];
			ois[j]=originalImages[j];
			ris[j]=rgbImages[j];
			sis[j]=spectrumImages[j];
			yis[j]=yuvImages[j];
			ws[j]=widths[j];
			hs[j]=heights[j];
		}
		for(int j=i; j<numImages; j++) {
			fs[j]=files[j+1];
			ois[j]=originalImages[j+1];
			ris[j]=rgbImages[j+1];
			sis[j]=spectrumImages[j+1];
			yis[j]=yuvImages[j+1];
			ws[j]=widths[j+1];
			hs[j]=heights[j+1];
		}
		files=fs;
		originalImages=ois;
		rgbImages=ris;
		spectrumImages=sis;
		yuvImages=yis;
		widths=ws;
		heights=hs;
	}
	
	public void changeSpectrum(ColorConverter YUVToSpectrum) {
		this.YUVToSpectrum=YUVToSpectrum;
		if(YUVToSpectrum==null) {
			spectrumImages=yuvImages;
		} else {
			float[] tmp=new float[3];
			for(int i=0; i<numImages; i++) {
				if(spectrumImages[i]==null || spectrumImages[i].length!=originalImages[i].length*3)
					spectrumImages[i]=new float[originalImages[i].length*3];
				for(int j=0; j<yuvImages[i].length; j+=3) {
					YUVToSpectrum.getColor(yuvImages[i][j],yuvImages[i][j+1],yuvImages[i][j+2],tmp);					
					spectrumImages[i][j]=tmp[0];
					spectrumImages[i][j+1]=tmp[1];
					spectrumImages[i][j+2]=tmp[2];						
				}
			}
		}
		
	}

	public int getNumImages() { return numImages; }
	public String[] getFileNames() { return files; }
	
	public BufferedImage[] getRGBImages() { return rgbImages; }
	public float[][] getSpectrumImages() { return spectrumImages; }
	public float[][] getYUVImages() { return yuvImages; }
	
	public BufferedImage getRGBImage(int i) { return rgbImages[i]; }
	public float[] getSpectrumImage(int i) { return spectrumImages[i]; }
	public float[] getYUVImage(int i) { return yuvImages[i]; }
		
	public int getHeight(int i) { return heights[i]; }
	public int getWidth(int i) { return widths[i]; }
	
}
